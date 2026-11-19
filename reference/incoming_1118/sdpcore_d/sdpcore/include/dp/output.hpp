// Блок вывода: циклическая передача команд движителям на контроллер по Modbus (TCP или RTU).
// Таблица «движитель → регистр» настраивается: единицы (проценты, мощность или иное), масштаб, смещение, пределы.
#pragma once
#include <algorithm>
#include <atomic>
#include <map>
#include <mutex>

#include "dp/control.hpp"
#include "dp/modbus.hpp"

namespace dp {

enum class OutField { Pct, AngleDeg };

struct RegEntry {
    std::string thrusterId;
    OutField field = OutField::Pct;
    int address = 0;                // адрес регистра (нумерация с нуля, как в протоколе)
    double unitPerPct = 1.0;        // для Pct: единиц величины на 1 % команды (проценты — 1; кВт — powerMaxKW/100)
    double scale = 1.0;             // регистр = round(значение · scale + offset)
    double offset = 0.0;
    bool isSigned = true;           // int16 или uint16
    long rawMin = -32768, rawMax = 32767;   // пределы кода
    bool angle360 = true;           // для AngleDeg: приводить к 0…360 (иначе −180…180)
    double safeValue = 0.0;         // значение, передаваемое при потере команд (для Pct)
};

struct LinkCfg {
    std::string id = "CTRL1";
    // транспорт
    bool serial = false;
    std::string host = "127.0.0.1"; int port = 502;                 // Modbus TCP
    std::string device; int baud = 9600; char parity = 'N'; int stopBits = 1;   // Modbus RTU
    int unitId = 1;
    int timeoutMs = 300;
    double periodMs = 100;          // период обмена для этой линии (задаётся под оборудование)
    double staleTimeoutS = 1.0;     // если команды не обновлялись дольше — передаём безопасные значения
    double retryS = 2.0;
    std::vector<RegEntry> entries;
    int heartbeatReg = -1;          // счётчик «жив», увеличивается каждый обмен (−1 — не используется)
    int modeReg = -1;               // код режима СДП
    int healthReg = -1;             // 1 — команды свежие, 0 — устарели
};

struct LinkStatus {
    std::string id;
    bool connected = false, stale = false;
    long long okCycles = 0, failedCycles = 0;
    std::string lastError;
};

class OutputLink {
public:
    explicit OutputLink(LinkCfg c) : cfg_(std::move(c)) {}
    ~OutputLink() { stop(); }

    void start() { if (running_) return; running_ = true; th_ = std::thread([this] { run(); }); }
    void stop() { running_ = false; if (th_.joinable()) th_.join(); }

    // Текущие команды от ядра. now — монотонное время, с.
    void setCommands(const std::vector<ThrusterCmd>& cmds, DpMode mode, double now) {
        std::lock_guard<std::mutex> lk(m_);
        cmds_.clear();
        for (const auto& c : cmds) cmds_[c.id] = c;
        mode_ = mode; lastSet_ = now; haveCmds_ = true;
    }
    LinkStatus status() const { std::lock_guard<std::mutex> lk(sm_); return st_; }

    // Перевод команды в код регистра (доступно для проверки).
    static uint16_t encode(const RegEntry& e, const ThrusterCmd& c) {
        double v;
        if (e.field == OutField::Pct) v = clampd(c.pct, -100.0, 100.0) * e.unitPerPct;
        else {
            double a = std::fmod(c.angleDeg, 360.0);
            if (e.angle360) { if (a < 0) a += 360.0; } else { if (a >= 180.0) a -= 360.0; if (a < -180.0) a += 360.0; }
            v = a;
        }
        long raw = std::lround(v * e.scale + e.offset);
        raw = std::max(e.rawMin, std::min(e.rawMax, raw));
        if (!e.isSigned && raw < 0) raw = 0;
        return static_cast<uint16_t>(raw);   // дополнительный код для отрицательных значений
    }
    static ThrusterCmd decode(const RegEntry& e, uint16_t reg) {   // обратное преобразование, для проверок
        ThrusterCmd c; c.id = e.thrusterId;
        const double raw = e.isSigned ? static_cast<double>(static_cast<int16_t>(reg)) : static_cast<double>(reg);
        const double v = (raw - e.offset) / e.scale;
        if (e.field == OutField::Pct) c.pct = v / e.unitPerPct; else c.angleDeg = v;
        return c;
    }

private:
    void setStatus(bool conn, bool stale, bool ok, const std::string& err) {
        std::lock_guard<std::mutex> lk(sm_);
        st_.id = cfg_.id; st_.connected = conn; st_.stale = stale;
        if (ok) ++st_.okCycles; else { ++st_.failedCycles; st_.lastError = err; }
    }

    void run() {
        std::unique_ptr<mb::Master> master;
        auto makeMaster = [&] {
            std::unique_ptr<mb::Transport> t;
            if (cfg_.serial) t = std::make_unique<mb::SerialTransport>(cfg_.device, cfg_.baud, cfg_.parity, cfg_.stopBits);
            else t = std::make_unique<mb::TcpTransport>(cfg_.host, cfg_.port);
            return std::make_unique<mb::Master>(std::move(t), cfg_.unitId, cfg_.timeoutMs);
        };
        master = makeMaster();
        auto next = std::chrono::steady_clock::now();
        uint16_t beat = 0;
        while (running_) {
            next += std::chrono::microseconds(static_cast<long long>(cfg_.periodMs * 1000));
            if (!master->isOpen()) {
                std::string err;
                if (!master->open(err)) {
                    setStatus(false, true, false, err);
                    for (double t = 0; t < cfg_.retryS && running_; t += 0.1) std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    next = std::chrono::steady_clock::now();
                    continue;
                }
            }
            // снимок команд
            std::map<std::string, ThrusterCmd> cmds; DpMode mode; double last; bool have;
            { std::lock_guard<std::mutex> lk(m_); cmds = cmds_; mode = mode_; last = lastSet_; have = haveCmds_; }
            const bool stale = !have || (monotonicSeconds() - last) > cfg_.staleTimeoutS;
            // регистры
            std::map<int, uint16_t> regs;
            for (const auto& e : cfg_.entries) {
                ThrusterCmd c; auto it = cmds.find(e.thrusterId);
                if (it != cmds.end()) c = it->second;
                if (stale) {                      // безопасное состояние: нулевая тяга, угол сохраняется
                    if (e.field == OutField::Pct) c.pct = e.safeValue;
                    else if (it == cmds.end()) c.angleDeg = 0;
                }
                regs[e.address] = encode(e, c);
            }
            if (cfg_.heartbeatReg >= 0) regs[cfg_.heartbeatReg] = ++beat;
            if (cfg_.modeReg >= 0) regs[cfg_.modeReg] = static_cast<uint16_t>(mode);
            if (cfg_.healthReg >= 0) regs[cfg_.healthReg] = stale ? 0 : 1;
            // непрерывные группы адресов — одной записью
            bool ok = true; std::string err;
            for (auto it = regs.begin(); it != regs.end() && ok;) {
                const int start = it->first; std::vector<uint16_t> blk;
                int a = start;
                while (it != regs.end() && it->first == a && blk.size() < 100) { blk.push_back(it->second); ++it; ++a; }
                ok = master->writeRegisters(start, blk, err);
            }
            if (!ok) master->close();            // переоткроем канал на следующем цикле
            setStatus(master->isOpen(), stale, ok, err);
            std::this_thread::sleep_until(next);
            if (std::chrono::steady_clock::now() > next + std::chrono::seconds(1)) next = std::chrono::steady_clock::now();
        }
        // при остановке — нулевая тяга
        if (master->isOpen()) {
            std::map<int, uint16_t> regs;
            for (const auto& e : cfg_.entries) { ThrusterCmd c; c.pct = e.safeValue; regs[e.address] = e.field == OutField::Pct ? encode(e, c) : 0; }
            if (cfg_.healthReg >= 0) regs[cfg_.healthReg] = 0;
            for (auto it = regs.begin(); it != regs.end();) {
                const int start = it->first; std::vector<uint16_t> blk; int a = start;
                while (it != regs.end() && it->first == a) { blk.push_back(it->second); ++it; ++a; }
                std::string err; master->writeRegisters(start, blk, err);
            }
            master->close();
        }
    }

    LinkCfg cfg_;
    std::atomic<bool> running_{false};
    std::thread th_;
    mutable std::mutex m_, sm_;
    std::map<std::string, ThrusterCmd> cmds_;
    DpMode mode_ = DpMode::Standby; double lastSet_ = 0; bool haveCmds_ = false;
    LinkStatus st_;
};

// Типовая таблица для демонстрации: на каждый движитель два регистра (упор в %, угол в градусах).
inline LinkCfg demoLink(const std::vector<ThrusterCfg>& th) {
    LinkCfg c; c.id = "CTRL1";
    int addr = 100;
    for (const auto& t : th) {
        RegEntry p; p.thrusterId = t.id; p.field = OutField::Pct; p.address = addr++; p.scale = 100.0;   // 0,01 % на единицу
        c.entries.push_back(p);
        if (t.type == ThrusterType::Azimuth) {
            RegEntry a; a.thrusterId = t.id; a.field = OutField::AngleDeg; a.address = addr++; a.scale = 10.0; a.isSigned = false; a.rawMin = 0; a.rawMax = 3600;
            c.entries.push_back(a);
        }
    }
    c.heartbeatReg = 10; c.modeReg = 11; c.healthReg = 12;
    return c;
}

}  // namespace dp
