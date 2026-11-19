// Проверка блока вывода: Modbus TCP и RTU против программного ведомого устройства, затем замкнутый контур
// «ядро → таблица регистров → контроллер (ведомое) → движители имитатора».
#include <atomic>
#include <cstdio>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "dp/output.hpp"
#include "dp/sim.hpp"

using namespace dp;
static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { std::printf("  [ПРОВАЛ] %s\n", msg); ++fails; } else std::printf("  [ок] %s\n", msg); } while (0)
static void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// Общая память ведомого устройства
struct Regs {
    std::mutex m; uint16_t r[400] = {0}; std::atomic<long long> writes{0};
    uint16_t get(int a) { std::lock_guard<std::mutex> lk(m); return r[a]; }
};

// Обработка PDU: возвращает ответный PDU
static std::vector<uint8_t> handlePdu(Regs& R, const std::vector<uint8_t>& p) {
    const int fc = p[0];
    auto exc = [&](int code) { return std::vector<uint8_t>{static_cast<uint8_t>(fc | 0x80), static_cast<uint8_t>(code)}; };
    const int addr = (p[1] << 8) | p[2];
    if (fc == 0x10) {
        const int qty = (p[3] << 8) | p[4];
        if (addr + qty > 400) return exc(2);
        { std::lock_guard<std::mutex> lk(R.m); for (int i = 0; i < qty; ++i) R.r[addr + i] = static_cast<uint16_t>((p[6 + 2 * i] << 8) | p[7 + 2 * i]); }
        ++R.writes;
        return {0x10, p[1], p[2], p[3], p[4]};
    }
    if (fc == 0x06) {
        if (addr >= 400) return exc(2);
        { std::lock_guard<std::mutex> lk(R.m); R.r[addr] = static_cast<uint16_t>((p[3] << 8) | p[4]); }
        return {0x06, p[1], p[2], p[3], p[4]};
    }
    if (fc == 0x03) {
        const int qty = (p[3] << 8) | p[4];
        if (addr + qty > 400) return exc(2);
        std::vector<uint8_t> o = {0x03, static_cast<uint8_t>(qty * 2)};
        std::lock_guard<std::mutex> lk(R.m);
        for (int i = 0; i < qty; ++i) { o.push_back(static_cast<uint8_t>(R.r[addr + i] >> 8)); o.push_back(static_cast<uint8_t>(R.r[addr + i] & 0xFF)); }
        return o;
    }
    return exc(1);
}

static bool recvAll(int c, uint8_t* d, size_t n) {
    size_t off = 0;
    while (off < n) { pollfd p{c, POLLIN, 0}; if (poll(&p, 1, 200) <= 0) return false; ssize_t r = recv(c, d + off, n - off, 0); if (r <= 0) return false; off += r; }
    return true;
}

struct TcpSlave {
    Regs& R; int port; std::atomic<bool> run{true}; std::thread th; int ls = -1; std::atomic<bool> dropNext{false};
    TcpSlave(Regs& r, int p) : R(r), port(p) {
        ls = socket(AF_INET, SOCK_STREAM, 0); int one = 1; setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        sockaddr_in b{}; b.sin_family = AF_INET; b.sin_port = htons(port); inet_pton(AF_INET, "127.0.0.1", &b.sin_addr);
        bind(ls, reinterpret_cast<sockaddr*>(&b), sizeof b); listen(ls, 4);
        th = std::thread([this] {
            while (run) {
                pollfd pa{ls, POLLIN, 0}; if (poll(&pa, 1, 100) <= 0) continue;
                int c = accept(ls, nullptr, nullptr);
                while (run) {
                    uint8_t h[7];
                    pollfd pc{c, POLLIN, 0}; int pr = poll(&pc, 1, 100); if (pr < 0) break; if (pr == 0) continue;
                    if (!recvAll(c, h, 7)) break;
                    const int len = (h[4] << 8) | h[5];
                    std::vector<uint8_t> pdu(static_cast<size_t>(len - 1));
                    if (!recvAll(c, pdu.data(), pdu.size())) break;
                    if (dropNext.exchange(false)) break;       // имитация обрыва связи без ответа
                    auto rsp = handlePdu(R, pdu);
                    std::vector<uint8_t> f = {h[0], h[1], 0, 0, static_cast<uint8_t>((rsp.size() + 1) >> 8), static_cast<uint8_t>((rsp.size() + 1) & 0xFF), h[6]};
                    f.insert(f.end(), rsp.begin(), rsp.end());
                    send(c, f.data(), f.size(), 0);
                }
                close(c);
            }
        });
    }
    ~TcpSlave() { run = false; th.join(); close(ls); }
};

struct RtuSlave {
    Regs& R; int fd; std::atomic<bool> run{true}; std::thread th; int unit;
    RtuSlave(Regs& r, int masterFd, int u) : R(r), fd(masterFd), unit(u) {
        th = std::thread([this] {
            while (run) {
                uint8_t b[260]; pollfd p{fd, POLLIN, 0};
                if (poll(&p, 1, 100) <= 0) continue;
                size_t n = 0;
                auto need = [&](size_t k) { while (n < k) { pollfd q{fd, POLLIN, 0}; if (poll(&q, 1, 100) <= 0) return false; ssize_t r = read(fd, b + n, k - n); if (r <= 0) return false; n += r; } return true; };
                if (!need(2)) continue;
                size_t total = 8;
                if (b[1] == 0x10) { if (!need(7)) continue; total = 9 + b[6]; }
                if (!need(total)) continue;
                if (mb::crc16(b, total - 2) != static_cast<uint16_t>(b[total - 2] | (b[total - 1] << 8))) continue;   // плохой CRC — молчим
                if (b[0] != unit) continue;
                std::vector<uint8_t> pdu(b + 1, b + total - 2);
                auto rsp = handlePdu(R, pdu);
                std::vector<uint8_t> f = {static_cast<uint8_t>(unit)}; f.insert(f.end(), rsp.begin(), rsp.end());
                uint16_t c = mb::crc16(f.data(), f.size()); f.push_back(c & 0xFF); f.push_back(c >> 8);
                (void)!write(fd, f.data(), f.size());
            }
        });
    }
    ~RtuSlave() { run = false; th.join(); }
};

static void testCodec() {
    std::printf("Преобразование команд в регистры\n");
    RegEntry p; p.thrusterId = "T"; p.field = OutField::Pct; p.scale = 100;
    ThrusterCmd c; c.id = "T"; c.pct = -37.25;
    uint16_t r = OutputLink::encode(p, c);
    CHECK(static_cast<int16_t>(r) == -3725 && std::fabs(OutputLink::decode(p, r).pct + 37.25) < 1e-9, "проценты со знаком, масштаб 0,01 %");
    c.pct = 250; CHECK(static_cast<int16_t>(OutputLink::encode(p, c)) == 10000, "команда ограничивается ±100 %");
    RegEntry kw = p; kw.unitPerPct = 1.8; kw.scale = 1;   // 180 кВт при 100 %
    c.pct = 50; CHECK(OutputLink::encode(kw, c) == 90, "единицы мощности: 50 % от 180 кВт = 90");
    RegEntry a; a.thrusterId = "T"; a.field = OutField::AngleDeg; a.scale = 10; a.isSigned = false; a.rawMin = 0; a.rawMax = 3600;
    c.angleDeg = -90; CHECK(OutputLink::encode(a, c) == 2700, "угол −90° → 270,0° → код 2700");
    c.angleDeg = 725.5; CHECK(OutputLink::encode(a, c) == 55, "угол приводится по модулю 360°");
    a.angle360 = false; a.isSigned = true; a.rawMin = -1800; a.rawMax = 1800; c.angleDeg = 270;
    CHECK(static_cast<int16_t>(OutputLink::encode(a, c)) == -900, "режим −180…180°");
    RegEntry o; o.scale = 1; o.offset = 4000; o.isSigned = false; o.rawMin = 0; o.rawMax = 20000; c.pct = 100; o.unitPerPct = 80;   // «4…20 мА»-подобная шкала
    CHECK(OutputLink::encode(o, c) == 12000, "смещение и пределы кода");
}

static void testTcp() {
    std::printf("Modbus TCP\n");
    Regs R; TcpSlave sl(R, 34600); sleepMs(200);
    mb::Master m(std::make_unique<mb::TcpTransport>("127.0.0.1", 34600), 1, 300);
    std::string err;
    CHECK(m.open(err), "соединение установлено");
    CHECK(m.writeRegisters(100, {1, 2, 0xFFFF, 4}, err) && R.get(102) == 0xFFFF, "запись нескольких регистров (0x10)");
    CHECK(m.writeRegister(50, 777, err) && R.get(50) == 777, "запись одного регистра (0x06)");
    std::vector<uint16_t> v; CHECK(m.readRegisters(100, 4, v, err) && v.size() == 4 && v[3] == 4, "чтение регистров (0x03)");
    CHECK(!m.writeRegisters(395, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}, err) && err.find("код 2") != std::string::npos, "исключение «неверный адрес» распознаётся");
    sl.dropNext = true;
    CHECK(!m.writeRegister(51, 1, err), "потеря ответа даёт ошибку по таймауту");
    m.close();

    // линия с обрывом: должна переподключиться сама
    LinkCfg lc; lc.host = "127.0.0.1"; lc.port = 34600; lc.periodMs = 10; lc.retryS = 0.2; lc.timeoutMs = 100; lc.heartbeatReg = 10;
    RegEntry e; e.thrusterId = "T"; e.address = 20; e.scale = 100; lc.entries.push_back(e);
    OutputLink link(lc); link.start();
    ThrusterCmd c; c.id = "T"; c.pct = 12.5;
    for (int i = 0; i < 30; ++i) { link.setCommands({c}, DpMode::AutoPosition, monotonicSeconds()); sleepMs(10); }
    CHECK(static_cast<int16_t>(R.get(20)) == 1250 && R.get(10) > 3, "линия передаёт команду и счётчик «жив»");
    sl.dropNext = true;
    for (int i = 0; i < 80; ++i) { link.setCommands({c}, DpMode::AutoPosition, monotonicSeconds()); sleepMs(10); }
    auto st = link.status();
    CHECK(st.failedCycles >= 1 && st.connected, "после обрыва связь восстановлена автоматически");
    // потеря команд: безопасное значение
    sleepMs(1300);
    CHECK(static_cast<int16_t>(R.get(20)) == 0, "при отсутствии новых команд передаётся нулевая тяга");
    CHECK(link.status().stale, "состояние «устарело» отмечено в статусе");
    link.stop();
}

static void testRtu() {
    std::printf("Modbus RTU (виртуальный COM)\n");
    int master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || grantpt(master) != 0 || unlockpt(master) != 0) { std::printf("  [пропуск] нет pty\n"); return; }
    std::string dev = ptsname(master);
    { termios t{}; tcgetattr(master, &t); cfmakeraw(&t); tcsetattr(master, TCSANOW, &t); }
    Regs R; RtuSlave sl(R, master, 7);
    mb::Master m(std::make_unique<mb::SerialTransport>(dev, 19200), 7, 300);
    std::string err;
    CHECK(m.open(err), "порт открыт");
    CHECK(m.writeRegisters(10, {5, 6, 7}, err) && R.get(12) == 7, "запись (0x10) с проверкой CRC");
    std::vector<uint16_t> v; CHECK(m.readRegisters(10, 3, v, err) && v[1] == 6, "чтение (0x03)");
    CHECK(!m.writeRegisters(399, {1, 2, 3}, err) && err.find("код 2") != std::string::npos, "исключение распознаётся в RTU");
    mb::Master wrong(std::make_unique<mb::SerialTransport>(dev, 19200), 9, 150);
    wrong.open(err);
    CHECK(!wrong.writeRegister(1, 1, err), "чужой адрес устройства: ответа нет");
    CHECK(mb::crc16(reinterpret_cast<const uint8_t*>("\x01\x03\x00\x00\x00\x0A"), 6) == 0xCDC5, "CRC16 эталонного кадра (C5 CD)");
    wrong.close(); m.close();
}

static void testLoop() {
    std::printf("Замкнутый контур через Modbus\n");
    EnvCfg env; env.windSpeed = 10; env.windFromDeg = 60; env.currentSpeed = 0.5; env.currentToDeg = 200; env.hs = 1.5; env.tp = 7; env.waveFromDeg = 60;
    VesselCfg v = demoVessel(); auto th = demoThrusters();
    Simulator sim(v, th, env); DpEngine eng(v, th);
    Regs R; TcpSlave sl(R, 34601); sleepMs(200);
    LinkCfg lc = demoLink(th); lc.host = "127.0.0.1"; lc.port = 34601; lc.periodMs = 2; lc.timeoutMs = 200;
    OutputLink link(lc); link.start();
    const double dt = 0.1; double t = 0, maxErr = 0;
    auto cycle = [&](bool engineOn) {
        sim.step(dt); t += dt;
        auto fr = sim.measure();
        auto o = eng.step(dt, fr);
        link.setCommands(o.cmds, eng.mode(), monotonicSeconds());
        sleepMs(engineOn ? 4 : 1);
        // «контроллер» читает регистры и подаёт команды на движители
        std::vector<ThrusterCmd> applied;
        std::map<std::string, ThrusterCmd> byId;
        for (const auto& e : lc.entries) {
            ThrusterCmd d = OutputLink::decode(e, R.get(e.address));
            auto& c = byId[e.thrusterId]; c.id = e.thrusterId;
            if (e.field == OutField::Pct) c.pct = d.pct; else c.angleDeg = d.angleDeg;
        }
        for (auto& kv : byId) applied.push_back(kv.second);
        sim.applyCommands(applied);
        return o;
    };
    for (int i = 0; i < 100; ++i) cycle(false);
    eng.setMode(DpMode::AutoPosition);
    eng.controller().setSetpoint({sim.truth().n, sim.truth().e, rad2deg(sim.truth().psi)});
    for (int i = 0; i < 5500; ++i) {
        auto o = cycle(true);
        if (t > 350) maxErr = std::max(maxErr, std::hypot(sim.truth().n - o.ctl.refN, sim.truth().e - o.ctl.refE));
    }
    std::printf("  максимальное отклонение при выводе через Modbus: %.2f м; режим в регистре: %u; записей принято: %lld\n", maxErr, R.get(lc.modeReg), R.writes.load());
    CHECK(maxErr < 1.5, "удержание позиции при выводе команд через регистры контроллера");
    CHECK(R.get(lc.modeReg) == static_cast<uint16_t>(DpMode::AutoPosition) && R.get(lc.healthReg) == 1, "режим и признак исправности переданы контроллеру");
    link.stop();
}

int main() {
    testCodec();
    testTcp();
    testRtu();
    testLoop();
    std::printf(fails ? "ИТОГ: есть провалы (%d)\n" : "ИТОГ: всё в порядке\n", fails);
    return fails ? 1 : 0;
}
