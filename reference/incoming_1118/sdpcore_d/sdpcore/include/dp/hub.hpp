// Блок приёма данных датчиков: сопоставляет входящие строки NMEA каналам и формирует SensorFrame для ядра.
#pragma once
#include <cctype>
#include <cmath>
#include <cstdio>
#include <deque>
#include <map>
#include <mutex>

#include "dp/nmea.hpp"
#include "dp/telegrams.hpp"
#include "dp/types.hpp"

namespace dp {

enum class ChannelKind { Gnss, LaserRadar, TautWire, Gyro, Wind, Mru };

struct ChannelCfg {
    std::string id;                 // имя датчика, например "GNSS1"
    ChannelKind kind = ChannelKind::Gnss;
    double sigma = 0.5;             // СКО положения, м (для Gyro — градусы, для Wind — м/с)
    V3 offset;                      // место установки относительно точки управления, м (x нос, y правый борт, z вниз)
    int minGnssQuality = 1;         // минимально допустимое качество GGA (1 — GPS, 2 — DGPS, 4 — RTK fix)
    double maxHdop = 10.0;
    bool requireChecksum = true;
    bool windMastCorrection = false; // зарезервировано
    std::string accept;             // принимаемые типы предложений через запятую (GGA,RMC…); пусто — набор по умолчанию для типа датчика
    std::string talkers;            // разрешённые talker ID через запятую (GP,GN…); пусто — любой
    double magVarDeg = 0;           // магнитное склонение для HDM, ° (восточное +)
    // приведение знаков МИО к внутреннему соглашению, как у K-Pos (крен + на правый борт вниз, дифферент + на нос вверх, heave + ВНИЗ):
    // -1 — если прибор выдаёт обратный знак. ⚠ сверить с описанием изготовителя конкретного прибора
    double rollSign = 1, pitchSign = 1, heaveSign = 1;
    // лазерный радар CyScan/RadaScan. Посылка $RLS,A,B,S1,XXX.XXX,S2,HHHH: A, B — положение ДАТЧИКА (м) в системе осей мишеней (руководство CyScan, прил. 6–7).
    // Режим rlsMode: 0 «судно» — оси привязаны к судну с началом в главной мишени (NMEA0183P, одна мишень): положение = ref + R(курс)·(A,B), курс от гирокомпаса;
    //   1 «ne» — мишени на неподвижной платформе в координатах север/восток: A — север, B — восток (положение = ref + (A,B));
    //   2 «frame» — оси, привязанные к конструкции с мишенями: A направлена по азимуту rlsAzDeg, B — на 90° по часовой стрелке от неё.
    // ⚠ Направление осей A/B для режима 0 в руководстве не определено — проверить на приборе (rls.swap, rls.signa, rls.signb).
    // ref — положение начала осей (главной мишени) в локальной системе, м. targets — мишени для $PGNMT: номер → (север, восток).
    double refN = 0, refE = 0, rlsSignA = 1, rlsSignB = 1, rlsAzDeg = 0;
    bool rlsSwap = false;           // поменять местами A и B
    int rlsMode = 0;
    // Fanbeam/Artemis (v0.22): поправка нуля пеленга (Fanbeam: пеленг от носа) или азимута (Artemis), градусы
    double brgOffDeg = 0;
    std::map<int, std::pair<double, double>> targets;
};

struct ChannelStats {
    std::string id;
    long long accepted = 0, rejected = 0, ignored = 0;
    double lastTime = -1e9;
    std::string lastReject;
};

// Строка, принятая каналом, с результатом разбора (для окна «Датчики», терминальный режим).
struct RawLine {
    long long seq = 0;
    double time = 0;
    std::string text;
    std::string result;     // "OK" — принята, "ИГНОР" — разобрана, но не используется, иначе причина отказа
};

// Сведения о канале для окна «Датчики».
struct ChannelInfo {
    std::string id;
    ChannelKind kind = ChannelKind::Gnss;
    V3 offset; double sigma = 0.5;
    long long accepted = 0, rejected = 0, ignored = 0;
    double rateHz = 0, ageS = 1e9;
    std::string lastReject, values;    // values — расшифрованные последние значения
    std::string kindText;
    std::string accept, talkers, known;   // действующий список типов, разрешённые talker, перечень разбираемых типов этого датчика
    bool checksum = true; int minQ = 1; double maxHdop = 10, magVar = 0, rollSign = 1, pitchSign = 1, heaveSign = 1;
};

// Типы предложений, которые разбирает ядро, по видам датчиков (для настройки в окне «Датчики»).
inline std::string knownSentences(ChannelKind k) {
    switch (k) {
        case ChannelKind::Gnss: return "GGA,GLL,RMC,VTG,GNS,GST,GSA,ZDA,DPGGA";
        case ChannelKind::LaserRadar: return "DPREL,RLS,PGNMT,PMDTG,MDL,CEG17,CEG16,ART16,ART17,ART22";
        case ChannelKind::TautWire: return "DPREL";
        case ChannelKind::Gyro: return "HDT,THS,HDM,VHW,ROT";
        case ChannelKind::Wind: return "MWV";
        case ChannelKind::Mru: return "SXN,HTRO";
    }
    return "";
}
// Набор по умолчанию — поведение до появления настройки (VTG, HDM и VHW включаются только вручную).
inline std::string defaultSentences(ChannelKind k) {
    switch (k) {
        case ChannelKind::Gnss: return "GGA,GLL,RMC";
        case ChannelKind::Gyro: return "HDT,THS";
        case ChannelKind::LaserRadar: return "DPREL,RLS,PGNMT";
        default: return knownSentences(k);
    }
}

class SensorHub {
public:
    void addChannel(const ChannelCfg& c) {
        std::lock_guard<std::mutex> lk(m_);
        ch_[c.id].cfg = c;
        ch_[c.id].stats.id = c.id;
        order_.push_back(c.id);
    }
    void setOrigin(double latDeg, double lonDeg) { std::lock_guard<std::mutex> lk(m_); frame_.setOrigin(latDeg, lonDeg); originFixed_ = true; }
    bool originSet() const { std::lock_guard<std::mutex> lk(m_); return frame_.valid(); }
    LocalFrame frame() const { std::lock_guard<std::mutex> lk(m_); return frame_; }

    // Вызывается из потоков ввода. now — монотонное время, с.
    void ingest(const std::string& channelId, const std::string& line, double now) {
        std::lock_guard<std::mutex> lk(m_);
        auto it = ch_.find(channelId);
        if (it == ch_.end()) return;
        Ch& c = it->second;
        const long long rej0 = c.stats.rejected;
        if (c.cfg.kind == ChannelKind::LaserRadar && !line.empty() && line[0] != '$') {   // «сырые» форматы Fanbeam/Artemis (не NMEA)
            const std::string acc = c.cfg.accept.empty() ? defaultSentences(c.cfg.kind) : c.cfg.accept;
            std::string use;
            for (const char* t : {"MDL", "CEG17", "CEG16", "ART16", "ART17", "ART22"}) if (("," + acc + ",").find(std::string(",") + t + ",") != std::string::npos) { use = t; break; }
            if (!use.empty()) {
                // если разрешено несколько «сырых» форматов, выбирается первый, чей разбор прошёл
                tele::RangeBearing rb; bool done = false;
                for (const char* t : {"MDL", "CEG17", "CEG16", "ART16", "ART17", "ART22"}) {
                    if (("," + acc + ",").find(std::string(",") + t + ",") == std::string::npos) continue;
                    rb = tele::parseRaw(t, line); if (rb.ok) { done = true; c.lastSent[t] = now; break; }
                }
                if (!done) { ++c.stats.rejected; c.stats.lastReject = rb.why; pushRaw(c, now, line, "ОШИБКА: " + c.stats.lastReject); return; }
                if (applyRangeBearing(c, rb)) { ++c.stats.accepted; c.stats.lastTime = now; c.arrivals.push_back(now); pushRaw(c, now, line, "OK"); }
                else pushRaw(c, now, line, "ОШИБКА: " + c.stats.lastReject);
                while (!c.arrivals.empty() && c.arrivals.front() < now - 10.0) c.arrivals.pop_front();
                return;
            }
        }
        auto s = nmea::parse(line, c.cfg.requireChecksum);
        if (!s) { ++c.stats.rejected; c.stats.lastReject = "ошибка формата или контрольной суммы"; pushRaw(c, now, line, "ОШИБКА: " + c.stats.lastReject); return; }
        {   // фильтры настройки посылок: talker ID и список принимаемых типов
            auto has = [](const std::string& list, const std::string& t) { return ("," + list + ",").find("," + t + ",") != std::string::npos; };
            if (!s->proprietary && !c.cfg.talkers.empty() && !has(c.cfg.talkers, s->talker)) { ++c.stats.ignored; pushRaw(c, now, line, "ИГНОР"); return; }
            const std::string acc = c.cfg.accept.empty() ? defaultSentences(c.cfg.kind) : c.cfg.accept;
            const std::string tname = (!s->proprietary && s->talker == "RL" && s->type == "S") ? "RLS" : s->type;   // $RLS — посылка CyScan, не talker+тип
            if (!has(acc, tname) && !has(acc, "P" + s->type)) { ++c.stats.ignored; pushRaw(c, now, line, "ИГНОР"); return; }
            c.lastSent[has(acc, tname) ? tname : "P" + s->type] = now;
        }
        bool used = false;
        switch (c.cfg.kind) {
            case ChannelKind::Gnss: case ChannelKind::LaserRadar: case ChannelKind::TautWire: used = handlePos(c, *s, now); break;
            case ChannelKind::Gyro: used = handleHdg(c, *s, now); break;
            case ChannelKind::Wind: used = handleWind(c, *s, now); break;
            case ChannelKind::Mru: used = handleMru(c, *s, now); break;
        }
        if (used) { ++c.stats.accepted; c.stats.lastTime = now; c.arrivals.push_back(now); pushRaw(c, now, line, "OK"); }
        else if (c.stats.rejected > rej0) pushRaw(c, now, line, "ОШИБКА: " + c.stats.lastReject);
        else { ++c.stats.ignored; pushRaw(c, now, line, "ИГНОР"); }
        while (!c.arrivals.empty() && c.arrivals.front() < now - 10.0) c.arrivals.pop_front();
    }

    // Последние строки канала с номерами больше afterSeq (не более maxN).
    std::vector<RawLine> rawTail(const std::string& id, long long afterSeq, size_t maxN) const {
        std::lock_guard<std::mutex> lk(m_);
        std::vector<RawLine> r;
        auto it = ch_.find(id); if (it == ch_.end()) return r;
        for (const auto& l : it->second.raw) if (l.seq > afterSeq) r.push_back(l);
        if (r.size() > maxN) r.erase(r.begin(), r.end() - static_cast<long>(maxN));
        return r;
    }

    // Типы предложений из списка приёма, которые приходили, но замолчали дольше timeoutS: пары (датчик, тип).
    std::vector<std::pair<std::string, std::string>> staleSentences(double now, double timeoutS) const {
        std::lock_guard<std::mutex> lk(m_);
        std::vector<std::pair<std::string, std::string>> r;
        if (timeoutS <= 0) return r;
        for (const auto& id : order_) {
            const Ch& c = ch_.at(id);
            for (const auto& kv : c.lastSent) {
                const std::string acc = c.cfg.accept.empty() ? defaultSentences(c.cfg.kind) : c.cfg.accept;
                if (("," + acc + ",").find("," + kv.first + ",") == std::string::npos) continue;
                if (now - kv.second > timeoutS) r.push_back({id, kv.first});
            }
        }
        return r;
    }

    // Место установки и шум можно менять во время работы (окно «Датчики»).
    bool setOffset(const std::string& id, double x, double y, double z) {
        std::lock_guard<std::mutex> lk(m_);
        auto it = ch_.find(id); if (it == ch_.end()) return false;
        it->second.cfg.offset = V3{x, y, z}; return true;
    }
    bool setSigma(const std::string& id, double s) {
        std::lock_guard<std::mutex> lk(m_);
        auto it = ch_.find(id); if (it == ch_.end() || !(s > 0.001 && s < 1000)) return false;
        it->second.cfg.sigma = s; return true;
    }

    // Настройка посылок: ключи accept, talkers, check, quality, hdop, magvar, sign.roll|pitch|heave. false — нет датчика или значение недопустимо.
    bool setParam(const std::string& id, const std::string& key, const std::string& val, std::string& why) {
        std::lock_guard<std::mutex> lk(m_);
        auto it = ch_.find(id); if (it == ch_.end()) { why = "Нет такого датчика: " + id; return false; }
        ChannelCfg& c = it->second.cfg;
        auto up = [](std::string x) { for (auto& ch : x) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch))); return x; };
        auto split = [](const std::string& x) { std::vector<std::string> r; std::string t; for (char ch : x) { if (ch == ',' || ch == ' ') { if (!t.empty()) r.push_back(t); t.clear(); } else t += ch; } if (!t.empty()) r.push_back(t); return r; };
        auto num = [&](double& v) { try { size_t k = 0; v = std::stod(val, &k); return k == val.size(); } catch (...) { return false; } };
        if (key == "accept") {
            if (up(val) == "DEFAULT" || val.empty()) { c.accept.clear(); return true; }
            const std::string known = "," + knownSentences(c.kind) + ","; std::string out; int n = 0;
            for (auto& t : split(up(val))) { if (known.find("," + t + ",") == std::string::npos) { why = "Предложение " + t + " этот датчик не разбирает; доступно: " + knownSentences(c.kind); return false; } out += (n++ ? "," : "") + t; }
            if (!n) { why = "Список предложений пуст"; return false; }
            c.accept = out; it->second.lastSent.clear(); return true;
        }
        if (key == "talkers") {
            if (up(val) == "ANY" || val.empty()) { c.talkers.clear(); return true; }
            std::string out; int n = 0;
            for (auto& t : split(up(val))) { if (t.size() != 2) { why = "Talker ID — две буквы (GP, GN, HE…): " + t; return false; } out += (n++ ? "," : "") + t; }
            c.talkers = out; return true;
        }
        double v = 0;
        if (key == "check") { if (up(val) == "ON") c.requireChecksum = true; else if (up(val) == "OFF") c.requireChecksum = false; else { why = "check on|off"; return false; } return true; }
        if (key == "quality") { if (!num(v) || v < 0 || v > 8) { why = "quality: от 0 до 8 (1 — GPS, 2 — DGPS, 4 — RTK)"; return false; } c.minGnssQuality = static_cast<int>(v); return true; }
        if (key == "hdop") { if (!num(v) || v < 0.5 || v > 99) { why = "hdop: от 0,5 до 99"; return false; } c.maxHdop = v; return true; }
        if (key == "magvar") { if (!num(v) || std::fabs(v) > 60) { why = "magvar: от −60 до 60°"; return false; } c.magVarDeg = v; return true; }
        if (key == "ref") {   // положение отражателя «N,E» в локальной системе, м
            const auto p = split(val); double n = 0, e = 0;
            try { if (p.size() != 2) throw 1; n = std::stod(p[0]); e = std::stod(p[1]); } catch (...) { why = "ref: два числа «север,восток», м"; return false; }
            if (std::fabs(n) > 1e6 || std::fabs(e) > 1e6) { why = "ref: не более 1 000 000 м"; return false; }
            c.refN = n; c.refE = e; return true;
        }
        if (key == "rls.signa" || key == "rls.signb") {
            if (!num(v) || (v != 1 && v != -1)) { why = "rls.sign: 1 или -1"; return false; }
            (key == "rls.signa" ? c.rlsSignA : c.rlsSignB) = v; return true;
        }
        if (key == "rls.mode") {
            const std::string m = up(val);
            if (m == "VESSEL") c.rlsMode = 0; else if (m == "NE") { c.rlsMode = 1; c.rlsAzDeg = 0; } else if (m == "FRAME") c.rlsMode = 2; else { why = "rls.mode: vessel, ne или frame"; return false; }
            return true;
        }
        if (key == "rls.az") { if (!num(v) || std::fabs(v) > 360) { why = "rls.az: азимут оси A, от −360 до 360°"; return false; } c.rlsAzDeg = v; return true; }
        if (key == "target") {   // «<номер> <север> <восток>» или «<номер> clear»; номер мишени CyScan/RadaScan
            const auto p = split(val); int id = 0;
            try { if (p.size() < 2) throw 1; size_t k = 0; id = std::stoi(p[0], &k); if (k != p[0].size() || id < 0 || id > 65535) throw 1;
                  if (up(p[1]) == "CLEAR") { c.targets.erase(id); return true; }
                  if (p.size() != 3) throw 1;
                  c.targets[id] = {std::stod(p[1]), std::stod(p[2])}; }
            catch (...) { why = "target: «номер север восток» (м) или «номер clear»"; return false; }
            return true;
        }
        if (key == "brg.off") { if (!num(v) || std::fabs(v) > 360) { why = "brg off: поправка нуля пеленга/азимута, от −360 до 360°"; return false; } c.brgOffDeg = v; return true; }
        if (key == "rls.swap") { if (up(val) == "1" || up(val) == "ON") c.rlsSwap = true; else if (up(val) == "0" || up(val) == "OFF") c.rlsSwap = false; else { why = "rls.swap: 0 или 1"; return false; } return true; }
        if (key.rfind("sign.", 0) == 0) {
            if (!num(v) || (v != 1 && v != -1)) { why = "sign: 1 или -1"; return false; }
            const std::string ax = key.substr(5);
            if (ax == "roll") c.rollSign = v; else if (ax == "pitch") c.pitchSign = v; else if (ax == "heave") c.heaveSign = v; else { why = "sign: roll, pitch или heave"; return false; }
            return true;
        }
        why = "Неизвестная настройка: " + key; return false;
    }
    ChannelCfg cfgOf(const std::string& id) const { std::lock_guard<std::mutex> lk(m_); auto it = ch_.find(id); return it == ch_.end() ? ChannelCfg{} : it->second.cfg; }

    std::vector<ChannelInfo> channels(double now) const {
        std::lock_guard<std::mutex> lk(m_);
        std::vector<ChannelInfo> r;
        for (const auto& id : order_) {
            const Ch& c = ch_.at(id);
            ChannelInfo ci; ci.id = id; ci.kind = c.cfg.kind; ci.offset = c.cfg.offset; ci.sigma = c.cfg.sigma;
            ci.accepted = c.stats.accepted; ci.rejected = c.stats.rejected; ci.ignored = c.stats.ignored; ci.lastReject = c.stats.lastReject;
            ci.ageS = c.stats.lastTime > -1e8 ? now - c.stats.lastTime : 1e9;
            ci.rateHz = c.arrivals.size() >= 2 ? (c.arrivals.size() - 1) / std::max(0.5, c.arrivals.back() - c.arrivals.front()) : 0.0;
            char b[160];
            switch (c.cfg.kind) {
                case ChannelKind::Gnss: ci.kindText = "GNSS"; std::snprintf(b, sizeof b, "N %.2f  E %.2f м", c.pos.n, c.pos.e); break;
                case ChannelKind::LaserRadar: ci.kindText = "Лазерный радар"; std::snprintf(b, sizeof b, "N %.2f  E %.2f м%s", c.pos.n, c.pos.e, c.extra.c_str()); break;
                case ChannelKind::TautWire: ci.kindText = "Натянутый трос"; std::snprintf(b, sizeof b, "N %.2f  E %.2f м", c.pos.n, c.pos.e); break;
                case ChannelKind::Gyro: ci.kindText = "Гирокомпас"; std::snprintf(b, sizeof b, "курс %.2f°", rad2deg(c.hdg.psi) < 0 ? rad2deg(c.hdg.psi) + 360.0 : rad2deg(c.hdg.psi)); break;
                case ChannelKind::Wind: ci.kindText = "Ветер"; std::snprintf(b, sizeof b, "%.1f м/с, %.0f° (отн.)", c.wind.speed, rad2deg(c.wind.fromRel)); break;
                case ChannelKind::Mru: ci.kindText = "МИО"; std::snprintf(b, sizeof b, "крен %.2f°  дифф. %.2f°  кач. %.2f м", rad2deg(c.mru.roll), rad2deg(c.mru.pitch), c.mru.heave); break;
            }
            std::string vs = b;
            if (c.cfg.kind == ChannelKind::Gnss && c.sogValid) { char q[64]; std::snprintf(q, sizeof q, "  ·  %.1f уз, %.0f°", c.sogKn, c.cogDeg); vs += q; }
            if (c.cfg.kind == ChannelKind::Gnss && !c.gnssInfo.empty()) vs += "  ·  " + c.gnssInfo;
            if (c.cfg.kind == ChannelKind::Gnss && c.gstSigma > 0) { char q[48]; std::snprintf(q, sizeof q, "  ·  GST σ %.2f м", c.gstSigma); vs += q; }
            if (c.cfg.kind == ChannelKind::Gyro && c.rotValid) { char q[48]; std::snprintf(q, sizeof q, "  ·  ROT %.1f°/мин", c.rotDegMin); vs += q; }
            ci.values = vs;
            ci.accept = c.cfg.accept.empty() ? defaultSentences(c.cfg.kind) : c.cfg.accept; ci.talkers = c.cfg.talkers; ci.known = knownSentences(c.cfg.kind);
            ci.checksum = c.cfg.requireChecksum; ci.minQ = c.cfg.minGnssQuality; ci.maxHdop = c.cfg.maxHdop; ci.magVar = c.cfg.magVarDeg;
            ci.rollSign = c.cfg.rollSign; ci.pitchSign = c.cfg.pitchSign; ci.heaveSign = c.cfg.heaveSign;
            r.push_back(ci);
        }
        return r;
    }

    // Состояние внешних устройств, которые приходят не по NMEA (электростанция, обратная связь движителей).
    void setPower(double availableKW) { std::lock_guard<std::mutex> lk(m_); power_ = availableKW; powerFresh_ = true; }
    void setThrusterFb(const ThrusterFb& fb) { std::lock_guard<std::mutex> lk(m_); fb_[fb.id] = fb; }

    // Формирует кадр для ядра; признак fresh выставляется однократно для данных, пришедших после прошлого вызова.
    SensorFrame buildFrame(double now) {
        std::lock_guard<std::mutex> lk(m_);
        SensorFrame fr;
        fr.time = now;
        for (const auto& id : order_) {
            Ch& c = ch_[id];
            switch (c.cfg.kind) {
                case ChannelKind::Gnss: case ChannelKind::LaserRadar: case ChannelKind::TautWire: {
                    PosMeas p = c.pos; p.id = id; p.sigma = std::max({c.cfg.sigma, c.sigmaDyn, (now - c.gstT <= 5.0 ? c.gstSigma : 0.0)}); p.offset = c.cfg.offset;
                    p.type = c.cfg.kind == ChannelKind::Gnss ? PosType::Gnss : c.cfg.kind == ChannelKind::LaserRadar ? PosType::LaserRadar : PosType::TautWire;
                    p.fresh = c.fresh; fr.pos.push_back(p); break;
                }
                case ChannelKind::Gyro: { HdgMeas h = c.hdg; h.id = id; h.fresh = c.fresh; fr.hdg.push_back(h); break; }
                case ChannelKind::Wind: { WindMeas w = c.wind; w.id = id; w.fresh = c.fresh; fr.wind.push_back(w); break; }
                case ChannelKind::Mru: { MruMeas m = c.mru; m.id = id; m.fresh = c.fresh; fr.mru.push_back(m); break; }
            }
            c.fresh = false;
        }
        if (power_ >= 0) { fr.power.availableKW = power_; fr.power.fresh = powerFresh_; powerFresh_ = false; }
        for (auto& kv : fb_) fr.fb.push_back(kv.second);
        return fr;
    }

    std::vector<ChannelStats> stats() const {
        std::lock_guard<std::mutex> lk(m_);
        std::vector<ChannelStats> r;
        for (const auto& id : order_) r.push_back(ch_.at(id).stats);
        return r;
    }

private:
    struct Ch {
        ChannelCfg cfg; ChannelStats stats;
        PosMeas pos; HdgMeas hdg; WindMeas wind; MruMeas mru;
        bool fresh = false;
        std::map<std::string, double> lastSent;                  // когда последний раз пришло предложение каждого принимаемого типа
        double sigmaDyn = 0;                                     // СКО текущего измерения от прибора (PGNMT), м; 0 — не задано
        std::string extra;                                       // дополнительная информация для показа (качество, возраст)
        bool hdgSeen = false;                                    // курс принят хотя бы раз (для пересчёта $RLS)
        double gstSigma = 0, gstT = -1e9;                       // СКО положения из GST (на ось, м) и время приёма
        std::string gnssInfo;                                    // DOP/тип решения/время из GSA, ZDA (только показ)
        bool rotValid = false; double rotDegMin = 0;             // скорость поворота из ROT (только показ)
        bool sogValid = false; double sogKn = 0, cogDeg = 0;    // скорость и курс над грунтом из VTG (только показ)
        std::deque<RawLine> raw; long long seq = 0; std::deque<double> arrivals;
    };

    void pushRaw(Ch& c, double now, const std::string& line, const std::string& res) {
        RawLine l; l.seq = ++c.seq; l.time = now; l.text = line.size() > 200 ? line.substr(0, 200) : line; l.result = res;
        while (!l.text.empty() && (l.text.back() == '\r' || l.text.back() == '\n')) l.text.pop_back();
        c.raw.push_back(std::move(l));
        while (c.raw.size() > 300) c.raw.pop_front();
    }

    bool headingNow(double& psi) const {   // курс первого гирокомпаса, от которого он хоть раз приходил
        for (const auto& id : order_) { const Ch& g = ch_.at(id); if (g.cfg.kind == ChannelKind::Gyro && g.hdgSeen) { psi = g.hdg.psi; return true; } }
        return false;
    }
    bool reject(Ch& c, const char* why) { ++c.stats.rejected; c.stats.lastReject = why; return false; }

    bool setPosGeo(Ch& c, double lat, double lon, double now) {
        (void)now;
        if (!frame_.valid()) frame_.setOrigin(lat, lon);
        double n, e; frame_.toLocal(lat, lon, n, e);
        c.pos.n = n; c.pos.e = e; c.fresh = true;
        return true;
    }

    bool handlePos(Ch& c, const nmea::Sentence& s, double now) {
        double lat, lon;
        if (s.type == "GNS" && s.f.size() >= 8) {   // $--GNS: режим — по символу на созвездие: N нет решения, A автономное, D дифф., P точное, R RTK fix, F RTK float, E счисление…
            int q = 0; bool any = false;
            for (char ch : s.f[5]) {
                int v = ch == 'A' ? 1 : ch == 'D' ? 2 : ch == 'P' ? 3 : ch == 'R' ? 4 : ch == 'F' ? 5 : ch == 'E' ? 6 : ch == 'M' ? 7 : ch == 'S' ? 8 : 0;
                if (v) { any = true; if (q == 0 || v > q) q = v; }
            }
            if (!any) return reject(c, "GNS: нет решения (режим N)");
            if (q < c.cfg.minGnssQuality) return reject(c, "качество решения GNS ниже допустимого");
            double hd; if (nmea::toDouble(s.f[7], hd) && hd > c.cfg.maxHdop) return reject(c, "HDOP выше допустимого");
            if (!nmea::parseAngle(s.f[1], s.f[2], true, lat) || !nmea::parseAngle(s.f[3], s.f[4], false, lon)) return reject(c, "нет координат в GNS");
            return setPosGeo(c, lat, lon, now);
        }
        if (s.type == "GST" && s.f.size() >= 7) {   // $--GST: статистика ошибок псевдодальностей; СКО по широте и долготе → реальное СКО положения
            double sl, so;
            if (!nmea::toDouble(s.f[5], sl) || !nmea::toDouble(s.f[6], so)) return reject(c, "GST: нет СКО по широте/долготе");
            if (sl < 0 || so < 0 || sl > 1000 || so > 1000) return reject(c, "GST: СКО вне диапазона");
            c.gstSigma = std::max(0.05, std::sqrt((sl * sl + so * so) / 2.0)); c.gstT = now; return true;
        }
        if (s.type == "GSA" && s.f.size() >= 17) {  // $--GSA: тип решения и DOP — только показ
            double p, h, v; const int fx = std::atoi(s.f[1].c_str());
            nmea::toDouble(s.f[14], p); nmea::toDouble(s.f[15], h); nmea::toDouble(s.f[16], v);
            char b[96]; std::snprintf(b, sizeof b, "%s PDOP %.1f HDOP %.1f VDOP %.1f", fx == 3 ? "3D" : fx == 2 ? "2D" : "нет решения", p, h, v);
            c.gnssInfo = b; return true;
        }
        if (s.type == "ZDA" && s.f.size() >= 4) {   // $--ZDA: время и дата UTC — только показ
            char b[48]; std::snprintf(b, sizeof b, "UTC %s %s.%s.%s", s.f[0].substr(0, 6).c_str(), s.f[1].c_str(), s.f[2].c_str(), s.f[3].c_str());
            c.gnssInfo = b; return true;
        }
        if ((s.type == "GGA" || (s.proprietary && s.type == "DPGGA")) && s.f.size() >= 8) {
            int q = std::atoi(s.f[5].c_str());
            if (q < c.cfg.minGnssQuality) return reject(c, "качество решения GGA ниже допустимого");
            double hd; if (nmea::toDouble(s.f[7], hd) && hd > c.cfg.maxHdop) return reject(c, "HDOP выше допустимого");
            if (!nmea::parseAngle(s.f[1], s.f[2], true, lat) || !nmea::parseAngle(s.f[3], s.f[4], false, lon)) return reject(c, "нет координат в GGA");
            return setPosGeo(c, lat, lon, now);
        }
        if (s.type == "GLL" && s.f.size() >= 6) {
            if (s.f[5] != "A") return reject(c, "GLL: данные недостоверны");
            if (!nmea::parseAngle(s.f[0], s.f[1], true, lat) || !nmea::parseAngle(s.f[2], s.f[3], false, lon)) return reject(c, "нет координат в GLL");
            return setPosGeo(c, lat, lon, now);
        }
        if (s.type == "RMC" && s.f.size() >= 6) {
            if (s.f[1] != "A") return reject(c, "RMC: данные недостоверны");
            if (!nmea::parseAngle(s.f[2], s.f[3], true, lat) || !nmea::parseAngle(s.f[4], s.f[5], false, lon)) return reject(c, "нет координат в RMC");
            return setPosGeo(c, lat, lon, now);
        }
        if (s.type == "VTG" && s.f.size() >= 5) {   // курс и скорость над грунтом: только показ, в решение не входит (не свежий отсчёт позиции)
            double cg, sg;
            if (!nmea::toDouble(s.f[0], cg) || !nmea::toDouble(s.f[4], sg)) return reject(c, "VTG: нет значений");
            c.cogDeg = cg; c.sogKn = sg; c.sogValid = true; return true;
        }
        if (!s.proprietary && s.talker == "RL" && s.type == "S" && s.f.size() >= 3) {   // CyScan: $RLS,A,B,A|V,XXX.XXX,A|V,HHHH — положение датчика в системе осей мишеней
            if (c.cfg.kind != ChannelKind::LaserRadar) return false;
            if (s.f[2] != "A") return reject(c, "RLS: данные недостоверны");
            double a, b;
            if (!nmea::toDouble(s.f[0], a) || !nmea::toDouble(s.f[1], b)) return reject(c, "RLS: нет значений");
            if (c.cfg.rlsSwap) std::swap(a, b);
            a *= c.cfg.rlsSignA; b *= c.cfg.rlsSignB;
            double th = 0;
            if (c.cfg.rlsMode == 0) { if (!headingNow(th)) return reject(c, "RLS: нет курса для пересчёта"); }
            else if (c.cfg.rlsMode == 2) th = deg2rad(c.cfg.rlsAzDeg);
            c.pos.n = c.cfg.refN + a * std::cos(th) - b * std::sin(th);
            c.pos.e = c.cfg.refE + a * std::sin(th) + b * std::cos(th);
            c.sigmaDyn = 0; c.extra.clear();
            c.fresh = true; return true;
        }
        if (s.proprietary && s.type == "GNMT" && s.f.size() >= 12) {   // RadaScan/CyScan MT: $PGNMT,T,время,счётчик,посл.,ID,статус,дальность,СКО,пеленг,СКО,качество,возраст,флаги
            if (c.cfg.kind != ChannelKind::LaserRadar) return false;
            if (s.f[5] != "A" && s.f[5] != "N") return reject(c, "PGNMT: данные недостоверны");
            double R, rs, B, bs, q, age; int id = 0;
            try { id = std::stoi(s.f[4]); } catch (...) { return reject(c, "PGNMT: нет номера мишени"); }
            if (!nmea::toDouble(s.f[6], R) || !nmea::toDouble(s.f[7], rs) || !nmea::toDouble(s.f[8], B) || !nmea::toDouble(s.f[9], bs)) return reject(c, "PGNMT: нет значений");
            if (!nmea::toDouble(s.f[10], q)) q = -1;
            if (!nmea::toDouble(s.f[11], age)) age = 0;
            if (age > 2000.0) return reject(c, "PGNMT: измерение устарело (возраст более 2 с)");
            auto t = c.cfg.targets.find(id);
            if (t == c.cfg.targets.end()) return reject(c, "PGNMT: положение мишени не задано (sensor <ID> target <номер> <север> <восток>)");
            double psi = 0; if (!headingNow(psi)) return reject(c, "PGNMT: нет курса для пересчёта");
            const double x = R * std::cos(deg2rad(B)), y = R * std::sin(deg2rad(B));        // от датчика к мишени в осях судна (пеленг по часовой от носа)
            c.pos.n = t->second.first - (x * std::cos(psi) - y * std::sin(psi));
            c.pos.e = t->second.second - (x * std::sin(psi) + y * std::cos(psi));
            c.sigmaDyn = std::hypot(rs, R * deg2rad(bs));
            char eb[96]; std::snprintf(eb, sizeof eb, "  мишень %d  кач. %.0f  возраст %.0f мс", id, q, age); c.extra = eb;
            c.fresh = true; return true;
        }
        if (s.proprietary && s.type == "MDTG" && s.f.size() >= 8) {   // Fanbeam $PMDTG,NN,RRRR.RR,BBB.BB,LLL,A,S,F,M,TTT
            if (c.cfg.kind != ChannelKind::LaserRadar) return false;
            tele::RangeBearing rb; rb.ok = true; rb.type = "PMDTG"; double R, B, L;
            if (!nmea::toDouble(s.f[1], R) || !nmea::toDouble(s.f[2], B)) return reject(c, "PMDTG: нет дальности или пеленга");
            rb.rangeM = R; rb.bearingDeg = B; rb.target = std::atoi(s.f[0].c_str());
            std::string ex; if (nmea::toDouble(s.f[3], L)) { char b[48]; std::snprintf(b, sizeof b, "  уровень %.0f", L); ex += b; }
            if (s.f.size() > 4 && !s.f[4].empty()) ex += "  точн. " + s.f[4];
            if (s.f.size() > 5 && !s.f[5].empty()) ex += "  уверенность " + s.f[5];
            if (s.f.size() > 6 && !s.f[6].empty()) ex += "  угроза " + s.f[6];
            return applyRangeBearing(c, rb, ex);
        }
        // Формат проекта для относительных систем (лазерный радар, натянутый трос): $PDPREL,N,E — метры в локальной системе.
        if (s.proprietary && s.type == "DPREL" && s.f.size() >= 2) {
            double n, e;
            if (!nmea::toDouble(s.f[0], n) || !nmea::toDouble(s.f[1], e)) return reject(c, "PDPREL: нет значений");
            c.pos.n = n; c.pos.e = e; c.fresh = true; return true;
        }
        return false;
    }

    // Привязка разобранной телеграммы «дальность—пеленг» (Fanbeam) или «дальность—азимут» (Artemis) к координатам.
    // Fanbeam: положение = мишень − R·(cos(курс+пеленг+поправка), sin(...)); положение мишени — из sensor <ID> target <№> <N> <E>.
    // Artemis: положение мобильной станции = положение Fix-станции (sensor <ID> ref N E) + R·(cos(азимут+поправка), sin(...)).
    bool applyRangeBearing(Ch& c, const tele::RangeBearing& rb, const std::string& extra = "") {
        if (!rb.valid) return reject(c, "данные отмечены прибором как недостоверные (нет захвата / READY = 0)");
        const bool artemis = rb.type.rfind("ART", 0) == 0;
        if (rb.rangeM <= 0.0) return reject(c, "дальность равна нулю");
        char eb[160];
        if (artemis) {
            if (!rb.hasAzimuth) return reject(c, "нет азимута (мобильная станция связана с маяком, а не с Fix-станцией)");
            const double az = deg2rad(rb.azimuthDeg + c.cfg.brgOffDeg);
            c.pos.n = c.cfg.refN + rb.rangeM * std::cos(az);
            c.pos.e = c.cfg.refE + rb.rangeM * std::sin(az);
            std::snprintf(eb, sizeof eb, "  %s  дальн. %.1f м  азим. %.3f°", rb.type.c_str(), rb.rangeM, rb.azimuthDeg);
        } else {
            auto t = c.cfg.targets.find(rb.target);
            if (t == c.cfg.targets.end()) return reject(c, "положение мишени не задано (sensor <ID> target <номер> <север> <восток>)");
            double psi = 0; if (!headingNow(psi)) return reject(c, "нет курса для пересчёта");
            const double a = psi + deg2rad(rb.bearingDeg + c.cfg.brgOffDeg);
            c.pos.n = t->second.first - rb.rangeM * std::cos(a);
            c.pos.e = t->second.second - rb.rangeM * std::sin(a);
            std::snprintf(eb, sizeof eb, "  %s  мишень %d  дальн. %.2f м  пел. %.2f°", rb.type.c_str(), rb.target, rb.rangeM, rb.bearingDeg);
        }
        c.extra = std::string(eb) + extra; c.sigmaDyn = 0; c.fresh = true; return true;
    }

    bool handleHdg(Ch& c, const nmea::Sentence& s, double now) {
        (void)now;
        double h;
        if (s.type == "HDT" && !s.f.empty()) {
            if (!nmea::toDouble(s.f[0], h)) return reject(c, "HDT: нет курса");
        } else if (s.type == "THS" && s.f.size() >= 2) {
            if (s.f[1] == "V") return reject(c, "THS: данные недостоверны");
            if (!nmea::toDouble(s.f[0], h)) return reject(c, "THS: нет курса");
        } else if (s.type == "HDM" && !s.f.empty()) {          // магнитный курс + склонение
            if (!nmea::toDouble(s.f[0], h)) return reject(c, "HDM: нет курса");
            h = std::fmod(h + c.cfg.magVarDeg + 720.0, 360.0);
        } else if (s.type == "VHW" && s.f.size() >= 3) {       // истинный курс, иначе магнитный + склонение
            if (!nmea::toDouble(s.f[0], h)) { if (!nmea::toDouble(s.f[2], h)) return reject(c, "VHW: нет курса"); h = std::fmod(h + c.cfg.magVarDeg + 720.0, 360.0); }
        } else if (s.type == "ROT" && !s.f.empty()) {          // скорость поворота: только показ
            double r; if (!nmea::toDouble(s.f[0], r)) return reject(c, "ROT: нет значения");
            if (s.f.size() >= 2 && s.f[1] == "V") return reject(c, "ROT: данные недостоверны");
            c.rotDegMin = r; c.rotValid = true; return true;
        } else return false;
        if (h < 0 || h >= 360.0001) return reject(c, "курс вне диапазона");
        c.hdg.psi = wrapPi(deg2rad(h)); c.fresh = true; c.hdgSeen = true; return true;
    }

    bool handleWind(Ch& c, const nmea::Sentence& s, double now) {
        (void)now;
        if (s.type == "MWV" && s.f.size() >= 5) {
            if (s.f[4] != "A") return reject(c, "MWV: данные недостоверны");
            if (s.f[1] != "R") return reject(c, "MWV: ожидается кажущийся (относительный) ветер");
            double ang, sp;
            if (!nmea::toDouble(s.f[0], ang) || !nmea::toDouble(s.f[2], sp)) return reject(c, "MWV: нет значений");
            const std::string u = s.f[3];
            if (u == "K") sp /= 3.6; else if (u == "N") sp *= 0.514444; else if (u != "M") return reject(c, "MWV: неизвестные единицы");
            c.wind.speed = sp; c.wind.fromRel = wrapPi(deg2rad(ang)); c.fresh = true; return true;
        }
        return false;
    }

    bool handleMru(Ch& c, const nmea::Sentence& s, double now) {
        (void)now;
        // Kongsberg/Seapath: $PSXN,23,roll,pitch,heading,heave
        if (s.proprietary && s.type == "SXN" && s.f.size() >= 5 && s.f[0] == "23") {
            double r, p, hv;
            if (!nmea::toDouble(s.f[1], r) || !nmea::toDouble(s.f[2], p)) return reject(c, "PSXN: нет значений");
            nmea::toDouble(s.f[4], hv);
            c.mru.roll = c.cfg.rollSign * deg2rad(r); c.mru.pitch = c.cfg.pitchSign * deg2rad(p); c.mru.heave = c.cfg.heaveSign * hv; c.fresh = true; return true;
        }
        // Sperry/Anschütz: $PHTRO,pitch,M|P,roll,B|T  (M — нос вниз, P — нос вверх; B — левый борт вверх? см. описание изготовителя)
        if (s.proprietary && s.type == "HTRO" && s.f.size() >= 4) {
            double p, r;
            if (!nmea::toDouble(s.f[0], p) || !nmea::toDouble(s.f[2], r)) return reject(c, "PHTRO: нет значений");
            if (s.f[1] == "M") p = -p;              // M — нос вниз
            if (s.f[3] == "B") r = -r;              // B — левый борт вниз (принято условно, сверить с изготовителем)
            c.mru.pitch = c.cfg.pitchSign * deg2rad(p); c.mru.roll = c.cfg.rollSign * deg2rad(r); c.fresh = true; return true;
        }
        return false;
    }

    mutable std::mutex m_;
    std::map<std::string, Ch> ch_;
    std::vector<std::string> order_;
    LocalFrame frame_;
    bool originFixed_ = false;
    double power_ = -1; bool powerFresh_ = false;
    std::map<std::string, ThrusterFb> fb_;
};

}  // namespace dp
