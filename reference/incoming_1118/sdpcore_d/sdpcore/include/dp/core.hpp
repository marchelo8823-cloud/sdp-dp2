// Ядро СДП: рабочий цикл. Датчики (или имитатор) → оценка состояния → регулятор → распределение упоров → команды движителям.
// Ядро запускается без данных и ждёт их по очереди:
//   НЕТ СУДНА    — данные судна не загружены (команда ship load <книга.xlsx>);
//   ЖДУ ДАТЧИКИ  — судно загружено, нет курса или положения;
//   ГОТОВ        — данные есть, режим ожидания: можно брать движители и включать режимы;
//   УДЕРЖАНИЕ    — режим курса или позиции. При потере всех систем положения удержание продолжается по счислению
//                  (тревога POS_DROPOUT), режим сам не снимается: снять упор без решения оператора опаснее.
// Имитатор запускается отдельно (sim start <книга имитатора.xlsx>) и подменяет собой датчики и движители загруженного судна.
#pragma once
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <sstream>

#include "dp/alarms.hpp"
#include "dp/capability.hpp"
#include "dp/shipbook.hpp"
#include "dp/simfeed.hpp"

namespace dp {

struct RunStats { double seconds = 0, rmsPos = 0, maxPos = 0, rmsHdg = 0, maxHdg = 0; };
struct CoreSettings { double cycleHz = 10; std::string logFile; };
enum class CoreState { NoShip, WaitSensors, Ready, Active };

inline const char* coreStateName(CoreState s) {
    switch (s) {
        case CoreState::NoShip: return "НЕТ СУДНА";
        case CoreState::WaitSensors: return "ЖДУ ДАТЧИКИ";
        case CoreState::Ready: return "ГОТОВ";
        case CoreState::Active: return "УДЕРЖАНИЕ";
    }
    return "";
}

class Core {
public:
    using Say = std::function<void(const std::string&)>;

    explicit Core(CoreSettings st = {}, Say say = {}) : set_(st), say_(std::move(say)), dt_(1.0 / clampd(st.cycleHz, 1, 100)) {}
    ~Core() { stop(); }
    Core(const Core&) = delete; Core& operator=(const Core&) = delete;

    void stop() { closeIo(); if (log_) log_.flush(); }

    // ---------------------------------------------------------------- данные судна
    // Загрузка книги судна. Только когда судна нет или ядро в ожидании. При ошибках прежнее судно остаётся.
    bool loadShip(const std::string& path) {
        if (sh_ && active()) { say("Судно не загружено: сначала режим ожидания (standby)."); return false; }
        AppCfg c; BookReport rep;
        const bool ok = loadShipBook(path, c, rep);
        lastRep_ = rep;
        for (auto& e : rep.errors) say("ОШИБКА: " + e);
        for (auto& w : rep.warnings) say("Предупреждение: " + w);
        if (!ok) { say("Данные судна не приняты: ошибок " + std::to_string(rep.errors.size()) + (sh_ ? ". Остаётся прежнее судно «" + sh_->cfg.vessel.name + "»." : ".")); return false; }
        const bool hadSim = static_cast<bool>(sim_);
        if (hadSim) { sim_.reset(); say("Имитатор остановлен: загружено другое судно."); }
        closeIo();
        sh_ = std::make_unique<Ship>(std::move(c));
        shipPath_ = path;
        openReal();
        resetRun();
        char b[240]; std::snprintf(b, sizeof b, "Судно «%s» загружено: движителей %zu, датчиков %zu (подключено %zu), значений по умолчанию %zu.",
                                    sh_->cfg.vessel.name.c_str(), sh_->cfg.thrusters.size(), sh_->sensorCount, sh_->cfg.inputs.size(), rep.defaults.size());
        say(b);
        if (sh_->cfg.inputs.empty()) say("У датчиков не заполнено «Подключение»: данные с судна не придут. Работа возможна только в имитаторе (sim start).");
        return true;
    }
    // Проверка книги без загрузки
    BookReport checkShip(const std::string& path) const { AppCfg c; BookReport rep; loadShipBook(path, c, rep); return rep; }

    // ---------------------------------------------------------------- имитатор
    bool simStart(const std::string& path) {
        if (!sh_) { say("Имитатор не запущен: сначала загрузите судно (ship load)."); return false; }
        if (active()) { say("Имитатор не запущен: сначала режим ожидания (standby)."); return false; }
        SimBook sb; BookReport rep;
        const bool ok = loadSimBook(path, sb, rep);
        for (auto& e : rep.errors) say("ОШИБКА: " + e);
        for (auto& w : rep.warnings) say("Предупреждение: " + w);
        if (!ok) { say("Имитатор не запущен: ошибок " + std::to_string(rep.errors.size()) + "."); return false; }
        closeIo();
        sh_->rebuild();
        const AppCfg& c = sh_->cfg;
        const double lat0 = c.originSet ? c.originLat : 0.0, lon0 = c.originSet ? c.originLon : 0.0;
        lf_ = LocalFrame(); lf_.setOrigin(lat0, lon0); sh_->hub->setOrigin(lat0, lon0);
        sim_ = std::make_unique<Simulator>(c.vessel, c.thrusters, sb.env, c.simSensors, seedOverride_ ? seedOverride_ : sb.seed);
        sim_->dyn().state().psi = deg2rad(sb.headingDeg);
        simSpeed_ = sb.speed;
        for (auto& g : c.simSensors.gnss) { ChannelCfg ch; ch.id = g.id; ch.kind = ChannelKind::Gnss; ch.offset = g.offset; ch.sigma = g.sigma; sh_->hub->addChannel(ch); }
        if (c.simSensors.laser) { ChannelCfg ch; ch.id = "LASER"; ch.kind = ChannelKind::LaserRadar; ch.offset = {22, 0, -8}; ch.sigma = c.simSensors.laserSigma; sh_->hub->addChannel(ch); }
        if (c.simSensors.wire) { ChannelCfg ch; ch.id = "WIRE"; ch.kind = ChannelKind::TautWire; ch.offset = {25, 0, 2}; ch.sigma = c.simSensors.wireSigma; sh_->hub->addChannel(ch); }
        for (int i = 1; i <= c.simSensors.gyros; ++i) { ChannelCfg ch; ch.id = "GYRO" + std::to_string(i); ch.kind = ChannelKind::Gyro; sh_->hub->addChannel(ch); }
        for (int i = 1; i <= c.simSensors.anemometers; ++i) { ChannelCfg ch; ch.id = "WIND" + std::to_string(i); ch.kind = ChannelKind::Wind; sh_->hub->addChannel(ch); }
        for (int i = 1; i <= c.simSensors.mrus; ++i) { ChannelCfg ch; ch.id = "MRU" + std::to_string(i); ch.kind = ChannelKind::Mru; sh_->hub->addChannel(ch); }
        t_ = 0; resetRun();
        char b[600]; std::snprintf(b, sizeof b, "Имитатор запущен: ветер %.1f м/с с %.0f°, течение %.1f м/с на %.0f°, волна %.1f м; ускорение ×%g. Датчики и движители судна заменены имитацией, выдача команд отключена.",
                                    sb.env.windSpeed, sb.env.windFromDeg, sb.env.currentSpeed, sb.env.currentToDeg, sb.env.hs, sb.speed);
        say(b);
        return true;
    }
    bool simStop() {
        if (!sim_) { say("Имитатор не запущен."); return false; }
        if (active()) { say("Имитатор не остановлен: сначала режим ожидания (standby)."); return false; }
        sim_.reset(); closeIo(); sh_->rebuild(); openReal(); resetRun();
        say("Имитатор остановлен: ядро ждёт датчики судна.");
        return true;
    }

    // ---------------------------------------------------------------- рабочий цикл
    void cycle() {
        if (!sh_) { t_ += dt_; return; }
        initTaken();
        DpEngine& eng = *sh_->eng; SensorHub& hub = *sh_->hub;
        SensorFrame fr;
        if (sim_) {
            sim_->step(dt_); t_ += dt_;
            SensorFrame raw = sim_->measure();
            for (auto& l : encodeNmea(raw, lf_)) hub.ingest(l.channel, l.sentence, t_);
            hub.setPower(sh_->cfg.simSensors.availableKW);
            for (auto f : raw.fb) { auto o = fbOverride_.find(f.id); if (o != fbOverride_.end()) { f.valid = true; f.pct = o->second; } hub.setThrusterFb(f); }
            fr = hub.buildFrame(t_);
        } else {
            t_ += dt_;
            fr = hub.buildFrame(monotonicSeconds());
        }
        out_ = eng.step(dt_, fr);
        if (active() && opSpSet_) opSp_ = eng.controller().setpoint();   // уставку мог перенести сам движок (here после возврата положения)
        for (const auto& s : out_.infos) { alarms_.info(clock(), s); say(s); }
        alarms_.update(clock(), out_.alarms);
        for (const auto& a : out_.alarms) if (shown_.insert({a.code, true}).second) say("ТРЕВОГА " + a.code + ": " + a.text);
        for (auto it = shown_.begin(); it != shown_.end();) {
            bool on = false; for (const auto& a : out_.alarms) if (a.code == it->first) on = true;
            if (!on) { say("Тревога снята: " + it->first); it = shown_.erase(it); } else ++it;
        }
        if (sim_) sim_->applyCommands(out_.cmds);
        else for (auto& l : sh_->links) l->setCommands(out_.cmds, eng.mode(), monotonicSeconds());

        const CoreState s = state();
        if (s != lastState_) { say(std::string("Состояние ядра: ") + coreStateName(s) + stateHint(s)); lastState_ = s; }

        if (active() && modeSince_ >= 0 && clock() - modeSince_ > 120.0) {      // статистика после переходного процесса
            double err = out_.ctl.errDist, eh = std::fabs(out_.ctl.errHdgDeg);
            if (sim_) {
                const auto& tr = sim_->truth();
                err = std::hypot(tr.n - out_.ctl.refN, tr.e - out_.ctl.refE); eh = std::fabs(rad2deg(wrapPi(tr.psi - deg2rad(out_.ctl.refPsiDeg))));
            }
            sum2_ += err * err; max_ = std::max(max_, err); h2_ += eh * eh; hmax_ = std::max(hmax_, eh); n_ += 1;
        }
        writeLog();
    }

    CoreState state() const {
        if (!sh_) return CoreState::NoShip;
        if (active()) return CoreState::Active;
        const EstimatorState& es = out_.est;
        if (!es.hdgValid || !es.posValid) return CoreState::WaitSensors;
        return CoreState::Ready;
    }

    static const char* helpText() {
        return "Команды:\n"
               "  ship load <книга.xlsx>              загрузить данные судна (только в ожидании)\n"
               "  ship check <книга.xlsx>             проверить книгу, не загружая\n"
               "  ship show                           загруженное судно и значения по умолчанию\n"
               "  sim start <книга.xlsx>              запустить имитатор вместо датчиков судна\n"
               "  sim stop                            остановить имитатор, вернуться к датчикам\n"
               "  sim wind|current|wave|off|on|thr|jump|freeze|fbfault|noise|gyro|mru|reset   управление имитатором\n"
               "  thr_take <id|all> | thr_release <id|all>   взять движитель под управление или снять\n"
               "  standby | heading | position        режим: ожидание, удержание курса, удержание позиции и курса\n"
               "  set <север м> <восток м> <курс °>   задать точку и курс\n"
               "  move <dС м> <dВ м> [dКурс °]        сдвинуть точку\n"
               "  hdg <курс °>                        задать курс\n"
               "  here                                после потери положения: держать здесь, а не возвращаться в заданную точку\n"
               "  pivot <вдоль м> <поперёк м>         центр вращения (+ к носу, + на правый борт)\n"
               "  rate rot <°/мин>|off, rate speed <уз>|off   предел скорости поворота и перемещения\n"
               "  law pid|lqr|mpc                     закон управления\n"
               "  gain high|low                       усиление контуров\n"
               "  cap [течение м/с]                   возможности удержания: предельный ветер по сценариям\n"
               "  class 2|3|off                       анализ последствий отказа (по умолчанию выключен)\n"
               "  ca [ветер м/с откуда ° [течение]]   анализ последствий сейчас; с погодой — по введённой погоде\n"
               "  status | help | quit\n";
    }

    void command(const std::string& line) {
        std::istringstream is(line); std::string c; is >> c;
        if (c.empty()) return;
        auto rest = [&]() { std::string r; std::getline(is, r); size_t i = 0; while (i < r.size() && r[i] == ' ') ++i; r = r.substr(i); while (!r.empty() && (r.back() == ' ' || r.back() == '\r')) r.pop_back();
                            if (r.size() >= 2 && (r.front() == '"' || r.front() == '\'') && r.back() == r.front()) r = r.substr(1, r.size() - 2);
                            return r; };
        if (c == "quit" || c == "exit") { quit_ = true; return; }
        if (c == "help") { say(helpText()); return; }
        if (c == "status") { statusRequested_ = true; return; }
        if (c == "ship") {
            std::string w; is >> w; const std::string p = rest();
            if (w == "load" && !p.empty()) loadShip(p);
            else if (w == "check" && !p.empty()) {
                const BookReport r = checkShip(p);
                for (auto& e : r.errors) say("ОШИБКА: " + e);
                for (auto& x : r.warnings) say("Предупреждение: " + x);
                for (auto& d : r.defaults) say("По умолчанию: " + d);
                for (auto& e : r.estimates) say("Оценка: " + e);
                say(r.ok() ? "Книга в порядке (не загружена)." : "В книге ошибок: " + std::to_string(r.errors.size()) + ".");
            }
            else if (w == "show") {
                if (!sh_) { say("Судно не загружено."); return; }
                say("Судно «" + sh_->cfg.vessel.name + "» из " + shipPath_);
                for (const auto& t : sh_->cfg.thrusters) { char b[200]; std::snprintf(b, sizeof b, "  %s: x %.1f, y %.1f м, упор %.0f/%.0f кН, %.0f кВт", t.id.c_str(), t.x, t.y, t.maxForward / 1000, t.maxReverse / 1000, t.powerMaxKW); say(b); }
                for (auto& d : lastRep_.defaults) say("По умолчанию: " + d);
            }
            else say("Формат: ship load <книга.xlsx> | ship check <книга.xlsx> | ship show");
            return;
        }
        if (c == "sim") {          // запуск и остановка имитатора; остальные команды sim — ниже, при загруженном судне
            std::istringstream is2(line); std::string c2, w; is2 >> c2 >> w;
            if (w == "start") { std::string p; std::getline(is2, p); size_t i = 0; while (i < p.size() && p[i] == ' ') ++i; p = p.substr(i); while (!p.empty() && (p.back() == ' ' || p.back() == '\r')) p.pop_back();
                                if (p.size() >= 2 && (p.front() == '"' || p.front() == '\'') && p.back() == p.front()) p = p.substr(1, p.size() - 2);
                                if (p.empty()) say("Формат: sim start <книга имитатора.xlsx>"); else simStart(p); return; }
            if (w == "stop") { simStop(); return; }
        }
        if (!sh_) { say("Судно не загружено: ship load <книга.xlsx>. Состояние ядра: НЕТ СУДНА."); return; }
        DpEngine& eng = *sh_->eng;
        const EstimatorState& es = out_.est;
        const bool activeNow = active();
        auto base = [&]() { if (opSpSet_) return opSp_; if (activeNow) return eng.controller().setpoint(); return eng.controller().holdFromCrp(es.n, es.e, rad2deg(es.psi)); };
        auto setSp = [&](const Setpoint& sp) {
            opSp_ = sp; opSpSet_ = true;
            if (activeNow) eng.controller().setSetpoint(sp);
            else { char b[160]; std::snprintf(b, sizeof b, "Уставка запомнена: N=%.1f E=%.1f курс=%.1f°; применится при включении режима.", sp.n, sp.e, sp.psiDeg); say(b); }
        };
        if (c == "thr_take" || c == "thr_release") { std::string w; is >> w; if (w.empty()) say("Формат: thr_take|thr_release <id|all>"); else takeThrusters(w, c == "thr_take"); }
        else if (c == "standby") { eng.setMode(DpMode::Standby); modeSince_ = -1; opSpSet_ = false; }
        else if (c == "position" || c == "heading") {
            bool okm; const DpMode m = parseMode(c, okm);
            if (state() == CoreState::WaitSensors && !activeNow) { say("Режим не включён: ядро ждёт датчики (нет курса или положения)."); return; }
            if (takenCount() == 0) { say("Режим не включён: нет движителей под управлением. Сначала thr_take all."); return; }
            if (!opSpSet_) opSp_ = eng.controller().holdFromCrp(es.n, es.e, rad2deg(es.psi));
            eng.setMode(m); modeSince_ = clock();
            eng.controller().setSetpoint(opSp_); opSpSet_ = true;
            n_ = sum2_ = max_ = h2_ = hmax_ = 0;
        }
        else if (c == "set") { double n, e, h; if (is >> n >> e >> h) setSp({n, e, h}); else say("Формат: set <север> <восток> <курс>"); }
        else if (c == "move") { double dn, de, dh = 0; if (is >> dn >> de) { is >> dh; const Setpoint b = base(); setSp({b.n + dn, b.e + de, b.psiDeg + dh}); } else say("Формат: move <dС> <dВ> [dКурс]"); }
        else if (c == "class") {
            std::string w; is >> w;
            say(eng.consequence().setClass(w == "2" ? 2 : w == "3" ? 3 : w == "off" ? 0 : -1));
        }
        else if (c == "ca") {
            const Consequence& ca = eng.consequence();
            double wind; bool hyp = false;
            if (is >> wind) {   // РС 8.9.7: наихудший отказ при введённой погоде
                double from = 0, cur = 0; is >> from; is >> cur; hyp = true;
                for (const auto& g : Consequence::makeGroups(eng.allocator().thrusters(), nullptr)) {
                    HypotCase h; h.failed = g.ids; h.windMS = clampd(wind, 0, 60); h.windFromDeg = from; h.currentMS = clampd(cur, 0, 5);
                    AppCfg cf = sh_->cfg; cf.thrusters = eng.allocator().thrusters();
                    const DriftResult r = capHypot(cf, h);
                    char b[120]; std::snprintf(b, sizeof b, "), ветер %.1f м/с с %.0f° от носа, течение %.1f м/с: ", h.windMS, from, h.currentMS);
                    say("Отказ «" + g.name + "» (" + Consequence::join(g.ids) + b + r.note);
                }
            }
            if (!hyp) {
                if (ca.dpClass() == 0) say("Анализ последствий выключен: class 2 или class 3. Сейчас — расчёт по текущему усилию регулятора:");
                else if (ca.ran()) { char b[160]; std::snprintf(b, sizeof b, "Анализ последствий: класс %d, последний расчёт %.0f с назад:", ca.dpClass(), clock() - ca.lastRunT()); say(b); }
                else say("Анализ последствий: класс " + std::to_string(ca.dpClass()) + ", расчёта ещё не было (нужен режим «Позиция» и 60 с при неизменных точке и курсе). Сейчас — по текущему усилию:");
                const auto gs = ca.ran() && ca.dpClass() ? ca.groups() : ca.evaluate(out_.ctl.tau, eng.allocator().thrusters());
                for (const auto& g : gs) {
                    char b[300]; std::snprintf(b, sizeof b, "  отказ «%s» (%s): %s — сила %.0f %%, момент %.0f %%%s", g.name.c_str(), Consequence::join(g.ids).c_str(),
                                               g.holds ? "удержится" : "НЕ ХВАТИТ упора", g.forcePct, g.momentPct, g.alarm ? ", тревога" : "");
                    say(b);
                }
                if (gs.empty()) say("  нет движителей в работе");
            }
        }
        else if (c == "here") { say(eng.holdHere()); if (activeNow) opSp_ = eng.controller().setpoint(); }
        else if (c == "hdg") { double h; if (is >> h) { const Setpoint b = base(); setSp({b.n, b.e, h}); } else say("Формат: hdg <курс>"); }
        else if (c == "pivot") {
            double px, py;
            if (!(is >> px >> py)) say("Формат: pivot <вдоль, м (+ к носу)> <поперёк, м (+ на правый борт)>");
            else if (std::fabs(px) > 300 || std::fabs(py) > 100) say("Центр вращения вне допустимых пределов (|вдоль| ≤ 300, |поперёк| ≤ 100 м)");
            else {
                Controller& ct = eng.controller();
                const Setpoint prev = opSpSet_ ? opSp_ : Setpoint{es.n, es.e, rad2deg(es.psi)};
                if (activeNow) { ct.setPivot(px, py); if (opSpSet_) opSp_ = ct.setpoint(); }
                else { const Setpoint crp = ct.crpFromHold(prev); ct.setPivotIdle(px, py); if (opSpSet_) opSp_ = ct.holdFromCrp(crp.n, crp.e, crp.psiDeg); }
                say("Центр вращения задан");
            }
        }
        else if (c == "rate") {
            std::string w, v; is >> w >> v;
            if ((w == "rot" || w == "speed") && !v.empty()) {
                double x = 0; bool ok = true;
                if (v != "off") { try { size_t k = 0; x = std::stod(v, &k); ok = k == v.size(); } catch (...) { ok = false; } }
                const double lo = w == "rot" ? 0.5 : 0.05, hi = w == "rot" ? 600 : 30;
                if (!ok || (x != 0 && (x < lo || x > hi))) say(std::string("Допустимо: ") + (w == "rot" ? "0,5…600 °/мин" : "0,05…30 уз") + " или off");
                else { (w == "rot" ? capRotDegMin_ : capSpeedKn_) = x; applyCaps(); say("Ограничение движения принято"); }
            } else say("Формат: rate rot <°/мин>|off | rate speed <узлов>|off");
        }
        else if (c == "law") {
            std::string l; is >> l; ControlCfg cc = eng.controller().cfg();
            if (l == "pid") cc.law = LawType::Pid; else if (l == "lqr") cc.law = LawType::Lqr; else if (l == "mpc") cc.law = LawType::Mpc; else { say("Законы: pid, lqr, mpc"); return; }
            eng.controller().configure(cc); say("Закон управления: " + l);
        }
        else if (c == "gain") {
            std::string w; is >> w;
            if (w == "high" || w == "low") { ControlCfg cc = eng.controller().cfg(); cc.highGain = (w == "high"); eng.controller().configure(cc); say(std::string("Усиление: ") + (cc.highGain ? "высокое" : "низкое")); }
            else say("Формат: gain high|low");
        }
        else if (c == "cap") {
            CapEnv e; double cur = 0; if (is >> cur) e.currentMS = clampd(cur, 0, 5);
            for (const auto& cv : capScenarios(sh_->cfg, e)) {
                double mn = 1e9, at = 0; for (size_t i = 0; i < cv.limitMS.size(); ++i) if (cv.limitMS[i] < mn) { mn = cv.limitMS[i]; at = cv.dirDeg[i]; }
                char b[240]; std::snprintf(b, sizeof b, "Возможности, «%s»: наименьший предельный ветер %.1f м/с (рабочий, 80 %% упора: %.1f м/с), с направления %.0f° от носа", cv.name.c_str(), mn, cv.minWork(), at); say(b);
            }
        }
        else if (c == "sim") {
            std::string w; is >> w;
            if (!sim_) { say("Имитатор не запущен: sim start <книга имитатора.xlsx>."); return; }
            EnvCfg& env = sim_->target();
            auto sensorKnown = [&](const std::string& id) { for (auto& p : simIds()) if (p == id) return true; return false; };
            if (w == "wind") { double sp, fr; if (is >> sp >> fr) { env.windSpeed = clampd(sp, 0, 60); env.windFromDeg = fr; } else say("Формат: sim wind <м/с> <откуда°>"); }
            else if (w == "current") { double sp, to; if (is >> sp >> to) { env.currentSpeed = clampd(sp, 0, 5); env.currentToDeg = to; } else say("Формат: sim current <м/с> <куда°>"); }
            else if (w == "wave") { double hs, tp, fr; if (is >> hs >> tp >> fr) { env.hs = clampd(hs, 0, 15); env.tp = clampd(tp, 2, 20); env.waveFromDeg = fr; } else say("Формат: sim wave <Hs м> <Tp с> <откуда°>"); }
            else if (w == "off" || w == "on") { std::string id; is >> id; if (sensorKnown(id)) sim_->failSensor(id, w == "off"); else say("Нет такого датчика: " + id); }
            else if (w == "thr") { std::string st, id; is >> st >> id; if (!sim_->failThrusterById(id, st == "off")) say("Нет такого движителя: " + id); }
            else if (w == "jump") { std::string id; double dn, de; if (is >> id >> dn >> de) { if (std::fabs(dn) < 1e-9 && std::fabs(de) < 1e-9) sim_->clearJump(id); else sim_->jumpSensor(id, dn, de); } else say("Формат: sim jump <GNSS1> <дС м> <дВ м> (0 0 — снять)"); }
            else if (w == "freeze") { std::string id, st; if (is >> id >> st) { if (sensorKnown(id)) sim_->freezeSensor(id, st == "on"); else say("Нет такого датчика: " + id); } else say("Формат: sim freeze <ID> on|off"); }
            else if (w == "fbfault") { std::string id, v; if (is >> id >> v) { if (v == "off") fbOverride_.erase(id); else { try { fbOverride_[id] = std::stod(v); } catch (...) { say("Формат: sim fbfault <движитель> <упор %>|off"); } } } else say("Формат: sim fbfault <движитель> <упор %>|off"); }
            else if (w == "noise") { std::string id; double k; if (is >> id >> k) sim_->noiseScale(id, clampd(k, 1, 50)); else say("Формат: sim noise <ID> <во сколько раз больше шум>"); }
            else if (w == "gyro") { std::string id; double d; if (is >> id >> d) sim_->gyroOffset(id, clampd(d, -30, 30)); else say("Формат: sim gyro <GYRO1> <сдвиг°>"); }
            else if (w == "mru") { std::string id; double r, p, h; if (is >> id >> r >> p >> h) sim_->biasMru(id, deg2rad(clampd(r, -20, 20)), deg2rad(clampd(p, -20, 20)), clampd(h, -5, 5)); else say("Формат: sim mru <MRU1> <крен°> <дифферент°> <heave м>"); }
            else if (w == "reset") {
                for (auto& id : simIds()) { sim_->failSensor(id, false); sim_->clearJump(id); sim_->freezeSensor(id, false); sim_->noiseScale(id, 1); sim_->gyroOffset(id, 0); sim_->biasMru(id, 0, 0, 0); }
                fbOverride_.clear();
                for (auto& t : sh_->cfg.thrusters) sim_->failThrusterById(t.id, false);
            }
            else say("sim: start, stop, wind, current, wave, off|on <ID>, thr off|on <ID>, jump, freeze, fbfault, noise, gyro, mru, reset");
        }
        else say("Неизвестная команда «" + c + "», help — список команд.");
    }

    std::vector<std::string> simIds() const {
        std::vector<std::string> r;
        if (!sim_) return r;
        const SimSensorsCfg& sc = sim_->sensors();
        for (const auto& g : sc.gnss) r.push_back(g.id);
        if (sc.laser) r.push_back("LASER");
        if (sc.wire) r.push_back("WIRE");
        for (int i = 1; i <= sc.gyros; ++i) r.push_back("GYRO" + std::to_string(i));
        for (int i = 1; i <= sc.anemometers; ++i) r.push_back("WIND" + std::to_string(i));
        for (int i = 1; i <= sc.mrus; ++i) r.push_back("MRU" + std::to_string(i));
        return r;
    }

    bool hasShip() const { return static_cast<bool>(sh_); }
    bool simRunning() const { return static_cast<bool>(sim_); }
    double simSpeed() const { return sim_ ? simSpeed_ : 1.0; }
    int takenCount() { if (!sh_) return 0; initTaken(); int n = 0; for (char c : taken_) n += c ? 1 : 0; return n; }
    bool taken(size_t i) const { return i < taken_.size() && taken_[i]; }   // движитель под управлением ядра (для пульта)
    double dt() const { return dt_; }
    double clock() const { return t_; }
    const EngineOutput& out() const { return out_; }
    DpEngine* engine() { return sh_ ? sh_->eng.get() : nullptr; }
    void setSimSeed(unsigned s) { seedOverride_ = s; }   // для проверок: начальное число имитатора вместо книги (0 — из книги)
    DpMode mode() const { return sh_ ? sh_->eng->mode() : DpMode::Standby; }
    const AppCfg* ship() const { return sh_ ? &sh_->cfg : nullptr; }
    Simulator* sim() { return sim_.get(); }
    AlarmMgr& alarms() { return alarms_; }
    bool quitRequested() const { return quit_; }
    bool takeStatusRequest() { const bool r = statusRequested_; statusRequested_ = false; return r; }
    std::vector<LinkStatus> linkStatus() const { std::vector<LinkStatus> v; if (sh_ && !sim_) for (auto& l : sh_->links) v.push_back(l->status()); return v; }
    RunStats stats() const {
        RunStats s; if (n_ <= 0) return s;
        s.seconds = n_ * dt_; s.rmsPos = std::sqrt(sum2_ / n_); s.maxPos = max_; s.rmsHdg = std::sqrt(h2_ / n_); s.maxHdg = hmax_; return s;
    }

private:
    unsigned seedOverride_ = 0;
    // Всё, что зависит от судна: пересоздаётся при загрузке судна и при запуске или остановке имитатора.
    struct Ship {
        AppCfg cfg; size_t sensorCount = 0;
        std::unique_ptr<DpEngine> eng; std::unique_ptr<SensorHub> hub;
        std::vector<std::unique_ptr<Channel>> chans; std::vector<std::unique_ptr<OutputLink>> links;
        explicit Ship(AppCfg c) : cfg(std::move(c)) {
            sensorCount = cfg.simSensors.gnss.size() + (cfg.simSensors.laser ? 1 : 0) + (cfg.simSensors.wire ? 1 : 0) + cfg.simSensors.gyros + cfg.simSensors.anemometers + cfg.simSensors.mrus;
            rebuild();
        }
        void rebuild() { eng = std::make_unique<DpEngine>(cfg.vessel, cfg.thrusters, cfg.control, cfg.est); hub = std::make_unique<SensorHub>(); }
    };

    bool active() const { return sh_ && sh_->eng->mode() != DpMode::Standby && sh_->eng->mode() != DpMode::Manual; }
    static std::string stateHint(CoreState s) {
        switch (s) {
            case CoreState::NoShip: return " — загрузите судно: ship load <книга.xlsx>";
            case CoreState::WaitSensors: return " — нет курса или положения от датчиков";
            case CoreState::Ready: return " — можно брать движители (thr_take all) и включать режим";
            default: return "";
        }
    }
    void say(const std::string& s) { if (say_) say_(s); }
    void openReal() {
        AppCfg& c = sh_->cfg; SensorHub& hub = *sh_->hub;
        if (c.originSet) hub.setOrigin(c.originLat, c.originLon);
        for (auto& in : c.inputs) {
            hub.addChannel(in.ch);
            std::unique_ptr<Channel> ch;
            if (in.transport == "udp") ch = std::make_unique<UdpChannel>(in.port, in.bind);
            else if (in.transport == "tcp") ch = std::make_unique<TcpChannel>(in.host, in.port);
            else ch = std::make_unique<SerialChannel>(in.device, in.baud);
            SensorHub* hp = &hub; const std::string id = in.ch.id;
            ch->start([hp, id](const std::string& line) { hp->ingest(id, line, monotonicSeconds()); });
            sh_->chans.push_back(std::move(ch));
        }
        for (auto& l : c.links) { sh_->links.push_back(std::make_unique<OutputLink>(l)); sh_->links.back()->start(); }
    }
    void closeIo() {
        if (!sh_) return;
        for (auto& l : sh_->links) l->stop();
        for (auto& c : sh_->chans) c->stop();
        sh_->links.clear(); sh_->chans.clear();
    }
    void resetRun() {
        taken_.clear(); out_ = EngineOutput(); alarms_ = AlarmMgr(); shown_.clear(); fbOverride_.clear();
        opSpSet_ = false; modeSince_ = -1; n_ = sum2_ = max_ = h2_ = hmax_ = 0; lastState_ = CoreState::NoShip;
        applyCaps();
        if (!set_.logFile.empty() && !log_.is_open()) {
            log_.open(set_.logFile);
            if (!log_) say("Предупреждение: не удалось открыть журнал " + set_.logFile);
            else log_ << "t_s,состояние,режим,N_м,E_м,курс_град,опорная_N,опорная_E,опорный_курс_град,ошибка_м,ошибка_курса_град,tauX_Н,tauY_Н,tauN_Нм,загрузка,ветер_мс\n";
        }
    }
    void applyCaps() { if (sh_) sh_->eng->controller().setMotionCaps(capSpeedKn_ * 0.514444, capRotDegMin_ / 60.0); }
    void writeLog() {
        if (!log_) return;
        const EstimatorState& es = out_.est;
        log_ << clock() << ',' << coreStateName(state()) << ',' << modeName(mode()) << ',' << es.n << ',' << es.e << ',' << rad2deg(es.psi) << ',' << out_.ctl.refN << ',' << out_.ctl.refE << ','
             << out_.ctl.refPsiDeg << ',' << out_.ctl.errDist << ',' << out_.ctl.errHdgDeg << ',' << out_.ctl.tau.X << ',' << out_.ctl.tau.Y << ',' << out_.ctl.tau.N << ','
             << out_.alloc.utilization << ',' << es.windSpeed << '\n';
        if (++cycles_ % 50 == 0) log_.flush();
    }
    void initTaken() {
        if (taken_.size() == sh_->cfg.thrusters.size()) return;
        taken_.assign(sh_->cfg.thrusters.size(), 0);
        for (size_t i = 0; i < taken_.size(); ++i) sh_->eng->setThrusterEnabled(i, false);
    }
    void takeThrusters(const std::string& which, bool take) {
        initTaken(); int hit = 0;
        const auto& th = sh_->cfg.thrusters;
        for (size_t i = 0; i < th.size(); ++i) {
            if (which != "all" && th[i].id != which) continue;
            ++hit;
            if (take && !th[i].enabled) { say("Не взят: " + th[i].id + " (в книге «В работе» — нет)"); continue; }
            taken_[i] = take ? 1 : 0;
            sh_->eng->setThrusterEnabled(i, take);
            say(std::string(take ? "Под управлением: " : "Снят с управления: ") + th[i].id);
        }
        if (!hit) say("Нет такого движителя: " + which);
        if (!take && takenCount() == 0 && active()) {
            sh_->eng->setMode(DpMode::Standby); modeSince_ = -1; opSpSet_ = false;
            say("Все движители сняты с управления: режим ожидания.");
        }
    }

    CoreSettings set_; Say say_; double dt_;
    std::unique_ptr<Ship> sh_; std::string shipPath_; BookReport lastRep_;
    std::unique_ptr<Simulator> sim_; double simSpeed_ = 1.0;
    LocalFrame lf_;
    AlarmMgr alarms_;
    std::map<std::string, bool> shown_;
    std::map<std::string, double> fbOverride_;
    std::vector<char> taken_;
    EngineOutput out_;
    std::ofstream log_;
    double t_ = 0, modeSince_ = -1;
    CoreState lastState_ = CoreState::NoShip;
    bool quit_ = false, statusRequested_ = false;
    Setpoint opSp_; bool opSpSet_ = false;
    double capRotDegMin_ = 0, capSpeedKn_ = 0;
    double sum2_ = 0, max_ = 0, h2_ = 0, hmax_ = 0, n_ = 0; long cycles_ = 0;
};

}  // namespace dp
