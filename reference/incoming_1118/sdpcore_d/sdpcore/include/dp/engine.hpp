// Движок СДП: оценка состояния → регулятор → распределение тяги → команды на движители.
#pragma once
#include "dp/forecast.hpp"
#include "dp/control.hpp"
#include "dp/consequence.hpp"

namespace dp {

// Приоритеты сообщений как в K-Pos: 0 — аварийное (Emergency), 1 — тревога (Alarm), 2 — предупреждение (Warning), 3 — информация.
// Если приоритет не задан (-1), он берётся из critical: критичное — тревога, остальное — предупреждение.
struct Alarm {
    std::string code;     // краткий код
    std::string text;     // сообщение оператору
    bool critical = false;
    int prio = -1;
    int level() const { return prio >= 0 ? prio : (critical ? 1 : 2); }
};

struct EngineOutput {
    std::vector<ThrusterCmd> cmds;
    ControlOutput ctl;
    AllocResult alloc;
    EstimatorState est;
    Capability cap;
    Wrench thrustFb;      // фактическая сумма сил движителей
    Wrench windModel;     // сила ветра по модели
    Wrench waveModel;     // средняя дрейфовая сила волн по оценке МИО (с учётом доли ffGain)
    double powerScale = 1;
    double powerKW = 0;       // оценка потребляемой мощности движителей по обратной связи по упору, кВт
    FcOut fc;                 // прогноз внешних сил (если включён)
    std::vector<Alarm> alarms;
    std::vector<std::string> infos;     // информационные сообщения шага (для журнала событий)
};

class DpEngine {
public:
    DpEngine(const VesselCfg& v, std::vector<ThrusterCfg> th, ControlCfg cc = {}, EstimatorCfg ec = {})
        : vcfg_(v), est_(v, ec), ctl_(v, cc), alloc_(std::move(th)) {
        for (const auto& t : alloc_.thrusters()) {
            ThrusterCmd c; c.id = t.id; c.angleDeg = t.type == ThrusterType::Azimuth ? 0.0 : rad2deg(thrusterBaseAngle(t));
            last_.push_back(c);
            want_.push_back(t.enabled ? 1 : 0);
        }
        fbLostT_.assign(last_.size(), 0.0); fbSeen_.assign(last_.size(), 0); fbOut_.assign(last_.size(), 0);
    }

    Controller& controller() { return ctl_; }
    const Estimator& estimator() const { return est_; }
    Estimator& estimatorMut() { return est_; }
    const Allocator& allocator() const { return alloc_; }
    void setHoldRadius(double r) { holdR_ = r > 0 ? r : 0; }
    double holdRadius() const { return holdR_; }
    Forecaster& forecast() { return fc_; }
    const Forecaster& forecast() const { return fc_; }
    // Предел курса, ° (0 — выключен): предупреждение при 80 %, аларм при выходе за предел. Только когда курс удерживается.
    void setHdgLimit(double d) { hdgLim_ = d > 0 ? d : 0; }
    double hdgLimit() const { return hdgLim_; }
    // Движитель под управлением ядра (решение оператора). Движитель без обратной связи ядро временно выводит из распределения само (fbOut_).
    void setThrusterEnabled(size_t i, bool on) { if (i < want_.size()) want_[i] = on ? 1 : 0; alloc_.setEnabled(i, on && !(i < fbOut_.size() && fbOut_[i])); }
    bool thrusterFbLost(size_t i) const { return i < fbOut_.size() && fbOut_[i]; }
    void setThrusterPreload(size_t i, double prefDeg, double pct) { alloc_.setPreload(i, prefDeg, pct); }
    void setMode(DpMode m) { ctl_.setMode(m, est_.state()); hereReq_ = false; }
    Consequence& consequence() { return ca_; }
    const Consequence& consequence() const { return ca_; }
    // «Держать здесь» (выбор оператора после потери положения, как Hold в NAVIS NavDP 4000): по умолчанию после возврата
    // измерений судно плавно возвращается в заданную точку (POS_RETURN). Эта команда вместо возврата делает заданной точкой
    // место, где судно находится: во время возврата — сразу; пока измерений нет — в момент их возврата (оценке по счислению
    // в это время верить нельзя). Курс заданный не меняется. Новая уставка оператора отменяет запрос. Возвращает текст для оператора.
    std::string holdHere() {
        const EstimatorState& es = est_.state();
        if (ctl_.mode() != DpMode::AutoPosition) return "Команда here действует только в режиме удержания позиции.";
        if (ctl_.returning()) {
            const Setpoint sp = ctl_.setpoint();
            ctl_.setSetpoint(ctl_.holdFromCrp(es.n, es.e, sp.psiDeg));
            hereReq_ = false;
            return "Возврат отменён: заданной точкой стало текущее положение судна.";
        }
        if (dropLatched_ || es.refLost) {
            hereReq_ = true; hereSp_ = ctl_.setpoint();
            return "Принято: когда системы положения вернутся, заданной точкой станет место, где окажется судно (без возврата).";
        }
        return "Команда here действует при потере положения (POS_ALL_REJ, POS_DROPOUT) и во время возврата (POS_RETURN).";
    }
    bool holdHereRequested() const { return hereReq_; }
    DpMode mode() const { return ctl_.mode(); }

    EngineOutput step(double dt, const SensorFrame& fr) {
        EngineOutput o;
        alloc_.setSpeedKn(std::hypot(est_.state().u, est_.state().v) / 0.514444);   // спад упора ПУ на ходу
        const auto& th = alloc_.thrusters();
        // --- мощность электростанции
        double need = 0;
        for (const auto& t : th) if (t.enabled) need += t.powerMaxKW;
        double ps = 1.0;
        if (fr.power.fresh) availKW_ = fr.power.availableKW;
        if (need > 0 && availKW_ < need) ps = clampd(availKW_ / need, 0.2, 1.0);
        // коэффициент по упору примерно кубический закон по мощности: упор ~ P^(2/3)
        const double thrustScale = thrustScaleForPower(th, ps);
        o.powerScale = thrustScale;

        // --- потеря обратной связи движителя: у движителя, от которого обратная связь приходила (или ожидается — есть запись в кадре),
        // нет достоверных данных дольше 2 с → тревога, движитель выводится из распределения упоров (команды ему 0), его упор в оценке — 0.
        // Возвращается в распределение сам, когда обратная связь снова достоверна.
        for (size_t i = 0; i < th.size() && i < want_.size(); ++i) {
            bool entry = false, valid = false;
            for (const auto& f : fr.fb) if (f.id == th[i].id) { entry = true; valid = valid || f.valid; }
            if (valid) {
                fbSeen_[i] = 1; fbLostT_[i] = 0;
                if (fbOut_[i]) { fbOut_[i] = 0; alloc_.setEnabled(i, want_[i]); o.infos.push_back("Движитель " + th[i].id + ": обратная связь восстановлена" + (want_[i] ? " — снова в распределении упоров" : "")); }
            } else if (fbSeen_[i] || entry) {
                fbLostT_[i] += dt;
                if (!fbOut_[i] && fbLostT_[i] >= 2.0) { fbOut_[i] = 1; alloc_.setEnabled(i, false); if (want_[i]) o.infos.push_back("Движитель " + th[i].id + ": нет обратной связи — выведен из распределения упоров"); }
            }
        }
        for (size_t i = 0; i < th.size() && i < want_.size(); ++i)
            if (fbOut_[i] && want_[i]) o.alarms.push_back({"THR_FB_LOST_" + th[i].id, "Движитель " + th[i].id + ": нет обратной связи — выведен из распределения упоров, остальные берут его долю", true, 1});

        // --- фактические углы и упоры по обратной связи
        std::vector<double> actAngle(th.size(), 0.0);
        Wrench thrust{};
        for (size_t i = 0; i < th.size(); ++i) {
            const auto& t = th[i];
            double pct = last_[i].pct, ang = last_[i].angleDeg;   // при отсутствии обратной связи — по последней команде
            for (const auto& f : fr.fb) if (f.id == t.id && f.valid) { pct = f.pct; ang = f.angleDeg; }
            actAngle[i] = ang;
            if (!t.enabled) continue;   // выведенный по потере обратной связи — тоже: его упор неизвестен, в оценку не идёт
            const double N = pct >= 0 ? pct / 100.0 * t.maxForward : pct / 100.0 * t.maxReverse;
            const double a = t.type == ThrusterType::Azimuth ? deg2rad(ang) : thrusterBaseAngle(t);
            const Wrench w = thrusterWrench(t, N, a);
            thrust.X += w.X; thrust.Y += w.Y; thrust.N += w.N;
        }
        o.thrustFb = thrust;
        for (size_t i = 0; i < th.size(); ++i) {   // потребляемая мощность по фактическому упору
            if (!th[i].enabled) continue;
            double pct = last_[i].pct; for (const auto& f : fr.fb) if (f.id == th[i].id && f.valid) pct = f.pct;
            o.powerKW += thrusterPowerKW(th[i], std::fabs(pct) / 100.0);
        }

        // --- расхождение команды и обратной связи движителя: больше порога fbDiffPct (азимут: угол больше fbDiffAngleDeg при команде выше 10 %) дольше fbDiffDelayS (по умолчанию 25 %, 30°, 30 с)
        if (!(ctl_.mode() == DpMode::Standby)) {
            auto g = [](double v) { char b[24]; std::snprintf(b, sizeof b, "%g", v); return std::string(b); };
            for (size_t i = 0; i < th.size() && i < last_.size(); ++i) {
                if (!th[i].enabled) continue;
                const ThrusterFb* f = nullptr; for (const auto& x : fr.fb) if (x.id == th[i].id && x.valid) f = &x;
                bool bad = false;
                if (f) {
                    bad = std::fabs(f->pct - last_[i].pct) > th[i].fbDiffPct;
                    if (!bad && th[i].type == ThrusterType::Azimuth && std::fabs(last_[i].pct) > 10.0)
                        bad = std::fabs(rad2deg(wrapPi(deg2rad(f->angleDeg - last_[i].angleDeg)))) > th[i].fbDiffAngleDeg;
                }
                if (mmT_.size() <= i) mmT_.resize(th.size(), 0.0);
                mmT_[i] = bad ? mmT_[i] + dt : 0.0;
                if (mmT_[i] >= th[i].fbDiffDelayS) o.alarms.push_back({"THR_FB_MISMATCH_" + th[i].id, "Движитель " + th[i].id + ": обратная связь расходится с командой (упор больше " + g(th[i].fbDiffPct) + " % или угол больше " + g(th[i].fbDiffAngleDeg) + "°) более " + g(th[i].fbDiffDelayS) + " с", false});
            }
        } else mmT_.assign(mmT_.size(), 0.0);

        // --- ветер по модели (по отфильтрованному кажущемуся ветру предыдущего шага)
        const EstimatorState& prev = est_.state();
        Wrench wind{};
        if (prev.windValid) wind = windForce(vcfg_, prev.windSpeed, prev.windFromRel);
        Wrench fcApplied{};      // поправка таблицы прогнозирования, внесённая в силу ветра (общая для оценки и закона управления)
        if (fc_.cfg().enabled && fc_.cfg().apply && prev.windValid && prev.hdgValid) { fcApplied = fc_.correction(prev.windSpeed, prev.windFromRel, lastCap_); wind.X += fcApplied.X; wind.Y += fcApplied.Y; wind.N += fcApplied.N; }
        o.windModel = wind;
        // --- дрейфовая сила волн по оценке, полученной из данных МИО (не из настроек имитатора)
        Wrench wave{}, wavePrev{};
        bool waveOn = false; double wHs = 0, wFrom = 0, wGain = 0;
        if (ctl_.cfg().waveFeedForward && prev.wave.valid && prev.wave.dirValid && prev.hdgValid && prev.wave.ffGain > 0) {
            waveOn = true; wHs = prev.wave.hs; wFrom = prev.wave.fromDeg; wGain = prev.wave.ffGain;
            wave = waveDriftForce(vcfg_, wHs, wFrom, prev.psi, prev.wave.tz);
            wave.X *= wGain; wave.Y *= wGain; wave.N *= wGain;
        }
        if (lastWaveOn_) {                       // прежняя оценка при нынешнем курсе: разница — только от уточнения оценки волн
            wavePrev = waveDriftForce(vcfg_, lastHs_, lastFrom_, prev.psi, lastTz_);
            wavePrev.X *= lastGain_; wavePrev.Y *= lastGain_; wavePrev.N *= lastGain_;
        }
        o.waveModel = wave;
        est_.absorbKnownChange(Wrench{wave.X - wavePrev.X, wave.Y - wavePrev.Y, wave.N - wavePrev.N});
        lastWaveOn_ = waveOn; lastTz_ = prev.wave.tz; lastHs_ = wHs; lastFrom_ = wFrom; lastGain_ = wGain;
        const Wrench known{wind.X + wave.X, wind.Y + wave.Y, wind.N + wave.N};     // известные внешние силы: ветер и волны

        // --- оценка
        // ветровая сила уже учтена в модели, поэтому неизвестная сила (bias) содержит только остаток
        est_.step(dt, fr.time, fr, thrust, known);
        const EstimatorState& es = est_.state();
        o.est = es;

        // --- возможности и управление
        const double effScale = thrustScale;     // допустимая доля упора определяется электростанцией
        const Capability cap = alloc_.capability(effScale);
        o.cap = cap;
        const DpMode m = ctl_.mode();
        const bool needPos = m == DpMode::AutoPosition || m == DpMode::Track || m == DpMode::FollowTarget;
        const bool needHdg = m != DpMode::Standby && m != DpMode::Manual;
        bool ok = true;
        if (needPos && !es.posValid) { ok = false; o.alarms.push_back({"POS_LOST", "Потеря положения — счисление по модели превысило допустимое время", true}); }
        if (needHdg && !es.hdgValid) { ok = false; o.alarms.push_back({"HDG_LOST", "Потеря курса — нет данных гирокомпасов, счисление курса закончилось: упор снят", true}); }
        if (needHdg && es.hdgDeadReckon) {
            const int left = static_cast<int>(std::max(0.0, est_.cfg().hdgDeadReckonS - es.hdgLostS));
            o.alarms.push_back({"HDG_DR", "Потеря курса — нет данных гирокомпасов " + std::to_string(static_cast<int>(es.hdgLostS)) + " с, курс по счислению (до снятия упора " + std::to_string(left) + " с)", true, 1});
        }
        if (es.gnssDisagree) o.alarms.push_back({"POS_DISAGREE", "Расхождение источников положения", false});
        // потеря опорных систем (K-Pos, «Position dropout»): сначала «Все системы положения отклонены», через posDropoutS — «Потеря положения»;
        // судно при этом удерживается по модели, уставка оператора не меняется. Когда измерения вернутся и судно окажется дальше
        // returnStartM от уставки, оно возвращается в неё плавно, не быстрее returnSpeedMS (ctl_.beginReturn)
        if (needPos && es.posValid && es.refLost) {
            if (es.posDropout) {
                char u[160]; std::snprintf(u, sizeof u, "вероятный уход ≈ %.1f м (оценка), погрешность фильтра до %.1f м", es.drDriftM, es.posUncertM);
                o.alarms.push_back({"POS_DROPOUT", "Потеря положения — нет измерений " + std::to_string(static_cast<int>(es.refLostS)) + " с, удержание по модели (счисление), " + u +
                                    (hereReq_ ? "; после возврата — держать здесь (выбор оператора)" : "; после возврата — в заданную точку, держать здесь — команда here"), true, 1});
                dropLatched_ = true;
            } else o.alarms.push_back({"POS_ALL_REJ", "Все системы положения отклонены или недоступны — удержание по модели", true, 1});
            if (ctl_.returning()) dropLatched_ = true;   // потеря во время возврата: после неё возврат начнётся заново от верной оценки
        }
        if (dropLatched_ && !es.refLost) {   // измерения положения вернулись после потери
            dropLatched_ = false;
            const bool here = hereReq_ && m == DpMode::AutoPosition && sameSp(ctl_.setpoint(), hereSp_);
            hereReq_ = false;
            if (here) {
                ctl_.setSetpoint(ctl_.holdFromCrp(es.n, es.e, ctl_.setpoint().psiDeg));
                o.infos.push_back("Положение восстановлено: заданной точкой стало текущее положение судна (выбор оператора, без возврата)");
            } else if (m == DpMode::AutoPosition) {
                const Setpoint crp = ctl_.crpFromHold(ctl_.setpoint());
                const double d = std::hypot(es.n - crp.n, es.e - crp.e);
                const ControlCfg& cc = ctl_.cfg();
                char b[320];
                // Возврат начинается заново и тогда, когда судно близко к точке, но возврат уже шёл (потеря во время возврата):
                // иначе опорная точка оставалась там, куда её привела ошибочная оценка по счислению (найдено 16.11.2026).
                const bool wasRet = ctl_.returning();
                if (d > cc.returnStartM || wasRet) {
                    ctl_.beginReturn(es, cc.returnSpeedMS, 3.0);
                    std::snprintf(b, sizeof b, "Положение восстановлено: судно в %.1f м от заданной точки, возврат со скоростью до %.1f м/с; держать здесь — команда here", d, cc.returnSpeedMS > 0 ? std::min(cc.returnSpeedMS, cc.refMaxSpeed) : cc.refMaxSpeed);
                } else std::snprintf(b, sizeof b, "Положение восстановлено: судно в %.1f м от заданной точки", d);
                o.infos.push_back(b);
            } else o.infos.push_back("Положение восстановлено");
        }
        if (wasReturning_ && !ctl_.returning() && m == DpMode::AutoPosition) o.infos.push_back("Возврат в заданную точку завершён");
        wasReturning_ = ctl_.returning();
        if (m == DpMode::AutoPosition && ctl_.returning()) {
            char b[200]; std::snprintf(b, sizeof b, "Возврат в заданную точку после потери положения: осталось %.1f м (держать здесь — команда here)", ctl_.returnLeftM());
            o.alarms.push_back({"POS_RETURN", b, false});
        }
        for (const auto& pa : es.posAlerts) o.alarms.push_back({pa.first, pa.second, false});
        if (es.softCorrecting) o.alarms.push_back({"POS_SOFT", "Плавная коррекция положения — после скачка опорной системы", false});
        if (es.gyroDisagree) o.alarms.push_back({"GYRO_DISAGREE", "Расхождение гирокомпасов", false});
        if (!fr.mru.empty() && !es.mruValid) o.alarms.push_back({"MRU_LOST", "Нет достоверных данных МИО: волны не оцениваются, наклон антенн не учитывается", false});
        if (es.mruDisagree) o.alarms.push_back({"MRU_DISAGREE", "Расхождение показаний МИО", false});
        if (ps < 0.999) o.alarms.push_back({"POWER_LIMIT", "Ограничение по мощности электростанции", false});

        ControlOutput co;
        if (ok) co = ctl_.compute(dt, es, known, cap);
        o.ctl = co;

        // --- анализ последствий отказа: в режиме «Позиция», опорная точка неподвижна (уставка достигнута), есть измерения положения, нет возврата
        {
            const bool inPos = m == DpMode::AutoPosition && ok;
            const double dpos = std::hypot(co.refN - caRefN_, co.refE - caRefE_), dpsi = std::fabs(rad2deg(wrapPi(deg2rad(co.refPsiDeg - caRefPsi_))));
            caRefN_ = co.refN; caRefE_ = co.refE; caRefPsi_ = co.refPsiDeg;
            const bool still = dt > 0 && dpos / dt < 0.005 && dpsi / dt < 0.01;
            std::vector<CaMsg> cm;
            ca_.step(fr.time, dt, inPos, inPos && still && !es.refLost && !ctl_.returning(), co.tau, alloc_.thrusters(), cm);
            for (const auto& x : cm) o.alarms.push_back({x.code, x.text, x.critical});
        }

        // --- распределение
        if (m == DpMode::Standby || m == DpMode::Manual || !ok) {
            // команды не формируем: в ручном режиме движители управляются с поста, при потере данных удерживаем нулевую тягу
            for (size_t i = 0; i < th.size(); ++i) {
                ThrusterCmd c; c.id = th[i].id; c.pct = 0; c.angleDeg = actAngle[i];
                if (m == DpMode::Standby && th[i].type == ThrusterType::Azimuth && th[i].parkDeg < 1e8) c.angleDeg = th[i].parkDeg;   // стоянка колонок
                o.cmds.push_back(c);
            }
            o.alloc.cmds = o.cmds;
        } else {
            o.alloc = alloc_.allocate(co.tau, actAngle, effScale);
            o.cmds = o.alloc.cmds;
            // Ограничение скорости изменения команд возможностями самого движителя: распределитель упоров
            // может переключать решение скачком (пара ВРК при малых усилиях), а движитель такой скачок всё равно не отработает.
            for (size_t i = 0; i < th.size() && i < o.cmds.size() && i < last_.size(); ++i) {
                if (!th[i].enabled) continue;
                const double dp = 100.0 * th[i].rateFracPerS * dt;
                // мягкая зона нечувствительности: малые изменения команды не передаются (меньше «дёрганья» упора и колонок)
                const double pd = th[i].pctDeadPct;
                const double dpct = pd > 0 ? Controller::dead(o.cmds[i].pct - last_[i].pct, pd) : o.cmds[i].pct - last_[i].pct;
                o.cmds[i].pct = last_[i].pct + clampd(dpct, -dp, dp);
                if (th[i].type == ThrusterType::Azimuth) {
                    const double da = th[i].azRateDegS * dt;
                    double err = rad2deg(wrapPi(deg2rad(o.cmds[i].angleDeg - last_[i].angleDeg)));
                    if (th[i].azDeadDeg > 0) err = Controller::dead(err, th[i].azDeadDeg);
                    o.cmds[i].angleDeg = last_[i].angleDeg + clampd(err, -da, da);
                }
            }
            for (const auto& id : o.alloc.softZone) o.alarms.push_back({"AZ_SOFT_ZONE_" + id, "Колонка " + id + " в нежелательном секторе (струя на соседнюю колонку): иначе запрос не выполнить", false});
            // Насыщение: запрос больше возможностей комплекса — тревога сразу. Если же запрос в пределах возможностей, а недодача только
            // оттого, что колонки разворачиваются (при торможении нужен разворот до 180°), — сообщение «Колонки разворачиваются»;
            // тревога — если недодача держится дольше времени разворота на 180° самой медленной колонки + 10 с
            // (колонка не поворачивается — это ещё и тревога расхождения команды и обратной связи THR_FB_MISMATCH).
            // Снимается тревога, когда насыщения нет 3 с подряд (без мигания при работе на пределе).
            // запрос регулятора до ограничения возможностями (регулятор сам обрезает команду до предела, поэтому по обрезанной не видно)
            const double rqX = o.ctl.law.X + o.ctl.windFF.X + o.ctl.bias.X, rqY = o.ctl.law.Y + o.ctl.windFF.Y + o.ctl.bias.Y, rqN = o.ctl.law.N + o.ctl.windFF.N + o.ctl.bias.N;
            const bool overCap = std::fabs(rqX) > cap.X || std::fabs(rqY) > cap.Y || std::fabs(rqN) > cap.N;
            double turnLimit = 10.0;
            for (const auto& t : th) if (t.enabled && t.type == ThrusterType::Azimuth) turnLimit = std::max(turnLimit, 180.0 / std::max(1.0, t.azRateDegS) + 10.0);
            satT_ = o.alloc.saturated ? satT_ + dt : 0.0;
            const bool turnOnly = o.alloc.saturated && o.alloc.turning && !overCap;
            const bool realSat = (o.alloc.saturated && (!turnOnly || satT_ > turnLimit)) || (overCap && o.alloc.utilization > 0.95);
            if (realSat) { satOn_ = true; satClrT_ = 0; }
            else if (satOn_) { satClrT_ += dt; if (satClrT_ >= 3.0) satOn_ = false; }
            if (satOn_) o.alarms.push_back({"SATURATION", "Запрошенное усилие превышает возможности комплекса", false});
            else if (turnOnly) o.alarms.push_back({"AZ_TURNING", "Колонки разворачиваются: упор временно набирается не полностью", false, 3});
            // тревога высокой загрузки с гистерезисом: включается после 3 с выше 90 % допустимой, снимается после 3 с ниже 80 %;
            // пока колонки разворачиваются при запросе в пределах возможностей, время не копится (кратковременная перекладка)
            const double u = o.alloc.utilization;
            if (!highLoad_) { onT_ = u > 0.9 && !(o.alloc.turning && !overCap) ? onT_ + dt : (u > 0.9 ? onT_ : 0.0); if (onT_ >= 3.0) { highLoad_ = true; offT_ = 0; } }
            else { offT_ = u < 0.8 ? offT_ + dt : 0.0; if (offT_ >= 3.0) { highLoad_ = false; onT_ = 0; } }
        }
        if (m == DpMode::Standby || m == DpMode::Manual || !ok) { highLoad_ = false; onT_ = offT_ = 0; satT_ = 0; satOn_ = false; satClrT_ = 0; }
        if (highLoad_) o.alarms.push_back({"HIGH_LOAD", "Загрузка движителей выше 90 % допустимой", false});
        // --- радиус удержания: предупреждение при 80 % радиуса, аларм при выходе за радиус (с выдержкой и гистерезисом)
        if (holdR_ > 0 && ok && (m == DpMode::AutoPosition || m == DpMode::Track)) {
            const double q = o.ctl.errDist / holdR_;
            auto upd = [&](bool& on, double& tOn, double& tOff, double hi, double lo, double holdOn, double holdOff) {
                if (!on) { tOn = q >= hi ? tOn + dt : 0.0; if (tOn >= holdOn) { on = true; tOff = 0; } }
                else { tOff = q < lo ? tOff + dt : 0.0; if (tOff >= holdOff) { on = false; tOn = 0; } }
            };
            upd(rOut_, rOutOn_, rOutOff_, 1.0, 0.9, 2.0, 3.0);
            upd(rWarn_, rWarnOn_, rWarnOff_, 0.8, 0.7, 3.0, 3.0);
            char b[200];
            if (rOut_) { std::snprintf(b, sizeof b, "Судно вышло за радиус удержания: %.1f м при радиусе %.1f м", o.ctl.errDist, holdR_); Alarm a{"RADIUS_OUT", b, true}; a.prio = 1; o.alarms.push_back(a); }
            else if (rWarn_) { std::snprintf(b, sizeof b, "Приближение к границе радиуса удержания: %.1f м из %.1f м", o.ctl.errDist, holdR_); o.alarms.push_back({"RADIUS_WARN", b, false}); }
        } else { rWarn_ = rOut_ = false; rWarnOn_ = rWarnOff_ = rOutOn_ = rOutOff_ = 0; }
        // --- предел курса: предупреждение при 80 % предела, аларм при выходе (с выдержкой и гистерезисом, как у радиуса)
        if (hdgLim_ > 0 && ok && (m == DpMode::AutoHeading || m == DpMode::AutoPosition || m == DpMode::Track)) {
            const double q = std::fabs(o.ctl.errHdgDeg) / hdgLim_;
            auto upd = [&](bool& on, double& tOn, double& tOff, double hi, double lo, double holdOn, double holdOff) {
                if (!on) { tOn = q >= hi ? tOn + dt : 0.0; if (tOn >= holdOn) { on = true; tOff = 0; } }
                else { tOff = q < lo ? tOff + dt : 0.0; if (tOff >= holdOff) { on = false; tOn = 0; } }
            };
            upd(hOut_, hOutOn_, hOutOff_, 1.0, 0.9, 2.0, 3.0);
            upd(hWarn_, hWarnOn_, hWarnOff_, 0.8, 0.7, 3.0, 3.0);
            char b[200];
            if (hOut_) { std::snprintf(b, sizeof b, "Курс вышел за предел: отклонение %.1f° при пределе %.1f°", std::fabs(o.ctl.errHdgDeg), hdgLim_); Alarm a{"HDG_OUT", b, true}; a.prio = 1; o.alarms.push_back(a); }
            else if (hWarn_) { std::snprintf(b, sizeof b, "Приближение к пределу курса: %.1f° из %.1f°", std::fabs(o.ctl.errHdgDeg), hdgLim_); o.alarms.push_back({"HDG_WARN", b, false}); }
        } else { hWarn_ = hOut_ = false; hWarnOn_ = hWarnOff_ = hOutOn_ = hOutOff_ = 0; }
        // --- прогнозирование внешних сил: накопление таблицы поправок и прогноз нагрузки на 1…10 мин
        {
            FcIn fi; fi.dt = dt; const EstimatorState& e2 = o.est;
            fi.learn = ok && m == DpMode::AutoPosition && e2.posValid && e2.hdgValid && e2.windValid && !lastSat_ && std::hypot(e2.vn, e2.ve) < 0.6;
            fi.windValid = e2.windValid; fi.hdgValid = e2.hdgValid; fi.windSpeed = e2.windSpeed; fi.windRel = e2.windFromRel; fi.psi = e2.psi;
            fi.biasN = e2.biasN; fi.biasE = e2.biasE; fi.biasM = e2.biasMoment; fi.applied = fcApplied; fi.cap = cap;
            fi.waveValid = waveOn; fi.hs = wHs; fi.waveFrom = wFrom; fi.tz = e2.wave.tz; fi.waveGain = wGain;
            o.fc = fc_.step(fi, vcfg_);
            if (o.fc.warn && (m == DpMode::AutoPosition || m == DpMode::Track)) {
                char b[200]; double l5 = 0; for (size_t k = 0; k < o.fc.horizonS.size(); ++k) if (std::fabs(o.fc.horizonS[k] - fc_.cfg().warnHorizonS) < 1.0) l5 = o.fc.load[k];
                std::snprintf(b, sizeof b, "Прогноз: через %.0f мин нагрузка комплекса составит %.0f %% допустимой", fc_.cfg().warnHorizonS / 60.0, 100.0 * l5);
                o.alarms.push_back({"FORECAST_LOAD", b, false});
            }
        }
        lastCap_ = cap; lastSat_ = o.alloc.saturated;
        last_ = o.cmds;
        return o;
    }

private:
    bool highLoad_ = false, dropLatched_ = false, wasReturning_ = false; double onT_ = 0, offT_ = 0;
    Consequence ca_; double caRefN_ = 0, caRefE_ = 0, caRefPsi_ = 0;
    bool hereReq_ = false; Setpoint hereSp_;
    static bool sameSp(const Setpoint& a, const Setpoint& b) { return std::fabs(a.n - b.n) < 1e-6 && std::fabs(a.e - b.e) < 1e-6 && std::fabs(a.psiDeg - b.psiDeg) < 1e-6; }
    std::vector<char> want_, fbSeen_, fbOut_; std::vector<double> fbLostT_;
    Forecaster fc_; Capability lastCap_; bool lastSat_ = false; double satT_ = 0, satClrT_ = 0; bool satOn_ = false;
    double hdgLim_ = 0; bool hWarn_ = false, hOut_ = false; double hWarnOn_ = 0, hWarnOff_ = 0, hOutOn_ = 0, hOutOff_ = 0;
    std::vector<double> mmT_;
    double holdR_ = 0; bool rWarn_ = false, rOut_ = false; double rWarnOn_ = 0, rWarnOff_ = 0, rOutOn_ = 0, rOutOff_ = 0;
    VesselCfg vcfg_;
    Estimator est_;
    Controller ctl_;
    Allocator alloc_;
    std::vector<ThrusterCmd> last_;
    double availKW_ = 1e9;
    bool lastWaveOn_ = false; double lastHs_ = 0, lastFrom_ = 0, lastGain_ = 0, lastTz_ = 0;
};

}  // namespace dp
