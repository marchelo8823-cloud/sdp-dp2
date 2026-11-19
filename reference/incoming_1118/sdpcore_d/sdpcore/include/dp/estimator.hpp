// Оценка состояния: объединение датчиков положения и курса, фильтры Калмана с оценкой
// неизвестной внешней силы (ветер, течение, волны), фильтрация ветра.
#pragma once
#include <array>
#include <cstdio>
#include <deque>
#include <map>
#include <string>

#include "dp/calibration.hpp"
#include "dp/model.hpp"
#include "dp/waves.hpp"

namespace dp {

// Фильтр Калмана с тремя состояниями: положение, скорость, неизвестная сила.
class Kf3 {
public:
    double p = 0, v = 0, b = 0;      // положение, скорость, смещение силы (Н)
    Mx P = Mx::eye(3);
    double qv = 1e-4, qb = 5e-4;    // интенсивности шума процесса (для скорости — (м/с)²/с, для силы — (Н/√с... в долях массы))
    bool inited = false;

    void init(double p0, double pVar, double bVar = 1e-4) {
        p = p0; v = 0; b = 0; inited = true;
        P = Mx(3, 3);
        P(0, 0) = pVar; P(1, 1) = 0.25; P(2, 2) = bVar;
    }

    // F — известная сила, Н; m, d — эффективная масса и линейное демпфирование.
    void predict(double dt, double F, double m, double d) {
        if (!inited) return;
        const double a = (F + b - d * v) / m;
        p += v * dt + 0.5 * a * dt * dt;
        v += a * dt;
        Mx A = Mx::eye(3);
        A(0, 1) = dt; A(1, 1) = 1.0 - d / m * dt; A(1, 2) = dt / m;
        Mx Q(3, 3);
        Q(1, 1) = qv * dt;
        const double sb = qb * m;                      // шум силы масштабируется массой
        Q(2, 2) = sb * sb * dt;
        P = A * P * A.T() + Q;
    }

    // Возвращает нормированную невязку (для отбраковки).
    double innovation(double z, double R) const {
        const double S = P(0, 0) + R;
        return (z - p) / std::sqrt(S);
    }

    void update(double z, double R, double m) {
        (void)m;
        const double S = P(0, 0) + R;
        double K[3] = {P(0, 0) / S, P(1, 0) / S, P(2, 0) / S};
        const double y = z - p;
        p += K[0] * y; v += K[1] * y; b += K[2] * y;
        Mx Pn = P;
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) Pn(i, j) = P(i, j) - K[i] * P(0, j);
        P = Pn;
    }
};

// Настройка одной системы положения (по образцу K-Pos): режим, вес, предел приёмки по предсказанию.
struct PosRefCfg {
    int mode = 0;      // 0 — используется, 1 — наблюдение (проверки идут, в решение не входит), 2 — выключена,
                       // 3 — «мобильная» (как K-Pos/GE «Mobile»: показывается и проверяется, но в удержании положения не участвует — например, маяк ROV)
    int manualWeight = 5;          // ручной вес 1…10 (только при weightMode = ручной; 5 — нейтральный)
    int weight = 0;    // только для показа: 1, если к системе применён пониженный вес GNSS (глобальная настройка, как в K-Pos)
    int accept = 1;    // предел приёмки по предсказанию: 0 — узкий, 1 — обычный, 2 — широкий
    bool autoRef = false;          // автоначало: первое измерение системы приводится к текущему решению (относительные системы — лазер, натяжной трос, гидроакустика)
    double biasN = 0, biasE = 0;   // поправка постоянного смещения системы (из калибровки), м: вычитается из приведённой позиции
};

struct EstimatorCfg {
    double posTimeoutS = 4.0;
    double biasRecoverS = 60.0, biasRecoverGain = 8.0;   // после возврата измерений: столько секунд шум процесса неизвестной силы × столько (0 с — выкл.)
    double posDropoutS = 20.0;        // «Position dropout» (K-Pos): столько секунд без принятых измерений положения
    double deadReckonMaxS = 0.0;      // 0 — счисление без ограничения по времени (как в K-Pos); иначе после этого времени положение считается потерянным ⚠
    double hdgTimeoutS = 2.0;
    double hdgDeadReckonS = 2.0;      // потеря курса: столько секунд (от последнего данного гирокомпаса) курс держится по счислению, затем курс
                                      // считается потерянным и упор снимается. 2 с — как «Heading dropout» в K-Pos (руководство 7.0, 8.1.7); было 30 с
    double drDrift5MinM = 14.0;       // вероятный уход судна за 5 мин счисления, м (оценка «k·t²»; по имитатору, уточняется на испытаниях)
    double windTimeoutS = 6.0;
    double gateSigma = 5.0;          // порог отбраковки по невязке
    int resetAfterRejects = 6;       // после стольких подряд отбракованных измерений принимаем скачок
    double gnssDisagreeM = 8.0;
    int freezeCount = 10;            // тест заморозки: столько одинаковых подряд измерений положения — система считается «зависшей» (0 — выкл.)
    // компиляция позиции: все значения по умолчанию дают прежнее поведение
    std::map<std::string, PosRefCfg> posRef;
    double acceptSigmaNarrow = 3.0, acceptSigmaWide = 8.0;   // пределы приёмки по предсказанию в сигмах (обычный — gateSigma)
    // «Reduced GPS» как в K-Pos: вес GNSS относительно остальных систем падает в 8 раз (0,33/0,33/0,33 → 0,10/0,10/0,80 для двух GNSS и одной другой системы);
    // пределы приёмки и медианы при этом не меняются
    int gnssWeight = 0;                                      // 0 — нормальный, 1 — пониженный вес GNSS
    double reducedGnssWeightRatio = 0.125;                   // относительный вес пониженных GNSS
    int medianMode = 0;              // медианная проверка (нужны три и более системы): 0 — выкл., 1 — предупреждение, 2 — отклонять
    double medianHystFrac = 0.7;     // отказ медианного теста снимается, когда расхождение ниже этой доли предела…
    double medianHoldS = 5.0;        // …и так держится не менее этого времени, с (против мигания тревоги)
    double medianFrac = 0.8;         // предел медианы — эта доля от наименьшего предела по предсказанию
    // Тест дисперсии (повторяемости) и веса по нему, как в K-Pos/Converteam. Повторяемость — СКО высокочастотного шума измерений:
    // по вторым разностям последних varWindow отсчётов (плавное движение судна вычитается), σ_rep = СКО(z[k]−2z[k−1]+z[k−2])/√6.
    int varTest = 0;                 // 0 — выкл., 1 — предупреждение, если σ_rep больше varFactor·заявленного СКО
    double varFactor = 3.0;
    int varWindow = 30, varMinSamples = 10;
    int weightMode = 0;              // 0 — обычные веса (по заявленному СКО), 1 — ручные 1…10 (PosRefCfg::manualWeight), 2 — автоматические по повторяемости
    //   автоматический вес системы = (заявленное СКО / max(заявленное СКО, σ_rep))² — только снижает вес зашумлённых систем, выше заявленного не поднимает
    // Тест расхождения (только предупреждение; нужны две и более системы в решении): предел в метрах, 0 — выкл.
    double divergeM = 0.0;
    double softRateMS = 0.0;         // мягкая коррекция: после скачка положение сдвигается не быстрее этой скорости, м/с (0 — выкл.)
    double softStartM = 1.0;         // коррекция меньше этого значения применяется сразу
    double gyroDisagreeDeg = 3.0;
    double windFilterS = 10.0;
    double mruTimeoutS = 1.5;        // МИО без свежих данных дольше этого считается неисправным
    double mruDisagreeDeg = 0.7;     // расхождение МИО по крену и дифференту, град
    double mruDisagreeHeaveM = 0.5;  // расхождение МИО по вертикальной качке, м
};

struct SensorStatus {
    std::string id;
    bool valid = false;
    double ageS = 1e9;
    int rejected = 0;
    std::string state;     // состояние для оператора: OK, PRED (отклонён по предсказанию), MEDIAN, MONITOR, OFF, STALE
};

struct EstimatorState {
    bool posValid = false, hdgValid = false, windValid = false;
    double n = 0, e = 0;              // положение точки управления, м
    double vn = 0, ve = 0;            // скорость, м/с
    double biasN = 0, biasE = 0;      // неизвестная сила в мировых осях, Н
    double psi = 0, r = 0, biasMoment = 0;
    double u = 0, v = 0;              // скорость в связанных осях
    double windSpeed = 0, windFromRel = 0;   // отфильтрованный кажущийся ветер
    double roll = 0, pitch = 0, heave = 0;   // слитые показания МИО (мгновенные): рад, рад, м
    bool mruValid = false, mruDisagree = false; int mruCount = 0;   // сколько МИО в согласии
    WaveEstimate wave;                // волнение по данным МИО
    std::string posSource;            // идентификаторы использованных источников
    bool gnssDisagree = false, gyroDisagree = false;
    std::vector<std::pair<std::string, std::string>> posAlerts;   // события компиляции позиции: (код, текст)
    // Диаграмма качества опорных систем: отклонения последних измерений каждой системы от решения (м) и повторяемость
    struct RefScatter { std::string id; std::vector<std::pair<float, float>> pts; double sdRep = 0, sigma = 0; bool varBad = false; int mode = 0; std::string state; };
    std::vector<RefScatter> scatter;
    bool softCorrecting = false;                                  // идёт мягкая коррекция после скачка
    // потеря опорных систем (как K-Pos): refLost — все системы положения отклонены или недоступны (положение держится по модели, «счисление»),
    // posDropout — так длится дольше posDropoutS (20 с). refLostS — сколько секунд положение не уточнялось измерениями.
    bool refLost = false, posDropout = false; double refLostS = 0;
    double posUncertM = 0;            // погрешность положения по фильтру (1 СКО, м): растёт при счислении; ⚠ для прогноза ухода сильно завышена
    double drDriftM = 0;              // вероятный уход при счислении, м: drDrift5MinM·(t/300 с)², t — сколько секунд нет измерений положения
    // курс: hdgMeasured — есть достоверные данные гирокомпасов; hdgDeadReckon — их нет, курс держится по счислению (не дольше hdgDeadReckonS);
    // hdgValid — курс пригоден для управления (измерен или по счислению); hdgLostS — сколько секунд нет данных гирокомпасов
    bool hdgMeasured = false, hdgDeadReckon = false; double hdgLostS = 0;
    double posSigma = 0;
    std::vector<SensorStatus> sensors;
};

class Estimator {
public:
    void rearmAutoRef(const std::string& id) { posSeen_[id].autoDone = false; }
    Estimator(const VesselCfg& cfg, EstimatorCfg ec = {}) : ec_(ec), in_(completeInertia(cfg)), waves_(cfg) {
        mEff_ = 0.5 * (in_.m11 + in_.m22);
        dEff_ = 0.5 * (in_.d11 + in_.d22);
    }

    // thrustBody — сумма сил движителей по обратной связи; windBody — сила ветра по модели (оба в связанных осях).
    void step(double dt, double time, const SensorFrame& fr, const Wrench& thrustBody, const Wrench& windBody) {
        // --- курс
        updateHeading(time, fr);
        const double psi = s_.hdgValid ? kpsi_.p : lastPsi_;
        // --- МИО: голосование по приборам, оценка волнения
        updateMru(dt, time, fr, psi);
        // --- положение
        const double cp = std::cos(psi), sp = std::sin(psi);
        const double Fx = thrustBody.X + windBody.X, Fy = thrustBody.Y + windBody.Y;
        const double Fn = Fx * cp - Fy * sp, Fe = Fx * sp + Fy * cp;
        // После долгой потери положения неизвестная сила (течение, порывы) успела измениться, а её оценка за время счисления — нет.
        // Поэтому recoverS секунд после возврата измерений шум процесса силы увеличен в recoverGain раз: оценка сходится быстрее,
        // и судно не разгоняет неучтённая сила (задача №17).
        const double qb0 = kn_.qb;
        if (recoverT_ > 0) { kn_.qb = ke_.qb = qb0 * ec_.biasRecoverGain; recoverT_ -= dt; }
        kn_.predict(dt, Fn, mEff_, dEff_);
        ke_.predict(dt, Fe, mEff_, dEff_);
        kn_.qb = ke_.qb = qb0;
        updatePosition(time, fr, psi);
        // курс: предсказание
        const double Nmom = thrustBody.N + windBody.N;
        if (kpsi_.inited && s_.hdgDeadReckon) {
            kpsi_.v = 0;   // счисление курса: ⚠ прогноз по модели моментов неустойчив (см. разбор), курс держится последним измеренным
            kpsi_.P(0, 0) += deg2rad(0.05) * deg2rad(0.05) * dt;
        } else if (kpsi_.inited) {
            const double a = (Nmom + kpsi_.b - in_.d66 * kpsi_.v) / in_.m66;
            kpsi_.p += kpsi_.v * dt + 0.5 * a * dt * dt;
            kpsi_.v += a * dt;
            Mx A = Mx::eye(3); A(0, 1) = dt; A(1, 1) = 1.0 - in_.d66 / in_.m66 * dt; A(1, 2) = dt / in_.m66;
            Mx Q(3, 3); Q(1, 1) = 1e-8 * dt; const double sb = 2e-3 * in_.m66; Q(2, 2) = sb * sb * dt;
            kpsi_.P = A * kpsi_.P * A.T() + Q;
            kpsi_.p = wrapPi(kpsi_.p);
        }
        correctHeading(fr);
        // --- ветер
        updateWind(dt, time, fr);
        publish(psi);
    }

    // Известная внешняя сила изменилась из-за уточнения оценки (например, волн), а не из-за реального изменения нагрузки:
    // чтобы суммарная сила в фильтре осталась непрерывной, неизвестная сила уменьшается на ту же величину (связанные оси → мировые).
    void absorbKnownChange(const Wrench& dBody) {
        if (!kn_.inited) return;
        const double cp = std::cos(s_.psi), sp = std::sin(s_.psi);
        kn_.b -= dBody.X * cp - dBody.Y * sp;
        ke_.b -= dBody.X * sp + dBody.Y * cp;
        if (kpsi_.inited) kpsi_.b -= dBody.N;
    }
    const EstimatorState& state() const { return s_; }
    Calibrator& calibrator() { return cal_; }
    const Calibrator& calibrator() const { return cal_; }
    EstimatorCfg& cfgMut() { return ec_; }          // настройки компиляции позиции меняются оператором во время работы
    const EstimatorCfg& cfg() const { return ec_; }
    void setOrigin(double n, double e) { (void)n; (void)e; }

private:
    Calibrator cal_;
    void updateMru(double dt, double time, const SensorFrame& fr, double psi) {
        struct V { std::string id; double r, p, h; size_t idx; };
        std::vector<V> v;
        for (const auto& m : fr.mru) {
            auto& st = mruSeen_[m.id];
            if (m.fresh) { st.last = time; st.r = m.roll; st.p = m.pitch; st.h = m.heave; st.have = true; }
        }
        for (const auto& m : fr.mru) {
            const auto& st = mruSeen_[m.id];
            if (st.have && time - st.last <= ec_.mruTimeoutS) v.push_back({m.id, st.r, st.p, st.h, v.size()});
        }
        std::vector<char> bad(v.size(), 0);
        s_.mruDisagree = false;
        const double thA = deg2rad(ec_.mruDisagreeDeg), thH = ec_.mruDisagreeHeaveM;
        if (v.size() >= 3) {                                   // голосование: прибор, далёкий от медианы, исключается
            std::vector<double> r, p, h;
            for (auto& x : v) { r.push_back(x.r); p.push_back(x.p); h.push_back(x.h); }
            const double mr = median(r), mp = median(p), mh = median(h);
            for (size_t i = 0; i < v.size(); ++i)
                if (std::fabs(v[i].r - mr) > thA || std::fabs(v[i].p - mp) > thA || std::fabs(v[i].h - mh) > thH) bad[i] = 1;
            for (char b : bad) s_.mruDisagree = s_.mruDisagree || b;
        } else if (v.size() == 2) {                            // два прибора: расхождение видно, виноватого не определить
            s_.mruDisagree = std::fabs(v[0].r - v[1].r) > thA || std::fabs(v[0].p - v[1].p) > thA || std::fabs(v[0].h - v[1].h) > thH;
        }
        double r = 0, p = 0, h = 0; int n = 0;
        for (size_t i = 0; i < v.size(); ++i) if (!bad[i]) { r += v[i].r; p += v[i].p; h += v[i].h; ++n; }
        s_.mruCount = n; s_.mruValid = n > 0;
        if (n > 0) { s_.roll = r / n; s_.pitch = p / n; s_.heave = h / n; mruLostS_ = 0; }
        else { s_.roll = s_.pitch = s_.heave = 0; mruLostS_ += dt; }
        s_.sensors.erase(std::remove_if(s_.sensors.begin(), s_.sensors.end(), [](const SensorStatus& x) { return x.id.rfind("M:", 0) == 0; }), s_.sensors.end());
        for (const auto& m : fr.mru) {
            auto& st = mruSeen_[m.id];
            bool isBad = false;
            for (size_t i = 0; i < v.size(); ++i) if (v[i].id == m.id) isBad = bad[i];
            if (isBad && !st.wasBad) ++st.rejects;
            st.wasBad = isBad;
            SensorStatus ss; ss.id = "M:" + m.id; ss.ageS = st.have ? time - st.last : 1e9;
            ss.valid = st.have && ss.ageS <= ec_.mruTimeoutS && !isBad; ss.rejected = st.rejects;
            s_.sensors.push_back(ss);
        }
        bool anyFresh = false;
        for (const auto& m : fr.mru) anyFresh = anyFresh || m.fresh;
        if (n > 0 && anyFresh && s_.hdgMeasured) waves_.push(time, psi, s_.roll, s_.pitch, -s_.heave);   // оценщик волн считает heave вверх
        if (mruLostS_ > 20.0) waves_.reset();                  // долго нет данных: накопленное окно уже не отражает море
        s_.wave = waves_.state();
        if (mruLostS_ > 20.0) { s_.wave.valid = false; s_.wave.dirValid = false; s_.wave.ffGain = 0; }
    }

    void updateHeading(double time, const SensorFrame& fr) {
        for (const auto& h : fr.hdg) {
            auto& st = hdgSeen_[h.id];
            if (h.fresh) { st.last = time; st.value = h.psi; st.have = true; }
        }
    }
    void correctHeading(const SensorFrame& fr) {
        std::vector<double> vals; std::vector<std::string> ids;
        for (const auto& h : fr.hdg) {
            const auto& st = hdgSeen_[h.id];
            if (st.have && fr.time - st.last <= ec_.hdgTimeoutS) { vals.push_back(st.value); ids.push_back(h.id); }
        }
        s_.sensors.erase(std::remove_if(s_.sensors.begin(), s_.sensors.end(), [](const SensorStatus& x) { return x.id.rfind("H:", 0) == 0; }), s_.sensors.end());
        for (const auto& h : fr.hdg) {
            const auto& st = hdgSeen_[h.id];
            SensorStatus ss; ss.id = "H:" + h.id; ss.ageS = st.have ? fr.time - st.last : 1e9; ss.valid = st.have && ss.ageS <= ec_.hdgTimeoutS;
            s_.sensors.push_back(ss);
        }
        const bool wasMeasured = s_.hdgMeasured;
        s_.hdgMeasured = !vals.empty();
        if (s_.hdgMeasured) lastHdgOkT_ = fr.time;
        // начало счисления курса: мгновенная скорость поворота по фильтру шумит (качка, волны), и прогноз от неё за десятки секунд
        // уводит курс на градусы; берётся сглаженная скорость поворота за последние ~10 с
        if (wasMeasured && !s_.hdgMeasured && kpsi_.inited) kpsi_.v = rSmooth_;
        s_.hdgLostS = kpsi_.inited && !s_.hdgMeasured ? std::max(0.0, fr.time - lastHdgOkT_) : 0.0;
        s_.hdgDeadReckon = kpsi_.inited && !s_.hdgMeasured && s_.hdgLostS <= ec_.hdgDeadReckonS;
        s_.hdgValid = s_.hdgMeasured || s_.hdgDeadReckon;
        s_.gyroDisagree = false;
        if (vals.empty()) return;
        // медиана по окружности относительно первого значения
        std::vector<double> rel;
        for (double v : vals) rel.push_back(wrapPi(v - vals[0]));
        const double psiMeas = wrapPi(vals[0] + median(rel));
        double spread = 0;
        for (double r : rel) spread = std::max(spread, std::fabs(r - median(rel)));
        s_.gyroDisagree = rad2deg(spread) > ec_.gyroDisagreeDeg && vals.size() > 1;
        // фильтр обновляем при свежих данных
        bool anyFresh = false;
        for (const auto& h : fr.hdg) anyFresh = anyFresh || h.fresh;
        lastPsi_ = psiMeas;
        if (!kpsi_.inited) { kpsi_.init(psiMeas, deg2rad(0.3) * deg2rad(0.3), std::pow(5e-3 * in_.m66, 2)); kpsi_.v = 0; return; }
        if (anyFresh) {
            const double R = deg2rad(0.15) * deg2rad(0.15);
            const double y = wrapPi(psiMeas - kpsi_.p);
            const double S = kpsi_.P(0, 0) + R;
            double K[3] = {kpsi_.P(0, 0) / S, kpsi_.P(1, 0) / S, kpsi_.P(2, 0) / S};
            kpsi_.p = wrapPi(kpsi_.p + K[0] * y); kpsi_.v += K[1] * y; kpsi_.b += K[2] * y;
            const double dtH = clampd(fr.time - lastHdgFuseT_, 0.01, 1.0); lastHdgFuseT_ = fr.time;
            rSmooth_ += (kpsi_.v - rSmooth_) * std::min(1.0, dtH / 10.0);
            Mx Pn = kpsi_.P;
            for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) Pn(i, j) = kpsi_.P(i, j) - K[i] * kpsi_.P(0, j);
            kpsi_.P = Pn;
        }
    }

    struct PosItem { std::string id; double n, e, sigma; bool monitor; double wrel = 1.0; bool mobile = false; };   // wrel — относительный вес
    PosRefCfg posCfg(const std::string& id) const { auto it = ec_.posRef.find(id); return it == ec_.posRef.end() ? PosRefCfg{} : it->second; }

    void updatePosition(double time, const SensorFrame& fr, double psi) {
        const double phi = s_.roll, th = s_.pitch;
        std::vector<PosItem> fresh, latest;
        for (const auto& p : fr.pos) {
            auto& st = posSeen_[p.id];
            const PosRefCfg pc = posCfg(p.id);
            if (p.fresh) {
                const V3 off = rotateBodyToNed(phi, th, psi, p.offset);   // смещение антенны с учётом крена и дифферента
                // тест заморозки: точно такое же значение приходит снова и снова — датчик завис, измерения не используются
                if (ec_.freezeCount > 0) {
                    if (st.have && p.n == st.rawN && p.e == st.rawE) ++st.ident; else st.ident = 0;
                    st.rawN = p.n; st.rawE = p.e;
                    st.frozen = st.ident + 1 >= ec_.freezeCount;
                } else { st.frozen = false; st.ident = 0; }
                if (pc.autoRef && !st.autoDone) {   // автоначало: смещение системы = её первое измерение минус текущее решение (если решения ещё нет — начало берётся от самой системы)
                    st.autoDone = true;
                    if (kn_.inited) { PosRefCfg& w = ec_.posRef[p.id]; w.biasN = (p.n - off.x) - kn_.p; w.biasE = (p.e - off.y) - ke_.p; }
                }
                const PosRefCfg pc2 = posCfg(p.id);
                st.n = p.n - off.x - pc2.biasN; st.e = p.e - off.y - pc2.biasE; st.sigma = p.sigma; st.last = time; st.have = true;
                if (!st.ok) st.ok = true;
                updateRepeatability(st, p.sigma);     // тест дисперсии: повторяемость по последним отсчётам
                double wv = 1.0;                      // вес по повторяемости или ручной (weightMode)
                if (ec_.weightMode == 2 && st.sdRep > 0) { const double q = p.sigma / std::max(p.sigma, st.sdRep); wv = std::max(0.05, q * q); }
                else if (ec_.weightMode == 1) wv = clampd(pc.manualWeight / 5.0, 0.2, 2.0);
                st.wrel = ((ec_.gnssWeight == 1 && p.type == PosType::Gnss) ? ec_.reducedGnssWeightRatio : 1.0) * wv;
                if (pc.mode != 2 && !st.frozen) fresh.push_back({p.id, st.n, st.e, st.sigma, pc.mode == 1 || pc.mode == 3, st.wrel, pc.mode == 3});
            }
        }
        s_.sensors.erase(std::remove_if(s_.sensors.begin(), s_.sensors.end(), [](const SensorStatus& x) { return x.id.rfind("P:", 0) == 0; }), s_.sensors.end());
        for (const auto& p : fr.pos) {
            const auto& st = posSeen_[p.id];
            const PosRefCfg pc = posCfg(p.id);
            SensorStatus ss; ss.id = "P:" + p.id; ss.ageS = st.have ? time - st.last : 1e9;
            ss.valid = st.have && ss.ageS <= ec_.posTimeoutS && !st.frozen; ss.rejected = st.rejects;
            ss.state = pc.mode == 2 ? "OFF" : st.frozen ? "FROZEN" : !ss.valid ? "STALE" : st.state;
            s_.sensors.push_back(ss);
            if (ss.valid && pc.mode != 2) latest.push_back({p.id, st.n, st.e, st.sigma, pc.mode == 1 || pc.mode == 3, st.wrel, pc.mode == 3});
        }
        { std::vector<Calibrator::Item> ci; for (auto& l : latest) ci.push_back({l.id, l.n, l.e}); cal_.tick(time, psi, ci); }   // калибровка (если включена)
        std::vector<PosItem> used0;                                 // системы, входящие в решение
        for (auto& l : latest) if (!l.monitor) used0.push_back(l);
        s_.posValid = kn_.inited;      // положение есть, пока работает фильтр; потеря опорных систем отслеживается ниже
        // инициализация
        std::vector<PosItem> initSet; for (auto& f : fresh) if (!f.monitor) initSet.push_back(f);
        if (!kn_.inited && !initSet.empty() && kpsi_.inited) {   // начальное положение — только когда известен курс (поправка на смещение антенны)
            double sn = 0, se = 0, sw = 0, swT = 0;
            for (auto& f : initSet) { const double w = f.wrel / (f.sigma * f.sigma); sn += w * f.n; se += w * f.e; sw += w; swT += 1 / (f.sigma * f.sigma); }
            const double bv = std::pow(2e-3 * mEff_, 2); kn_.init(sn / sw, 1.0 / swT, bv); ke_.init(se / sw, 1.0 / swT, bv);
            s_.posValid = true; lastAccT_ = time;
        }
        // расхождение источников
        s_.gnssDisagree = false;
        double maxd = 0;
        for (size_t i = 0; i < used0.size(); ++i) for (size_t j = i + 1; j < used0.size(); ++j)
            maxd = std::max(maxd, std::hypot(used0[i].n - used0[j].n, used0[i].e - used0[j].e));
        s_.gnssDisagree = maxd > ec_.gnssDisagreeM;
        s_.softCorrecting = false;
        if (!fresh.empty() && kn_.inited) fusePosition(time, fresh, used0);
        // потеря опорных систем: положение не уточнялось дольше posTimeoutS — держимся по модели (счисление)
        s_.refLostS = kn_.inited ? std::max(0.0, time - lastAccT_) : 0.0;
        s_.refLost = kn_.inited && s_.refLostS > ec_.posTimeoutS;
        const bool wasDrop = s_.posDropout;
        s_.posDropout = kn_.inited && s_.refLostS >= ec_.posDropoutS;
        if (wasDrop && !s_.refLost) recoverT_ = ec_.biasRecoverS;   // измерения вернулись после Position dropout
        if (kn_.inited && ec_.deadReckonMaxS > 0 && s_.refLostS > ec_.deadReckonMaxS) s_.posValid = false;
        { const double q = s_.refLost ? s_.refLostS / 300.0 : 0.0; s_.drDriftM = ec_.drDrift5MinM * q * q; }
        // события для оператора: сохраняются, пока последнее измерение системы не прошло проверку
        s_.posAlerts.clear();
        for (const auto& p : fr.pos) {
            const auto& st = posSeen_[p.id];
            if (posCfg(p.id).mode == 2 || !st.have || time - st.last > ec_.posTimeoutS) continue;
            if (st.frozen) s_.posAlerts.push_back({"POS_FROZEN", "Тест заморозки: " + p.id + " — одинаковые измерения " + std::to_string(st.ident + 1) + " раз подряд, система не используется"});
            else if (st.state == "PRED") s_.posAlerts.push_back({"POS_PRED", "Ошибка предсказания положения: " + p.id + " — измерение отличается от прогноза больше допуска, отклонено"});
            else if (st.state == "MEDIAN") s_.posAlerts.push_back({"POS_MEDIAN", "Медианный тест: " + p.id + " отклонена — система расходится с остальными"});
            else if (st.state == "MEDIANW") s_.posAlerts.push_back({"POS_MEDIAN_WARN", "Медианный тест: расхождение " + p.id + " — ещё не отброшена, но расходится с остальными"});
            else if (st.state == "JUMP") s_.posAlerts.push_back({"POS_JUMP", "Скачок положения: " + p.id + " — принят после серии повторов, возможен сброс"});
        }
        // тест дисперсии: разброс измерений заметно больше заявленного СКО
        if (ec_.varTest > 0) {
            for (const auto& p : fr.pos) {
                const auto& st = posSeen_[p.id]; const PosRefCfg pc = posCfg(p.id);
                if (pc.mode == 2 || pc.mode == 3 || !st.have || time - st.last > ec_.posTimeoutS || !st.varBad) continue;
                char b[160]; std::snprintf(b, sizeof b, "Тест дисперсии: %s — разброс измерений %.2f м при заявленном СКО %.2f м (предел ×%.1f)", p.id.c_str(), st.sdRep, st.sigma, ec_.varFactor);
                s_.posAlerts.push_back({"POS_VARIANCE", b});
            }
        }
        // тест расхождения: пары систем, входящих в решение, расходятся больше предела (в отличие от медианы — без отбраковки, только предупреждение)
        if (ec_.divergeM > 0 && used0.size() >= 2) {
            for (size_t i = 0; i < used0.size(); ++i) for (size_t j = i + 1; j < used0.size(); ++j) {
                const double d = std::hypot(used0[i].n - used0[j].n, used0[i].e - used0[j].e);
                if (d > ec_.divergeM) {
                    char b[200]; std::snprintf(b, sizeof b, "Тест расхождения: %s и %s — системы расходятся на %.1f м (предел %.1f м)", used0[i].id.c_str(), used0[j].id.c_str(), d, ec_.divergeM);
                    s_.posAlerts.push_back({"POS_DIVERGE", b});
                }
            }
        }
        // диаграмма качества: отклонения последних измерений от решения
        for (const auto& p : fr.pos) {
            auto& st = posSeen_[p.id];
            if (p.fresh && st.have && kn_.inited) { st.scat.push_back({static_cast<float>(st.n - kn_.p), static_cast<float>(st.e - ke_.p)}); while (st.scat.size() > 120) st.scat.pop_front(); }
        }
        s_.scatter.clear();
        for (const auto& p : fr.pos) {
            const auto& st = posSeen_[p.id];
            EstimatorState::RefScatter rs; rs.id = p.id; rs.sdRep = st.sdRep; rs.sigma = st.sigma; rs.varBad = st.varBad; rs.mode = posCfg(p.id).mode; rs.state = st.state;
            for (const auto& q : st.scat) rs.pts.push_back(q);
            s_.scatter.push_back(std::move(rs));
        }
    }

    // Повторяемость измерений системы: СКО вторых разностей последних отсчётов, делённое на √6.
    template <class S> void updateRepeatability(S& st, double sigma) {
        st.hist.push_back({st.n, st.e});
        while (static_cast<int>(st.hist.size()) > std::max(5, ec_.varWindow) + 2) st.hist.pop_front();
        const int n = static_cast<int>(st.hist.size());
        if (n < std::max(5, ec_.varMinSamples) + 2) { st.sdRep = 0; st.varBad = false; return; }
        double sn = 0, se = 0; int k = 0;
        for (int i = 2; i < n; ++i) {
            const double dn = st.hist[i][0] - 2 * st.hist[i - 1][0] + st.hist[i - 2][0], de = st.hist[i][1] - 2 * st.hist[i - 1][1] + st.hist[i - 2][1];
            sn += dn * dn; se += de * de; ++k;
        }
        st.sdRep = std::sqrt((sn + se) / (2.0 * k) / 6.0);
        const double lim = ec_.varFactor * sigma;
        if (!st.varBad) { if (st.sdRep > lim) st.varBad = true; }
        else if (st.sdRep < 0.7 * lim) st.varBad = false;
    }

    // Проверки по предсказанию и по медиане, веса, мягкая коррекция. Порядок как в K-Pos: сначала предсказание, затем медиана.
    void fusePosition(double time, const std::vector<PosItem>& fresh, const std::vector<PosItem>& used0) {
        auto sigK = [&](int a) { return a == 0 ? ec_.acceptSigmaNarrow : a == 2 ? ec_.acceptSigmaWide : ec_.gateSigma; };
        // медиана по системам, входящим в решение (нужны три и более)
        bool medOn = ec_.medianMode > 0 && used0.size() >= 3;
        double medN = 0, medE = 0, sigMed = 0;
        if (medOn) {
            std::vector<double> vn, ve, sg;
            for (auto& u : used0) { vn.push_back(u.n); ve.push_back(u.e); sg.push_back(u.sigma); }
            medN = median(vn); medE = median(ve); sigMed = median(sg);
        }
        // предел медианы для системы — по её собственному СКО и СКО самой медианы (раньше брался наименьший предел среди всех систем:
        // точная система, например лазерная, делала предел меньше шума GNSS, и те отклонялись постоянно)
        auto medLimOf = [&](const std::string& id, double sigma) { return ec_.medianFrac * sigK(posCfg(id).accept) * 1.4142 * std::sqrt(kn_.P(0, 0) + sigma * sigma + sigMed * sigMed); };
        double sn = 0, se = 0, sw = 0, swTrue = 0; std::string used;
        bool jumpAccepted = false;
        for (const auto& f : fresh) {
            auto& st = posSeen_[f.id];
            const double yn = kn_.innovation(f.n, f.sigma * f.sigma), ye = ke_.innovation(f.e, f.sigma * f.sigma);
            bool jump = false;
            // медианная проверка первой: при трёх и более системах она точнее показывает, какая система уклонилась
            bool medBad = false;
            if (medOn && !f.monitor) {            // гистерезис и выдержка: признак ставится сразу, снимается при расхождении ниже medianHystFrac·предел в течение medianHoldS
                const double dm = std::hypot(f.n - medN, f.e - medE), medLim = medLimOf(f.id, f.sigma);
                if (!st.medBad) { if (dm > medLim) { st.medBad = true; st.medOkSince = -1; } }
                else if (dm < ec_.medianHystFrac * medLim) { if (st.medOkSince < 0) st.medOkSince = time; if (time - st.medOkSince >= ec_.medianHoldS) st.medBad = false; }
                else st.medOkSince = -1;
                medBad = st.medBad;
            } else { st.medBad = false; st.medOkSince = -1; }
            if (medBad && ec_.medianMode == 2) { st.state = "MEDIAN"; continue; }
            if (std::hypot(yn, ye) > sigK(posCfg(f.id).accept) * 1.4142) {
                if (++st.rejects >= ec_.resetAfterRejects) { st.rejects = 0; jump = true; }   // потеряли согласие: примем скачок
                else { st.state = "PRED"; continue; }
            } else st.rejects = 0;
            st.state = medBad ? "MEDIANW" : jump ? "JUMP" : "OK";
            if (f.monitor) { if (st.state == "OK") st.state = f.mobile ? "MOBILE" : "MONITOR"; continue; }
            if (jump) jumpAccepted = true;
            const double w = f.wrel / (f.sigma * f.sigma);
            sn += w * f.n; se += w * f.e; sw += w; swTrue += 1 / (f.sigma * f.sigma); used += (used.empty() ? "" : "+") + f.id;
        }
        if (sw <= 0) return;
        const double R = 1.0 / swTrue;      // шум решения — по настоящим СКО; вес влияет только на распределение между системами
        double zn = sn / sw, ze = se / sw;
        // мягкая коррекция: большое расхождение отрабатывается не быстрее softRateMS, чтобы скачок датчика не вызвал резкого упора
        const double dtm = clampd(time - lastFuseT_, 0.05, 2.0);
        if (ec_.softRateMS > 0) {
            const double y = std::hypot(zn - kn_.p, ze - ke_.p), step = ec_.softRateMS * dtm;
            if (y > ec_.softStartM && y > step) { const double k = step / y; zn = kn_.p + (zn - kn_.p) * k; ze = ke_.p + (ze - ke_.p) * k; s_.softCorrecting = true; }
        }
        (void)jumpAccepted;
        lastFuseT_ = time; lastAccT_ = time;
        kn_.update(zn, R, mEff_); ke_.update(ze, R, mEff_);
        s_.posSource = used; s_.posSigma = std::sqrt(R);
    }

    void updateWind(double dt, double time, const SensorFrame& fr) {
        std::vector<double> sp, cx, cy;
        for (const auto& w : fr.wind) {
            auto& st = windSeen_[w.id];
            if (w.fresh) { st.last = time; st.speed = w.speed; st.from = w.fromRel; st.have = true; }
        }
        for (const auto& w : fr.wind) {
            const auto& st = windSeen_[w.id];
            if (st.have && time - st.last <= ec_.windTimeoutS) { sp.push_back(st.speed); cx.push_back(std::cos(st.from)); cy.push_back(std::sin(st.from)); }
        }
        s_.sensors.erase(std::remove_if(s_.sensors.begin(), s_.sensors.end(), [](const SensorStatus& x) { return x.id.rfind("W:", 0) == 0; }), s_.sensors.end());
        for (const auto& w : fr.wind) {
            const auto& st = windSeen_[w.id];
            SensorStatus ss; ss.id = "W:" + w.id; ss.ageS = st.have ? time - st.last : 1e9; ss.valid = st.have && ss.ageS <= ec_.windTimeoutS;
            s_.sensors.push_back(ss);
        }
        s_.windValid = !sp.empty();
        if (sp.empty()) return;
        const double speed = median(sp), c = median(cx), s = median(cy);
        const double from = std::atan2(s, c);
        if (!windInit_) { fs_ = speed; fcx_ = std::cos(from); fcy_ = std::sin(from); windInit_ = true; }
        const double a = dt / (ec_.windFilterS + dt);
        fs_ += a * (speed - fs_);
        fcx_ += a * (std::cos(from) - fcx_); fcy_ += a * (std::sin(from) - fcy_);
        s_.windSpeed = fs_; s_.windFromRel = std::atan2(fcy_, fcx_);
    }

    void publish(double psi) {
        s_.psi = kpsi_.inited ? kpsi_.p : psi;
        s_.r = kpsi_.v; s_.biasMoment = kpsi_.b;
        s_.posUncertM = kn_.inited ? std::sqrt(std::max(0.0, kn_.P(0, 0)) + std::max(0.0, ke_.P(0, 0))) : 0.0;
        s_.n = kn_.p; s_.e = ke_.p; s_.vn = kn_.v; s_.ve = ke_.v; s_.biasN = kn_.b; s_.biasE = ke_.b;
        const double cp = std::cos(s_.psi), sp = std::sin(s_.psi);
        s_.u = s_.vn * cp + s_.ve * sp; s_.v = -s_.vn * sp + s_.ve * cp;
    }

    struct PosSeen { std::deque<std::array<double, 2>> hist; std::deque<std::pair<float, float>> scat; double sdRep = 0; bool varBad = false; double n = 0, e = 0, sigma = 1, wrel = 1, last = -1e9; double rawN = 0, rawE = 0; int ident = 0; bool frozen = false, autoDone = false; bool have = false, ok = false; int rejects = 0; std::string state = "OK"; bool medBad = false; double medOkSince = -1; };
    struct HdgSeen { double value = 0, last = -1e9; bool have = false; };
    struct WindSeen { double speed = 0, from = 0, last = -1e9; bool have = false; };
    struct MruSeen { double r = 0, p = 0, h = 0, last = -1e9; bool have = false, wasBad = false; int rejects = 0; };

    EstimatorCfg ec_;
    Inertia in_;
    double mEff_ = 1, dEff_ = 0;
    Kf3 kn_, ke_, kpsi_;
    double recoverT_ = 0;   // осталось секунд ускоренного схождения оценки силы после потери положения
    EstimatorState s_;
    std::map<std::string, PosSeen> posSeen_;
    std::map<std::string, HdgSeen> hdgSeen_;
    std::map<std::string, WindSeen> windSeen_;
    std::map<std::string, MruSeen> mruSeen_;
    WaveEstimator waves_;
    double mruLostS_ = 1e9, lastFuseT_ = -1e9, lastAccT_ = -1e9, lastHdgOkT_ = -1e9, lastHdgFuseT_ = -1e9, rSmooth_ = 0;
    double lastPsi_ = 0, fs_ = 0, fcx_ = 1, fcy_ = 0;
    bool windInit_ = false;
};

}  // namespace dp
