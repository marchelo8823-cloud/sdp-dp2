// Блок управления: законы управления (ПИД, LQR, MPC), задатчик траектории и режимы работы.
#pragma once
#include "dp/estimator.hpp"
#include "dp/thrust.hpp"
#include "dp/predict.hpp"

#include <array>
#include <memory>

namespace dp {

enum class DpMode { Standby, Manual, Joystick, AutoHeading, AutoPosition, Track, FollowTarget, Autopilot };
enum class LawType { Pid, Lqr, Mpc };

struct ControlCfg {
    LawType law = LawType::Pid;
    // ПИД: желаемая собственная частота и затухание контура
    double wnPos = 0.10, zetaPos = 0.9;
    double wnHdg = 0.12, zetaHdg = 0.9;
    // LQR и MPC: допустимые отклонения (правило Брайсона)
    double posTolM = 2.0, velTolMS = 0.3;
    double hdgTolDeg = 4.0, rotTolDegS = 0.5;
    double effortFrac = 1.0;          // доля предельного усилия, считающаяся «номинальной»
    int mpcHorizon = 20;
    double mpcStepS = 1.0;
    double mpcRateWeight = 0.5;       // вес изменения управления (относительно веса самого управления)
    // общее
    double forceFilterS = 1.0;
    double biasFilterS = 8.0;         // сглаживание оценки неизвестной силы: она меняется медленно, а оценка дрожит
    double biasLimitFrac = 0.5;       // предел компенсации неизвестной силы от возможностей комплекса
    // усиление (как High/Low gain в промышленных СДП): «высокое» ускоряет контуры позиции и курса в gainHighK раз
    bool highGain = false; double gainHighK = 1.3;
    // минимальная мощность (Minpower): малые отклонения не отрабатываются (мягкая зона нечувствительности), меньше работа движителей
    bool minPower = false;
    double minPosDeadM = 0.5, minHdgDeadDeg = 0.5;
    // чувствительность: мягкая зона нечувствительности (малые отклонения не отрабатываются), общая для всех режимов пропульсии
    double posDeadM = 0.0, hdgDeadDeg = 0.0;
    // зона удержания (Hold Area, как в NavDP 4000): внутри «радиуса остановки» (доля радиуса) регулятор позиции не работает, судну разрешён дрейф;
    // у предупредительной границы (доля радиуса) — тревога; за радиусом — аварийная тревога. 0 — выключена
    double holdAreaM = 0.0, holdWarnPct = 80.0, holdStopPct = 30.0;
    // предсказание внешних сил (ветер + волны): упреждение идёт по среднему и тренду, а не по мгновенному значению
    PredictCfg predict;
    bool windFeedForward = true;
    bool waveFeedForward = true;      // упреждение по дрейфовой силе волн (оценка по МИО)
    bool refCapTotal = false;         // ограничение оператора на скорость: полная скорость опорной точки (с упреждением по цели) не больше refMaxSpeed
    double refMaxSpeed = 0.5;         // м/с, скорость перемещения опорной точки при смене уставки
    double refAccel = 0.02;           // м/с²
    double refMaxRotDegS = 1.0;       // °/с
    double refRotAccelDegS2 = 0.05;
    // возврат в заданную точку после потери положения: если судно ушло дальше returnStartM, опорная точка начинается с того места,
    // где судно оказалось, и идёт к заданной точке не быстрее returnSpeedMS (0 — без ограничения, только refMaxSpeed)
    double returnSpeedMS = 0.2, returnStartM = 2.0;
    // торможение дрейфа в начале возврата: опорная точка стартует со скоростью судна и гасит её с этим ускорением, м/с²
    // (не догоняет судно полным упором; решение владельца 16.11.2026)
    double returnBrakeAccel = 0.05;
    double returnSettleM = 1.0;       // выдержка в начале возврата длится, пока погрешность фильтра больше этого, м (не дольше 30 с сверх)
    double returnErrMaxM = 1.0;       // при возврате ошибка положения для закона не больше этого, м
    double returnVelGain = 1.5;       // притормаживание сверх предела возврата: ускорение = это × превышение скорости к точке, 1/с (не больше returnBrakeAccel)
    // автопилот
    double apSpeedKp = 0.15, apSpeedKi = 0.01;   // доли массы на (м/с)
};

// ---------------------------------------------------------------- законы по одной степени свободы
struct AxisParams {
    double m = 1, d = 0;          // эффективная масса и линейное демпфирование
    double wn = 0.1, zeta = 0.9;  // для ПИД
    double xtol = 1, vtol = 0.1;  // для LQR/MPC
    double ucap = 1;              // номинальное усилие
    int horizon = 20; double step = 1.0; double rateW = 0.5;
};

class AxisLaw {
public:
    virtual ~AxisLaw() = default;
    // e, ev — отклонение и его скорость; umax — предельное усилие. Возвращает управляющую силу (Н или Н·м).
    virtual double compute(double e, double ev, double umax) = 0;
    virtual void reset() {}
};

class PidLaw : public AxisLaw {
public:
    explicit PidLaw(const AxisParams& p) {
        kp_ = p.m * p.wn * p.wn;
        const double kdIdeal = 2.0 * p.zeta * p.wn * p.m;
        kd_ = std::max(kdIdeal - p.d, 0.25 * kdIdeal);
    }
    double compute(double e, double ev, double umax) override {
        return clampd(-kp_ * e - kd_ * ev, -umax, umax);
    }
    double kp() const { return kp_; }
    double kd() const { return kd_; }
private:
    double kp_ = 0, kd_ = 0;
};

// LQR для модели «масса с демпфированием»: x' = v, v' = (-d v + u)/m. Коэффициенты по уравнению Риккати.
class LqrLaw : public AxisLaw {
public:
    explicit LqrLaw(const AxisParams& p) {
        const double dt = 0.1, a = p.d / p.m, b = 1.0 / p.m;
        const double q1 = 1.0 / (p.xtol * p.xtol), q2 = 1.0 / (p.vtol * p.vtol), r = 1.0 / (p.ucap * p.ucap);
        const double A[2][2] = {{1, dt}, {0, 1 - a * dt}};
        const double B[2] = {0, b * dt};
        double P[2][2] = {{q1, 0}, {0, q2}};
        for (int it = 0; it < 40000; ++it) {
            // K = (R + BᵀPB)⁻¹ BᵀPA
            double PB[2] = {P[0][1] * B[1], P[1][1] * B[1]};
            const double s = r + B[1] * PB[1];
            double BtPA[2];
            for (int j = 0; j < 2; ++j) BtPA[j] = PB[0] * A[0][j] + PB[1] * A[1][j];
            double K[2] = {BtPA[0] / s, BtPA[1] / s};
            double N[2][2];
            for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j) {
                double AtPA = 0;
                for (int k = 0; k < 2; ++k) for (int l = 0; l < 2; ++l) AtPA += A[k][i] * P[k][l] * A[l][j];
                N[i][j] = AtPA - BtPA[i] * K[j];
            }
            N[0][0] += q1; N[1][1] += q2;
            double diff = 0;
            for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j) { diff += std::fabs(N[i][j] - P[i][j]); P[i][j] = N[i][j]; }
            k1_ = K[0]; k2_ = K[1];
            if (diff < 1e-12 * (std::fabs(P[0][0]) + 1)) break;
        }
    }
    double compute(double e, double ev, double umax) override { return clampd(-k1_ * e - k2_ * ev, -umax, umax); }
    double k1() const { return k1_; }
    double k2() const { return k2_; }
private:
    double k1_ = 0, k2_ = 0;
};

// MPC с ограничением на управление: горизонт N, решается методом проекции градиента.
class MpcLaw : public AxisLaw {
public:
    explicit MpcLaw(const AxisParams& p) : N_(std::max(3, p.horizon)) {
        const double dt = p.step, a = p.d / p.m, b = 1.0 / p.m;
        double A[2][2], B[2];
        if (a > 1e-9) {
            const double ex = std::exp(-a * dt), g = (1 - ex) / a;
            A[0][0] = 1; A[0][1] = g; A[1][0] = 0; A[1][1] = ex;
            B[0] = b * (dt - g) / a; B[1] = b * g;
        } else {
            A[0][0] = 1; A[0][1] = dt; A[1][0] = 0; A[1][1] = 1;
            B[0] = b * dt * dt / 2; B[1] = b * dt;
        }
        const double q1 = 1.0 / (p.xtol * p.xtol), q2 = 1.0 / (p.vtol * p.vtol), r = 1.0 / (p.ucap * p.ucap);
        // Φ[k][j] — вклад управления j в состояние на шаге k+1
        std::vector<std::array<double, 2>> Phi(static_cast<size_t>(N_ * N_), std::array<double, 2>{0.0, 0.0});
        std::vector<std::array<double, 4>> Ak(static_cast<size_t>(N_) + 1);
        Ak[0] = {1, 0, 0, 1};
        for (int k = 1; k <= N_; ++k) {
            const auto& m = Ak[k - 1];
            Ak[k] = {A[0][0] * m[0] + A[0][1] * m[2], A[0][0] * m[1] + A[0][1] * m[3],
                     A[1][0] * m[0] + A[1][1] * m[2], A[1][0] * m[1] + A[1][1] * m[3]};
        }
        for (int k = 0; k < N_; ++k)
            for (int j = 0; j <= k; ++j) {
                const auto& m = Ak[k - j];
                Phi[static_cast<size_t>(k * N_ + j)] = {m[0] * B[0] + m[1] * B[1], m[2] * B[0] + m[3] * B[1]};
            }
        H_.assign(static_cast<size_t>(N_ * N_), 0.0);
        G_.assign(static_cast<size_t>(N_) * 2, 0.0);
        for (int j = 0; j < N_; ++j) {
            for (int l = 0; l < N_; ++l) {
                double s = 0;
                for (int k = std::max(j, l); k < N_; ++k) {
                    const auto &pj = Phi[static_cast<size_t>(k * N_ + j)], &pl = Phi[static_cast<size_t>(k * N_ + l)];
                    s += q1 * pj[0] * pl[0] + q2 * pj[1] * pl[1];
                }
                H_[static_cast<size_t>(j * N_ + l)] = s;
            }
            H_[static_cast<size_t>(j * N_ + j)] += r;
            for (int c = 0; c < 2; ++c) {
                double s = 0;
                for (int k = j; k < N_; ++k) {
                    const auto& pj = Phi[static_cast<size_t>(k * N_ + j)];
                    const auto& m = Ak[k + 1];
                    s += q1 * pj[0] * m[c] + q2 * pj[1] * m[2 + c];
                }
                G_[static_cast<size_t>(j) * 2 + c] = s;
            }
        }
        // штраф на изменение управления
        const double rd = p.rateW * r;
        for (int j = 0; j < N_; ++j) {
            H_[static_cast<size_t>(j * N_ + j)] += (j == 0 ? 1 : 2) * rd;
            if (j + 1 < N_) { H_[static_cast<size_t>(j * N_ + j + 1)] -= rd; H_[static_cast<size_t>((j + 1) * N_ + j)] -= rd; }
        }
        double L = 0;
        for (int j = 0; j < N_; ++j) { double s = 0; for (int l = 0; l < N_; ++l) s += std::fabs(H_[static_cast<size_t>(j * N_ + l)]); L = std::max(L, s); }
        step_ = 1.0 / L;
        u_.assign(static_cast<size_t>(N_), 0.0);
    }
    double compute(double e, double ev, double umax) override {
        // сдвиг предыдущего решения
        for (int j = 0; j + 1 < N_; ++j) u_[static_cast<size_t>(j)] = u_[static_cast<size_t>(j) + 1];
        for (auto& v : u_) v = clampd(v, -umax, umax);
        std::vector<double> g(static_cast<size_t>(N_));
        for (int j = 0; j < N_; ++j) g[static_cast<size_t>(j)] = G_[static_cast<size_t>(j) * 2] * e + G_[static_cast<size_t>(j) * 2 + 1] * ev;
        // ускоренный проекционный градиент
        std::vector<double> y = u_, un = u_;
        double t = 1.0;
        for (int it = 0; it < 120; ++it) {
            std::vector<double> grad(static_cast<size_t>(N_));
            for (int j = 0; j < N_; ++j) {
                double s = g[static_cast<size_t>(j)];
                for (int l = 0; l < N_; ++l) s += H_[static_cast<size_t>(j * N_ + l)] * y[static_cast<size_t>(l)];
                grad[static_cast<size_t>(j)] = s;
            }
            std::vector<double> nx(static_cast<size_t>(N_));
            for (int j = 0; j < N_; ++j) nx[static_cast<size_t>(j)] = clampd(y[static_cast<size_t>(j)] - step_ * grad[static_cast<size_t>(j)], -umax, umax);
            const double tn = 0.5 * (1 + std::sqrt(1 + 4 * t * t));
            for (int j = 0; j < N_; ++j) y[static_cast<size_t>(j)] = nx[static_cast<size_t>(j)] + (t - 1) / tn * (nx[static_cast<size_t>(j)] - un[static_cast<size_t>(j)]);
            un = nx; t = tn;
        }
        u_ = un;
        return u_[0];
    }
    void reset() override { std::fill(u_.begin(), u_.end(), 0.0); }
private:
    int N_;
    std::vector<double> H_, G_, u_;
    double step_ = 1.0;
};

inline std::unique_ptr<AxisLaw> makeLaw(LawType t, const AxisParams& p) {
    switch (t) {
        case LawType::Lqr: return std::make_unique<LqrLaw>(p);
        case LawType::Mpc: return std::make_unique<MpcLaw>(p);
        default: return std::make_unique<PidLaw>(p);
    }
}

// ---------------------------------------------------------------- задатчик опорной точки
struct Reference {
    double n = 0, e = 0, vn = 0, ve = 0;      // положение и скорость опорной точки
    double an = 0, ae = 0;                    // ускорение опорной точки (для упреждения)
    double psi = 0, r = 0;                    // курс и скорость поворота
    bool inited = false;

    void reset(double n0, double e0, double psi0) { n = n0; e = e0; vn = ve = 0; an = ae = 0; psi = psi0; r = 0; inited = true; }
    // то же, но со скоростью: опорная точка начинает движение так, как движется судно (возврат после потери положения)
    void resetMoving(double n0, double e0, double psi0, double vn0, double ve0) { reset(n0, e0, psi0); vn = vn0; ve = ve0; }

    // Перемещение к целевой точке с ограничением скорости и ускорения; velFF — скорость цели.
    void moveTo(double dt, double tn, double te, double vtn, double vte, const ControlCfg& c) {
        const double dn = tn - n, de = te - e;
        const double dist = std::hypot(dn, de);
        double vdn = vtn, vde = vte;
        if (dist > 1e-6) {
            const double want = std::min(c.refMaxSpeed, std::sqrt(2.0 * c.refAccel * dist) * 0.8 + 0.0);
            const double k = std::min(want, 0.2 * dist) ;
            vdn += dn / dist * k; vde += de / dist * k;
        }
        if (c.refCapTotal) { const double m = std::hypot(vdn, vde); if (m > c.refMaxSpeed && m > 1e-9) { vdn *= c.refMaxSpeed / m; vde *= c.refMaxSpeed / m; } }
        const double ddn = vdn - vn, dde = vde - ve, dd = std::hypot(ddn, dde);
        const double lim = c.refAccel * dt;
        const double pvn = vn, pve = ve;
        if (dd > lim) { vn += ddn / dd * lim; ve += dde / dd * lim; } else { vn = vdn; ve = vde; }
        an = (vn - pvn) / dt; ae = (ve - pve) / dt;
        n += vn * dt; e += ve * dt;
    }
    void turnTo(double dt, double target, const ControlCfg& c) {
        const double err = wrapPi(target - psi);
        const double rmax = deg2rad(c.refMaxRotDegS), ra = deg2rad(c.refRotAccelDegS2);
        const double want = clampd(std::sqrt(2.0 * ra * std::fabs(err)) * 0.8, 0.0, rmax) * sgn(err);
        const double lim = ra * dt;
        r += clampd(want - r, -lim, lim);
        psi = wrapPi(psi + r * dt);
    }
};

struct Joystick { double x = 0, y = 0, n = 0; bool holdHeading = true; };

struct Setpoint {
    double n = 0, e = 0, psiDeg = 0;
};

struct Waypoint { double n = 0, e = 0; };

struct TrackCfg {
    std::vector<Waypoint> wp;
    double speed = 1.0;            // м/с
    bool headingAlongTrack = false;
    double acceptRadius = 8.0;
};

struct TargetInfo {
    double n = 0, e = 0, vn = 0, ve = 0;
    double offsetBearingDeg = 180, offsetDist = 50;   // положение судна относительно цели: пеленг от цели и дистанция
    bool valid = false;
};

struct ControlOutput {
    Wrench tau;
    Wrench law;                 // составляющая регулятора
    Wrench windFF;              // упреждающая компенсация ветра
    Wrench bias;                // компенсация неизвестной силы
    double errN = 0, errE = 0, errDist = 0, errHdgDeg = 0;
    double errAlong = 0, errCross = 0;
    double refN = 0, refE = 0, refPsiDeg = 0;
    bool trackDone = false;
};

class Controller {
public:
    Controller(const VesselCfg& v, ControlCfg c) : vcfg_(v), cfg_(std::move(c)), in_(completeInertia(v)) { build(); }

    void configure(const ControlCfg& c) { cfg_ = c; build(); }
    const ControlCfg& cfg() const { return cfg_; }
    // Ограничения оператора на движение: максимальная скорость (м/с) и скорость поворота (°/с); 0 — не ограничено. Только уменьшают заданное в настройках.
    void setMotionCaps(double speedMS, double rotDegS) { capSpeed_ = std::max(0.0, speedMS); capRot_ = std::max(0.0, rotDegS); }
    double capSpeed() const { return capSpeed_; }
    double capRot() const { return capRot_; }
    DpMode mode() const { return mode_; }

    void setMode(DpMode m, const EstimatorState& est) {
        if (m == mode_) return;
        mode_ = m;
        resetLaws();
        ref_.reset(est.n, est.e, est.psi);
        sp_ = holdFromCrp(est.n, est.e, rad2deg(est.psi));
        { double dn, de; bodyToNe(pvx_, pvy_, est.psi, dn, de); pref_.reset(est.n + dn, est.e + de, est.psi); }
        filt_ = Wrench{};
        wpIdx_ = 0; trackDone_ = false; trackStarted_ = false;
        speedInt_ = 0; biasInit_ = false;
        holdPsi_ = est.psi; holdValid_ = false;
        returning_ = false;
    }
    void setSetpoint(const Setpoint& s) { sp_ = s; returning_ = false; }   // новая уставка от оператора отменяет возврат
    // Возврат в заданную точку после потери положения: уставка не меняется, опорная точка (и курс) начинают с того места, где судно
    // оказалось по оценке, holdS секунд стоят на нём (оценка успокаивается после первых измерений), затем идут к уставке не быстрее speedMS.
    // Опорная точка стартует не с нулевой, а со скоростью судна по оценке: иначе дрейфующее судно уходило от неподвижной опорной
    // точки, регулятор догонял его полным упором и судно проскакивало заданную точку (найдено на Mac владельца 16.11.2026).
    void beginReturn(const EstimatorState& est, double speedMS, double holdS) {
        ref_.resetMoving(est.n, est.e, est.psi, est.vn, est.ve);
        { double dn, de; bodyToNe(pvx_, pvy_, est.psi, dn, de); pref_.resetMoving(est.n + dn, est.e + de, est.psi, est.vn, est.ve); }
        returning_ = true; returnCap_ = std::max(0.0, speedMS); returnHoldS_ = returnHold0_ = std::max(0.0, holdS); holdLastSet_ = false; holdExtS_ = 0;
    }
    bool returning() const { return returning_; }
    // расстояние от опорной точки до уставки (точки центра вращения), м — сколько осталось вернуться
    double returnLeftM() const { return std::hypot(sp_.n - pref_.n, sp_.e - pref_.e); }
    // Центр вращения (точка судна: вдоль + к носу, поперёк + к правому борту, м). Уставка положения относится к этой точке:
    // удерживается она, а судно при смене курса поворачивается вокруг неё. Мидель (0; 0) — точка управления.
    double pivotX() const { return pvx_; }
    double pivotY() const { return pvy_; }
    bool pivotOff() const { return std::fabs(pvx_) > 0.05 || std::fabs(pvy_) > 0.05; }
    static void bodyToNe(double x, double y, double psiRad, double& n, double& e) { const double c = std::cos(psiRad), s = std::sin(psiRad); n = x * c - y * s; e = x * s + y * c; }
    // уставка для точки управления (n, e) при курсе psiDeg → уставка точки центра вращения, и обратно
    Setpoint holdFromCrp(double n, double e, double psiDeg) const { double dn, de; bodyToNe(pvx_, pvy_, deg2rad(psiDeg), dn, de); return {n + dn, e + de, psiDeg}; }
    Setpoint crpFromHold(const Setpoint& h) const { double dn, de; bodyToNe(pvx_, pvy_, deg2rad(h.psiDeg), dn, de); return {h.n - dn, h.e - de, h.psiDeg}; }
    void setPivotIdle(double px, double py) { pvx_ = px; pvy_ = py; }   // режим не включён: только запомнить
    // Смена центра вращения на ходу: судно остаётся на месте, точка удержания переносится в новую точку судна.
    void setPivot(double px, double py) {
        const Setpoint crp = crpFromHold(sp_);
        pvx_ = px; pvy_ = py;
        sp_ = holdFromCrp(crp.n, crp.e, crp.psiDeg);
        double dn, de; bodyToNe(pvx_, pvy_, ref_.psi, dn, de);
        pref_ = ref_; pref_.n = ref_.n + dn; pref_.e = ref_.e + de;
    }
    const Setpoint& setpoint() const { return sp_; }
    void setTrack(const TrackCfg& t) { track_ = t; wpIdx_ = 0; trackDone_ = false; trackStarted_ = false; }
    void setTarget(const TargetInfo& t) { target_ = t; }
    void setJoystick(const Joystick& j) { joy_ = j; }
    void setAutopilot(double courseDeg, double speedMS) { apCourse_ = deg2rad(courseDeg); apSpeed_ = speedMS; }

    static double dead(double e, double d) { return e > d ? e - d : (e < -d ? e + d : 0.0); }
    ControlOutput compute(double dt, const EstimatorState& est, const Wrench& windBody, const Capability& cap) {
        ControlOutput o;
        if (mode_ == DpMode::Standby || mode_ == DpMode::Manual) {
            o.refN = est.n; o.refE = est.e; o.refPsiDeg = rad2deg(est.psi);
            filt_ = Wrench{};
            for (auto& p : pr_) p.reset();
            return o;
        }
        const double cp = std::cos(est.psi), sp = std::sin(est.psi);
        auto toBodyX = [&](double n, double e) { return n * cp + e * sp; };
        auto toBodyY = [&](double n, double e) { return -n * sp + e * cp; };

        Wrench law{};
        bool posCtl = false, hdgCtl = false;
        double hdgTarget = ref_.psi;
        double velFFx = 0, velFFy = 0;

        switch (mode_) {
            case DpMode::AutoPosition:
                if (returning_ && returnHoldS_ > 0) {   // выдержка после возврата измерений: опорная точка — там, где судно, и с его скоростью
                    // Пока оценка положения ещё поправляется по вернувшимся измерениям (скачок или плавная коррекция после счисления),
                    // выдержка продлевается: иначе опорная точка оставалась на старой, ошибочной оценке, и судно гнали к ней полным упором.
                    const double jn = est.n - (holdLastN_ + holdLastVn_ * dt), je = est.e - (holdLastE_ + holdLastVe_ * dt);
                    if (holdLastSet_ && holdExtS_ < 30.0 && (std::hypot(jn, je) > 0.3 || est.softCorrecting)) { holdExtS_ += std::max(0.0, returnHold0_ - returnHoldS_); returnHoldS_ = returnHold0_; }   // не дольше 30 с сверх выдержки
                    holdLastN_ = est.n; holdLastE_ = est.e; holdLastVn_ = est.vn; holdLastVe_ = est.ve; holdLastSet_ = true;
                    // и пока погрешность фильтра после счисления больше returnSettleM — оценке ещё нельзя верить (задача №17)
                    if (holdExtS_ < 30.0 && est.posUncertM > cfg_.returnSettleM && returnHoldS_ < dt + 1e-9) { holdExtS_ += dt; returnHoldS_ += dt; }
                    returnHoldS_ -= dt;
                    // опорная точка стоит на оценке, а её скорость гасится от скорости судна с ускорением returnBrakeAccel —
                    // дрейф тормозится сразу и мягко, а не рывком после выдержки (задача №17)
                    double hvn = ref_.vn, hve = ref_.ve; const double hv = std::hypot(hvn, hve), dv = cfg_.returnBrakeAccel * dt;
                    if (hv > dv) { hvn -= hvn / hv * dv; hve -= hve / hv * dv; } else { hvn = hve = 0; }
                    ref_.resetMoving(est.n, est.e, est.psi, hvn, hve);
                    double dn, de; bodyToNe(pvx_, pvy_, est.psi, dn, de); pref_.resetMoving(est.n + dn, est.e + de, est.psi, hvn, hve);
                }
                if (pivotOff()) {   // удерживается точка центра вращения; положение судна следует за курсом задания
                    pref_.moveTo(dt, sp_.n, sp_.e, 0, 0, eff());
                    ref_.turnTo(dt, deg2rad(sp_.psiDeg), eff());
                    const double c = std::cos(ref_.psi), s = std::sin(ref_.psi);
                    ref_.n = pref_.n - (pvx_ * c - pvy_ * s); ref_.e = pref_.e - (pvx_ * s + pvy_ * c);
                    ref_.vn = pref_.vn + ref_.r * (pvx_ * s + pvy_ * c); ref_.ve = pref_.ve - ref_.r * (pvx_ * c - pvy_ * s);
                    ref_.an = pref_.an; ref_.ae = pref_.ae;
                } else {
                    ref_.moveTo(dt, sp_.n, sp_.e, 0, 0, eff());
                    ref_.turnTo(dt, deg2rad(sp_.psiDeg), eff());
                    pref_ = ref_;
                }
                if (returning_ && returnHoldS_ <= 0 && returnLeftM() < 0.1 && std::hypot(pref_.vn, pref_.ve) < 0.02 &&
                    std::fabs(rad2deg(wrapPi(deg2rad(sp_.psiDeg) - ref_.psi))) < 0.2) returning_ = false;
                posCtl = hdgCtl = true;
                break;
            case DpMode::AutoHeading:
                ref_.turnTo(dt, deg2rad(sp_.psiDeg), eff());
                hdgCtl = true;
                break;
            case DpMode::Track: {
                posCtl = hdgCtl = true;
                advanceTrack(dt, est);
                o.trackDone = trackDone_;
                break;
            }
            case DpMode::FollowTarget: {
                posCtl = hdgCtl = true;
                if (target_.valid) {
                    const double b = deg2rad(target_.offsetBearingDeg);
                    const double tn = target_.n + target_.offsetDist * std::cos(b), te = target_.e + target_.offsetDist * std::sin(b);
                    ref_.moveTo(dt, tn, te, target_.vn, target_.ve, eff());
                    // курс — на цель
                    const double toward = std::atan2(target_.e - ref_.e, target_.n - ref_.n);
                    ref_.turnTo(dt, toward, eff());
                } else {
                    ref_.moveTo(dt, ref_.n, ref_.e, 0, 0, eff());
                }
                break;
            }
            case DpMode::Joystick: {
                const double fx = joy_.x * cap.X, fy = joy_.y * cap.Y;
                law.X = fx; law.Y = fy;
                if (std::fabs(joy_.n) > 0.02) { law.N = joy_.n * cap.N; holdValid_ = false; ref_.psi = est.psi; ref_.r = 0; }
                else if (joy_.holdHeading) {
                    if (!holdValid_) { holdPsi_ = est.psi; ref_.psi = est.psi; ref_.r = 0; holdValid_ = true; }
                    hdgCtl = true; hdgTarget = holdPsi_;
                    ref_.psi = holdPsi_; ref_.r = 0;
                }
                break;
            }
            case DpMode::Autopilot: {
                const double eu = apSpeed_ - est.u;
                speedInt_ = clampd(speedInt_ + eu * dt, -3.0, 3.0);
                const double Xff = in_.d11 * apSpeed_ + in_.q11 * std::fabs(apSpeed_) * apSpeed_;
                law.X = clampd(Xff + cfg_.apSpeedKp * in_.m11 * eu + cfg_.apSpeedKi * in_.m11 * speedInt_, -cap.X, cap.X);
                law.Y = clampd(-0.25 * in_.m22 * est.v, -cap.Y, cap.Y);
                ref_.turnTo(dt, apCourse_, eff());
                hdgCtl = true;
                break;
            }
            default: break;
        }

        if (posCtl) {
            const double eN = est.n - ref_.n, eE = est.e - ref_.e;
            const double vN = est.vn - ref_.vn, vE = est.ve - ref_.ve;
            const double ex = toBodyX(eN, eE), ey = toBodyY(eN, eE);
            const double vx = toBodyX(vN, vE), vy = toBodyY(vN, vE);
            const double umaxX = 0.95 * cap.X, umaxY = 0.95 * cap.Y;
            // Возврат после потери положения (задача №17): ошибка положения, на которую работает закон, ограничена returnErrMaxM —
            // отставшее от опорной точки судно догоняет её не быстрее ≈ wn·returnErrMaxM/(2ζ), а не разгоняется до 0,6 м/с
            double exl = ex, eyl = ey;
            if (returning_ && returnCap_ > 0) { const double em = std::hypot(ex, ey); if (em > cfg_.returnErrMaxM && em > 1e-9) { exl *= cfg_.returnErrMaxM / em; eyl *= cfg_.returnErrMaxM / em; } }
            law.X = lawX_->compute(dead(exl, posDead()), vx, umaxX);
            law.Y = lawY_->compute(dead(eyl, posDead()), vy, umaxY);
            // и если судно идёт к заданной точке быстрее предела возврата — притормаживание по его скорости (не сильнее returnBrakeAccel)
            if (returning_ && returnHoldS_ <= 0 && returnCap_ > 0) {
                const double tn = sp_.n - est.n, te = sp_.e - est.e, td = std::hypot(tn, te);
                if (td > 0.5) {
                    const double vt = (est.vn * tn + est.ve * te) / td, over = vt - 0.9 * returnCap_;   // с запасом 10 % на волну и порывы
                    if (over > 0) {
                        const double acc = std::min(cfg_.returnVelGain * over, cfg_.returnBrakeAccel);
                        law.X -= in_.m11 * acc * toBodyX(tn / td, te / td); law.Y -= in_.m22 * acc * toBodyY(tn / td, te / td);
                    }
                }
            }
            // упреждение по скорости опорной точки
            velFFx = toBodyX(ref_.vn, ref_.ve); velFFy = toBodyY(ref_.vn, ref_.ve);
            law.X += in_.d11 * velFFx + in_.q11 * std::fabs(velFFx) * velFFx + in_.m11 * toBodyX(ref_.an, ref_.ae);
            law.Y += in_.d22 * velFFy + in_.q22 * std::fabs(velFFy) * velFFy + in_.m22 * toBodyY(ref_.an, ref_.ae);
            o.errN = eN; o.errE = eE; o.errDist = std::hypot(eN, eE);
            o.errAlong = -ex; o.errCross = -ey;
        }
        if (hdgCtl) {
            const double eh = wrapPi(est.psi - hdgTarget);
            const double rr = est.r - ref_.r;
            if (!est.hdgDeadReckon) law.N += lawN_->compute(dead(eh, deg2rad(hdgDead())), rr, 0.95 * cap.N);
            o.errHdgDeg = rad2deg(eh);
        }

        // упреждение по ветру и компенсация неизвестной силы
        Wrench ff{}, bs{};
        if (cfg_.windFeedForward) {
            Wrench w = windBody;
            if (cfg_.predict.enabled) {          // среднее и тренд считаются в земных осях: при повороте судна сила пересчитывается без скачка
                const double fn = w.X * cp - w.Y * sp, fe = w.X * sp + w.Y * cp;
                const double pn = pr_[0].step(dt, fn, cfg_.predict), pe = pr_[1].step(dt, fe, cfg_.predict), pm = pr_[2].step(dt, w.N, cfg_.predict);
                w.X = pn * cp + pe * sp; w.Y = -pn * sp + pe * cp; w.N = pm;
            } else { for (auto& p : pr_) p.reset(); }
            ff.X = -w.X; ff.Y = -w.Y; ff.N = -w.N;
        }
        if (posCtl || mode_ == DpMode::Joystick || mode_ == DpMode::Autopilot || hdgCtl) {
            // оценка неизвестной силы дрожит (шум измерений), поэтому в команду берём её сглаженной (в мировых осях)
            if (!biasInit_) { biasF_[0] = est.biasN; biasF_[1] = est.biasE; biasF_[2] = est.biasMoment; biasInit_ = true; }
            else { const double ab = dt / (cfg_.biasFilterS + dt); biasF_[0] += ab * (est.biasN - biasF_[0]); biasF_[1] += ab * (est.biasE - biasF_[1]); biasF_[2] += ab * (est.biasMoment - biasF_[2]); }
            const double bx = biasF_[0] * cp + biasF_[1] * sp, by = -biasF_[0] * sp + biasF_[1] * cp;
            const double lim = cfg_.biasLimitFrac;
            if (posCtl) { bs.X = clampd(-bx, -lim * cap.X, lim * cap.X); bs.Y = clampd(-by, -lim * cap.Y, lim * cap.Y); }
            if (hdgCtl) bs.N = clampd(-biasF_[2], -lim * cap.N, lim * cap.N);
        }
        if (!posCtl) { ff.X = (mode_ == DpMode::Autopilot ? 0 : 0); ff.Y = 0; }
        Wrench raw{law.X + ff.X + bs.X, law.Y + ff.Y + bs.Y, law.N + ff.N + bs.N};
        raw.X = clampd(raw.X, -cap.X, cap.X); raw.Y = clampd(raw.Y, -cap.Y, cap.Y); raw.N = clampd(raw.N, -cap.N, cap.N);
        // счисление курса (гирокомпасов нет): курс не наблюдается, и замкнутый контур курса по прогнозу уводит судно (оценка момента
        // шумит, а регулятор отрабатывает её как настоящую ошибку). Поэтому момент держится таким, каким он был в среднем за ~10 с до потери
        if (hdgCtl && est.hdgDeadReckon) raw.N = nHeld_;
        else if (hdgCtl) nHeld_ += (raw.N - nHeld_) * std::min(1.0, dt / 10.0);
        else nHeld_ = raw.N;
        const double a = dt / (cfg_.forceFilterS + dt);
        filt_.X += a * (raw.X - filt_.X); filt_.Y += a * (raw.Y - filt_.Y); filt_.N += a * (raw.N - filt_.N);
        o.tau = filt_; o.law = law; o.windFF = ff; o.bias = bs;
        o.refN = ref_.n; o.refE = ref_.e; o.refPsiDeg = rad2deg(ref_.psi);
        return o;
    }

    const Reference& reference() const { return ref_; }
    double posDead() const { return std::max({cfg_.posDeadM, cfg_.minPower ? cfg_.minPosDeadM : 0.0, cfg_.holdAreaM > 0 ? cfg_.holdStopPct / 100.0 * cfg_.holdAreaM : 0.0}); }
    double hdgDead() const { return std::max(cfg_.hdgDeadDeg, cfg_.minPower ? cfg_.minHdgDeadDeg : 0.0); }

private:
    void build() {
        const double fz = cfg_.effortFrac;
        AxisParams px, py, pn;
        px.m = in_.m11; px.d = in_.d11; py.m = in_.m22; py.d = in_.d22; pn.m = in_.m66; pn.d = in_.d66;
        const double gk = cfg_.highGain ? cfg_.gainHighK : 1.0, wP = cfg_.wnPos * gk, wH = cfg_.wnHdg * gk;
        px.wn = py.wn = wP; px.zeta = py.zeta = cfg_.zetaPos;
        pn.wn = wH; pn.zeta = cfg_.zetaHdg;
        px.xtol = py.xtol = cfg_.posTolM; px.vtol = py.vtol = cfg_.velTolMS;
        pn.xtol = deg2rad(cfg_.hdgTolDeg); pn.vtol = deg2rad(cfg_.rotTolDegS);
        // номинальное усилие выбираем так, чтобы полоса пропускания совпадала с ПИД (wn² · m · допуск)
        px.ucap = fz * px.m * wP * wP * px.xtol; py.ucap = fz * py.m * wP * wP * py.xtol;
        pn.ucap = fz * pn.m * wH * wH * pn.xtol;
        for (AxisParams* p : {&px, &py, &pn}) { p->horizon = cfg_.mpcHorizon; p->step = cfg_.mpcStepS; p->rateW = cfg_.mpcRateWeight; }
        lawX_ = makeLaw(cfg_.law, px); lawY_ = makeLaw(cfg_.law, py); lawN_ = makeLaw(cfg_.law, pn);
    }
    void resetLaws() { lawX_->reset(); lawY_->reset(); lawN_->reset(); }

    // Движение опорной точки строго по линии пути: скорость нарастает с ограниченным ускорением,
    // перед поворотными пунктами и в конце трека снижается.
    void advanceTrack(double dt, const EstimatorState& est) {
        if (track_.wp.empty()) { ref_.moveTo(dt, ref_.n, ref_.e, 0, 0, eff()); return; }
        if (!trackStarted_) { segFromN_ = ref_.n; segFromE_ = ref_.e; trackV_ = 0; trackStarted_ = true; }
        if (wpIdx_ >= track_.wp.size()) {
            trackDone_ = true;
            ref_.moveTo(dt, track_.wp.back().n, track_.wp.back().e, 0, 0, eff());
            return;
        }
        const auto& w = track_.wp[wpIdx_];
        const double dn = w.n - ref_.n, de = w.e - ref_.e, dist = std::hypot(dn, de);
        const bool last = wpIdx_ + 1 == track_.wp.size();
        if (dist < 0.3) {
            segFromN_ = w.n; segFromE_ = w.e; ++wpIdx_;
            if (last) trackDone_ = true;
            return;
        }
        const double ux = dn / dist, uy = de / dist;
        // допустимая скорость у конца участка
        double vEnd = 0.0;
        if (!last) {
            const auto& nx = track_.wp[wpIdx_ + 1];
            const double ln = nx.n - w.n, le = nx.e - w.e, l = std::hypot(ln, le);
            if (l > 1e-6) {
                const double cosT = (ux * ln + uy * le) / l;          // 1 — прямо, −1 — разворот
                vEnd = track_.speed * std::max(0.0, cosT) * 0.5;
            }
        }
        const double vBrake = std::sqrt(vEnd * vEnd + 2.0 * cfg_.refAccel * dist);
        const double vWant = std::min(capSpeed_ > 0 ? std::min(track_.speed, capSpeed_) : track_.speed, vBrake);
        const double pv = trackV_;
        trackV_ += clampd(vWant - trackV_, -cfg_.refAccel * dt, cfg_.refAccel * dt);
        const double pvn = ref_.vn, pve = ref_.ve;
        ref_.vn = ux * trackV_; ref_.ve = uy * trackV_;
        ref_.an = (ref_.vn - pvn) / dt; ref_.ae = (ref_.ve - pve) / dt;
        (void)pv;
        const double step = std::min(trackV_ * dt, dist);
        ref_.n += ux * step; ref_.e += uy * step;
        if (!last && dist < 0.5 + trackV_ * dt) { segFromN_ = w.n; segFromE_ = w.e; ++wpIdx_; }
        // ограничение отставания: если судно сильно отстало, опорная точка ждёт
        const double lag = std::hypot(est.n - ref_.n, est.e - ref_.e);
        if (lag > 30.0) trackV_ *= 0.9;
        if (track_.headingAlongTrack && dist > 1.0) ref_.turnTo(dt, std::atan2(de, dn), eff());
        else ref_.turnTo(dt, deg2rad(sp_.psiDeg), eff());
    }

    ControlCfg eff() const {
        ControlCfg c = cfg_;
        if (capSpeed_ > 0) { c.refMaxSpeed = std::min(c.refMaxSpeed, capSpeed_); c.refCapTotal = true; }
        if (returning_ && returnCap_ > 0) { c.refMaxSpeed = std::min(c.refMaxSpeed, returnCap_); c.refCapTotal = true; }
        // при возврате тормозит опорная точка с returnBrakeAccel, а разгоняется к точке — с обычным refAccel (не догоняет рывком)
        if (returning_ && std::hypot(ref_.vn, ref_.ve) > returnCap_ + 0.01) c.refAccel = std::max(c.refAccel, c.returnBrakeAccel);
        if (capRot_ > 0) c.refMaxRotDegS = std::min(c.refMaxRotDegS, capRot_);
        return c;
    }
    double capSpeed_ = 0, capRot_ = 0;
    bool returning_ = false; double returnCap_ = 0, returnHoldS_ = 0, returnHold0_ = 0;
    bool holdLastSet_ = false; double holdExtS_ = 0, holdLastN_ = 0, holdLastE_ = 0, holdLastVn_ = 0, holdLastVe_ = 0;
    double nHeld_ = 0;   // момент по курсу, сглаженный за ~10 с: держится при счислении курса
    VesselCfg vcfg_;
    ControlCfg cfg_;
    Inertia in_;
    DpMode mode_ = DpMode::Standby;
    std::unique_ptr<AxisLaw> lawX_, lawY_, lawN_;
    Reference ref_, pref_;               // pref_ — опорная точка центра вращения
    double pvx_ = 0, pvy_ = 0;           // центр вращения
    Setpoint sp_;
    TrackCfg track_;
    TargetInfo target_;
    Joystick joy_;
    Wrench filt_;
    Predictor pr_[3];
    double biasF_[3] = {0, 0, 0}; bool biasInit_ = false;
    size_t wpIdx_ = 0;
    bool trackStarted_ = false;
    double segFromN_ = 0, segFromE_ = 0, trackV_ = 0;
    bool trackDone_ = false;
    double apCourse_ = 0, apSpeed_ = 0, speedInt_ = 0, holdPsi_ = 0;
    bool holdValid_ = false;
};

}  // namespace dp
