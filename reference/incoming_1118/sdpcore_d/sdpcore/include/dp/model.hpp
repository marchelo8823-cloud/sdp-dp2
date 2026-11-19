// Математическая модель судна: параметры, ветровая нагрузка, дрейфовая сила волн,
// силы движителей и динамика в шести степенях свободы.
#pragma once
#include <algorithm>
#include <random>

#include "dp/types.hpp"

namespace dp {

// ---------------------------------------------------------------- достройка параметров судна
struct Inertia {
    double m11, m22, m33, m44, m55, m66;   // масса + присоединённая масса по 6 степеням
    double d11, d22, d33, d44, d55, d66;   // линейное демпфирование
    double q11, q22, q66;                  // квадратичное демпфирование
    double kz, kphi, ktheta;               // коэффициенты восстанавливающих сил
};

inline Inertia completeInertia(const VesselCfg& v) {
    Inertia r{};
    const double m = v.mass, L = v.length, B = v.beam;
    const double Ixx = v.Ixx > 0 ? v.Ixx : m * 0.35 * 0.35 * B * B;
    const double Iyy = v.Iyy > 0 ? v.Iyy : m * 0.25 * 0.25 * L * L;
    const double Izz = v.Izz > 0 ? v.Izz : m * 0.25 * 0.25 * L * L;
    r.m11 = m + (v.addSurge > 0 ? v.addSurge : 0.10 * m);
    r.m22 = m + (v.addSway > 0 ? v.addSway : 0.60 * m);
    r.m33 = m + (v.addHeave > 0 ? v.addHeave : 0.80 * m);
    r.m44 = Ixx + (v.addRoll > 0 ? v.addRoll : 0.20 * Ixx);
    r.m55 = Iyy + (v.addPitch > 0 ? v.addPitch : 0.50 * Iyy);
    r.m66 = Izz + (v.addYaw > 0 ? v.addYaw : 0.50 * Izz);
    // линейное демпфирование по умолчанию — доля от массы на характерной частоте (≈ 0.02–0.1 с⁻¹)
    r.d11 = v.Xu > 0 ? v.Xu : 0.02 * r.m11;
    r.d22 = v.Yv > 0 ? v.Yv : 0.06 * r.m22;
    r.d33 = v.Zw > 0 ? v.Zw : 0.20 * r.m33;
    r.d44 = v.Kp > 0 ? v.Kp : 0.10 * r.m44;
    r.d55 = v.Mq > 0 ? v.Mq : 0.10 * r.m55;
    r.d66 = v.Nr > 0 ? v.Nr : 0.08 * r.m66;
    if (v.currentRS) {   // Руководство РС (ДУ1): Cx = 0,07 (по B·T), Cy = 0,6 (по площади боковой проекции подводной части)
        const double syh = v.currentLateralArea > 0 ? v.currentLateralArea : 0.85 * L * v.draft;
        r.q11 = v.Xuu > 0 ? v.Xuu : 0.5 * RHO_WATER * 0.07 * (B * v.draft);
        r.q22 = v.Yvv > 0 ? v.Yvv : 0.5 * RHO_WATER * 0.6 * syh;
    } else {
        r.q11 = v.Xuu > 0 ? v.Xuu : 0.5 * RHO_WATER * 0.02 * (B * v.draft) * 2.0;
        r.q22 = v.Yvv > 0 ? v.Yvv : 0.5 * RHO_WATER * 1.0 * (L * v.draft);
    }
    r.q66 = v.Nrr > 0 ? v.Nrr : 0.05 * RHO_WATER * L * L * L * L * v.draft * 0.01;
    r.kz = RHO_WATER * G * v.waterplaneCoef * L * B;
    r.kphi = m * G * v.gmT;
    r.ktheta = m * G * v.gmL;
    return r;
}

// ---------------------------------------------------------------- ветер
// alphaFrom — откуда дует кажущийся ветер относительно носа (рад, по часовой, 0 — встречный).
inline Wrench windForce(const VesselCfg& v, double speed, double alphaFrom) {
    Wrench w;
    if (speed <= 0) return w;
    const double q = 0.5 * RHO_AIR * speed * speed;
    const double a = wrapPi(alphaFrom);
    double cx, cy, cn;
    const double aa = std::fabs(a);
    if (!v.windTable.cx.empty()) {
        const auto& t = v.windTable;
        const double pos = clampd(rad2deg(aa) / t.stepDeg, 0.0, static_cast<double>(t.cx.size() - 1));
        const size_t i0 = static_cast<size_t>(pos);
        const size_t i1 = std::min(i0 + 1, t.cx.size() - 1);
        const double f = pos - static_cast<double>(i0);
        auto lerp = [&](const std::vector<double>& c) { return c[i0] + f * (c[i1] - c[i0]); };
        cx = lerp(t.cx);
        cy = t.cy.empty() ? 0.0 : lerp(t.cy);
        cn = t.cn.empty() ? 0.0 : lerp(t.cn);
        w.X = -q * v.frontalArea * cx;                         // таблица уже содержит знак/форму по углу
        w.Y = -sgn(a) * q * v.lateralArea * cy;
        w.N = -sgn(a) * q * v.lateralArea * v.length * cn;
        return w;
    }
    cx = v.cx;
    cy = v.cy;
    cn = v.cn;
    w.X = -q * v.frontalArea * cx * std::cos(a);
    w.Y = -q * v.lateralArea * cy * std::sin(a);
    if (v.windMomentRS) {   // РС, п. 2.2.1: Cmz = Cy(α)·(Xya/Lpp + 0,3·(1 − 2·απ/π)); Cy = −cy·sin α
        const double lpp = v.lpp > 0 ? v.lpp : 0.92 * v.length;
        w.N = w.Y * (v.xWind + 0.3 * lpp * (1.0 - 2.0 * aa / PI));
    } else {
        w.N = -q * v.lateralArea * v.length * cn * std::sin(2.0 * a);
    }
    return w;
}

// ---------------------------------------------------------------- движители
inline double thrusterMax(const ThrusterCfg& t, double signedFrac) {  // предел упора для знака
    return signedFrac >= 0 ? t.maxForward : t.maxReverse;
}

// КПД колонки (0..1) при направлении упора angleDeg по таблице effAngle; без таблицы — 1. Интерполяция линейная по кругу.
inline double effAt(const ThrusterCfg& t, double angleDeg) {
    if (t.effAngle.empty()) return 1.0;
    std::vector<AnglePt> p = t.effAngle;
    for (auto& q : p) q.angleDeg = rad2deg(wrap2Pi(deg2rad(q.angleDeg)));
    std::sort(p.begin(), p.end(), [](const AnglePt& a, const AnglePt& b) { return a.angleDeg < b.angleDeg; });
    const double a = rad2deg(wrap2Pi(deg2rad(angleDeg)));
    const size_t n = p.size();
    size_t hi = 0; while (hi < n && p[hi].angleDeg < a) ++hi;
    const AnglePt& B = p[hi % n]; const AnglePt& A = p[(hi + n - 1) % n];
    double a0 = A.angleDeg, a1 = B.angleDeg, x = a;
    if (a1 <= a0) { a1 += 360.0; if (x < a0) x += 360.0; }
    const double k = a1 - a0 < 1e-9 ? 0.0 : (x - a0) / (a1 - a0);
    return clampd(A.pct + (B.pct - A.pct) * k, 0.0, 100.0) / 100.0;
}

// Сила и момент от одного движителя в связанных осях: упор thrustN вдоль угла angleRad.
inline Wrench thrusterWrench(const ThrusterCfg& t, double thrustN, double angleRad) {
    Wrench w;
    if (t.type == ThrusterType::Azimuth && !t.effIsLimit && !t.effAngle.empty()) thrustN *= effAt(t, rad2deg(angleRad));   // потеря упора по углу
    w.X = thrustN * std::cos(angleRad);
    w.Y = thrustN * std::sin(angleRad);
    w.N = t.x * w.Y - t.y * w.X;
    return w;
}

inline double thrusterBaseAngle(const ThrusterCfg& t) {
    switch (t.type) {
        case ThrusterType::Tunnel: return deg2rad(t.fixedAngleDeg != 0 ? t.fixedAngleDeg : 90.0);
        case ThrusterType::Fixed: return deg2rad(t.fixedAngleDeg);
        default: return 0.0;
    }
}

inline bool angleForbidden(const ThrusterCfg& t, double angleDeg) {
    const double a = rad2deg(wrap2Pi(deg2rad(angleDeg)));
    for (const auto& s : t.forbidden) {
        const double f = rad2deg(wrap2Pi(deg2rad(s.fromDeg))), to = rad2deg(wrap2Pi(deg2rad(s.toDeg)));
        if (f <= to ? (a >= f && a <= to) : (a >= f || a <= to)) return true;
    }
    return false;
}

// Угол в «жёстком» ядре запретного сектора (вокруг его середины, ширина hardCoreDeg): здесь колонка не остаётся ни при каких условиях.
inline bool angleHardForbidden(const ThrusterCfg& t, double angleDeg) {
    const double a = rad2deg(wrap2Pi(deg2rad(angleDeg)));
    for (const auto& s : t.forbidden) {
        const double f = rad2deg(wrap2Pi(deg2rad(s.fromDeg))), to = rad2deg(wrap2Pi(deg2rad(s.toDeg)));
        const bool in = f <= to ? (a >= f && a <= to) : (a >= f || a <= to);
        if (!in) continue;
        const double span = f <= to ? to - f : to - f + 360.0;
        const double c = std::fmod(f + span / 2.0, 360.0);
        const double d = std::fabs(rad2deg(wrapPi(deg2rad(a - c))));
        if (d <= t.hardCoreDeg / 2.0 || t.hardCoreDeg >= span) return true;
    }
    return false;
}

// Ближайший допустимый угол с учётом запретных секторов (шаг поиска 1°).
inline double nearestAllowedAngle(const ThrusterCfg& t, double angleDeg) {
    if (!angleForbidden(t, angleDeg)) return angleDeg;
    for (int d = 1; d <= 180; ++d) {
        if (!angleForbidden(t, angleDeg + d)) return angleDeg + d;
        if (!angleForbidden(t, angleDeg - d)) return angleDeg - d;
    }
    return angleDeg;
}

// ---------------------------------------------------------------- динамика (имитатор «истинного» судна)
struct VesselState {
    double n = 0, e = 0, d = 0;          // положение, м
    double phi = 0, theta = 0, psi = 0;  // крен, дифферент, курс, рад
    double u = 0, v = 0, w = 0, p = 0, q = 0, r = 0;
};

// Средний период по нулям Tz по значительной высоте (ветроволновая статистика Северного моря, Руководство РС, прил. 2, табл. 1).
inline double tzFromHs(double hs) {
    static const double H[] = {0, 0.1, 0.4, 0.8, 1.3, 2.1, 3.1, 4.2, 5.7, 7.4, 9.5, 12.1};
    static const double T[] = {2.5, 2.5, 3.2, 3.9, 4.6, 5.3, 6.0, 6.4, 7.1, 7.4, 8.1, 8.5};
    if (hs <= H[0]) return T[0];
    for (int i = 1; i < 12; ++i) if (hs <= H[i]) return T[i - 1] + (T[i] - T[i - 1]) * (hs - H[i - 1]) / (H[i] - H[i - 1]);
    return T[11];
}

// Средняя дрейфовая сила волн в связанных осях. hs — высота значительная, м; fromDeg — откуда идут волны (истинный пеленг);
// psi — курс, рад; tz — средний период по нулям, с (0 — по статистике из hs). Одна и та же формула используется
// в имитаторе («истина») и в ядре (оценка по данным МИО).
// Модель 1 — Руководство РС (ДУ1, п. 4) и DNV-ST-0111 (2021); модель 0 — прежняя упрощённая.
inline Wrench waveDriftForce(const VesselCfg& cfg, double hs, double fromDeg, double psi, double tz = 0) {
    Wrench w;
    if (hs <= 0) return w;
    const double rel = deg2rad(fromDeg) + PI - psi;   // направление распространения в связанных осях
    if (cfg.waveModel == 0) {
        const double width = std::fabs(std::sin(rel)) * cfg.length + std::fabs(std::cos(rel)) * cfg.beam;
        const double F = (1.0 / 16.0) * RHO_WATER * G * hs * hs * cfg.waveReflection * width;
        w.X = F * std::cos(rel);
        w.Y = F * std::sin(rel);
        w.N = 0.02 * F * cfg.length * std::sin(2 * rel);
        return w;
    }
    const double lpp = cfg.lpp > 0 ? cfg.lpp : 0.92 * cfg.length;
    const double los = cfg.los > 0 ? cfg.los : lpp;
    const double T = tz > 0 ? tz : tzFromHs(hs);
    const double al = wrapPi(rel - PI);                 // откуда идут волны относительно носа, по часовой (0 — встречные)
    const double alp = std::fabs(al);                   // απ ∈ [0, π]
    const double h3 = 1.32 * hs;                        // высота 3 % обеспеченности
    auto f = [](double t) { return t < 1.0 ? 1.0 : std::pow(t, -3.0) * std::exp(1.0 - std::pow(t, -3.0)); };
    const double h1A = 0.8 * std::pow(cfg.waveBeta, 0.45);
    const double cw = clampd(cfg.waveCwla, 0.85, 1.15);
    const double h1B = 0.7 * cw * cw;
    const double h1 = h1A + alp / PI * (h1B - h1A);
    const double h2 = 0.05 + 0.95 * std::atan(1.45 * (alp - 1.75));
    const double Tx = T / (0.9 * std::pow(lpp, 0.33)), Ty = T / (0.75 * std::sqrt(cfg.beam));
    const double rg = RHO_WATER * G;
    w.X = rg * cfg.beam * h3 * h3 * 0.026 * h1 * h2 * f(Tx);                 // встречные волны: h2 < 0 — сила к корме
    const double Cy = -0.026 * std::sin(al) * los / lpp * f(Ty);             // волны с правого борта — сила на левый
    w.Y = rg * lpp * h3 * h3 * Cy;
    const double Cmz = Cy * (cfg.xLos / lpp + (0.05 - 0.14 * alp / PI) * los / lpp);
    w.N = rg * lpp * lpp * h3 * h3 * Cmz;
    return w;
}

class WaveField {
public:
    // Состав волнения (случайные доли частот и начальные фазы) задаётся один раз; высота, период и направление
    // потом меняются плавно, фазы накапливаются — без скачков ординаты при смене настроек.
    void configure(double hs, double tp, double fromDeg, unsigned seed = 7) {
        comps_.clear();
        PortableRng rng(seed);
        for (int i = 0; i < 10; ++i) { Comp c; c.ratio = rng.uniform(0.75, 1.35); c.phase = rng.uniform(0, 2 * PI); c.acc = 0; comps_.push_back(c); }
        hs_ = hs; tp_ = tp > 0 ? tp : 8.0; from_ = deg2rad(fromDeg);
        refresh();
    }
    void update(double dt, double hs, double tp, double fromDeg) {
        hs_ = hs; tp_ = tp > 0 ? tp : 8.0; from_ = deg2rad(fromDeg);
        for (auto& c : comps_) c.acc += 2 * PI / tp_ * c.ratio * dt;
        refresh();
    }
    // Ордината и уклоны волны в точке (n, e); уклоны по осям NED.
    void sample(double /*t*/, double n, double e, double& zeta, double& slopeN, double& slopeE) const {
        zeta = slopeN = slopeE = 0;
        const double toN = std::cos(from_ + PI), toE = std::sin(from_ + PI);   // направление распространения
        for (const auto& c : comps_) {
            const double arg = c.acc - c.k * (n * toN + e * toE) + c.phase;
            zeta += amp_ * std::cos(arg);
            const double ds = amp_ * c.k * std::sin(arg);
            slopeN += ds * toN;
            slopeE += ds * toE;
        }
    }
    bool active() const { return amp_ > 1e-6; }
    double tz() const {   // средний период по нулям составного волнения
        double m2 = 0; for (const auto& c : comps_) m2 += c.ratio * c.ratio;
        return comps_.empty() ? 0.0 : tp_ / std::sqrt(m2 / static_cast<double>(comps_.size()));
    }
private:
    struct Comp { double ratio, phase, acc, k = 0; };
    void refresh() {
        amp_ = comps_.empty() || hs_ <= 0 ? 0.0 : std::sqrt(2.0 * (hs_ * hs_ / 16.0) / static_cast<double>(comps_.size()));
        for (auto& c : comps_) { const double w = 2 * PI / tp_ * c.ratio; c.k = w * w / G; }
    }
    std::vector<Comp> comps_;
    double amp_ = 0, hs_ = 0, tp_ = 8, from_ = 0;
};

class VesselDynamics {
public:
    VesselDynamics(const VesselCfg& cfg, const EnvCfg& env) : cfg_(cfg), env_(env), in_(completeInertia(cfg)) {
        waves_.configure(env.hs, env.tp, env.waveFromDeg);
    }
    VesselState& state() { return s_; }
    const VesselState& state() const { return s_; }
    EnvCfg& env() { return env_; }
    void reconfigureWaves() {}   // волны меняются плавно сами при изменении env()
    const Inertia& inertia() const { return in_; }

    // Кажущийся ветер в точке мачты: скорость и направление «откуда» относительно носа.
    void apparentWind(double& speed, double& fromRel) const {
        const double to = deg2rad(env_.windFromDeg) + gustDir_ + PI;
        const double ws0 = std::max(0.0, env_.windSpeed + gustSpd_);
        const double vn = ws0 * std::cos(to), ve = ws0 * std::sin(to);
        // скорость судна в мире
        const double cp = std::cos(s_.psi), sp = std::sin(s_.psi);
        const double sn = s_.u * cp - s_.v * sp, se = s_.u * sp + s_.v * cp;
        const double an = vn - sn, ae = ve - se;                        // воздух относительно судна, куда дует
        const double ub = an * cp + ae * sp, vb = -an * sp + ae * cp;   // в связанных осях
        speed = std::hypot(ub, vb);
        fromRel = std::atan2(-vb, -ub);                                  // откуда дует
    }

    // Шаг интегрирования. tau — силы движителей в связанных осях (X,Y,N); t — время, с.
    void step(double dt, const Wrench& tau, double t) {
        const double cp = std::cos(s_.psi), sp = std::sin(s_.psi);
        // течение в связанных осях
        updateTurbulence(dt);
        waves_.update(dt, env_.hs, env_.tp, env_.waveFromDeg);
        const double ctoN = std::cos(deg2rad(env_.currentToDeg)) * env_.currentSpeed + curVarN_;
        const double ctoE = std::sin(deg2rad(env_.currentToDeg)) * env_.currentSpeed + curVarE_;
        const double uc = ctoN * cp + ctoE * sp, vc = -ctoN * sp + ctoE * cp;
        const double ur = s_.u - uc, vr = s_.v - vc;

        // ветер
        double ws, wf;
        apparentWind(ws, wf);
        const Wrench wf_ = windForce(cfg_, ws, wf);

        // дрейфовая сила волн (средняя)
        Wrench wd = waveDriftForce(cfg_, env_.hs, env_.waveFromDeg, s_.psi, waves_.tz());
        if (env_.turbulence > 0 && (slowN_ != 0 || slowE_ != 0)) {          // медленный дрейф: вторая гармоника волн, периоды 1–2 мин
            wd.X += slowN_ * cp + slowE_ * sp; wd.Y += -slowN_ * sp + slowE_ * cp;
        }
        meanWave_ = std::hypot(wd.X, wd.Y);

        const Inertia& I = in_;
        const double X = tau.X + wf_.X + wd.X - I.d11 * ur - I.q11 * std::fabs(ur) * ur;
        const double Y = tau.Y + wf_.Y + wd.Y - I.d22 * vr - I.q22 * std::fabs(vr) * vr;
        double Ncur = 0;
        if (cfg_.currentRS) {   // РС п. 3.2.1: момент течения Mz = Fy·(Xyh + max(min(0,4·(1 − 2απ/π), 0,25), −0,2)·Lpp)
            const double lpp = cfg_.lpp > 0 ? cfg_.lpp : 0.92 * cfg_.length;
            const double alp = std::fabs(std::atan2(vr, ur));
            const double Fyc = -I.q22 * std::fabs(vr) * vr;
            Ncur = Fyc * (cfg_.xCurrent + clampd(0.4 * (1.0 - 2.0 * alp / PI), -0.2, 0.25) * lpp);
        }
        const double N = tau.N + wf_.N + wd.N + Ncur - I.d66 * s_.r - I.q66 * std::fabs(s_.r) * s_.r;
        const double du = (X + I.m22 * s_.v * s_.r) / I.m11;
        const double dv = (Y - I.m11 * s_.u * s_.r) / I.m22;
        const double dr = (N - (I.m22 - I.m11) * s_.u * s_.v) / I.m66;

        // волновое возбуждение вертикальных движений
        double zeta = 0, sN = 0, sE = 0;
        if (waves_.active()) waves_.sample(t, s_.n, s_.e, zeta, sN, sE);
        const double slopeX = sN * cp + sE * sp, slopeY = -sN * sp + sE * cp;
        const double Fz = -I.kz * cfg_.raoHeave * zeta - I.d33 * s_.w - I.kz * s_.d;   // d — вниз, ζ — подъём поверхности (вверх)
        const double Kx = -I.kphi * cfg_.raoRoll * slopeY - I.d44 * s_.p - I.kphi * s_.phi;
        const double Mx_ = I.ktheta * cfg_.raoPitch * slopeX - I.d55 * s_.q - I.ktheta * s_.theta;

        s_.u += du * dt;
        s_.v += dv * dt;
        s_.r += dr * dt;
        s_.w += Fz / I.m33 * dt;
        s_.p += Kx / I.m44 * dt;
        s_.q += Mx_ / I.m55 * dt;
        s_.n += (s_.u * cp - s_.v * sp) * dt;
        s_.e += (s_.u * sp + s_.v * cp) * dt;
        s_.d += s_.w * dt;
        s_.phi += s_.p * dt;
        s_.theta += s_.q * dt;
        s_.psi = wrapPi(s_.psi + s_.r * dt);
    }

private:
    // Случайные медленные процессы (Орнштейна — Уленбека): порывы ветра, изменение течения, медленный дрейф от волн.
    double ou(double x, double dt, double tau, double sigma) {
        const double a = std::exp(-dt / tau);
        return x * a + sigma * std::sqrt(1.0 - a * a) * rng_.normal();
    }
    void updateTurbulence(double dt) {
        const double k = env_.turbulence;
        if (k <= 0) { gustSpd_ = gustDir_ = curVarN_ = curVarE_ = slowN_ = slowE_ = 0; return; }
        gustSpd_ = ou(gustSpd_, dt, 25.0, k * std::max(0.3, 0.08 * env_.windSpeed) * (env_.windSpeed > 0.5 ? 1.0 : 0.0));
        gustDir_ = ou(gustDir_, dt, 60.0, k * deg2rad(2.5) * (env_.windSpeed > 1.0 ? 1.0 : 0.0));
        const double cs = k * (0.02 + 0.05 * env_.currentSpeed) * (env_.currentSpeed > 0.05 ? 1.0 : 0.0);
        curVarN_ = ou(curVarN_, dt, 120.0, cs); curVarE_ = ou(curVarE_, dt, 120.0, cs);
        const double sf = k * 0.35 * meanWave_;
        slowN_ = ou(slowN_, dt, 50.0, sf); slowE_ = ou(slowE_, dt, 50.0, sf);
    }

    VesselCfg cfg_;
    EnvCfg env_;
    Inertia in_;
    VesselState s_;
    WaveField waves_;
    PortableRng rng_{4242};
    double gustSpd_ = 0, gustDir_ = 0, curVarN_ = 0, curVarE_ = 0, slowN_ = 0, slowE_ = 0, meanWave_ = 0;
};

}  // namespace dp
