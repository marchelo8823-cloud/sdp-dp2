// Возможности судна (capability plot): предельная скорость ветра по направлениям, при которой комплекс ещё удерживает судно.
// Для каждого направления ветра (волны и течение — с того же направления) ищется наибольшая скорость, при которой распределитель
// движителей набирает нужные силу и момент: «предел» — при полном упоре, «рабочий» — при 80 % упора.
// Сценарии: все движители, отказ каждого движителя, отказ шины A или B (если описана электростанция).
// ⚠ Расчёт по средним силам (без порывов и качки), курс судна удерживается постоянным, направление ветра, волн и течения совпадает.
//   Это оценка по модели ядра, а не расчёт по правилам классификационного общества.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include "dp/config.hpp"
#include "dp/thrust.hpp"

namespace dp {

struct CapEnv {
    double currentMS = 0.0;     // скорость течения, м/с
    bool wavesWithWind = true;  // волна по ветру: Hs = 0,0246·V² (развитое волнение, не более 12 м), иначе без волн
    double maxWindMS = 40.0;    // верхняя граница поиска, м/с
    double stepDeg = 10.0;      // шаг направления, °
};

struct CapCurve {
    std::string name;                 // «Все движители», «Отказ PU1», «Отказ шины A»
    std::vector<double> dirDeg;       // направление, откуда дует ветер относительно носа, °
    std::vector<double> limitMS;      // предельный ветер при полном упоре, м/с
    std::vector<double> workMS;       // рабочий ветер при 80 % упора, м/с
    double minLimit() const { return limitMS.empty() ? 0.0 : *std::min_element(limitMS.begin(), limitMS.end()); }
    double minWork() const { return workMS.empty() ? 0.0 : *std::min_element(workMS.begin(), workMS.end()); }
};

// Силы окружающей среды на неподвижное судно с курсом 0 при ветре, волне и течении «откуда» alphaDeg (по часовой от носа).
inline Wrench capEnvForce(const VesselCfg& v, const Inertia& in, double windMS, double hs, double currentMS, double alphaDeg) {
    Wrench w = windForce(v, windMS, deg2rad(alphaDeg));
    const Wrench wd = waveDriftForce(v, hs, alphaDeg, 0.0);
    w.X += wd.X; w.Y += wd.Y; w.N += wd.N;
    if (currentMS > 0) {
        const double to = deg2rad(alphaDeg) + PI;                  // куда течёт вода относительно носа
        const double uc = currentMS * std::cos(to), vc = currentMS * std::sin(to);
        const double Fx = in.q11 * std::fabs(uc) * uc, Fy = in.q22 * std::fabs(vc) * vc;
        w.X += Fx; w.Y += Fy;
        if (v.currentRS) {
            const double lpp = v.lpp > 0 ? v.lpp : 0.92 * v.length;
            const double alp = std::fabs(std::atan2(-vc, -uc));
            w.N += Fy * (v.xCurrent + clampd(0.4 * (1.0 - 2.0 * alp / PI), -0.2, 0.25) * lpp);
        }
    }
    return w;
}


// Хватает ли комплексу усилий для удержания при заданном ветре: сравниваются набранные и нужные сила и момент.
// Колонки считаются уже повёрнутыми на нужные углы (установившийся режим): распределение повторяется, пока углы не перестанут меняться.
inline bool capHolds(const Allocator& al, const Wrench& envF, double scale) {
    if (scale <= 0) return false;
    const Wrench tau{-envF.X, -envF.Y, -envF.N};
    const size_t nt = al.thrusters().size();
    std::vector<double> act(nt, 0.0);
    AllocResult r;
    for (int it = 0; it < 6; ++it) {
        r = al.allocate(tau, act, scale);
        bool moved = false;
        for (size_t i = 0; i < nt && i < r.cmds.size(); ++i) {
            if (al.thrusters()[i].type != ThrusterType::Azimuth) continue;
            if (std::fabs(rad2deg(wrapPi(deg2rad(r.cmds[i].angleDeg - act[i])))) > 1.0) moved = true;
            act[i] = r.cmds[i].angleDeg;
        }
        if (!moved) break;
    }
    const double fn = std::hypot(tau.X, tau.Y), mx = std::hypot(tau.X - r.achieved.X, tau.Y - r.achieved.Y);
    return mx <= 0.02 * fn + 500.0 && std::fabs(tau.N - r.achieved.N) <= 0.03 * std::fabs(tau.N) + 5000.0;
}

inline double capLimitWind(const VesselCfg& v, const Inertia& in, const Allocator& al, const CapEnv& e, double alphaDeg, double scale) {
    auto holds = [&](double V) {
        const double hs = e.wavesWithWind ? std::min(0.0246 * V * V, 12.0) : 0.0;
        return capHolds(al, capEnvForce(v, in, V, hs, e.currentMS, alphaDeg), scale);
    };
    if (!holds(0.0)) return 0.0;                 // не держит даже при штиле (течение слишком сильное)
    if (holds(e.maxWindMS)) return e.maxWindMS;
    double lo = 0, hi = e.maxWindMS;
    for (int i = 0; i < 14; ++i) { const double mid = 0.5 * (lo + hi); (holds(mid) ? lo : hi) = mid; }
    return lo;
}

inline CapCurve capCurve(const VesselCfg& v, const std::vector<ThrusterCfg>& th, double scale, const CapEnv& e, const std::string& name) {
    CapCurve c; c.name = name;
    const Inertia in = completeInertia(v);
    Allocator al(th);
    for (double a = 0; a < 360.0 - 1e-9; a += e.stepDeg) {
        c.dirDeg.push_back(a);
        c.limitMS.push_back(capLimitWind(v, in, al, e, a, scale));
        c.workMS.push_back(capLimitWind(v, in, al, e, a, scale * 0.8));
    }
    return c;
}

// Все сценарии: «Все движители», затем отказ каждого движителя .
inline std::vector<CapCurve> capScenarios(const AppCfg& cfg, const CapEnv& e) {
    std::vector<ThrusterCfg> base; for (const auto& t : cfg.thrusters) if (t.maxForward > 0 && t.enabled) base.push_back(t);
    std::vector<CapCurve> out;
    out.push_back(capCurve(cfg.vessel, base, 1.0, e, "Все движители"));
    for (size_t i = 0; i < base.size(); ++i) {
        std::vector<ThrusterCfg> t2 = base; t2[i].enabled = false;
        out.push_back(capCurve(cfg.vessel, t2, 1.0, e, "Отказ " + base[i].id));
    }
    return out;
}


// ------------------------------------------------------------ гипотетический анализ и дрейф (по образцу Capability/Drift Analysis NavDP 4000)
// Оператор задаёт отказы движителей и шин, предел упора, ветер, течение — и видит: держит ли комплекс, какое усилие не добирается,
// куда и с какой скоростью судно дрейфует и за какое время дойдёт до заданного радиуса.
struct HypotCase {
    std::vector<std::string> failed;      // отказавшие движители (id)
    bool deadBus[2] = {false, false};     // отказ шины A / B
    double thrustLimitPct = 100.0;        // общий предел упора, % (ограничение оператора)
    double windMS = 10.0, windFromDeg = 0.0;   // ветер: скорость и направление «откуда» относительно носа, °
    double currentMS = 0.0;               // течение (с того же направления, что и ветер)
    bool wavesWithWind = true;
    double radiusM = 10.0;                // радиус, до которого считается время дрейфа
};

struct DriftResult {
    bool holds = false;                   // удержание сохраняется
    Wrench envForce;                      // внешняя сила на судно (связанные оси), Н, Н·м
    Wrench achieved;                      // набранная тяга комплекса
    Wrench net;                           // равнодействующая: внешняя + тяга
    double residualN = 0.0;               // модуль несбалансированной силы, Н
    double yawResidualNm = 0.0;           // несбалансированный момент, Н·м
    double driftSpeedMS = 0.0;            // установившаяся скорость дрейфа, м/с (квадратичное сопротивление)
    double driftDirDeg = 0.0;             // куда дрейфует, относительно носа, °
    double timeToRadiusS = -1.0;          // время до радиуса, с (−1 — не дойдёт за час / не дрейфует)
    std::string note;
    std::vector<double> trackT, trackD;   // график дрейфа: время, с, и пройденное расстояние, м (до двух радиусов)
};

inline DriftResult capHypot(const AppCfg& cfg, const HypotCase& h) {
    DriftResult r;
    std::vector<ThrusterCfg> th;
    for (const auto& t : cfg.thrusters) {
        if (t.maxForward <= 0 || !t.enabled) continue;
        if (std::find(h.failed.begin(), h.failed.end(), t.id) != h.failed.end()) continue;
        th.push_back(t);
    }
    const bool deadAny = h.deadBus[0] || h.deadBus[1];
    if (deadAny) for (auto& t : th) if (t.bus >= 0 && t.bus <= 1 && h.deadBus[t.bus]) t.enabled = false;
    double scale = 1.0 * clampd(h.thrustLimitPct, 0.0, 100.0) / 100.0;
    const Inertia in = completeInertia(cfg.vessel);
    const double hs = h.wavesWithWind ? std::min(0.0246 * h.windMS * h.windMS, 12.0) : 0.0;
    r.envForce = capEnvForce(cfg.vessel, in, h.windMS, hs, h.currentMS, h.windFromDeg);
    const Wrench tau{-r.envForce.X, -r.envForce.Y, -r.envForce.N};
    Allocator al(th);
    AllocResult ar; const size_t nt = th.size(); std::vector<double> act(nt, 0.0);
    if (nt > 0 && scale > 0) {
        for (int it = 0; it < 6; ++it) {
            ar = al.allocate(tau, act, scale);
            bool moved = false;
            for (size_t i = 0; i < nt && i < ar.cmds.size(); ++i) {
                if (th[i].type != ThrusterType::Azimuth) continue;
                if (std::fabs(rad2deg(wrapPi(deg2rad(ar.cmds[i].angleDeg - act[i])))) > 1.0) moved = true;
                act[i] = ar.cmds[i].angleDeg;
            }
            if (!moved) break;
        }
        r.achieved = ar.achieved;
    }
    r.net = Wrench{r.envForce.X + r.achieved.X, r.envForce.Y + r.achieved.Y, r.envForce.N + r.achieved.N};
    r.residualN = std::hypot(r.net.X, r.net.Y); r.yawResidualNm = r.net.N;
    r.holds = nt > 0 && scale > 0 && capHolds(al, r.envForce, scale);
    if (r.holds) { r.note = "Удержание сохраняется"; return r; }
    // дрейф: сила тянет судно, сопротивление квадратичное; интегрируем вдоль равнодействующей
    const double F = r.residualN;
    if (F < 1.0) { r.note = "Не держит по курсу: поперечного дрейфа нет, остаётся несбалансированный момент"; return r; }
    const double ux = r.net.X / F, uy = r.net.Y / F;
    const double q = std::max(1.0, std::hypot(in.q11 * ux, in.q22 * uy)), m = std::max(1.0, std::hypot(in.m11 * ux, in.m22 * uy));
    r.driftSpeedMS = std::sqrt(F / q); r.driftDirDeg = std::fmod(rad2deg(std::atan2(uy, ux)) + 360.0, 360.0);
    double v = 0, d = 0;
    for (double t = 0; t < 3600.0; t += 0.5) {
        v += 0.5 * (F - q * v * v) / m; d += 0.5 * v;
        if (static_cast<long>(t * 2) % 10 == 0) { r.trackT.push_back(t + 0.5); r.trackD.push_back(d); }
        if (d >= h.radiusM && r.timeToRadiusS < 0) r.timeToRadiusS = t + 0.5;
        if (d >= 2.0 * h.radiusM) break;
    }
    char b[200]; std::snprintf(b, sizeof b, "Не держит: дрейф %.2f м/с в направлении %.0f° (от носа), до %.0f м — %s", r.driftSpeedMS, r.driftDirDeg, h.radiusM,
        r.timeToRadiusS > 0 ? (std::to_string(static_cast<int>(r.timeToRadiusS)) + " с").c_str() : "более часа");
    r.note = b;
    return r;
}

}  // namespace dp
