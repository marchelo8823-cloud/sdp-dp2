// Распределение тяги между движителями и модель отклика движителей.
#pragma once
#include <map>

#include "dp/model.hpp"

namespace dp {

// --- кривая движителя (необязательная). Столбцы: 0 — обороты, 1 — упор, 2 — мощность, всё в %.
inline double curveMap(const std::vector<CurvePt>& c, double x, int from, int to) {
    auto col = [](const CurvePt& p, int k) { return k == 0 ? p.rpm : k == 1 ? p.thrust : p.power; };
    if (c.empty()) return x;
    if (x <= col(c.front(), from)) return col(c.front(), to);
    for (size_t i = 1; i < c.size(); ++i) {
        const double a = col(c[i - 1], from), b = col(c[i], from);
        if (x <= b) {
            if (b - a < 1e-9) return col(c[i], to);
            return col(c[i - 1], to) + (col(c[i], to) - col(c[i - 1], to)) * (x - a) / (b - a);
        }
    }
    return col(c.back(), to);
}
inline double limitFrac(const ThrusterCfg& t) { return clampd(t.limitPct, 1.0, 100.0) / 100.0 * clampd(t.speedFactor, 0.0, 1.0); }
// Множитель максимального упора по скорости судна (уз): до fadeKn — 1, далее линейно до fadeFloorPct при 2·fadeKn и выше.
inline double speedFactorFor(const ThrusterCfg& t, double speedKn) {
    if (t.fadeKn <= 0.0 || speedKn <= t.fadeKn) return 1.0;
    const double fl = clampd(t.fadeFloorPct, 0.0, 100.0) / 100.0;
    return 1.0 - (1.0 - fl) * clampd((speedKn - t.fadeKn) / t.fadeKn, 0.0, 1.0);
}
// Мощность движителя, кВт, при заданной доле упора (по модулю, 0..1). Без кривой — P ~ T^1,5 (как в таблице Praxis).
inline double thrusterPowerKW(const ThrusterCfg& t, double thrustFrac) {
    const double f = clampd(std::fabs(thrustFrac), 0.0, 1.0);
    const double idle = t.powerMaxKW * clampd(t.idlePct, 0.0, 100.0) / 100.0;
    const double p = t.curve.empty() ? t.powerMaxKW * std::pow(f, 1.5) : t.powerMaxKW * curveMap(t.curve, 100.0 * f, 1, 2) / 100.0;
    return std::max(p, idle);
}
// Обороты, % от максимальных, нужные для заданной доли упора (0..1). Без кривой — 100·√f (упор ~ n²).
inline double thrusterRpmPct(const ThrusterCfg& t, double thrustFrac) {
    const double f = clampd(std::fabs(thrustFrac), 0.0, 1.0);
    if (t.curve.empty()) return 100.0 * std::sqrt(f);
    return curveMap(t.curve, 100.0 * f, 1, 0);
}

// Доля упора, которую допускает доступная мощность: ps — доля мощности (0..1) от суммарной на максимальных упорах.
// Без кривых упор ~ мощность^(2/3); с кривыми доля находится так, чтобы суммарная мощность по кривым равнялась ps·Σ P_max.
inline double thrustScaleForPower(const std::vector<ThrusterCfg>& th, double ps) {
    if (ps >= 1.0) return 1.0;
    bool anyCurve = false; double need = 0;
    for (const auto& t : th) if (t.enabled) { anyCurve = anyCurve || !t.curve.empty(); need += t.powerMaxKW; }
    if (!anyCurve) return std::pow(std::max(ps, 0.0), 2.0 / 3.0);
    double lo = 0, hi = 1;
    for (int it = 0; it < 30; ++it) {
        const double mid = 0.5 * (lo + hi); double sum = 0;
        for (const auto& t : th) if (t.enabled) sum += thrusterPowerKW(t, mid);
        (sum <= ps * need ? lo : hi) = mid;
    }
    return lo;
}

struct Capability {   // оценка предельных усилий комплекса в связанных осях
    double X = 0, Y = 0, N = 0;
};

struct AllocResult {
    std::vector<ThrusterCmd> cmds;
    Wrench achieved;
    bool saturated = false;
    bool turning = false;         // колонка ещё разворачивается (больше 15° до нужного угла): упор временно только вдоль её нынешнего направления
    double utilization = 0;       // максимальная загрузка движителя, 0..1
    std::vector<std::string> softZone;   // колонки, оставленные в «нежелательной» части запретного сектора (иначе запрос не выполнить)
};

class Allocator {
public:
    explicit Allocator(std::vector<ThrusterCfg> th) : th_(std::move(th)) {}
    const std::vector<ThrusterCfg>& thrusters() const { return th_; }
    void setSpeedKn(double kn) { for (auto& t : th_) t.speedFactor = speedFactorFor(t, kn); }   // спад упора ПУ с ростом скорости судна
    void setPreload(size_t i, double prefDeg, double pct) { if (i < th_.size()) { th_[i].prefDeg = prefDeg; th_[i].preloadPct = pct; } }   // предпочтительный угол и предварительный упор (режимы TAL)
    void setEnabled(size_t i, bool on) { if (i < th_.size()) th_[i].enabled = on; }   // участие движителя в распределении (взятие под управление ДП)

    Capability capability(double powerScale = 1.0) const {
        Capability c;
        for (const auto& t : th_) {
            if (!t.enabled) continue;
            const double mx = std::max(t.maxForward, t.maxReverse) * powerScale * limitFrac(t);
            if (t.type == ThrusterType::Azimuth) {
                c.X += mx; c.Y += mx; c.N += mx * std::hypot(t.x, t.y);
            } else {
                const double a = thrusterBaseAngle(t);
                c.X += mx * std::fabs(std::cos(a));
                c.Y += mx * std::fabs(std::sin(a));
                c.N += mx * std::fabs(t.x * std::sin(a) - t.y * std::cos(a));
            }
        }
        c.X *= 0.8; c.Y *= 0.8; c.N *= 0.8;
        return c;
    }

    // tau — требуемые усилия; actualAngleDeg — фактические углы азимутальных движителей (по порядку в списке th_);
    // powerScale — доля доступной мощности (1 — без ограничения).
    AllocResult allocate(const Wrench& tau, const std::vector<double>& actualAngleDeg, double powerScale = 1.0) const {
        bool hasEff = false; for (const auto& t : th_) hasEff = hasEff || (t.type == ThrusterType::Azimuth && !t.effAngle.empty());
        if (!hasEff) return allocateSoft(tau, actualAngleDeg, powerScale);
        // КПД по углу: итерации — угол из прошлого результата задаёт КПД, он уменьшает предел колонки; команда упора делится на КПД
        std::vector<double> eff(th_.size(), 1.0);
        AllocResult r = allocateSoft(tau, actualAngleDeg, powerScale);
        for (int it = 0; it < 3; ++it) {
            for (size_t i = 0; i < th_.size() && i < r.cmds.size(); ++i) {
                if (th_[i].type != ThrusterType::Azimuth || th_[i].effAngle.empty()) continue;
                const double act = i < actualAngleDeg.size() ? actualAngleDeg[i] : r.cmds[i].angleDeg;
                const double err = std::fabs(rad2deg(wrapPi(deg2rad(r.cmds[i].angleDeg - act))));
                eff[i] = std::max(0.05, effAt(th_[i], err > 15.0 ? act : r.cmds[i].angleDeg));
            }
            Allocator tmp = *this;
            for (size_t i = 0; i < th_.size(); ++i) if (eff[i] < 1.0) tmp.th_[i].speedFactor = clampd(th_[i].speedFactor, 0.0, 1.0) * eff[i];
            r = tmp.allocateSoft(tau, actualAngleDeg, powerScale);
        }
        for (size_t i = 0; i < th_.size() && i < r.cmds.size(); ++i)
            if (!th_[i].effIsLimit && eff[i] < 1.0) r.cmds[i].pct = clampd(r.cmds[i].pct / eff[i], -100.0, 100.0);   // физический упор = команда·КПД
        return r;
    }

private:
    AllocResult allocateSoft(const Wrench& tau, const std::vector<double>& actualAngleDeg, double powerScale) const {
        std::vector<char> snapped;
        AllocResult r = allocateImpl(tau, actualAngleDeg, powerScale, nullptr, &snapped);
        bool any = false; for (char c : snapped) any = any || c;
        if (!any) return r;
        // недодача усилия: если колонка отодвинута из нежелательной части сектора и из-за этого запрос не выполняется — разрешаем остаться в ней
        auto miss = [&](const AllocResult& a) { return std::hypot(tau.X - a.achieved.X, tau.Y - a.achieved.Y) + 0.1 * std::fabs(tau.N - a.achieved.N); };
        const double tn = std::hypot(tau.X, tau.Y) + 0.1 * std::fabs(tau.N) + 1.0;
        if (miss(r) < 0.05 * tn) return r;
        std::vector<char> none;
        AllocResult r2 = allocateImpl(tau, actualAngleDeg, powerScale, &snapped, &none);
        if (miss(r2) < 0.9 * miss(r)) {
            for (size_t i = 0; i < th_.size() && i < snapped.size(); ++i) if (snapped[i]) r2.softZone.push_back(th_[i].id);
            return r2;
        }
        return r;
    }

private:
    // soft — колонки, которым разрешено остаться в нежелательной (не жёсткой) части сектора; snapped — кого отодвинули из такой части
    AllocResult allocateImpl(const Wrench& tau, const std::vector<double>& actualAngleDeg, double powerScale, const std::vector<char>* soft, std::vector<char>* snapped) const {
        const size_t nt = th_.size();
        if (snapped) snapped->assign(nt, 0);
        auto allowedAngle = [&](size_t i, double a) {
            if (!angleForbidden(th_[i], a)) return a;
            if (angleHardForbidden(th_[i], a)) return nearestAllowedAngle(th_[i], a);
            if (soft && (*soft)[i]) return a;
            if (snapped) (*snapped)[i] = 1;
            return nearestAllowedAngle(th_[i], a);
        };
        std::vector<int> mode(nt, 0);                 // 0 — свободный азимут (2 переменные), 1 — движение вдоль линии
        std::vector<double> lineAngle(nt, 0.0);       // угол линии для режима 1, рад
        std::vector<double> desiredDeg(nt, 0.0);
        bool turning = false;

        // проход 1: все азимутальные свободны
        Sol s1 = solveScaled(tau, mode, lineAngle, powerScale);
        for (size_t i = 0; i < nt; ++i) {
            const auto& t = th_[i];
            if (!t.enabled || t.type != ThrusterType::Azimuth) continue;
            const double act = i < actualAngleDeg.size() ? actualAngleDeg[i] : 0.0;
            const double fx = s1.fx[i], fy = s1.fy[i];
            double des = (std::hypot(fx, fy) > 1e-3 * std::max(1.0, t.maxForward)) ? rad2deg(std::atan2(fy, fx)) : act;
            const double allowed = allowedAngle(i, des);
            desiredDeg[i] = allowed;
            const double err = std::fabs(rad2deg(wrapPi(deg2rad(allowed - act))));
            if (err > 15.0) {                         // колонка ещё не повернулась — упор по текущему направлению
                mode[i] = 1; lineAngle[i] = deg2rad(act); turning = true;
            } else if (std::fabs(allowed - des) > 0.5) {  // запретный сектор — ограничиваем линией
                mode[i] = 1; lineAngle[i] = deg2rad(allowed);
            }
        }
        // проход 2
        Sol s2 = solveScaled(tau, mode, lineAngle, powerScale);

        AllocResult res;
        res.achieved = s2.achieved;
        res.saturated = s2.saturated;
        res.turning = turning;
        for (size_t i = 0; i < nt; ++i) {
            const auto& t = th_[i];
            ThrusterCmd c;
            c.id = t.id;
            if (!t.enabled) { res.cmds.push_back(c); continue; }
            const double mf = t.maxForward * powerScale * limitFrac(t), mr = t.maxReverse * powerScale * limitFrac(t);
            if (t.type == ThrusterType::Azimuth) {
                if (mode[i] == 0) {
                    const double F = std::hypot(s2.fx[i], s2.fy[i]);
                    c.pct = mf > 0 ? 100.0 * F / t.maxForward : 0.0;
                    c.angleDeg = F > 1e-3 * std::max(1.0, t.maxForward) ? rad2deg(std::atan2(s2.fy[i], s2.fx[i])) : desiredDeg[i];
                    c.angleDeg = allowedAngle(i, c.angleDeg);
                    res.utilization = std::max(res.utilization, F / std::max(1.0, mf));
                } else {
                    const double f = s2.line[i];
                    c.pct = f >= 0 ? 100.0 * f / t.maxForward : -100.0 * (-f) / std::max(1.0, t.maxReverse);
                    c.angleDeg = desiredDeg[i];
                    res.utilization = std::max(res.utilization, f >= 0 ? f / std::max(1.0, mf) : -f / std::max(1.0, mr));
                }
            } else {
                const double f = s2.line[i];
                c.pct = f >= 0 ? 100.0 * f / std::max(1.0, t.maxForward) : 100.0 * f / std::max(1.0, t.maxReverse);
                c.angleDeg = rad2deg(thrusterBaseAngle(t));
                res.utilization = std::max(res.utilization, f >= 0 ? f / std::max(1.0, mf) : -f / std::max(1.0, mr));
            }
            res.cmds.push_back(c);
        }
        return res;
    }

    struct Sol {
        std::vector<double> fx, fy, line;   // для свободных азимутальных — fx,fy; для остальных — line
        Wrench achieved;
        bool saturated = false;
    };

    struct Var {            // одна переменная оптимизации
        size_t th;          // индекс движителя
        int kind;           // 0 — линия (одна переменная), 1 — fx, 2 — fy
        double bx, by, bn;  // столбец матрицы эффективности
        double lo, hi;      // пределы (для kind 0)
        double w;           // вес
        double mag;         // предел модуля (для пары fx,fy)
        double f0 = 0.0;    // предпочтительное значение (предварительный упор): решение стремится к нему
    };

    std::vector<Var> buildVars(const std::vector<int>& mode, const std::vector<double>& lineAngle, double ps) const {
        std::vector<Var> v;
        for (size_t i = 0; i < th_.size(); ++i) {
            const auto& t = th_[i];
            if (!t.enabled) continue;
            const double mf = std::max(1.0, t.maxForward * ps * limitFrac(t)), mr = std::max(1.0, t.maxReverse * ps * limitFrac(t));
            const double w = t.weight / (mf * mf);
            if (t.type == ThrusterType::Azimuth && mode[i] == 0) {
                v.push_back({i, 1, 1.0, 0.0, -t.y, 0, 0, w, mf});
                v.push_back({i, 2, 0.0, 1.0, t.x, 0, 0, w, mf});
                if (t.prefDeg < 1e8 && t.preloadPct > 0.0) {      // предварительный упор вдоль предпочтительного угла
                    const double P = clampd(t.preloadPct, 0.0, 50.0) / 100.0 * mf;
                    v[v.size() - 2].f0 = P * std::cos(deg2rad(t.prefDeg));
                    v[v.size() - 1].f0 = P * std::sin(deg2rad(t.prefDeg));
                }
            } else {
                const double a = t.type == ThrusterType::Azimuth ? lineAngle[i] : thrusterBaseAngle(t);
                v.push_back({i, 0, std::cos(a), std::sin(a), t.x * std::sin(a) - t.y * std::cos(a), -mr, mf, w, mf});
            }
        }
        return v;
    }

    // Взвешенное решение методом наименьших квадратов с учётом пределов (метод активных ограничений).
    bool solveOnce(const Wrench& tau, const std::vector<Var>& vars, std::vector<double>& f) const {
        const size_t nv = vars.size();
        f.assign(nv, 0.0);
        std::vector<char> fixedv(nv, 0);
        for (size_t iter = 0; iter < nv + 3; ++iter) {
            std::vector<size_t> fr;
            for (size_t k = 0; k < nv; ++k) if (!fixedv[k]) fr.push_back(k);
            double rx = tau.X, ry = tau.Y, rn = tau.N;
            for (size_t k = 0; k < nv; ++k) {
                const double fk = fixedv[k] ? f[k] : vars[k].f0;      // свободные переменные стремятся к предварительному упору
                rx -= vars[k].bx * fk; ry -= vars[k].by * fk; rn -= vars[k].bn * fk;
            }
            if (fr.empty()) break;
            Mx A(3, 3);
            for (size_t k : fr) {
                const double bx[3] = {vars[k].bx, vars[k].by, vars[k].bn};
                for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) A(i, j) += bx[i] * bx[j] / vars[k].w;
            }
            // нормировка для регуляризации
            double sc = 0; for (int i = 0; i < 3; ++i) sc = std::max(sc, A(i, i));
            for (int i = 0; i < 3; ++i) A(i, i) += 1e-9 * (sc > 0 ? sc : 1.0);
            Mx rhs(3, 1); rhs(0, 0) = rx; rhs(1, 0) = ry; rhs(2, 0) = rn;
            Mx lam;
            if (!solveLinear(A, rhs, lam)) return false;
            for (size_t k : fr) f[k] = vars[k].f0 + (vars[k].bx * lam(0, 0) + vars[k].by * lam(1, 0) + vars[k].bn * lam(2, 0)) / vars[k].w;

            // проверка пределов
            double worst = 1.0; int wk = -1; bool pair = false; size_t wk2 = 0;
            for (size_t k : fr) {
                const Var& v = vars[k];
                if (v.kind == 0) {
                    const double r = f[k] > 0 ? f[k] / v.hi : (v.lo < 0 ? f[k] / v.lo : 0.0);
                    if (r > worst + 1e-9) { worst = r; wk = static_cast<int>(k); pair = false; }
                } else if (v.kind == 1 && k + 1 < nv && !fixedv[k + 1] && vars[k + 1].th == v.th) {
                    const double r = std::hypot(f[k], f[k + 1]) / v.mag;
                    if (r > worst + 1e-9) { worst = r; wk = static_cast<int>(k); pair = true; wk2 = k + 1; }
                }
            }
            if (wk < 0) return true;
            const size_t k = static_cast<size_t>(wk);
            if (pair) {
                const double F = std::hypot(f[k], f[wk2]);
                f[k] *= vars[k].mag / F; f[wk2] *= vars[k].mag / F;
                fixedv[k] = fixedv[wk2] = 1;
            } else {
                f[k] = f[k] > 0 ? vars[k].hi : vars[k].lo;
                fixedv[k] = 1;
            }
        }
        return true;
    }

    Sol evaluate(const Wrench& tau, const std::vector<Var>& vars, const std::vector<double>& f) const {
        Sol s;
        s.fx.assign(th_.size(), 0.0); s.fy.assign(th_.size(), 0.0); s.line.assign(th_.size(), 0.0);
        double X = 0, Y = 0, N = 0;
        for (size_t k = 0; k < vars.size(); ++k) {
            const Var& v = vars[k];
            X += v.bx * f[k]; Y += v.by * f[k]; N += v.bn * f[k];
            if (v.kind == 0) s.line[v.th] = f[k];
            else if (v.kind == 1) s.fx[v.th] = f[k];
            else s.fy[v.th] = f[k];
        }
        s.achieved = {X, Y, N};
        const double scale = std::max({std::fabs(tau.X), std::fabs(tau.Y), std::fabs(tau.N), 1.0});
        s.saturated = std::fabs(X - tau.X) > 0.01 * scale || std::fabs(Y - tau.Y) > 0.01 * scale || std::fabs(N - tau.N) > 0.01 * scale;
        return s;
    }

    // Если требуемое недостижимо — уменьшаем продольную и поперечную силы, сохраняя момент (приоритет курса).
    Sol solveScaled(const Wrench& tau, const std::vector<int>& mode, const std::vector<double>& lineAngle, double ps) const {
        const auto vars = buildVars(mode, lineAngle, ps);
        std::vector<double> f;
        if (vars.empty()) { Sol s; s.fx.assign(th_.size(), 0); s.fy.assign(th_.size(), 0); s.line.assign(th_.size(), 0); return s; }
        solveOnce(tau, vars, f);
        Sol best = evaluate(tau, vars, f);
        if (!best.saturated) return best;
        double lo = 0.0, hi = 1.0;
        std::vector<double> fl;
        solveOnce({0, 0, tau.N}, vars, fl);
        Sol sl = evaluate({0, 0, tau.N}, vars, fl);
        Sol okSol = sl; std::vector<double> okF = fl;
        for (int it = 0; it < 14; ++it) {
            const double mid = 0.5 * (lo + hi);
            const Wrench t2{tau.X * mid, tau.Y * mid, tau.N};
            std::vector<double> fm;
            solveOnce(t2, vars, fm);
            Sol sm = evaluate(t2, vars, fm);
            if (!sm.saturated) { lo = mid; okSol = sm; okF = fm; } else hi = mid;
        }
        okSol.saturated = true;
        return okSol;
    }

    std::vector<ThrusterCfg> th_;
};

// ---------------------------------------------------------------- отклик движителя (имитатор)
class ThrusterSim {
public:
    explicit ThrusterSim(const ThrusterCfg& c) : cfg_(c) { angleDeg_ = c.type == ThrusterType::Azimuth ? 0.0 : rad2deg(thrusterBaseAngle(c)); }
    void command(const ThrusterCmd& c) { cmd_ = c; }
    void setSpeedKn(double kn) { cfg_.speedFactor = speedFactorFor(cfg_, kn); }
    void step(double dt, double powerScale = 1.0) {
        if (!cfg_.enabled || failed_) { thrustN_ = 0; return; }
        // угол
        if (cfg_.type == ThrusterType::Azimuth) {
            const double err = rad2deg(wrapPi(deg2rad(cmd_.angleDeg - angleDeg_)));
            const double step = clampd(err, -cfg_.azRateDegS * dt, cfg_.azRateDegS * dt);
            angleDeg_ = rad2deg(wrapPi(deg2rad(angleDeg_ + step)));
        }
        // упор
        const double pct = clampd(cmd_.pct, -100.0, 100.0);
        const double mf = cfg_.maxForward * powerScale * limitFrac(cfg_), mr = cfg_.maxReverse * powerScale * limitFrac(cfg_);
        double target = pct >= 0 ? pct / 100.0 * cfg_.maxForward : pct / 100.0 * cfg_.maxReverse;
        target = clampd(target, -mr, mf);
        const double lag = dt / std::max(dt, cfg_.timeConstS);
        double want = thrustN_ + lag * (target - thrustN_);
        const double maxStep = cfg_.rateFracPerS * std::max(cfg_.maxForward, cfg_.maxReverse) * dt;
        thrustN_ += clampd(want - thrustN_, -maxStep, maxStep);
    }
    double thrustN() const { return thrustN_; }
    double angleDeg() const { return angleDeg_; }
    double pct() const { return thrustN_ >= 0 ? 100.0 * thrustN_ / std::max(1.0, cfg_.maxForward) : 100.0 * thrustN_ / std::max(1.0, cfg_.maxReverse); }
    Wrench wrench() const {
        const double a = cfg_.type == ThrusterType::Azimuth ? deg2rad(angleDeg_) : thrusterBaseAngle(cfg_);
        return thrusterWrench(cfg_, thrustN_, a);
    }
    const ThrusterCfg& cfg() const { return cfg_; }
    void setFailed(bool f) { failed_ = f; }   // имитация отказа: упор исчезает, обратная связь недостоверна
    bool failed() const { return failed_; }
private:
    ThrusterCfg cfg_;
    bool failed_ = false;
    ThrusterCmd cmd_;
    double thrustN_ = 0, angleDeg_ = 0;
};

}  // namespace dp
