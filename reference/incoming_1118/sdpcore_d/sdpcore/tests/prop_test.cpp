// Усиление (High/Low gain), минимальная мощность (Minpower) и гистерезис тревоги HIGH_LOAD.
#include <cstdio>
#include <cmath>
#include <vector>
#include "dp/sim.hpp"

using namespace dp;
static int fails = 0;
#define CHECK(c, msg) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", msg); if (!ok_) ++fails; } while (0)

struct R { double maxPct = 0, rmsPos = 0, meanPct = 0, activity = 0; int alarmEdges = 0; };

static R run(bool high, bool minp, double wind, double T = 500) {
    VesselCfg v = demoVessel(); auto th = demoThrusters();
    ControlCfg cc; cc.highGain = high; cc.minPower = minp;
    EnvCfg env; env.windSpeed = wind; env.windFromDeg = 60; env.currentSpeed = 0.5; env.currentToDeg = 200; env.hs = 1.5; env.tp = 7; env.waveFromDeg = 60;
    DpEngine eng(v, th, cc); Simulator sim(v, th, env);
    sim.dyn().state().psi = deg2rad(10);
    const double dt = 0.1;
    for (int i = 0; i < 100; ++i) { sim.step(dt); eng.step(dt, sim.measure()); }
    eng.setMode(DpMode::AutoPosition);
    eng.controller().setSetpoint({sim.truth().n, sim.truth().e, 10.0});
    R r; bool prev = false; int cnt = 0; std::vector<double> last;
    for (double t = 0; t < T; t += dt) {
        sim.step(dt); auto o = eng.step(dt, sim.measure()); sim.applyCommands(o.cmds);
        bool hi = false; for (auto& a : o.alarms) if (a.code == "HIGH_LOAD") hi = true;
        if (hi != prev) ++r.alarmEdges;
        prev = hi;
        if (last.size() != o.cmds.size()) last.assign(o.cmds.size(), 0.0);
        if (t > 150) {
            double s = 0, act = 0;
            for (size_t i = 0; i < o.cmds.size(); ++i) { r.maxPct = std::max(r.maxPct, std::fabs(o.cmds[i].pct)); s += std::fabs(o.cmds[i].pct); act += std::fabs(o.cmds[i].pct - last[i]); }
            r.meanPct += s / o.cmds.size(); r.activity += act / o.cmds.size();
            const auto& st = sim.truth(); r.rmsPos += std::pow(std::hypot(st.n - o.ctl.refN, st.e - o.ctl.refE), 2); ++cnt;
        }
        for (size_t i = 0; i < o.cmds.size(); ++i) last[i] = o.cmds[i].pct;
    }
    r.rmsPos = std::sqrt(r.rmsPos / std::max(1, cnt)); r.meanPct /= std::max(1, cnt); r.activity /= std::max(1, cnt);
    return r;
}

int main() {
    std::printf("== Усиление и минимальная мощность ==\n");
    for (double w : {8.0, 16.0, 30.0}) {
        const R lo = run(false, false, w), hi = run(true, false, w), mp = run(false, true, w);
        std::printf("   ветер %.0f м/с: низкое усиление СКО %.2f м, активность %.3f; высокое СКО %.2f м, активность %.3f; мин. мощность СКО %.2f м, активность %.3f; смен тревоги: %d / %d / %d\n",
                    w, lo.rmsPos, lo.activity, hi.rmsPos, hi.activity, mp.rmsPos, mp.activity, lo.alarmEdges, hi.alarmEdges, mp.alarmEdges);
        CHECK(hi.rmsPos <= lo.rmsPos + 0.05, "высокое усиление держит не хуже низкого");
        CHECK(mp.activity <= lo.activity + 1e-9, "минимальная мощность: работа движителей не выше обычной");
        CHECK(lo.alarmEdges <= 4 && hi.alarmEdges <= 4 && mp.alarmEdges <= 4, "тревога высокой загрузки не дребезжит (не больше 2 включений за 500 с)");
    }
    { const R lo = run(false, false, 8.0), hi = run(true, false, 8.0); CHECK(hi.activity >= lo.activity - 1e-9, "высокое усиление: движители работают активнее, чем при низком"); }
    // команды не ограничены режимом: при сильном ветре упор может превышать 80 %
    { const R s = run(false, false, 36.0); CHECK(s.maxPct > 80.5, "сильный ветер: упор не ограничен 80 %"); }
    std::printf(fails ? "ИТОГ: ошибок %d\n" : "ИТОГ: всё в порядке\n", fails);
    return fails ? 1 : 0;
}
