// Поворот вокруг точки судна: выбранная точка остаётся на месте на всём протяжении поворота.
#include <cstdio>
#include <cmath>
#include <algorithm>
#include "dp/sim.hpp"

using namespace dp;
static int fails = 0;
#define CHECK(c, msg) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", msg); if (!ok_) ++fails; } while (0)

// Возвращает наибольшее смещение точки (px, py) судна от начального положения за время поворота и после него.
// usePivot — центр вращения задан в ядре (удерживается эта точка); иначе удерживается точка управления (центр судна).
static void run(bool usePivot, double px, double py, double turn, double& maxDev, double& endDev, double& endHdgErr) {
    VesselCfg v = demoVessel(); auto th = demoThrusters();
    ControlCfg cc; EnvCfg env; env.windSpeed = 12.5; env.windFromDeg = 45; env.hs = 1.4; env.tp = 7; env.waveFromDeg = 60; env.currentSpeed = 0.3; env.currentToDeg = 200;
    DpEngine eng(v, th, cc); Simulator sim(v, th, env);
    const double dt = 0.1;
    for (int i = 0; i < 100; ++i) { sim.step(dt); eng.step(dt, sim.measure()); }
    if (usePivot) eng.controller().setPivotIdle(px, py);
    eng.setMode(DpMode::AutoPosition);
    for (double t = 0; t < 120; t += dt) { sim.step(dt); auto o = eng.step(dt, sim.measure()); sim.applyCommands(o.cmds); }
    auto pt = [&](double& n, double& e) { const auto& s = sim.truth(); n = s.n + px * std::cos(s.psi) - py * std::sin(s.psi); e = s.e + px * std::sin(s.psi) + py * std::cos(s.psi); };
    double n0, e0; pt(n0, e0);
    const double h0 = eng.controller().setpoint().psiDeg;
    Setpoint sp = eng.controller().setpoint(); sp.psiDeg = h0 + turn; eng.controller().setSetpoint(sp);     // только курс, положение уставки то же
    maxDev = 0;
    for (double t = 0; t < 400; t += dt) {
        sim.step(dt); auto o = eng.step(dt, sim.measure()); sim.applyCommands(o.cmds);
        double n, e; pt(n, e); maxDev = std::max(maxDev, std::hypot(n - n0, e - e0));
    }
    double n, e; pt(n, e); endDev = std::hypot(n - n0, e - e0);
    endHdgErr = std::fabs(rad2deg(wrapPi(sim.truth().psi - deg2rad(h0 + turn))));
}

int main() {
    std::printf("== Поворот вокруг точки судна ==\n");
    for (double px : {30.0, -25.0}) {
        double m0, e0, h0, m1, e1, h1;
        run(false, px, 0, 30, m0, e0, h0); run(true, px, 0, 30, m1, e1, h1);
        std::printf("   точка %.0f м, поворот на 30°: вокруг центра судна — наибольшее смещение точки %.1f м; вокруг этой точки — %.1f м (итог %.1f м, ошибка курса %.1f°)\n", px, m0, m1, e1, h1);
        CHECK(h1 < 1.0, "курс достигнут");
        CHECK(m0 > 10.0, "без центра вращения точка уходит далеко (контроль теста)");
        CHECK(m1 < 4.0 && m1 < 0.3 * m0, "с центром вращения точка почти не смещается");
    }
    { // смена центра вращения на ходу: судно остаётся на месте
        VesselCfg v = demoVessel(); auto th = demoThrusters(); ControlCfg cc; EnvCfg env;
        DpEngine eng(v, th, cc); Simulator sim(v, th, env);
        for (int i = 0; i < 100; ++i) { sim.step(0.1); eng.step(0.1, sim.measure()); }
        eng.setMode(DpMode::AutoPosition);
        for (double t = 0; t < 40; t += 0.1) { sim.step(0.1); auto o = eng.step(0.1, sim.measure()); sim.applyCommands(o.cmds); }
        const Setpoint before = eng.controller().crpFromHold(eng.controller().setpoint());
        eng.controller().setPivot(25.0, 0.0);
        const Setpoint after = eng.controller().crpFromHold(eng.controller().setpoint());
        CHECK(std::hypot(before.n - after.n, before.e - after.e) < 1e-6, "смена центра вращения не сдвигает судно (уставка точки управления прежняя)");
        CHECK(std::hypot(eng.controller().setpoint().n - before.n, eng.controller().setpoint().e - before.e) > 20.0, "точка удержания переехала на нос");
    }
    std::printf(fails ? "ИТОГ: ошибок %d\n" : "ИТОГ: всё в порядке\n", fails);
    return fails ? 1 : 0;
}
