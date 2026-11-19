// Замкнутые испытания ядра СДП в имитаторе.
#include <cstdio>
#include <set>
#include "dp/sim.hpp"

using namespace dp;

struct Result { double rmsPos = 0, maxPos = 0, rmsHdg = 0, maxHdg = 0, meanUtil = 0; };

static Result run(LawType law, const EnvCfg& env, double T, double settle, bool failThr = false, bool jumpGnss = false,
                  DpMode mode = DpMode::AutoPosition, const char* tag = "") {
    VesselCfg v = demoVessel();
    auto th = demoThrusters();
    ControlCfg cc; cc.law = law;
    DpEngine eng(v, th, cc);
    Simulator sim(v, th, env);
    sim.dyn().state().psi = deg2rad(10);
    const double dt = 0.1;
    // прогрев: накопление данных датчиков
    for (int i = 0; i < 100; ++i) { sim.step(dt); auto fr = sim.measure(); eng.step(dt, fr); }
    eng.setMode(mode);
    eng.controller().setSetpoint({sim.truth().n, sim.truth().e, 10.0});
    Result r; int cnt = 0; double ut = 0;
    std::set<std::string> alarms;
    for (double t = 0; t < T; t += dt) {
        if (failThr && std::fabs(t - T * 0.5) < dt / 2) sim.failThruster(0);
        if (jumpGnss && std::fabs(t - T * 0.5) < dt / 2) sim.jumpSensor("GNSS1", 15, -10);
        if (jumpGnss && std::fabs(t - T * 0.5 - 60) < dt / 2) sim.clearJump("GNSS1");
        sim.step(dt);
        auto fr = sim.measure();
        auto o = eng.step(dt, fr);
        sim.applyCommands(o.cmds);
        for (auto& a : o.alarms) alarms.insert(a.code);
        if (t >= settle) {
            const auto& s = sim.truth();
            const double ep = std::hypot(s.n - o.ctl.refN, s.e - o.ctl.refE);
            const double eh = std::fabs(rad2deg(wrapPi(s.psi - deg2rad(o.ctl.refPsiDeg))));
            r.rmsPos += ep * ep; r.rmsHdg += eh * eh; r.maxPos = std::max(r.maxPos, ep); r.maxHdg = std::max(r.maxHdg, eh);
            ut += o.alloc.utilization; ++cnt;
        }
    }
    r.rmsPos = std::sqrt(r.rmsPos / cnt); r.rmsHdg = std::sqrt(r.rmsHdg / cnt); r.meanUtil = ut / cnt;
    std::printf("  %-22s rmsPos=%.2f maxPos=%.2f rmsHdg=%.2f maxHdg=%.2f util=%.2f alarms:", tag, r.rmsPos, r.maxPos, r.rmsHdg, r.maxHdg, r.meanUtil);
    for (auto& a : alarms) std::printf(" %s", a.c_str());
    std::printf("\n");
    return r;
}

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { std::printf("  [ПРОВАЛ] %s\n", msg); ++fails; } else std::printf("  [ок] %s\n", msg); } while (0)


// Смена уставки: перемещение на 40 м и поворот на 60°, затем прохождение по треку.
static void manoeuvres(LawType law, const char* name, const EnvCfg& env) {
    VesselCfg v = demoVessel(); auto th = demoThrusters();
    ControlCfg cc; cc.law = law;
    DpEngine eng(v, th, cc); Simulator sim(v, th, env);
    const double dt = 0.1;
    for (int i = 0; i < 100; ++i) { sim.step(dt); eng.step(dt, sim.measure()); }
    eng.setMode(DpMode::AutoPosition);
    const double n0 = sim.truth().n, e0 = sim.truth().e;
    eng.controller().setSetpoint({n0 + 40, e0 + 20, 60.0});
    double maxUtil = 0, overshoot = 0;
    const double tgtDist = std::hypot(40, 20);
    for (double t = 0; t < 1500; t += dt) {
        sim.step(dt); auto o = eng.step(dt, sim.measure()); sim.applyCommands(o.cmds);
        maxUtil = std::max(maxUtil, o.alloc.utilization);
        const double d = std::hypot(sim.truth().n - n0, sim.truth().e - e0);
        overshoot = std::max(overshoot, d - tgtDist);
    }
    const double ep = std::hypot(sim.truth().n - (n0 + 40), sim.truth().e - (e0 + 20));
    const double eh = std::fabs(rad2deg(wrapPi(sim.truth().psi - deg2rad(60))));
    std::printf("  %-4s смена уставки: итоговая ошибка=%.2f м, по курсу=%.2f°, перерегулирование=%.2f м, макс. загрузка=%.2f\n", name, ep, eh, overshoot, maxUtil);
    CHECK(ep < 1.0 && eh < 1.0, "приход в новую точку");
    // трек
    eng.controller().setTrack({{{n0 + 40, e0 + 20}, {n0 + 120, e0 + 20}, {n0 + 120, e0 + 100}}, 1.0, false, 8.0});
    eng.setMode(DpMode::Track);
    eng.controller().setSetpoint({n0 + 40, e0 + 20, 60.0});
    double maxCross = 0;
    for (double t = 0; t < 700; t += dt) {
        sim.step(dt); auto o = eng.step(dt, sim.measure()); sim.applyCommands(o.cmds);
        if (t > 60) maxCross = std::max(maxCross, o.ctl.errDist);
    }
    const double ef = std::hypot(sim.truth().n - (n0 + 120), sim.truth().e - (e0 + 100));
    std::printf("  %-4s трек: макс. отклонение от опорной точки=%.2f м, расстояние до конечной точки=%.2f м\n", name, maxCross, ef);
    CHECK(ef < 3.0, "трек пройден до конечной точки");
}

int main() {
    EnvCfg calm; calm.windSpeed = 8; calm.windFromDeg = 60; calm.currentSpeed = 0.5; calm.currentToDeg = 200; calm.hs = 1.0; calm.tp = 7; calm.waveFromDeg = 60;
    EnvCfg storm = calm; storm.windSpeed = 15; storm.currentSpeed = 0.8; storm.hs = 2.5; storm.tp = 9;
    const char* names[3] = {"ПИД", "LQR", "MPC"};
    const LawType laws[3] = {LawType::Pid, LawType::Lqr, LawType::Mpc};
    for (int i = 0; i < 3; ++i) {
        std::printf("Закон: %s\n", names[i]);
        auto r1 = run(laws[i], calm, 900, 400, false, false, DpMode::AutoPosition, "умеренная погода");
        CHECK(r1.maxPos < 6.0 && r1.maxHdg < 3.0, "удержание в умеренной погоде");
        auto r2 = run(laws[i], storm, 900, 400, false, false, DpMode::AutoPosition, "сильная погода");
        CHECK(r2.maxPos < 12.0 && r2.maxHdg < 6.0, "удержание в сильной погоде");
    }
    std::printf("Отказ движителя (ПИД)\n");
    auto rf = run(LawType::Pid, calm, 900, 600, true, false, DpMode::AutoPosition, "отказ AZ1");
    CHECK(rf.maxPos < 15.0, "удержание после отказа движителя");
    std::printf("Скачок GNSS (ПИД)\n");
    auto rj = run(LawType::Pid, calm, 900, 400, false, true, DpMode::AutoPosition, "скачок GNSS1");
    CHECK(rj.maxPos < 8.0, "отбраковка скачка одного приёмника");
    std::printf("Удержание курса (ПИД)\n");
    auto rh = run(LawType::Pid, calm, 600, 300, false, false, DpMode::AutoHeading, "курс");
    CHECK(rh.maxHdg < 3.0, "удержание курса");
    std::printf("Манёвры\n");
    for (int i = 0; i < 3; ++i) manoeuvres(laws[i], names[i], calm);
    std::printf(fails ? "ИТОГ: есть провалы (%d)\n" : "ИТОГ: всё в порядке\n", fails);
    return fails ? 1 : 0;
}
