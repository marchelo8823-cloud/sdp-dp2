// Проверка оценки волнения по трём МИО: высота, период, направление, голосование приборов и упреждение по дрейфовой силе.
#include <cstdio>
#include "dp/sim.hpp"

using namespace dp;

static int fails = 0;
#define CHECK(c, msg) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", msg); if (!ok_) ++fails; } while (0)

struct Out { double maxAfterTurn = 0; WaveEstimate w; int mruCount = 0; bool disagree = false, mruValid = false; double rmsPos = 0, meanTiltErr = 0; std::set<std::string> alarms; Wrench waveFF; };

static Out run(double hs, double tp, double fromDeg, double T, int failAt = -1, int biasAt = -1, bool waveFF = true, double turnAt = -1) {
    VesselCfg v = demoVessel(); auto th = demoThrusters();
    ControlCfg cc; cc.waveFeedForward = waveFF;
    EnvCfg env; env.windSpeed = 8; env.windFromDeg = 120; env.hs = hs; env.tp = tp; env.waveFromDeg = fromDeg;
    DpEngine eng(v, th, cc); Simulator sim(v, th, env);
    sim.dyn().state().psi = deg2rad(10);
    const double dt = 0.1;
    for (int i = 0; i < 100; ++i) { sim.step(dt); eng.step(dt, sim.measure()); }
    eng.setMode(DpMode::AutoPosition);
    eng.controller().setSetpoint({sim.truth().n, sim.truth().e, 10.0});
    Out r; int cnt = 0;
    for (double t = 0; t < T; t += dt) {
        if (failAt >= 0 && std::fabs(t - failAt) < dt / 2) sim.failSensor("MRU2", true);
        if (biasAt >= 0 && std::fabs(t - biasAt) < dt / 2) sim.biasMru("MRU3", deg2rad(3.0), 0, 0);
        if (turnAt >= 0 && std::fabs(t - turnAt) < dt / 2) { Setpoint sp = eng.controller().setpoint(); sp.psiDeg = 100.0; eng.controller().setSetpoint(sp); }
        sim.step(dt); auto fr = sim.measure(); auto o = eng.step(dt, fr); sim.applyCommands(o.cmds);
        if (turnAt >= 0 && t > turnAt) { const auto& s = sim.truth(); r.maxAfterTurn = std::max(r.maxAfterTurn, std::hypot(s.n - o.ctl.refN, s.e - o.ctl.refE)); }
        for (auto& a : o.alarms) r.alarms.insert(a.code);
        r.w = o.est.wave; r.mruCount = o.est.mruCount; r.disagree = o.est.mruDisagree; r.mruValid = o.est.mruValid; r.waveFF = o.waveModel;
        if (t > 200) { const auto& s = sim.truth(); r.rmsPos += std::pow(std::hypot(s.n - o.ctl.refN, s.e - o.ctl.refE), 2); r.meanTiltErr += std::fabs(rad2deg(s.phi - o.est.roll)); ++cnt; }
    }
    r.rmsPos = std::sqrt(r.rmsPos / std::max(1, cnt)); r.meanTiltErr /= std::max(1, cnt);
    return r;
}

static double angDiff(double a, double b) { return std::fabs(std::fmod(a - b + 540.0, 360.0) - 180.0); }

int main() {
    std::printf("== Оценка волн по МИО ==\n");
    for (double from : {60.0, 200.0, 320.0}) {
        Out r = run(2.0, 8.0, from, 600);
        std::printf("   истина: Hs 2.0 м, Tp 8 с, откуда %.0f°; оценка: Hs %.2f, Tz %.1f, Tp %.1f, откуда %.0f°, уверенность %.2f, окно %.0f с, доля упреждения %.2f\n",
                    from, r.w.hs, r.w.tz, r.w.tp, r.w.fromDeg, r.w.conf, r.w.seconds, r.w.ffGain);
        char b[160];
        std::snprintf(b, sizeof b, "направление %.0f°: оценка в пределах 25°", from); CHECK(r.w.dirValid && angDiff(r.w.fromDeg, from) < 25.0, b);
        std::snprintf(b, sizeof b, "направление %.0f°: высота в пределах ±30 %%", from); CHECK(r.w.valid && std::fabs(r.w.hs - 2.0) < 0.6, b);
    }
    { Out r = run(0.0, 8.0, 60.0, 240); CHECK(r.w.valid && r.w.hs < 0.3 && !r.w.dirValid, "штиль: высота мала, направление не выдумывается"); }
    { Out r = run(2.0, 8.0, 60.0, 600, 250); CHECK(r.mruCount == 2 && r.w.dirValid && angDiff(r.w.fromDeg, 60) < 25, "отказ MRU2: остаётся два прибора, оценка сохраняется"); }
    { Out r = run(2.0, 8.0, 60.0, 600, -1, 250);
      CHECK(r.mruCount == 2 && r.disagree && r.alarms.count("MRU_DISAGREE"), "MRU3 с ошибкой 3°: исключён голосованием, тревога MRU_DISAGREE");
      CHECK(r.w.dirValid && angDiff(r.w.fromDeg, 60) < 25, "оценка волн не ухудшилась из-за неисправного прибора"); }
    { Out a = run(2.5, 8.0, 60.0, 600, -1, -1, true), b = run(2.5, 8.0, 60.0, 600, -1, -1, false);
      std::printf("   положение, СКО после 200 с: с упреждением по волнам %.3f м, без %.3f м; ошибка наклона по МИО %.3f°\n", a.rmsPos, b.rmsPos, a.meanTiltErr);
      CHECK(a.rmsPos < 0.15, "с упреждением по волнам удержание в пределах 0.15 м (СКО)");
      CHECK(a.meanTiltErr < 0.1, "наклон по МИО совпадает с истинным (среднее расхождение < 0.1°)"); }
    { Out a = run(2.5, 8.0, 60.0, 700, -1, -1, true, 350), b = run(2.5, 8.0, 60.0, 700, -1, -1, false, 350);
      std::printf("   разворот на 90° в волнах: наибольшее отклонение с упреждением %.3f м, без %.3f м\n", a.maxAfterTurn, b.maxAfterTurn);
      CHECK(a.maxAfterTurn < 0.5, "разворот в волнах: отклонение остаётся меньше 0.5 м"); }
    std::printf(fails ? "\nОШИБОК: %d\n" : "\nвсе проверки пройдены\n", fails);
    return fails ? 1 : 0;
}
