// Реалистичность среды имитатора: плавная смена ветра и волн (без скачков), порывы, переменное течение, медленный дрейф.
#include <cstdio>
#include "dp/sim.hpp"

using namespace dp;
static int fails = 0;
#define CHECK(c, msg) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", msg); if (!ok_) ++fails; } while (0)

struct R { double rms = 0, maxErr = 0, maxStepM = 0, maxRotDegS = 0, maxAccel = 0, windAt5 = 0, windAt200 = 0, minEnv = 1e9; double spreadWind = 0; };

static R run(double turb, bool calm, double T) {
    VesselCfg v = demoVessel(); auto th = demoThrusters();
    EnvCfg env; env.windSpeed = 14; env.windFromDeg = 60; env.currentSpeed = 0.6; env.currentToDeg = 200; env.hs = 1.5; env.tp = 7; env.waveFromDeg = 60; env.turbulence = turb;
    DpEngine eng(v, th); Simulator sim(v, th, env);
    sim.dyn().state().psi = deg2rad(10);
    const double dt = 0.1;
    for (int i = 0; i < 100; ++i) { sim.step(dt); eng.step(dt, sim.measure()); }
    eng.setMode(DpMode::AutoPosition);
    eng.controller().setSetpoint({sim.truth().n, sim.truth().e, 10.0});
    R r; int cnt = 0; double pn = sim.truth().n, pe = sim.truth().e, pu = 0, pv = 0; double wmin = 1e9;
    const double tCalm = 300;
    for (double t = 0; t < T; t += dt) {
        if (calm && std::fabs(t - tCalm) < dt / 2) { sim.target().windSpeed = 0; sim.target().currentSpeed = 0; sim.target().hs = 0; }
        sim.step(dt); auto o = eng.step(dt, sim.measure()); sim.applyCommands(o.cmds);
        const auto& s = sim.truth();
        if (t > 150) {
            r.maxStepM = std::max(r.maxStepM, std::hypot(s.n - pn, s.e - pe));
            r.maxRotDegS = std::max(r.maxRotDegS, std::fabs(rad2deg(s.r)));
            r.maxAccel = std::max(r.maxAccel, std::hypot(s.u - pu, s.v - pv) / dt);
            const double err = std::hypot(s.n - o.ctl.refN, s.e - o.ctl.refE);
            r.maxErr = std::max(r.maxErr, err);
            if (!calm || t < tCalm) { r.rms += err * err; ++cnt; }
            if (!calm) { wmin = std::min(wmin, sim.dyn().env().windSpeed); }
        }
        if (calm && std::fabs(t - (tCalm + 5)) < dt / 2) r.windAt5 = sim.dyn().env().windSpeed;
        if (calm && std::fabs(t - (tCalm + 200)) < dt / 2) r.windAt200 = sim.dyn().env().windSpeed;
        pn = s.n; pe = s.e; pu = s.u; pv = s.v;
    }
    r.rms = std::sqrt(r.rms / std::max(1, cnt));
    return r;
}

int main() {
    std::printf("== Среда имитатора ==\n");
    R flat = run(0.0, false, 600), rough = run(1.0, false, 600);
    std::printf("   удержание при ветре 14 м/с: ровная среда СКО %.2f м (наиб. %.2f м); с порывами и дрейфом СКО %.2f м (наиб. %.2f м)\n", flat.rms, flat.maxErr, rough.rms, rough.maxErr);
    CHECK(rough.rms > flat.rms * 2 && rough.rms < 3.0, "порывы и дрейф дают реалистичные отклонения (больше ровной среды, но в пределах 3 м)");
    R c = run(1.0, true, 700);
    std::printf("   штиль с t=300 с: ветер через 5 с %.1f м/с, через 200 с %.1f м/с; наибольший шаг %.3f м за 0.1 с; наибольшая угловая скорость %.2f°/с; наиб. ускорение %.3f м/с²; наибольшее отклонение %.2f м\n",
                c.windAt5, c.windAt200, c.maxStepM, c.maxRotDegS, c.maxAccel, c.maxErr);
    CHECK(c.windAt5 > 10.0, "через 5 с после «штиля» ветер ещё не упал (плавное изменение, не скачок)");
    CHECK(c.windAt200 < 0.05, "ветер доходит до нуля");
    CHECK(c.maxStepM < 0.2, "положение не прыгает (шаг за 0.1 с меньше 0.2 м)");
    CHECK(c.maxErr < 8.0, "при смене на штиль отклонение остаётся умеренным");
    {   // дрейфовая сила волн по Руководству РС / DNV (проверка знаков и величины)
        VesselCfg v; v.length = 90.7; v.beam = 19.8; v.lpp = 81.6; v.los = 84.7;
        const Wrench head = waveDriftForce(v, 2.1, 0, 0, 5.3), beam = waveDriftForce(v, 2.1, 90, 0, 5.3), astern = waveDriftForce(v, 2.1, 180, 0, 5.3);
        std::printf("   волны Hs 2.1 м, Tz 5.3 с: встречные X %.1f кН; с правого борта Y %.1f кН; в корму X %.1f кН\n", head.X / 1e3, beam.Y / 1e3, astern.X / 1e3);
        CHECK(head.X < 0 && std::fabs(head.X / 1e3 + 22.1) < 1.0, "встречные волны сносят судно к корме (≈22 кН)");
        CHECK(beam.Y < 0 && std::fabs(beam.Y / 1e3 + 89.9) < 2.0, "волны с правого борта сносят на левый (≈90 кН)");
        CHECK(astern.X > 0, "волны в корму толкают вперёд");
        CHECK(tzFromHs(2.1) > 5.2 && tzFromHs(2.1) < 5.4, "Tz по статистике РС для Hs 2.1 м ≈ 5.3 с");
    }
    std::printf(fails ? "ИТОГ: ошибок %d\n" : "ИТОГ: всё в порядке\n", fails);
    return fails ? 1 : 0;
}
