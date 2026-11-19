// Необязательные поля движителя: предел нагрузки (limitPct) и кривая «обороты — упор — мощность» (curve).
#include <cstdio>
#include "dp/sim.hpp"
#include "dp/vesselio.hpp"

using namespace dp;
static int fails = 0;
#define CHECK(c, msg) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", msg); if (!ok_) ++fails; } while (0)

int main() {
    // 1. кривая Praxis (Thr_Set1): интерполяция
    ThrusterCfg t; t.id = "A"; t.type = ThrusterType::Azimuth; t.maxForward = t.maxReverse = 482800; t.powerMaxKW = 2874.8;
    t.curve = {{0,0,0},{13.22,1.66,0.14},{25.62,6.21,0.52},{38.02,14.5,5.57},{50.41,24.85,13.04},{62.81,39.45,24.78},{75.21,56.56,42.54},{87.6,76.74,67.23},{100,100,100}};
    CHECK(std::fabs(thrusterRpmPct(t, 0.2485) - 50.41) < 0.05, "обороты для упора 24,85 % = 50,41 %");
    CHECK(std::fabs(thrusterPowerKW(t, 0.5656) - 0.4254 * 2874.8) < 1.0, "мощность при упоре 56,56 % по кривой");
    CHECK(std::fabs(thrusterPowerKW(t, 1.0) - 2874.8) < 1e-6, "мощность при полном упоре");
    CHECK(thrusterPowerKW(t, -0.5) == thrusterPowerKW(t, 0.5), "мощность не зависит от знака упора");
    // 2. без кривой: P ~ T^1,5, обороты ~ sqrt
    ThrusterCfg u = t; u.curve.clear();
    CHECK(std::fabs(thrusterPowerKW(u, 0.5) - 2874.8 * std::pow(0.5, 1.5)) < 1e-6, "без кривой P = Pmax·T^1,5");
    CHECK(std::fabs(thrusterRpmPct(u, 0.25) - 50.0) < 1e-9, "без кривой обороты = 100·sqrt(T)");
    // 3. чтение/запись профиля (круг)
    ThrusterCfg w = t; w.limitPct = 80; w.id = "B";
    auto js = writeThruster(w);
    json::Report rep; json::Value root; std::string perr;
    CHECK(json::parse(js, root, perr), "запись движителя — корректный JSON");
    json::Obj o(&root, "движитель", &rep);
    ThrusterCfg r; readThruster(o, r);
    CHECK(rep.errors.empty(), "профиль с limitPct и curve читается без ошибок");
    CHECK(r.limitPct == 80 && r.curve.size() == t.curve.size() && std::fabs(r.curve[4].thrust - 24.85) < 1e-9, "limitPct и curve сохраняются и читаются");
    ThrusterCfg plain; plain.id = "C"; plain.maxForward = 1000;
    CHECK(writeThruster(plain).find("curve") == std::string::npos && writeThruster(plain).find("limitPct") == std::string::npos, "без полей запись прежняя");
    // 4. предел нагрузки в распределителе и имитаторе
    std::vector<ThrusterCfg> th(1); th[0].id = "T"; th[0].type = ThrusterType::Fixed; th[0].fixedAngleDeg = 0; th[0].maxForward = th[0].maxReverse = 100000;
    Allocator a0(th); th[0].limitPct = 50; Allocator a1(th);
    CHECK(std::fabs(a1.capability().X - 0.5 * a0.capability().X) < 1e-6, "предел 50 % уменьшает возможности вдвое");
    auto r1 = a1.allocate({90000, 0, 0}, {});
    CHECK(r1.saturated && std::fabs(r1.cmds[0].pct - 100.0 * 0.5) < 0.5, "команда не превышает предел (50 %)");
    ThrusterSim sim(th[0]); sim.command({"T", 100.0, 0.0});
    for (int i = 0; i < 2000; ++i) sim.step(0.1);
    CHECK(std::fabs(sim.thrustN() - 50000) < 50, "имитатор не выдаёт упор выше предела");
    // 5. ядро: при нехватке мощности доля упора по кривой Praxis, без кривой — прежний закон P^(2/3)
    {
        VesselCfg v = demoVessel(); auto thr = demoThrusters();
        for (auto& x : thr) x.powerMaxKW = 1000;
        auto runScale = [&](bool curve) {
            auto tt = thr; if (curve) for (auto& x : tt) x.curve = {{0,0,0},{100,100,100}};   // мощность линейна по упору
            DpEngine eng(v, tt, ControlCfg{});
            Simulator sm(v, tt, EnvCfg{});
            auto fr = sm.measure(); fr.power.fresh = true; fr.power.availableKW = 0.5 * 1000 * tt.size();
            return eng.step(0.1, fr);
        };
        auto o0 = runScale(false), o1 = runScale(true);
        CHECK(std::fabs(o0.powerScale - std::pow(0.5, 2.0 / 3.0)) < 1e-9, "без кривой доля упора = ps^(2/3) (поведение прежнее)");
        CHECK(std::fabs(o1.powerScale - 0.5) < 1e-3, "с кривой (P линейна по упору) доля упора = 0,5, а не 0,63");
        CHECK(o0.powerKW == 0.0, "в покое потребление мощности равно нулю");
    }
    std::printf(fails ? "ПРОВАЛ: %d\n" : "Все проверки пройдены\n", fails);
    return fails ? 1 : 0;
}
