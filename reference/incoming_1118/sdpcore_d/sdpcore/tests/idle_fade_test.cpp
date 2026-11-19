// Холостая мощность ПУ с ВРШ и спад упора ПУ на ходу
#include "dp/thrust.hpp"
#include "dp/vesselio.hpp"
#include <cstdio>
#include <cmath>
using namespace dp;
static int fails = 0;
#define CHECK(c, m) do { if (!(c)) { std::printf("ОШИБКА: %s\n", m); ++fails; } } while (0)
int main() {
    ThrusterCfg t; t.type = ThrusterType::Tunnel; t.powerMaxKW = 1000; t.maxForward = 100000; t.idlePct = 10; t.fadeKn = 5; t.fadeFloorPct = 50;
    CHECK(std::fabs(thrusterPowerKW(t, 0.0) - 100.0) < 1e-9, "холостая мощность 10 % при нулевом упоре");
    CHECK(std::fabs(thrusterPowerKW(t, 1.0) - 1000.0) < 1e-9, "полная мощность при полном упоре");
    CHECK(std::fabs(thrusterPowerKW(t, 0.1) - 100.0) < 1e-6 || thrusterPowerKW(t, 0.1) >= 100.0, "мощность не ниже холостой");
    ThrusterCfg u = t; u.idlePct = 0; CHECK(thrusterPowerKW(u, 0.0) == 0.0, "без idlePct холостой мощности нет");
    CHECK(speedFactorFor(t, 3.0) == 1.0 && speedFactorFor(t, 5.0) == 1.0, "до 5 уз упор полный");
    CHECK(std::fabs(speedFactorFor(t, 7.5) - 0.75) < 1e-9, "7,5 уз — 75 %");
    CHECK(std::fabs(speedFactorFor(t, 10.0) - 0.5) < 1e-9 && std::fabs(speedFactorFor(t, 20.0) - 0.5) < 1e-9, "10 уз и выше — 50 %");
    ThrusterCfg w = t; w.fadeKn = 0; CHECK(speedFactorFor(w, 15.0) == 1.0, "без fadeKn спада нет");
    t.speedFactor = speedFactorFor(t, 10.0); CHECK(std::fabs(limitFrac(t) - 0.5) < 1e-9, "limitFrac учитывает скорость");
    std::printf(fails ? "idle_fade_test: ОШИБКИ\n" : "idle_fade_test: ok\n"); return fails ? 1 : 0;
}
