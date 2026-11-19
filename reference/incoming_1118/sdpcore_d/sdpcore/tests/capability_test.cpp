// Возможности судна (capability.hpp): предельный ветер по направлениям и при отказах.
// Аргумент: <книга судна>
#include <cstdio>
#include <cmath>
#include "dp/shipbook.hpp"
#include "dp/capability.hpp"
using namespace dp;
static int fails = 0;
#define CHECK(c, m) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", m); if (!ok_) ++fails; } while (0)

int main(int argc, char** argv) {
    if (argc < 2) { std::printf("нужна книга судна\n"); return 2; }
    AppCfg cfg; BookReport rep;
    if (!loadShipBook(argv[1], cfg, rep)) { std::printf("FAIL книга не прочитана\n"); return 1; }
    CapEnv e; e.stepDeg = 30.0;
    const auto sc = capScenarios(cfg, e);
    size_t nth = 0; for (const auto& t : cfg.thrusters) if (t.maxForward > 0 && t.enabled) ++nth;
    CHECK(sc.size() == nth + 1, "сценарии: все движители и отказ каждого");
    const CapCurve& all = sc[0];
    CHECK(all.dirDeg.size() == 12, "направления через 30°");

    bool range = true, workLe = true, failLe = true;
    for (const auto& c : sc)
        for (size_t i = 0; i < c.dirDeg.size(); ++i) {
            if (c.limitMS[i] < 0 || c.limitMS[i] > e.maxWindMS) range = false;
            if (c.workMS[i] > c.limitMS[i] + 0.05) workLe = false;
        }
    for (size_t k = 1; k < sc.size(); ++k)
        for (size_t i = 0; i < all.dirDeg.size(); ++i)
            if (sc[k].limitMS[i] > all.limitMS[i] + 0.05) failLe = false;
    CHECK(range, "предельный ветер в пределах 0…верхней границы поиска");
    CHECK(workLe, "рабочий ветер (80 % упора) не больше предельного");
    CHECK(failLe, "при отказе движителя предел не больше, чем со всеми движителями");
    CHECK(all.minLimit() > 5.0, "со всеми движителями судно держит хотя бы 5 м/с с любого направления");
    std::printf("     наименьший предел: все %.1f м/с", all.minLimit());
    for (size_t k = 1; k < sc.size(); ++k) std::printf(", %s %.1f", sc[k].name.c_str(), sc[k].minLimit());
    std::printf("\n");
    

    // боковой ветер тяжелее встречного: у судна площадь борта больше площади носа
    CHECK(all.limitMS[3] < all.limitMS[0], "предел при ветре в борт (90°) меньше, чем в нос (0°)");

    // больше ветра — больше силы
    const Inertia in = completeInertia(cfg.vessel);
    const Wrench f10 = capEnvForce(cfg.vessel, in, 10, 0, 0, 90), f20 = capEnvForce(cfg.vessel, in, 20, 0, 0, 90);
    CHECK(std::fabs(f20.Y) > 3.0 * std::fabs(f10.Y), "сила ветра растёт примерно как квадрат скорости");

    // слишком сильное течение: не держит даже в штиль
    CapEnv strong = e; strong.currentMS = 15.0;
    std::vector<ThrusterCfg> th; for (const auto& t : cfg.thrusters) if (t.maxForward > 0 && t.enabled) th.push_back(t);
    Allocator al(th);
    CHECK(capLimitWind(cfg.vessel, in, al, strong, 90, 1.0) == 0.0, "течение 15 м/с в борт: предел 0 (не держит в штиль)");
    CHECK(!capHolds(al, Wrench{0, 0, 0}, 0.0), "без упора (предел 0 %) не держит");
    CHECK(capHolds(al, Wrench{0, 0, 0}, 1.0), "в штиль без течения держит");

    // меньше упора — меньше предел
    const double l100 = capLimitWind(cfg.vessel, in, al, e, 90, 1.0), l50 = capLimitWind(cfg.vessel, in, al, e, 90, 0.5);
    CHECK(l50 < l100, "при пределе упора 50 % предельный ветер меньше");

    std::printf(fails ? "ЕСТЬ ОШИБКИ: %d\n" : "все проверки прошли\n", fails);
    return fails ? 1 : 0;
}
