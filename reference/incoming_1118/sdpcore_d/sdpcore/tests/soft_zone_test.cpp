// Запретный сектор колонки: жёсткое ядро (hardCoreDeg) и «нежелательная» остальная часть сектора.
#include <cstdio>
#include "dp/thrust.hpp"
#include "dp/vesselio.hpp"

using namespace dp;
static int fails = 0;
#define CHECK(c, msg) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", msg); if (!ok_) ++fails; } while (0)

static AllocResult run(double core, double dirDeg, double F) {
    ThrusterCfg t; t.id = "VRK_P"; t.type = ThrusterType::Azimuth; t.x = -24; t.y = -3.5; t.maxForward = t.maxReverse = 343000; t.powerMaxKW = 2060;
    t.forbidden = {{240, 300}}; t.hardCoreDeg = core;
    Allocator a({t});
    Wrench w; w.X = F * std::cos(deg2rad(dirDeg)); w.Y = F * std::sin(deg2rad(dirDeg)); w.N = t.x * w.Y - t.y * w.X;
    return a.allocate(w, {dirDeg}, 1.0);
}
static double wrap360(double a) { return std::fmod(a + 720.0, 360.0); }

int main() {
    const double F = 100000;
    // 1. весь сектор жёсткий (по умолчанию): запрос в секторе уводится из него, предупреждения нет
    auto r1 = run(360, 250, F);
    const double a1 = wrap360(r1.cmds[0].angleDeg);
    CHECK(!(a1 > 240.5 && a1 < 299.5) && r1.softZone.empty(), "жёсткий сектор: колонка выведена из сектора");
    // 2. жёсткое ядро 20°: запрос на 250° (нежелательная часть) выполняется, колонка остаётся там с предупреждением
    auto r2 = run(20, 250, F);
    const double a2 = wrap360(r2.cmds[0].angleDeg);
    CHECK(std::fabs(a2 - 250) < 5.0 && r2.softZone.size() == 1, "нежелательная часть: запрос выполнен, колонка осталась, выдано предупреждение");
    // 3. запрос в ядре (270°): колонка не остаётся в ядре даже если запрос не выполнить
    auto r3 = run(20, 270, F);
    const double a3 = wrap360(r3.cmds[0].angleDeg);
    CHECK(!(a3 > 259.9 && a3 < 280.1), "жёсткое ядро: колонка не удерживается в ядре");
    // 4. запрос вне сектора: без изменений и без предупреждения
    auto r4 = run(20, 200, F);
    CHECK(std::fabs(wrap360(r4.cmds[0].angleDeg) - 200) < 5.0 && r4.softZone.empty(), "вне сектора: угол как запрошен, предупреждения нет");
    // 5. чтение и запись профиля
    ThrusterCfg w; w.id = "B"; w.type = ThrusterType::Azimuth; w.maxForward = 1000; w.hardCoreDeg = 20; w.forbidden = {{60, 120}};
    json::Report rep; json::Value root; std::string perr;
    CHECK(json::parse(writeThruster(w), root, perr), "запись корректна");
    json::Obj o(&root, "движитель", &rep); ThrusterCfg rd; readThruster(o, rd);
    CHECK(rep.errors.empty() && rd.hardCoreDeg == 20, "hardCoreDeg сохраняется и читается");
    std::printf(fails ? "ПРОВАЛ: %d\n" : "Все проверки пройдены\n", fails);
    return fails ? 1 : 0;
}
