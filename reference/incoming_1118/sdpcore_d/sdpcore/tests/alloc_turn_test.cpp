// Распределитель: признак «колонки разворачиваются» и встречный упор (bias).
// Аргумент: <книга судна>
#include <cstdio>
#include <cmath>
#include "dp/shipbook.hpp"
using namespace dp;
static int fails = 0;
#define CHECK(c, m) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", m); if (!ok_) ++fails; } while (0)

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    AppCfg cfg; BookReport rep;
    if (!loadShipBook(argv[1], cfg, rep)) { std::printf("FAIL книга\n"); return 1; }
    Allocator al(cfg.thrusters);
    const Wrench astern{-60000, 0, 0};                     // небольшая сила назад: 60 кН при возможностях около 290 кН
    std::vector<double> fwd(cfg.thrusters.size(), 0.0);    // колонки смотрят вперёд (0°): сила назад требует разворота на 180°
    std::vector<double> back(cfg.thrusters.size(), 180.0);
    const AllocResult r1 = al.allocate(astern, fwd), r2 = al.allocate(astern, back);
    std::printf("     колонки вперёд: разворот %d, насыщение %d; колонки назад: разворот %d, насыщение %d\n", r1.turning, r1.saturated, r2.turning, r2.saturated);
    CHECK(r1.turning, "колонки смотрят не туда: признак «разворачиваются»");
    CHECK(!r2.turning && !r2.saturated, "колонки уже развёрнуты: признака нет, запрос выполнен");
    const Capability cap = al.capability(1.0);
    CHECK(std::fabs(astern.X) < cap.X, "запрос в пределах возможностей комплекса (для ядра это не настоящая нехватка)");

    // встречный упор: при нулевом запросе колонки держат упор вдоль заданных углов, но в сумме сила почти ноль
    std::vector<ThrusterCfg> th = cfg.thrusters;
    for (auto& t : th) if (t.type == ThrusterType::Azimuth) { t.preloadPct = 20; t.prefDeg = t.y < 0 ? 90 : 270; }   // пара колонок толкает навстречу друг другу
    Allocator ab(th);
    std::vector<double> pref(th.size(), 0.0); for (size_t i = 0; i < th.size(); ++i) if (th[i].type == ThrusterType::Azimuth) pref[i] = th[i].prefDeg;
    const AllocResult rb = ab.allocate(Wrench{0, 0, 0}, pref);
    double azPct = 0; for (size_t i = 0; i < th.size(); ++i) if (th[i].type == ThrusterType::Azimuth) azPct = std::max(azPct, std::fabs(rb.cmds[i].pct));
    std::printf("     встречный упор 20 %%: колонки до %.0f %%, итог X %.0f Н, Y %.0f Н\n", azPct, rb.achieved.X, rb.achieved.Y);
    CHECK(azPct > 10, "при встречном упоре колонки работают и при нулевом запросе");
    CHECK(std::hypot(rb.achieved.X, rb.achieved.Y) < 2000, "суммарная сила при этом почти ноль (колонки гасят друг друга)");
    // малое изменение силы при встречном упоре — без разворота колонок
    const AllocResult rs = ab.allocate(Wrench{-20000, 0, 0}, pref), rn = al.allocate(Wrench{-20000, 0, 0}, fwd);
    double turnB = 0, turnN = 0;
    for (size_t i = 0; i < th.size(); ++i) if (th[i].type == ThrusterType::Azimuth) {
        turnB = std::max(turnB, std::fabs(rad2deg(wrapPi(deg2rad(rs.cmds[i].angleDeg - pref[i])))));
        turnN = std::max(turnN, std::fabs(rad2deg(wrapPi(deg2rad(rn.cmds[i].angleDeg - fwd[i])))));
    }
    std::printf("     сила назад 20 кН: разворот колонок со встречным упором до %.0f°, без него до %.0f°\n", turnB, turnN);
    CHECK(turnB < 45 && turnN > 90, "со встречным упором малая сила назад — поворот колонок на десятки градусов, а не на 180°");
    std::printf(fails ? "ЕСТЬ ОШИБКИ: %d\n" : "все проверки прошли\n", fails);
    return fails ? 1 : 0;
}
