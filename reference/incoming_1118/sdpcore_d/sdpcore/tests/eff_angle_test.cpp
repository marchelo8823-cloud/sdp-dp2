// КПД колонки по углу упора: таблица effAngle (потеря упора / предел команды).
#include <cstdio>
#include "dp/thrust.hpp"
#include "dp/vesselio.hpp"

using namespace dp;
static int fails = 0;
#define CHECK(c, msg) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", msg); if (!ok_) ++fails; } while (0)

static ThrusterCfg mk(bool limit) {
    ThrusterCfg t; t.id = "VRK_P"; t.type = ThrusterType::Azimuth; t.x = -24; t.y = -3.5; t.maxForward = t.maxReverse = 343000; t.powerMaxKW = 2060;
    t.effAngle = {{0, 100}, {90, 100}, {180, 60}, {270, 100}};   // струя назад-вперёд: потеря в секторе вокруг 180°
    t.effIsLimit = limit; return t;
}

int main() {
    ThrusterCfg t = mk(false);
    CHECK(std::fabs(effAt(t, 0) - 1.0) < 1e-9 && std::fabs(effAt(t, 180) - 0.6) < 1e-9, "в узлах таблицы значения точные");
    CHECK(std::fabs(effAt(t, 135) - 0.8) < 1e-9, "между узлами — линейно");
    CHECK(std::fabs(effAt(t, -180) - 0.6) < 1e-9 && std::fabs(effAt(t, 540) - 0.6) < 1e-9, "углы приводятся по кругу");
    ThrusterCfg none; none.type = ThrusterType::Azimuth;
    CHECK(effAt(none, 123) == 1.0, "без таблицы КПД = 100 %");
    // физика: упор в направлении 180° с потерей 40 %
    Wrench w = thrusterWrench(t, 100000, deg2rad(180));
    CHECK(std::fabs(w.X + 60000) < 1.0, "потеря упора: физический упор 60 % от команды");
    Wrench wl = thrusterWrench(mk(true), 100000, deg2rad(180));
    CHECK(std::fabs(wl.X + 100000) < 1.0, "режим предела: физика не режется (режет распределитель)");
    // распределитель: запрос 90 кН назад (угол 180°). Колонка предельно 343 кН, 60 % = 206 кН — запрос выполним, команда выше 26 %
    Wrench tau; tau.X = -90000; tau.Y = 0; tau.N = t.x * 0 - t.y * tau.X;
    Allocator a({t});
    AllocResult r = a.allocate(tau, {180.0}, 1.0);
    const double cmd = r.cmds[0].pct;
    CHECK(std::fabs(r.achieved.X + 90000) < 3000, "запрос выполнен");
    CHECK(cmd > 100.0 * 90000 / 343000 * 1.5 && cmd < 100.0 * 90000 / 343000 / 0.6 * 1.1, "команда увеличена за счёт потери КПД (≈ 26 % / 0,6)");
    // насыщение: запрос 300 кН назад недостижим (максимум 206 кН); команда не больше 100 %
    Wrench big; big.X = -300000; big.Y = 0; big.N = -t.y * big.X;
    AllocResult rb = a.allocate(big, {180.0}, 1.0);
    CHECK(std::fabs(rb.cmds[0].pct) <= 100.0 + 1e-6 && std::fabs(rb.achieved.X) < 215000, "предел по углу: сила не больше 60 % максимума");
    // запрос вперёд — без потерь
    Wrench fw; fw.X = 300000; fw.Y = 0; fw.N = -t.y * fw.X;
    AllocResult rf = a.allocate(fw, {0.0}, 1.0);
    CHECK(rf.achieved.X > 280000, "вперёд без потерь: полный упор");
    // режим предела: команда не растёт, но сила ограничена таблицей
    Allocator al({mk(true)});
    AllocResult rl = al.allocate(big, {180.0}, 1.0);
    CHECK(rl.cmds[0].pct < 62.0 && std::fabs(rl.achieved.X) < 215000, "режим предела: команда ≤ 60 %");
    // чтение/запись
    json::Report rep; json::Value root; std::string perr;
    CHECK(json::parse(writeThruster(t), root, perr), "запись корректна");
    json::Obj o(&root, "движитель", &rep); ThrusterCfg rd; readThruster(o, rd);
    CHECK(rep.errors.empty() && rd.effAngle.size() == 4 && !rd.effIsLimit, "таблица сохраняется и читается");
    std::printf(fails ? "ПРОВАЛ: %d\n" : "Все проверки пройдены\n", fails);
    return fails ? 1 : 0;
}
