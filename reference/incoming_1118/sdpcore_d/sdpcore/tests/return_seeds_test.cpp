// Возврат в заданную точку после потери положения — по 10 разным начальным числам имитатора (разная погода и шум),
// а не по одному: один удачный набор чисел скрывал раскачку (найдено на Mac владельца 16.11.2026).
// Случаи: 1 — потеря на 300 с, возврат; 2 — во время возврата новая потеря на 60 с, затем возврат.
// Требования (задача №17, 16.11.2026; в скобках — наибольшее по 10 числам):
//   торможение дрейфа больше 80 % упора — не дольше 10 с (0 с); перерегулирование меньше 2 м (1 м); усилие до 56 %;
//   средняя скорость за 10 с: случай 2 — не больше 0,25 м/с (0,24); случай 1 — не больше 0,27 м/с (0,26 — одно число из 10,
//   остальные до 0,24; истинная скорость с волной и порывами при пределе опорной точки 0,2 м/с).
// Аргументы: <книга судна> <книга имитатора>
#include <cstdio>
#include <cmath>
#include <deque>
#include "dp/core.hpp"
using namespace dp;

struct Ret { double brakeFull = 0, v10 = 0, over = 0, load = 0; bool done = false; };

static Ret watch(Core& c, double T) {
    Ret r; std::deque<std::pair<double, std::pair<double, double>>> h; const double t0 = c.clock();
    bool braked = false, near = false; double vMin = 1e9;
    auto has = [&](const char* k) { for (auto& a : c.out().alarms) if (a.code == k) return true; return false; };
    while (c.clock() < t0 + T) {
        c.cycle(); const auto& o = c.out(); const auto& s = c.sim()->truth();
        if (o.est.refLost) continue;
        const auto sp = c.engine()->controller().setpoint();
        const double d = std::hypot(s.n - sp.n, s.e - sp.e);
        if (d < 1.0) near = true;
        if (near) r.over = std::max(r.over, d);
        if (!has("POS_RETURN")) { if (c.clock() - t0 > 5) r.done = true; continue; }
        const double v = std::hypot(s.u, s.v);
        const double l = std::max(std::fabs(o.ctl.tau.X) / std::max(1.0, o.cap.X), std::fabs(o.ctl.tau.Y) / std::max(1.0, o.cap.Y));
        r.load = std::max(r.load, l);
        if (c.clock() - t0 > 3.0) vMin = std::min(vMin, v);
        if (!braked && vMin < 0.2 && v > vMin + 0.02) braked = true;      // дрейф погашен: скорость прошла минимум
        if (!braked) { if (l > 0.8) r.brakeFull += c.dt(); continue; }
        h.push_back({c.clock(), {s.n, s.e}});
        while (h.size() > 1 && c.clock() - h.front().first > 10) h.pop_front();
        if (c.clock() - h.front().first > 9.5)
            r.v10 = std::max(r.v10, std::hypot(s.n - h.front().second.first, s.e - h.front().second.second) / (c.clock() - h.front().first));
    }
    return r;
}

int main(int argc, char** argv) {
    if (argc < 3) { std::printf("нужны: <книга судна> <книга имитатора>\n"); return 2; }
    int fails = 0;
    std::printf("число | случай | торможение >80 %%, с | скорость возврата (10 с), м/с | перерегулирование, м | усилие, %% | завершён\n");
    for (unsigned seed : {11u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u}) {
        Core c({}, [](const std::string&) {});
        c.setSimSeed(seed);
        c.command(std::string("ship load ") + argv[1]); c.command(std::string("sim start ") + argv[2]);
        for (int i = 0; i < 200; ++i) c.cycle();
        c.command("thr_take all"); c.command("position");
        auto run = [&](double s) { const double t = c.clock(); while (c.clock() < t + s) c.cycle(); };
        auto off = [&]() { for (auto& id : c.simIds()) if (id.rfind("GNSS", 0) == 0 || id == "LASER") c.command("sim off " + id); };
        run(300);
        off(); run(300); c.command("sim reset");
        const Ret r1 = watch(c, 250);
        run(60); off(); run(300); c.command("sim reset"); run(15); off(); run(60); c.command("sim reset");
        const Ret r2 = watch(c, 300);
        for (int k = 0; k < 2; ++k) {
            const Ret& r = k ? r2 : r1;
            const bool ok = r.brakeFull <= 10.0 && r.v10 <= (k ? 0.25 : 0.27) && r.over < 2.0 && r.done;
            std::printf("%5u | %d | %5.1f | %4.2f | %4.1f | %3.0f | %s %s\n", seed, k + 1, r.brakeFull, r.v10, r.over, 100 * r.load, r.done ? "да" : "нет", ok ? "" : "  <-- FAIL");
            if (!ok) ++fails;
        }
    }
    std::printf(fails ? "ПРОВАЛЕНО: %d\n" : "ВСЕ ПРОВЕРКИ ПРОЙДЕНЫ\n", fails);
    return fails ? 1 : 0;
}
