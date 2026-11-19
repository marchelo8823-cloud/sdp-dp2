// Калибровка систем положения: известное смещение и ошибка места антенны находятся в имитации.
#include <cstdio>
#include <cmath>
#include "dp/sim.hpp"

using namespace dp;
static int fails = 0;
#define CHECK(c, msg) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", msg); if (!ok_) ++fails; } while (0)

static std::vector<std::string> ids(const SensorFrame& fr) { std::vector<std::string> r; for (auto& p : fr.pos) r.push_back(p.id); return r; }

// rotate — поворот курса на 60° за время; biasId получает смещение (bn, be); antId — ошибка места антенны (dx, dy: истинное минус заданное)
static CalView run(bool rotate, const std::string& ref, double bn, double be, double dx, double dy, double T = 900, EstimatorCfg* keep = nullptr, DpEngine** out = nullptr) {
    (void)keep; (void)out;
    VesselCfg v = demoVessel(); auto th = demoThrusters();
    ControlCfg cc; EnvCfg env;
    DpEngine eng(v, th, cc); Simulator sim(v, th, env);
    eng.estimatorMut().calibrator().setRef(ref);
    eng.estimatorMut().calibrator().start();
    const double dt = 0.1; std::vector<std::string> pid;
    for (double t = 0; t < T; t += dt) {
        sim.dyn().state().psi = deg2rad(rotate ? 60.0 * t / T : 0.0);
        sim.step(dt);
        SensorFrame fr = sim.measure();
        for (auto& p : fr.pos) {
            if (p.id == "GNSS2") { p.n += bn; p.e += be; }
            if (p.id == "LASER") p.offset = V3{p.offset.x - dx, p.offset.y - dy, p.offset.z};
        }
        pid = ids(fr);
        eng.step(dt, fr);
    }
    return eng.estimator().calibrator().view(pid);
}
static const CalSensor* find(const CalView& v, const std::string& id) { for (auto& s : v.s) if (s.id == id) return &s; return nullptr; }

int main() {
    std::printf("== Калибровка систем положения ==\n");
    { // смещение GNSS2, курс постоянный: определяется только смещение
        const CalView v = run(false, "", 2.0, -1.0, 0, 0, 600);
        const CalSensor* g = find(v, "GNSS2"); const CalSensor* g1 = find(v, "GNSS1");
        std::printf("   GNSS2: bn %.2f be %.2f, СКО %.2f, отсчётов %d, ready %d; GNSS1: bn %.2f be %.2f\n", g ? g->bn : 0, g ? g->be : 0, g ? g->sigma : 0, g ? g->n : 0, g ? g->ready : 0, g1 ? g1->bn : 0, g1 ? g1->be : 0);
        CHECK(g && std::fabs(g->bn - 2.0) < 0.25 && std::fabs(g->be + 1.0) < 0.25, "смещение GNSS2 найдено с точностью 0,25 м");
        CHECK(g && !g->fitAnt, "без поворота курса ошибка антенны не определяется");
        CHECK(g && g->ready, "после 300 отсчётов поправка готова");
        CHECK(g1 && std::fabs(g1->bn) < 0.25 && std::fabs(g1->be) < 0.25, "исправная система остаётся с малым смещением");
    }
    { // поворот курса 60°: ошибка места антенны лазера
        const CalView v = run(true, "", 0, 0, 1.5, -0.8, 900);
        const CalSensor* l = find(v, "LASER");
        std::printf("   LASER: dx %.2f dy %.2f, bn %.2f be %.2f, размах курса %.0f°\n", l ? l->dx : 0, l ? l->dy : 0, l ? l->bn : 0, l ? l->be : 0, v.hdgSpanDeg);
        CHECK(v.hdgSpanDeg >= 30.0, "размах курса достаточен");
        CHECK(l && l->fitAnt && std::fabs(l->dx - 1.5) < 0.3 && std::fabs(l->dy + 0.8) < 0.3, "ошибка места антенны лазера найдена с точностью 0,3 м");
        CHECK(l && std::fabs(l->bn) < 0.3 && std::fabs(l->be) < 0.3, "постоянное смещение лазера около нуля");
    }
    { // ручной эталон
        const CalView v = run(false, "GNSS1", 2.0, -1.0, 0, 0, 600);
        const CalSensor* g = find(v, "GNSS2"); const CalSensor* r = find(v, "GNSS1");
        CHECK(r && r->isRef && r->n == 0, "эталон отмечен и не калибруется");
        CHECK(g && std::fabs(g->bn - 2.0) < 0.3 && std::fabs(g->be + 1.0) < 0.3, "смещение найдено относительно выбранного эталона");
    }
    std::printf(fails ? "ИТОГ: ошибок %d\n" : "ИТОГ: всё в порядке\n", fails);
    return fails ? 1 : 0;
}
