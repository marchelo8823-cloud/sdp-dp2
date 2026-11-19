// Анализ последствий отказа (РС ч. XV 8.9.5–8.9.7): группы из данных судна, тревога при недостатке упора, снятие с запасом, пауза при смене точки.
// Аргументы: <книга судна с группами «ГРЩ-1», «ГРЩ-2»> <книга имитатора>
#include <cstdio>
#include <cmath>
#include "dp/core.hpp"
using namespace dp;
static int fails = 0;
#define CHECK(c, m) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", m); if (!ok_) ++fails; } while (0)
static bool said(const std::vector<std::string>& v, const std::string& p) { for (auto& s : v) if (s.find(p) != std::string::npos) return true; return false; }

int main(int argc, char** argv) {
    if (argc < 3) { std::printf("нужны два аргумента\n"); return 2; }
    std::vector<std::string> log;
    Core c({}, [&](const std::string& s) { log.push_back(s); });
    c.command(std::string("ship load ") + argv[1]);
    c.command(std::string("sim start ") + argv[2]);
    for (int i = 0; i < 200; ++i) c.cycle();
    auto has = [&](const char* code) { for (auto& a : c.out().alarms) if (a.code == code) return true; return false; };
    auto run = [&](double s) { const double t = c.clock(); while (c.clock() < t + s) c.cycle(); };
    DpEngine& e = *c.engine();

    std::printf("-- группы из данных судна --\n");
    bool single = true; const auto g = Consequence::makeGroups(c.ship()->thrusters, &single);
    CHECK(!single && g.size() == 2 && g[0].name == "ГРЩ-1" && g[0].ids.size() == 2, "две группы: ГРЩ-1 (AZ1, BOW1), ГРЩ-2 (AZ2, BOW2)");
    std::vector<ThrusterCfg> noG = c.ship()->thrusters; for (auto& t : noG) t.failGroups.clear();
    const auto g1 = Consequence::makeGroups(noG, &single);
    CHECK(single && g1.size() == noG.size(), "без групп — одиночный отказ каждого движителя");

    std::printf("-- по умолчанию выключен --\n");
    c.command("thr_take all"); c.command("position");
    c.command("sim wind 8 60");
    run(200);
    CHECK(e.consequence().dpClass() == 0 && !e.consequence().ran() && !has("CA_DRIFT_OFF"), "без class 2 анализа нет");

    std::printf("-- класс 2, ветер 8 м/с: запас есть --\n");
    log.clear(); c.command("class 2");
    CHECK(said(log, "класс 2"), "class 2 принят");
    run(130);
    CHECK(e.consequence().ran() && !has("CA_DRIFT_OFF") && !has("CA_NO_GROUPS"), "расчёт прошёл, тревоги нет");
    bool allHold = true; for (const auto& x : e.consequence().groups()) allHold = allHold && x.holds;
    CHECK(allHold, "обе группы удерживают при 8 м/с");

    std::printf("-- ветер растёт: тревога CA_DRIFT_OFF, пока полный состав ещё держит --\n");
    double wOn = -1, errMax = 0; bool sat = false;
    for (double w = 14; w <= 36 && wOn < 0; w += 3) {
        c.command("sim wind " + std::to_string(w) + " 90");
        const double t0 = c.clock();
        while (c.clock() < t0 + 200) {
            c.cycle();
            if (c.clock() > t0 + 60) { const auto& s = c.sim()->truth(); errMax = std::max(errMax, std::hypot(s.n - c.out().ctl.refN, s.e - c.out().ctl.refE)); sat = sat || has("SATURATION"); }
            if (has("CA_DRIFT_OFF")) { wOn = w; break; }
        }
    }
    std::printf("   тревога при ветре %.0f м/с с 90°; полный состав: отклонение до %.2f м, насыщение: %s\n", wOn, errMax, sat ? "было" : "нет");
    CHECK(wOn > 0, "тревога CA_DRIFT_OFF при недостатке упора у одной из групп");
    CHECK(!sat && errMax < 3.0, "тревога раньше, чем полный состав перестаёт держать (нет насыщения, меньше 3 м)");
    { std::string txt; bool crit = false; for (auto& a : c.out().alarms) if (a.code == "CA_DRIFT_OFF") { txt = a.text; crit = a.critical; }
      std::printf("   %s\n", txt.c_str());
      CHECK(txt.find("ГРЩ-") != std::string::npos && txt.find("силы") != std::string::npos, "в тексте — какая группа и сколько наберётся");
      CHECK(crit, "CA_DRIFT_OFF — тревога, не предупреждение"); }
    { HypotCase h; h.windMS = wOn; h.windFromDeg = 90; h.wavesWithWind = false;
      std::printf("   сверка с моделью без волн (capHypot): полный состав при %.0f м/с — %s\n", wOn, capHypot(*c.ship(), h).holds ? "держит" : "не держит"); }
    log.clear(); c.command("ca");
    CHECK(said(log, "НЕ ХВАТИТ упора"), "команда ca показывает группу с недостатком");

    std::printf("-- ветер стих: тревога снимается --\n");
    c.command("sim wind 6 90");
    { double tOff = -1; const double t0 = c.clock(); while (c.clock() < t0 + 400) { c.cycle(); if (!has("CA_DRIFT_OFF") && tOff < 0) tOff = c.clock() - t0; }
      std::printf("   снята через %.0f с\n", tOff);
      CHECK(tOff > 0, "при запасе больше 5 % тревога снимается"); }

    std::printf("-- смена точки: пауза --\n");
    run(70); const double r0 = e.consequence().lastRunT();
    c.command("move 15 0");
    double moveEnd = -1; { double pn = c.out().ctl.refN; const double t0 = c.clock(); while (c.clock() < t0 + 200) { c.cycle(); if (std::fabs(c.out().ctl.refN - pn) > 1e-4) moveEnd = c.clock(); pn = c.out().ctl.refN; } }
    const double r1 = e.consequence().lastRunT();
    std::printf("   последний расчёт до сдвига %.0f с, после %.0f с, точка неподвижна с %.0f с\n", r0, r1, moveEnd);
    CHECK(moveEnd > r0 && (r1 <= r0 || r1 >= moveEnd + 59.0), "пока точка движется, расчёта нет; после остановки — не раньше чем через 60 с");

    std::printf("-- погода, введённая вручную (РС 8.9.7) --\n");
    log.clear(); c.command("ca 30 90");
    CHECK(said(log, "Отказ «ГРЩ-1»") && said(log, "Отказ «ГРЩ-2»"), "ca <ветер> <откуда> — расчёт по каждой группе");

    std::printf("-- выход из режима и выключение --\n");
    c.command("standby"); for (int i = 0; i < 20; ++i) c.cycle();
    CHECK(!has("CA_DRIFT_OFF") && e.consequence().groups().empty(), "в ожидании результаты сброшены");
    log.clear(); c.command("class off");
    CHECK(said(log, "остановлен") && e.consequence().dpClass() == 0, "class off — анализ остановлен");
    log.clear(); c.command("class 5");
    CHECK(said(log, "2, 3 или off") && e.consequence().dpClass() == 0, "неверный класс отклоняется");

    std::printf(fails ? "ПРОВАЛЕНО: %d\n" : "ВСЕ ПРОВЕРКИ ПРОЙДЕНЫ\n", fails);
    return fails ? 1 : 0;
}
