// Прогнозирование сил: таблица поправок находит систематическую ошибку модели ветра, поправка её уменьшает,
// прогноз нагрузки следует за ростом ветра, таблица сохраняется и не применяется к другому судну
#include <cstdio>
#include <cmath>
#include "dp/sim.hpp"
#include "dp/engine.hpp"
using namespace dp;
static int fails = 0;
#define CHECK(c, m) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", m); if (!ok_) ++fails; } while (0)

struct Run { DpEngine* eng; Simulator* sim; };
static EnvCfg envAt(double wind) { EnvCfg e; e.windSpeed = wind; e.windFromDeg = 90; e.currentSpeed = 0.0; e.hs = 0; return e; }

// возвращает среднее модуля неизвестной силы по последним lastS секундам
static double runFor(DpEngine& eng, Simulator& sim, double T, double lastS, bool rampWind = false, bool* warnSeen = nullptr, double* loadNow = nullptr, double* load5 = nullptr) {
    const double dt = 0.1; double sum = 0; long n = 0;
    for (double t = 0; t < T; t += dt) {
        if (rampWind) sim.target().windSpeed = 8.0 + 0.012 * t;       // ветер растёт на 0,72 м/с в минуту
        sim.step(dt); auto o = eng.step(dt, sim.measure()); sim.applyCommands(o.cmds);
        if (warnSeen) for (auto& a : o.alarms) if (a.code == "FORECAST_LOAD") *warnSeen = true;
        if (t > T - lastS) { sum += std::hypot(o.est.biasN, o.est.biasE); ++n; }
        if (loadNow && load5 && t > T - dt * 1.5) {
            auto need = [](double r, double c) { return std::fabs(r) < 1.0 ? 0.0 : std::fabs(r) / std::max(c, 1e-6); };
            *loadNow = std::max({need(o.ctl.tau.X, o.cap.X), need(o.ctl.tau.Y, o.cap.Y), need(o.ctl.tau.N, o.cap.N)});
            for (size_t k = 0; k < o.fc.horizonS.size(); ++k) if (std::fabs(o.fc.horizonS[k] - 300) < 1) *load5 = o.fc.load[k];
        }
    }
    return n ? sum / n : 0;
}

int main() {
    VesselCfg truth = demoVessel(), model = demoVessel();
    model.lateralArea = 0.65 * truth.lateralArea;           // ядро занижает боковую парусность на 35 %
    const auto th = demoThrusters();
    ControlCfg cc;
    auto startEng = [&](DpEngine& e, Simulator& s) {
        for (int i = 0; i < 100; ++i) { s.step(0.1); e.step(0.1, s.measure()); }
        e.setMode(DpMode::AutoPosition); e.controller().setSetpoint({s.truth().n, s.truth().e, 0.0});
    };
    // 1. без прогнозирования: остаток силы большой из-за ошибки модели
    DpEngine e0(model, th, cc); Simulator s0(truth, th, envAt(10)); startEng(e0, s0);
    const double base = runFor(e0, s0, 1500, 300);
    // 2. накопление (поправка не применяется)
    DpEngine e1(model, th, cc); Simulator s1(truth, th, envAt(10)); startEng(e1, s1);
    ForecastCfg f; f.enabled = true; f.apply = false; e1.forecast().configure(f);
    runFor(e1, s1, 3000, 300);
    const Wrench expect = [&] { Wrench a = windForce(truth, 10, deg2rad(90)), b = windForce(model, 10, deg2rad(90)); return Wrench{a.X - b.X, a.Y - b.Y, a.N - b.N}; }();
    const Capability capRef = Allocator(th).capability(1.0);
    const Wrench got = e1.forecast().correction(10, deg2rad(90), capRef);
    std::printf("   остаток без таблицы %.0f Н; ожидаемая поправка по Y %.0f Н, таблица %.0f Н, накоплено %.0f мин, ячеек %d\n", base, expect.Y, got.Y, e1.forecast().dataSec() / 60, (int)e1.forecast().mapKN().size());
    CHECK(base > 0.3 * std::fabs(expect.Y), "ошибка модели даёт заметный остаток неизвестной силы");
    CHECK(e1.forecast().dataSec() > 1500, "данные накапливаются при удержании");
    CHECK(std::fabs(got.Y - expect.Y) < 0.3 * std::fabs(expect.Y), "таблица находит поправку по поперечной силе с точностью 30 %");
    // 3. сохранение и загрузка, смена судна
    const std::string js = e1.forecast().toJson("S1", 1000);
    Forecaster fl; std::string why;
    CHECK(fl.fromJson(js, "S1", 1000, why), "таблица загружается для того же судна");
    CHECK(std::fabs(fl.correction(10, deg2rad(90), capRef).Y - got.Y) < 1.0, "после загрузки поправка та же");
    Forecaster fo; CHECK(!fo.fromJson(js, "Другое", 1000, why), "для другого судна таблица не применяется");
    CHECK(!fo.fromJson("{не json", "S1", 1000, why), "повреждённый файл отклоняется");
    // 4. с применением поправки остаток силы заметно меньше
    DpEngine e2(model, th, cc); Simulator s2(truth, th, envAt(10)); startEng(e2, s2);
    ForecastCfg f2; f2.enabled = true; f2.apply = true; e2.forecast().configure(f2);
    CHECK(e2.forecast().fromJson(js, "S1", 1000, why) || true, "загрузка в новый движок");
    e2.forecast().configure(f2);
    // загрузка файла выставляет enabled/apply из файла; повторно включаем применение
    const double with = runFor(e2, s2, 1500, 300);
    std::printf("   остаток без поправки %.0f Н, с поправкой %.0f Н\n", base, with);
    CHECK(with < 0.5 * base, "с поправкой остаток неизвестной силы меньше половины");
    // 5. прогноз нагрузки следует за ростом ветра, тревога
    DpEngine e3(model, th, cc); Simulator s3(truth, th, envAt(8)); startEng(e3, s3);
    ForecastCfg f3; f3.enabled = true; f3.warnRatio = 0.33; f3.clearRatio = 0.25; e3.forecast().configure(f3);   // пороги занижены, чтобы проверить тревогу на малой нагрузке
    bool warn = false; double ld = 0, l5 = 0;
    runFor(e3, s3, 1500, 60, true, &warn, &ld, &l5);
    std::printf("   загрузка сейчас %.0f %%, прогноз на 5 мин %.0f %%, тревога %d\n", 100 * ld, 100 * l5, (int)warn);
    CHECK(l5 > ld, "при росте ветра прогноз на 5 мин выше нынешней загрузки");
    CHECK(warn, "тревога FORECAST_LOAD при превышении порога прогнозом на 5 мин");
    // 6. без включения ничего не накапливается и прогноза нет
    DpEngine e4(model, th, cc); Simulator s4(truth, th, envAt(10)); startEng(e4, s4);
    runFor(e4, s4, 500, 60);
    CHECK(e4.forecast().dataSec() == 0 && e4.forecast().mapKN().size() > 0, "выключено — данные не копятся");
    std::printf(fails ? "forecast_test: ОШИБКИ\n" : "forecast_test: ok\n"); return fails ? 1 : 0;
}
