// Основа редактора sdp_ship без Qt: площади по фигурам, запись книги и её загрузка ядром, чертёж в книге,
// подсказка контура по картинке, запрос и разбор ответа Claude.
#include <cstdio>
#include <cmath>
#include "dp/contour.hpp"
#include "dp/shipbook.hpp"
#include "dp/shipdesign.hpp"
#include "dp/vision.hpp"
using namespace dp;
using namespace dp::design;
static int fails = 0;
#define CHECK(c, m) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", m); if (!ok_) ++fails; } while (0)
static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
static Shape rect(int view, double a0, double b0, double a1, double b1) { return {view, "", {{a0, b0}, {a1, b0}, {a1, b1}, {a0, b1}}}; }

int main(int argc, char** argv) {
    if (argc < 3) { std::printf("нужны: <пример книги> <папка для временных файлов>\n"); return 2; }
    const std::string tmp = argv[2];
    std::printf("-- площади --\n");
    {   ShipDesign d;
        d.shapes.push_back(rect(Side, -30, -4, 30, 3));          // корпус: 60 м, от киля −4 до палубы +3
        d.shapes.push_back(rect(Side, 0, 0, 10, 8));             // надстройка 10 × 8, нижние 3 м перекрыты корпусом
        d.shapes.push_back({Front, "", {{-7, -4}, {7, -4}, {7, 3}, {-7, 3}}});
        d.shapes.push_back(rect(Front, -4, 3, 4, 8));
        d.shapes.push_back({Top, "", {{-30, -7}, {22, -7}, {30, 0}, {22, 7}, {-30, 7}}});   // корпус с носовым клином
        const DrawCalc c = calc(d);
        std::printf("   сбоку: над водой %.2f м², центр x %.2f; под водой %.2f м², осадка %.2f; спереди %.2f м²; сверху L %.2f B %.2f S %.2f\n",
                    c.lateral, c.xWind, c.underwater, c.draft, c.frontal, c.length, c.beam, c.waterplane);
        CHECK(near(c.lateral, 60 * 3 + 10 * 5, 0.5), "боковая парусность: перекрытие надстройки и корпуса не удваивается (230 м²)");
        CHECK(near(c.xWind, (180 * 0 + 50 * 5) / 230.0, 0.05), "центр парусности по длине");
        CHECK(near(c.underwater, 240, 0.5) && near(c.draft, 4, 0.01), "подводная площадь 240 м², осадка 4 м по нижней точке");
        CHECK(near(c.frontal, 14 * 3 + 8 * 5, 0.3), "лобовая парусность 82 м²");
        CHECK(near(c.length, 60, 0.01) && near(c.beam, 14, 0.01) && near(c.waterplane, 52 * 14 + 8 * 7, 0.5), "сверху: длина, ширина, площадь ватерлинии");
    }
    std::printf("-- схема по введённым числам --\n");
    {   ShipDesign d; d.hull = {{"Длина по ватерлинии", "60"}, {"Ширина", "14"}, {"Осадка", "4"}, {"Коэффициент полноты ватерлинии", "0,9"},
                                {"Боковая площадь парусности", "380"}, {"Центр парусности от миделя", "6"}, {"Лобовая площадь парусности", "120"}};
        d.thr = {{{"Обозначение", "BOW1"}, {"X от миделя, м", "22"}}};
        const Scheme s = scheme(d); ShipDesign v; v.shapes = s.shapes; const DrawCalc c = calc(v);
        std::printf("   сверху %.1f м², полнота %.3f; сбоку над водой %.1f м², центр %.2f; под водой %.1f м²; спереди %.1f м²\n", c.waterplane, c.cw, c.lateral, c.xWind, c.underwater, c.frontal);
        CHECK(near(c.length, 60, 0.01) && near(c.beam, 14, 0.01) && near(c.cw, 0.9, 0.002), "сверху: длина, ширина и полнота как введены");
        CHECK(near(c.lateral, 380, 1) && near(c.xWind, 6, 0.05), "сбоку: блок площадью боковой парусности с центром на 6 м");
        CHECK(near(c.underwater, 0.85 * 60 * 4, 1) && near(c.draft, 4, 0.01), "подводная часть по умолчанию 0,85·L·T, осадка 4 м");
        CHECK(near(c.frontal, 120, 0.5), "спереди: лобовая парусность");
        bool bow = false; for (auto& p : s.points) bow = bow || (p.label == "BOW1" && near(p.a, 22, 1e-9)); CHECK(bow, "движитель отмечен на виде сбоку по своему X");
        ShipDesign e; e.hull = {{"Ширина", "14"}}; const Scheme s2 = scheme(e);
        CHECK(s2.shapes.empty() && s2.missing.size() == 2, "без длины и осадки схема не строится, недостающее названо");
    }
    std::printf("-- книга: запись и загрузка ядром --\n");
    {   ShipDesign d;
        d.hull = {{"Название судна", "Проверочное судно «Север»"}, {"Длина по ватерлинии", "60"}, {"Ширина", "14"}, {"Осадка", "4,2"}, {"Водоизмещение", "2200"},
                  {"Лобовая площадь парусности", "120"}, {"Боковая площадь парусности", "380"}, {"Широта района работ", "60"}, {"Долгота района работ", "29.5"}};
        d.thr = {{{"Обозначение", "AZ1"}, {"Тип", "поворотный"}, {"X от миделя, м", "-22"}, {"Y от ДП, м", "-4"}, {"Упор вперёд, кН", "180"}, {"Упор назад, кН", "150"}, {"Мощность, кВт", "700"}, {"Предпочтительный угол, °", "90"}, {"Встречный упор, %", "20"}, {"Группа отказа", "ГРЩ-1"}},
                 {{"Обозначение", "AZ2"}, {"Тип", "поворотный"}, {"X от миделя, м", "-22"}, {"Y от ДП, м", "4"}, {"Упор вперёд, кН", "180"}, {"Упор назад, кН", "150"}, {"Мощность, кВт", "700"}},
                 {{"Обозначение", "BOW1"}, {"Тип", "туннельный"}, {"X от миделя, м", "22"}, {"Y от ДП, м", "0"}, {"Упор вперёд, кН", "100"}, {"Упор назад, кН", "100"}, {"Мощность, кВт", "400"}}};
        d.sen = {{{"Обозначение", "GNSS1"}, {"Вид", "спутниковый"}, {"X от миделя, м", "20"}, {"Y от ДП, м", "3"}, {"Z вниз от палубы, м", "-15"}, {"СКО", "0.5"}},
                 {{"Обозначение", "GYRO1"}, {"Вид", "гирокомпас"}}, {{"Обозначение", "GYRO2"}, {"Вид", "гирокомпас"}}, {{"Обозначение", "WIND1"}, {"Вид", "ветер"}}};
        d.shapes.push_back(rect(Side, -30, -4, 30, 3));
        d.under[Side].path = "профиль.png"; d.under[Side].cal = true; d.under[Side].px1 = 10; d.under[Side].a2 = 30;
        const std::string path = tmp + "/design_book.xlsx"; std::string err;
        CHECK(writeBook(d, path, err), "книга записана");
        AppCfg cfg; BookReport rep; const bool ok = loadShipBook(path, cfg, rep);
        for (auto& e : rep.errors) std::printf("     %s\n", e.c_str());
        CHECK(ok, "ядро принимает книгу, записанную редактором");
        CHECK(cfg.vessel.name == "Проверочное судно «Север»" && near(cfg.vessel.draft, 4.2, 1e-9) && near(cfg.vessel.mass, 2.2e6, 1), "название, осадка с запятой, водоизмещение");
        CHECK(cfg.thrusters.size() == 3 && cfg.thrusters[2].type == ThrusterType::Tunnel && near(cfg.thrusters[0].maxForward, 180000, 1), "движители");
        CHECK(cfg.simSensors.gnss.size() == 1 && cfg.simSensors.gyros == 2 && cfg.simSensors.anemometers == 1, "датчики");
        CHECK(near(cfg.thrusters[0].prefDeg, 90, 1e-9) && near(cfg.thrusters[0].preloadPct, 20, 1e-9) && cfg.thrusters[0].failGroups == "ГРЩ-1", "встречный упор и группа отказа сохраняются редактором");
        ShipDesign r; std::vector<std::string> notes;
        CHECK(readBook(path, r, err, &notes) && notes.empty(), "редактор читает свою книгу");
        CHECK(r.hull["Осадка"] == "4,2" && r.thr.size() == 3 && r.thr[1]["Y от ДП, м"] == "4" && r.sen.size() == 4, "таблицы прочитаны обратно без потерь");
        CHECK(r.shapes.size() == 1 && r.shapes[0].pts.size() == 4 && near(r.shapes[0].pts[2].b, 3, 1e-9), "чертёж сохранён в книге и прочитан");
        CHECK(r.under[Side].path == "профиль.png" && r.under[Side].cal && near(r.under[Side].a2, 30, 1e-9), "подложка и привязка сохранены");
        ShipDesign t; CHECK(readBook(argv[1], t, err) && t.thr.size() == 4 && t.hull["Название судна"] == "Демонстрационное судно" && t.shapes.empty(), "редактор читает книгу из шаблона");
        CHECK(t.thr[0]["Группа отказа"] == "ГРЩ-1", "редактор читает группу отказа из примера");
    }
    std::printf("-- подсказка контура --\n");
    {   const int W = 400, H = 200; std::vector<uint8_t> g(W * H, 235);
        for (int y = 60; y < 150; ++y) for (int x = 40; x < 360; ++x) g[y * W + x] = 40;           // тёмный прямоугольник 320 × 90
        for (int y = 20; y < 60; ++y) for (int x = 200; x < 270; ++x) g[y * W + x] = 40;           // надстройка
        for (int y = 90; y < 110; ++y) for (int x = 100; x < 120; ++x) g[y * W + x] = 235;         // светлое окно внутри — дыра
        for (int x = 0; x < W; ++x) g[120 * W + x] = 60, g[121 * W + x] = 60;                        // тонкая линия ватерлинии через весь рисунок
        for (int y = 5; y < 60; ++y) g[y * W + 30] = 60;                                             // выносная линия размера
        const contour::Result r = contour::find(g, W, H, 10, 5, 390, 195);
        double area = 0; for (size_t i = 0, j = r.pts.size() - 1; i < r.pts.size(); j = i++) area += (r.pts[j].x + r.pts[i].x) * (r.pts[j].y - r.pts[i].y);
        area = std::fabs(area) / 2;
        std::printf("   вершин %zu, площадь %.0f (ожидается %d)\n", r.pts.size(), area, 320 * 90 + 70 * 40);
        CHECK(r.ok && r.pts.size() >= 6 && r.pts.size() <= 14, "контур из небольшого числа вершин");
        CHECK(near(area, 320 * 90 + 70 * 40, 0.04 * (320 * 90 + 70 * 40)), "площадь контура совпадает с предметом: дыры заполнены, тонкие линии не прилипли");
        std::vector<uint8_t> flat(W * H, 128);
        CHECK(!contour::find(flat, W, H, 0, 0, W, H).ok, "на ровном фоне контур не находится");
    }
    std::printf("-- распознавание Claude: запрос и ответ --\n");
    {   const std::string body = vision::requestBody("claude-sonnet-5-5", Side, 1200, 400, "image/png", "QUJD");
        json::Value v; std::string e; CHECK(json::parse(body, v, e), "тело запроса — правильный JSON");
        CHECK(body.find("\"media_type\":\"image/png\"") != std::string::npos && body.find("1200×400") != std::string::npos, "в запросе картинка и её размер");
        const std::string resp = "{\"id\":\"msg_1\",\"content\":[{\"type\":\"text\",\"text\":\"Вот результат:\\n```json\\n{\\\"figures\\\":[{\\\"name\\\":\\\"корпус\\\",\\\"points\\\":[[10,200],[1100,200],[1150,260],[10,260]]},{\\\"name\\\":\\\"линия\\\",\\\"points\\\":[[1,1],[2,2]]}],\\\"waterline_y\\\":240,\\\"dimensions\\\":{\\\"length_m\\\":64.4,\\\"beam_m\\\":null,\\\"draft_m\\\":4.5,\\\"displacement_t\\\":null},\\\"notes\\\":\\\"мачта не видна\\\"}\\n```\"}]}";
        const vision::Answer a = vision::parseResponse(resp);
        CHECK(a.ok && a.figures.size() == 1 && a.figures[0].name == "корпус" && a.figures[0].px.size() == 4, "фигура прочитана, вырожденная отброшена");
        CHECK(a.hasWaterline && near(a.waterlineY, 240, 1e-9) && a.dims.size() == 2 && a.dims[0].first == "Длина по ватерлинии" && near(a.dims[1].second, 4.5, 1e-9), "ватерлиния и подписанные размеры (пустые пропущены)");
        CHECK(a.notes == "мачта не видна", "замечание прочитано");
        const vision::Answer er = vision::parseResponse("{\"type\":\"error\",\"error\":{\"type\":\"authentication_error\",\"message\":\"invalid x-api-key\"}}");
        CHECK(!er.ok && er.error.find("invalid x-api-key") != std::string::npos, "ошибка сервера передаётся понятно");
    }
    std::printf(fails ? "ОШИБОК: %d\n" : "ВСЕ ПРОВЕРКИ ПРОЙДЕНЫ\n", fails);
    return fails ? 1 : 0;
}
