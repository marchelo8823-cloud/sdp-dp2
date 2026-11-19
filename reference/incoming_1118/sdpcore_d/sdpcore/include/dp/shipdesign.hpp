// Данные судна для редактора: таблицы книги (корпус, движители, датчики, выдача) и чертёж из простых фигур.
// Чертёж — три вида в метрах:
//   сверху  — a = X (от миделя, + к носу), b = Y (от ДП, + на правый борт);
//   сбоку   — a = X, b = высота над ватерлинией (+ вверх);
//   спереди — a = Y (+ правый борт), b = высота над ватерлинией.
// По фигурам считаются: длина, ширина, площадь ватерлинии (сверху); боковая парусность и её центр, подводная площадь
// и осадка (сбоку, фигуры режутся по ватерлинии); лобовая парусность (спереди). Пересечения фигур считаются один раз.
// Книга записывается в том же виде, что шаблон «Данные_судна»; чертёж хранится в ней на скрытом листе «Рисунок».
#pragma once
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "dp/json.hpp"
#include "dp/xlsx.hpp"
#include "dp/xlsxwrite.hpp"

namespace dp {
namespace design {

enum View { Top = 0, Side = 1, Front = 2 };
inline const char* viewName(int v) { return v == Top ? "сверху" : v == Side ? "сбоку" : "спереди"; }

struct Pt { double a = 0, b = 0; };
struct Shape { int view = Side; std::string name; std::vector<Pt> pts; };
// Подложка вида: картинка и привязка двумя точками (пиксели картинки → метры вида)
struct Underlay {
    std::string path; bool cal = false; double opacity = 0.6;
    double px1 = 0, py1 = 0, px2 = 0, py2 = 0;     // пиксели (x вправо, y вниз)
    double a1 = 0, b1 = 0, a2 = 0, b2 = 0;         // соответствующие точки вида, м
};

// ---------------------------------------------------------------- описание книги (то же, что в шаблоне)
struct KvSpec { const char* label; const char* unit; bool req; const char* note; };
struct ColSpec { const char* head; bool req; double width; const char* note; };

inline const std::vector<KvSpec>& hullSpec() {
    static const std::vector<KvSpec> v = {
        {"Название судна", "", true, "Как называть судно в сообщениях"},
        {"Длина по ватерлинии", "м", true, "Расчётная длина корпуса"},
        {"Ширина", "м", true, "Наибольшая ширина по ватерлинии"},
        {"Осадка", "м", true, "Средняя осадка в рабочем состоянии"},
        {"Водоизмещение", "т", true, "Масса судна в рабочем состоянии"},
        {"Лобовая площадь парусности", "м²", true, "Проекция надводной части на плоскость мидель-шпангоута"},
        {"Боковая площадь парусности", "м²", true, "Проекция надводной части на диаметральную плоскость"},
        {"Широта района работ", "°", true, "Градусы с долями, северная +, южная −. Начало местных координат"},
        {"Долгота района работ", "°", true, "Градусы с долями, восточная +, западная −"},
        {"Длина между перпендикулярами", "м", false, "Если пусто — равна длине по ватерлинии"},
        {"Центр парусности от миделя", "м", false, "+ к носу. Если пусто — на миделе"},
        {"Поперечная метацентрическая высота", "м", false, "Если пусто — оценка ядра по размерам"},
        {"Продольная метацентрическая высота", "м", false, "Если пусто — оценка ядра по размерам"},
        {"Коэффициент полноты ватерлинии", "", false, "Если пусто — 0,8"},
        {"Коэффициент ветра продольный Cx", "", false, "По продувкам или расчёту; если пусто — значение ядра"},
        {"Коэффициент ветра поперечный Cy", "", false, "То же"},
        {"Коэффициент момента ветра Cn", "", false, "То же"},
        {"Боковая площадь подводной части", "м²", false, "Если пусто — 0,85 × длина × осадка"}};
    return v;
}
inline const std::vector<ColSpec>& thrSpec() {
    static const std::vector<ColSpec> v = {
        {"Обозначение", true, 14, "До 16 знаков, без пробелов"}, {"Тип", true, 15, "туннельный | поворотный | неповоротный"}, {"В работе", false, 10, "да | нет; пусто — да"},
        {"X от миделя, м", true, 13, "+ к носу"}, {"Y от ДП, м", true, 11, "+ на правый борт"}, {"Упор вперёд, кН", true, 13, "Наибольший упор"}, {"Упор назад, кН", true, 13, "Для туннельного — на другой борт"},
        {"Мощность, кВт", true, 12, "При наибольшем упоре"}, {"Угол установки, °", false, 13, "Только для неповоротного (обязательно): 0 — вдоль судна к носу"},
        {"Темп упора, %/с", false, 12, "Пусто — 25"}, {"Скорость поворота, °/с", false, 14, "Для поворотного; пусто — 10"}, {"Постоянная времени, с", false, 13, "Пусто — 2"},
        {"Предел нагрузки, %", false, 12, "Пусто — 100"}, {"Запретные сектора, °", false, 18, "Для поворотного: «от-до; от-до», направление силы на судно"},
        {"Порог расхождения, %", false, 13, "Уставка и факт; пусто — 25"}, {"Порог по углу, °", false, 12, "Пусто — 30"}, {"Выдержка расхождения, с", false, 14, "Пусто — 30"},
        {"Предпочтительный угол, °", false, 14, "Для поворотного: встречный упор (bias) — угол постоянного упора; у пары колонок — навстречу друг другу, например 90 и 270; пусто — нет"},
        {"Встречный упор, %", false, 12, "Для поворотного, вместе с углом: 5–50 %; пусто — нет"},
        {"Группа отказа", false, 16, "Анализ последствий: движители одной группы отказывают вместе (ГРЩ, машинное отделение); несколько — через «;»; пусто у всех — одиночные отказы"}};
    return v;
}
inline const std::vector<ColSpec>& senSpec() {
    static const std::vector<ColSpec> v = {
        {"Обозначение", true, 14, "Например GNSS1"}, {"Вид", true, 14, "спутниковый | гирокомпас | ветер | качка | лазер | трос"},
        {"X от миделя, м", false, 13, "+ к носу; обязательно для систем положения"}, {"Y от ДП, м", false, 11, "+ на правый борт"}, {"Z вниз от палубы, м", false, 14, "Вниз +, антенна на мачте — отрицательное"},
        {"СКО", false, 9, "Погрешность: м (положение), ° (курс), м/с (ветер). Обязательно для систем положения"},
        {"Подключение", false, 13, "udp | tcp | serial; пусто — датчик только для имитатора"}, {"Адрес", false, 15, "Для tcp"}, {"Порт", false, 9, "Для udp и tcp"},
        {"Устройство", false, 14, "Для serial: COM3, /dev/ttyUSB0"}, {"Скорость, бод", false, 12, "Для serial"}};
    return v;
}
inline const std::vector<KvSpec>& outKvSpec() {
    static const std::vector<KvSpec> v = {
        {"Подключение", "", false, "tcp | serial. Если лист не заполнен, команды никуда не передаются"}, {"Адрес", "", false, "Для tcp"}, {"Порт", "", false, "Для tcp"},
        {"Устройство", "", false, "Для serial"}, {"Скорость, бод", "", false, "Для serial"}, {"Номер устройства Modbus", "", false, "Адрес контроллера"},
        {"Период передачи, мс", "мс", false, "Пусто — 100"}, {"Регистр счётчика «жив»", "", false, "Необязательно"}, {"Регистр режима", "", false, "Необязательно"}};
    return v;
}
inline const std::vector<ColSpec>& outSpec() {
    static const std::vector<ColSpec> v = {
        {"Движитель", true, 14, "Обозначение с листа «Движители»"}, {"Величина", true, 11, "упор | угол"}, {"Адрес регистра", true, 13, ""}, {"Масштаб", true, 10, "Единиц регистра на 1 % или на 1°"},
        {"Со знаком", false, 10, "да | нет; пусто — да"}, {"Наибольшее значение", false, 14, "Предел значения регистра"}};
    return v;
}

using Row = std::map<std::string, std::string>;
struct ShipDesign {
    std::map<std::string, std::string> hull;
    std::vector<Row> thr, sen, outRows;
    std::map<std::string, std::string> outKv;
    std::vector<Shape> shapes;
    Underlay under[3];
};

// ---------------------------------------------------------------- площади по фигурам
struct Area { bool any = false; double area = 0, ca = 0, cb = 0, amin = 0, amax = 0, bmin = 0, bmax = 0; };

// Площадь объединения многоугольников в полосе b ∈ [bLo, bHi] и её центр. Счёт по горизонтальным строкам:
// в каждой строке интервалы всех фигур объединяются, поэтому перекрытия не удваиваются.
inline Area unionArea(const std::vector<const std::vector<Pt>*>& polys, double bLo = -1e18, double bHi = 1e18, int rows = 4000) {
    Area r; double lo = 1e18, hi = -1e18;
    for (auto* p : polys) for (const auto& q : *p) { lo = std::min(lo, q.b); hi = std::max(hi, q.b); }
    lo = std::max(lo, bLo); hi = std::min(hi, bHi);
    if (!(hi > lo)) return r;
    const double h = (hi - lo) / rows;
    r.amin = 1e18; r.amax = -1e18; r.bmin = 1e18; r.bmax = -1e18;
    std::vector<std::pair<double, double>> iv; std::vector<double> xs;
    for (int k = 0; k < rows; ++k) {
        const double y = lo + (k + 0.5) * h;
        iv.clear();
        for (auto* p : polys) {
            const auto& P = *p; if (P.size() < 3) continue;
            xs.clear();
            for (size_t i = 0, j = P.size() - 1; i < P.size(); j = i++) {
                const Pt &A = P[i], &B = P[j];
                if ((A.b > y) != (B.b > y)) xs.push_back(A.a + (y - A.b) * (B.a - A.a) / (B.b - A.b));
            }
            std::sort(xs.begin(), xs.end());
            for (size_t i = 0; i + 1 < xs.size(); i += 2) iv.push_back({xs[i], xs[i + 1]});
        }
        if (iv.empty()) continue;
        std::sort(iv.begin(), iv.end());
        double s = iv[0].first, e = iv[0].second;
        auto flush = [&]() { const double L = e - s; if (L <= 0) return; r.area += L * h; r.ca += 0.5 * (s + e) * L * h; r.cb += y * L * h;
                             r.amin = std::min(r.amin, s); r.amax = std::max(r.amax, e); r.bmin = std::min(r.bmin, y - h / 2); r.bmax = std::max(r.bmax, y + h / 2); };
        for (size_t i = 1; i < iv.size(); ++i) { if (iv[i].first <= e) e = std::max(e, iv[i].second); else { flush(); s = iv[i].first; e = iv[i].second; } }
        flush();
    }
    if (r.area > 0) { r.any = true; r.ca /= r.area; r.cb /= r.area; }
    return r;
}

// Что получается из чертежа
struct DrawCalc {
    bool top = false, side = false, sideUnder = false, front = false;
    double length = 0, beam = 0, waterplane = 0, cw = 0;             // сверху
    double lateral = 0, xWind = 0, zWind = 0;                         // сбоку, над водой
    double underwater = 0, xUnder = 0, draft = 0;                     // сбоку, под водой
    double frontal = 0;                                               // спереди, над водой
};
inline DrawCalc calc(const ShipDesign& d) {
    DrawCalc c; std::vector<const std::vector<Pt>*> v[3];
    for (const auto& s : d.shapes) if (s.view >= 0 && s.view <= 2 && s.pts.size() >= 3) v[s.view].push_back(&s.pts);
    const Area t = unionArea(v[Top]);
    if (t.any) { c.top = true; c.length = t.amax - t.amin; c.beam = t.bmax - t.bmin; c.waterplane = t.area; c.cw = c.length > 0 && c.beam > 0 ? t.area / (c.length * c.beam) : 0; }
    const Area up = unionArea(v[Side], 0.0, 1e18), dn = unionArea(v[Side], -1e18, 0.0);
    if (up.any) { c.side = true; c.lateral = up.area; c.xWind = up.ca; c.zWind = up.cb; }
    if (dn.any) { c.sideUnder = true; c.underwater = dn.area; c.xUnder = dn.ca; c.draft = -dn.bmin; }
    const Area f = unionArea(v[Front], 0.0, 1e18);
    if (f.any) { c.front = true; c.frontal = f.area; }
    return c;
}
inline double shapeArea(const Shape& s) { std::vector<const std::vector<Pt>*> v{&s.pts}; return unionArea(v).area; }


// ---------------------------------------------------------------- схема по введённым числам
// Что следует из полей корпуса, нарисованное простыми фигурами. В расчёт «по чертежу» схема не входит.
// Надводная часть — условный блок нужной площади с центром в заданном месте, а не форма судна.
struct SchemePoint { int view = Top; double a = 0, b = 0; std::string label; bool thruster = true; };
struct Scheme { std::vector<Shape> shapes; std::vector<SchemePoint> points; std::vector<std::string> missing; };

inline Scheme scheme(const ShipDesign& d) {
    Scheme s;
    auto val = [&](const char* k, double& v) {
        auto it = d.hull.find(k); if (it == d.hull.end()) return false;
        std::string t; for (char c : it->second) { if (c == ',') t += '.'; else if (c != ' ') t += c; }
        try { size_t n = 0; v = std::stod(t, &n); return n == t.size() && std::isfinite(v); } catch (...) { return false; }
    };
    double L = 0, B = 0, T = 0, cw = 0.8, al = 0, xw = 0, au = 0, af = 0;
    const bool hasL = val("Длина по ватерлинии", L) && L > 0, hasB = val("Ширина", B) && B > 0, hasT = val("Осадка", T) && T > 0;
    if (!hasL) s.missing.push_back("длина");
    if (!hasB) s.missing.push_back("ширина");
    if (!hasT) s.missing.push_back("осадка");
    if (!(val("Коэффициент полноты ватерлинии", cw) && cw > 0.3 && cw <= 1.0)) cw = 0.8;
    const bool hasAL = val("Боковая площадь парусности", al) && al > 0, hasAF = val("Лобовая площадь парусности", af) && af > 0;
    if (!val("Центр парусности от миделя", xw)) xw = 0;
    if (!(val("Боковая площадь подводной части", au) && au > 0) && hasL && hasT) au = 0.85 * L * T;
    // сверху: прямая часть и нос клином; длина клина подобрана под полноту ватерлинии
    if (hasL && hasB) {
        const double lb = std::min(L, 2.0 * L * (1.0 - cw)), xb = L / 2 - lb;
        Shape t; t.view = Top; t.name = "корпус по данным";
        if (lb < 1e-6) t.pts = {{-L / 2, -B / 2}, {L / 2, -B / 2}, {L / 2, B / 2}, {-L / 2, B / 2}};
        else t.pts = {{-L / 2, -B / 2}, {xb, -B / 2}, {L / 2, 0}, {xb, B / 2}, {-L / 2, B / 2}};
        s.shapes.push_back(t);
    }
    // сбоку: подводная часть (срез днища к носу подогнан под подводную площадь) и надводный блок
    if (hasL && hasT) {
        const double c = std::clamp(2.0 * (L * T - au) / T, 0.0, L);
        Shape u; u.view = Side; u.name = "подводная часть по данным";
        u.pts = {{-L / 2, 0}, {L / 2, 0}, {L / 2 - c, -T}, {-L / 2, -T}};
        s.shapes.push_back(u);
    }
    if (hasL && hasAL) {
        const double l = std::clamp(L - 2 * std::fabs(xw), 0.3 * L, 0.9 * L), x0 = std::clamp(xw, -L / 2 + l / 2, L / 2 - l / 2), h = al / l;   // короче блок — дальше может стоять центр
        Shape w; w.view = Side; w.name = "надводная часть по данным";
        w.pts = {{x0 - l / 2, 0}, {x0 + l / 2, 0}, {x0 + l / 2, h}, {x0 - l / 2, h}};
        s.shapes.push_back(w);
        s.points.push_back({Side, xw, h / 2, "центр парусности", false});
    }
    // спереди: подводная часть по ширине и осадке, надводный блок по лобовой площади
    if (hasB && hasT) { Shape f; f.view = Front; f.name = "подводная часть по данным"; f.pts = {{-B / 2, 0}, {B / 2, 0}, {B / 2, -T}, {-B / 2, -T}}; s.shapes.push_back(f); }
    if (hasB && hasAF) { const double h = af / B; Shape f; f.view = Front; f.name = "надводная часть по данным"; f.pts = {{-B / 2, 0}, {B / 2, 0}, {B / 2, h}, {-B / 2, h}}; s.shapes.push_back(f); }
    // движители на виде сбоку — у днища по своему X
    if (hasT) for (const auto& r : d.thr) {
        auto id = r.find("Обозначение"), x = r.find("X от миделя, м"); if (id == r.end() || x == r.end()) continue;
        std::string t; for (char ch : x->second) t += ch == ',' ? '.' : ch;
        try { s.points.push_back({Side, std::stod(t), -0.8 * T, id->second, true}); } catch (...) {}
    }
    return s;
}

// ---------------------------------------------------------------- чертёж ↔ JSON
inline std::string num(double v) { char b[40]; std::snprintf(b, sizeof b, "%.6g", v); return b; }
inline std::string esc(const std::string& s) { std::string o; for (char c : s) { if (c == '"' || c == '\\') o += '\\'; if (static_cast<unsigned char>(c) >= 32) o += c; } return o; }
inline std::string drawingJson(const ShipDesign& d) {
    std::string j = "{\"version\":1,\"shapes\":[";
    for (size_t i = 0; i < d.shapes.size(); ++i) {
        const Shape& s = d.shapes[i];
        j += std::string(i ? "," : "") + "{\"view\":" + std::to_string(s.view) + ",\"name\":\"" + esc(s.name) + "\",\"pts\":[";
        for (size_t k = 0; k < s.pts.size(); ++k) j += std::string(k ? "," : "") + "[" + num(s.pts[k].a) + "," + num(s.pts[k].b) + "]";
        j += "]}";
    }
    j += "],\"underlays\":[";
    for (int v = 0; v < 3; ++v) {
        const Underlay& u = d.under[v];
        j += std::string(v ? "," : "") + "{\"view\":" + std::to_string(v) + ",\"path\":\"" + esc(u.path) + "\",\"cal\":" + (u.cal ? "true" : "false") + ",\"opacity\":" + num(u.opacity)
           + ",\"px\":[" + num(u.px1) + "," + num(u.py1) + "," + num(u.px2) + "," + num(u.py2) + "],\"m\":[" + num(u.a1) + "," + num(u.b1) + "," + num(u.a2) + "," + num(u.b2) + "]}";
    }
    return j + "]}";
}
inline bool parseDrawing(const std::string& text, ShipDesign& d, std::string& err) {
    json::Value root; if (!json::parse(text, root, err)) return false;
    json::Report rep; json::Obj R(&root, "рисунок", &rep);
    d.shapes.clear();
    for (auto o : R.objs("shapes")) {
        Shape s; s.view = o.integer("view", Side); s.name = o.str("name", "");
        for (auto& p : o.numLists("pts")) if (p.size() == 2) s.pts.push_back({p[0], p[1]});
        if (s.view >= 0 && s.view <= 2) d.shapes.push_back(s);
    }
    for (auto o : R.objs("underlays")) {
        const int v = o.integer("view", -1); if (v < 0 || v > 2) continue;
        Underlay& u = d.under[v]; u.path = o.str("path", ""); u.cal = o.boolean("cal", false); u.opacity = o.num("opacity", 0.6);
        auto px = o.nums("px"), m = o.nums("m");
        if (px.size() == 4) { u.px1 = px[0]; u.py1 = px[1]; u.px2 = px[2]; u.py2 = px[3]; }
        if (m.size() == 4) { u.a1 = m[0]; u.b1 = m[1]; u.a2 = m[2]; u.b2 = m[3]; }
    }
    return true;
}

// ---------------------------------------------------------------- книга ↔ ShipDesign
inline std::vector<xlsxw::Sheet> bookSheets(const ShipDesign& d) {
    using namespace xlsxw;
    std::vector<Sheet> out;
    {   Sheet s; s.name = "Читать первым"; s.widths = {120};
        const char* lines[] = {"Данные судна для ядра СДП", "",
            "Заполняются листы «Корпус», «Движители», «Датчики»; лист «Выдача» — когда есть контроллер движителей.",
            "Чтобы датчик давал данные с судна, заполните у него «Подключение». Без подключения датчик используется только имитатором.",
            "Жёлтые ячейки обязательны: пока хоть одна пуста, ядро не загрузит судно и назовёт лист и строку.",
            "Зелёные ячейки необязательны: если пусто, ядро берёт значение по умолчанию и сообщает об этом в отчёте.",
            "Подписи в столбце A и заголовки таблиц не меняйте: ядро находит данные по ним.",
            "Десятичный знак — запятая или точка. Единицы указаны рядом с каждой величиной.",
            "Оси судна: X — вдоль, + к носу, от миделя; Y — поперёк, + на правый борт, от диаметральной плоскости; Z — вниз.",
            "Углы — в градусах по часовой стрелке, 0 — к носу. Направления ветра и волны — откуда, течения — куда.",
            "", "Чертёж судна (если он сделан в редакторе sdp_ship) хранится на скрытом листе «Рисунок». Ядро его не читает.",
            "", "Проверка без запуска ядра:  sdp_run --check Данные_судна.xlsx", "Загрузка в работающее ядро:  ship load Данные_судна.xlsx"};
        for (size_t i = 0; i < sizeof(lines) / sizeof(lines[0]); ++i) s.set(i, 0, lines[i], i == 0 ? Title : Plain);
        out.push_back(s); }
    auto kvSheet = [&](Sheet& s, const std::vector<KvSpec>& spec, const std::map<std::string, std::string>& val, bool reqCol, size_t r0) {
        const char* h5[] = {"Параметр", "Значение", "Единица", "Обязательно", "Пояснение"};
        const char* h4[] = {"Параметр", "Значение", "Единица", "Пояснение"};
        if (reqCol) for (int i = 0; i < 5; ++i) s.set(r0, i, h5[i], Head); else for (int i = 0; i < 4; ++i) s.set(r0, i, h4[i], Head);
        size_t r = r0;
        for (const auto& k : spec) {
            ++r; auto it = val.find(k.label);
            s.set(r, 0, k.label, Bold); s.value(r, 1, it == val.end() ? "" : it->second, k.req ? Required : Optional); s.set(r, 2, k.unit, Plain);
            if (reqCol) { s.set(r, 3, k.req ? "да" : "нет", Plain); s.set(r, 4, k.note, Plain); } else s.set(r, 3, k.note, Plain);
        }
        return r;
    };
    auto table = [&](Sheet& s, const std::vector<ColSpec>& spec, const std::vector<Row>& rows, size_t r0, size_t minRows) {
        for (size_t i = 0; i < spec.size(); ++i) { s.set(r0, i, spec[i].head, Head); if (s.widths.size() <= i) s.widths.resize(i + 1, 10); s.widths[i] = std::max(s.widths[i], spec[i].width); }
        const size_t n = std::max(minRows, rows.size());
        for (size_t k = 0; k < n; ++k)
            for (size_t i = 0; i < spec.size(); ++i) {
                std::string v; if (k < rows.size()) { auto it = rows[k].find(spec[i].head); if (it != rows[k].end()) v = it->second; }
                s.value(r0 + 1 + k, i, v, spec[i].req ? Required : Optional);
            }
        size_t r = r0 + n + 2;
        s.set(r, 0, "Пояснения к столбцам (эту часть ядро не читает):", Bold);
        for (const auto& c : spec) { ++r; s.set(r, 1, c.head, Bold); s.set(r, 2, c.req ? "обязательно" : "необязательно"); s.set(r, 3, c.note); }
    };
    {   Sheet s; s.name = "Корпус"; s.widths = {40, 26, 10, 13, 70}; s.freeze = "B2"; kvSheet(s, hullSpec(), d.hull, true, 0); out.push_back(s); }
    {   Sheet s; s.name = "Движители"; s.freeze = "B2"; table(s, thrSpec(), d.thr, 0, 12);
        const size_t n = std::max<size_t>(12, d.thr.size()) + 1;
        s.lists.push_back({"B2:B" + std::to_string(n), {"туннельный", "поворотный", "неповоротный"}}); s.lists.push_back({"C2:C" + std::to_string(n), {"да", "нет"}});
        out.push_back(s); }
    {   Sheet s; s.name = "Датчики"; s.freeze = "B2"; table(s, senSpec(), d.sen, 0, 16);
        const size_t n = std::max<size_t>(16, d.sen.size()) + 1;
        s.lists.push_back({"B2:B" + std::to_string(n), {"спутниковый", "гирокомпас", "ветер", "качка", "лазер", "трос"}}); s.lists.push_back({"G2:G" + std::to_string(n), {"udp", "tcp", "serial"}});
        out.push_back(s); }
    {   Sheet s; s.name = "Выдача"; s.widths = {30, 20, 14, 60, 12, 16};
        const size_t r = kvSheet(s, outKvSpec(), d.outKv, false, 0);
        table(s, outSpec(), d.outRows, r + 2, 16);
        for (size_t i = 0; i < s.widths.size(); ++i) s.widths[i] = std::max(s.widths[i], i < 4 ? std::vector<double>{30, 20, 14, 60}[i] : 12.0);
        out.push_back(s); }
    {   Sheet s; s.name = "Рисунок"; s.hidden = true; s.widths = {12, 100};
        const std::string j = drawingJson(d); size_t r = 0;
        s.set(r++, 0, "Чертёж судна для редактора sdp_ship. Не изменяйте этот лист вручную.");
        for (size_t p = 0; p < j.size(); p += 30000) s.set(r++, 1, j.substr(p, 30000)), s.set(r - 1, 0, "часть");
        out.push_back(s); }
    return out;
}
inline bool writeBook(const ShipDesign& d, const std::string& path, std::string& err) { return xlsxw::writeFile(path, bookSheets(d), err); }

inline bool readBook(const std::string& path, ShipDesign& d, std::string& err, std::vector<std::string>* notes = nullptr) {
    xlsx::Book b; if (!xlsx::readFile(path, b, err)) return false;
    d = ShipDesign();
    auto note = [&](const std::string& s) { if (notes) notes->push_back(s); };
    auto cell = [](const xlsx::Row& r, size_t c) { return c < r.size() ? r[c] : std::string(); };
    if (const xlsx::Sheet* s = b.find("Корпус")) {
        for (const auto& r : s->rows) { if (r.empty() || r[0].empty() || r[0] == "Параметр") continue; d.hull[r[0]] = cell(r, 1); }
    } else note("в книге нет листа «Корпус»");
    auto table = [&](const xlsx::Sheet* s, const char* first, size_t from, std::vector<Row>& out) {
        if (!s) return;
        size_t h = s->rows.size();
        for (size_t i = from; i < s->rows.size(); ++i) if (!s->rows[i].empty() && s->rows[i][0] == first) { h = i; break; }
        if (h == s->rows.size()) return;
        const xlsx::Row& head = s->rows[h];
        for (size_t i = h + 1; i < s->rows.size(); ++i) {
            const xlsx::Row& r = s->rows[i]; bool any = false; for (auto& c : r) any = any || !c.empty();
            if (!any) break;
            Row x; for (size_t c = 0; c < head.size(); ++c) if (!head[c].empty()) x[head[c]] = cell(r, c);
            out.push_back(x);
        }
    };
    table(b.find("Движители"), "Обозначение", 0, d.thr);
    table(b.find("Датчики"), "Обозначение", 0, d.sen);
    if (const xlsx::Sheet* s = b.find("Выдача")) {
        for (const auto& r : s->rows) { if (r.empty() || r[0].empty() || r[0] == "Параметр") continue; if (r[0] == "Движитель") break; d.outKv[r[0]] = cell(r, 1); }
        table(s, "Движитель", 0, d.outRows);
    }
    if (const xlsx::Sheet* s = b.find("Рисунок")) {
        std::string j; for (const auto& r : s->rows) if (cell(r, 0) == "часть") j += cell(r, 1);
        if (!j.empty()) { std::string e; if (!parseDrawing(j, d, e)) note("чертёж на листе «Рисунок» не прочитан: " + e); }
    }
    return true;
}

}  // namespace design
}  // namespace dp
