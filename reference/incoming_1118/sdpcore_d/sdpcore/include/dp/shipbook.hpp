// Данные судна из книги Excel. Листы: «Корпус», «Движители», «Датчики», «Выдача» (необязателен).
// Данные имитатора — в отдельной книге с листом «Имитатор» (loadSimBook).
// Книга переводится в те же настройки, что читает ядро, и проверяется дважды: заполнено ли обязательное
// и физически ли согласованы значения. Заглушек нет: пока обязательное поле пусто, ядро не запускается.
// Единицы в книге — как удобно человеку (т, кН, %/с); в ядре — СИ (кг, Н, доля в секунду).
#pragma once
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "dp/config.hpp"
#include "dp/profile.hpp"
#include "dp/xlsx.hpp"

namespace dp {

struct BookReport {
    std::vector<std::string> errors, warnings, defaults, estimates;
    bool ok() const { return errors.empty(); }
};

namespace shipbook {

inline std::string esc(const std::string& s) { std::string o; for (char c : s) { if (c == '"' || c == '\\') o += '\\'; if (static_cast<unsigned char>(c) >= 32) o += c; } return o; }
// число из ячейки: допускается десятичная запятая и пробелы-разделители
inline bool number(std::string s, double& out) {
    std::string t; for (char c : s) { if (c == ',') t += '.'; else if (c != ' ' && c != '\t') t += c; }
    if (t.empty()) return false;
    try { size_t k = 0; out = std::stod(t, &k); return k == t.size() && std::isfinite(out); } catch (...) { return false; }
}
inline std::string num(double v) { char b[40]; std::snprintf(b, sizeof b, "%.10g", v); return b; }
inline bool yes(const std::string& s, bool def, bool& bad) {
    bad = false; if (s.empty()) return def;
    if (s == "да" || s == "Да" || s == "ДА" || s == "1" || s == "yes" || s == "true") return true;
    if (s == "нет" || s == "Нет" || s == "НЕТ" || s == "0" || s == "no" || s == "false") return false;
    bad = true; return def;
}

// Лист «параметр — значение»: подпись в столбце A, значение в столбце B
struct KeyVal {
    std::map<std::string, std::string> v; const std::string sheet; BookReport& rep; bool present = false;
    KeyVal(const xlsx::Book& b, const std::string& name, BookReport& r) : sheet(name), rep(r) {
        const xlsx::Sheet* s = b.find(name); if (!s) return; present = true;
        for (const auto& row : s->rows) if (!row.empty() && !row[0].empty()) v[row[0]] = row.size() > 1 ? row[1] : "";
    }
    std::string where(const std::string& k) const { return "лист «" + sheet + "», «" + k + "»"; }
    // обязательное число: пусто или не число — ошибка
    bool need(const std::string& k, double& out, double scale = 1.0) {
        auto it = v.find(k);
        if (it == v.end()) { rep.errors.push_back(where(k) + ": строка не найдена (не меняйте подписи в столбце A)"); return false; }
        if (it->second.empty()) { rep.errors.push_back(where(k) + ": не заполнено"); return false; }
        if (!number(it->second, out)) { rep.errors.push_back(where(k) + ": «" + it->second + "» — не число"); return false; }
        out *= scale; return true;
    }
    // необязательное число: пусто — берётся значение по умолчанию, и это записывается в отчёт
    bool opt(const std::string& k, double& out, double scale, const std::string& defText) {
        auto it = v.find(k);
        if (it == v.end() || it->second.empty()) { if (!defText.empty()) rep.defaults.push_back(where(k) + ": не задано, принято " + defText); return false; }
        if (!number(it->second, out)) { rep.errors.push_back(where(k) + ": «" + it->second + "» — не число"); return false; }
        out *= scale; return true;
    }
    std::string str(const std::string& k) const { auto it = v.find(k); return it == v.end() ? "" : it->second; }
};

// Лист-таблица: строка заголовков начинается с ячейки first в столбце A; дальше строки данных до первой пустой
struct Table {
    std::vector<std::string> head; std::vector<xlsx::Row> rows; std::vector<int> rowNo; const std::string sheet; BookReport& rep; bool present = false;
    Table(const xlsx::Book& b, const std::string& name, const std::string& first, BookReport& r) : sheet(name), rep(r) {
        const xlsx::Sheet* s = b.find(name); if (!s) return;
        size_t h = s->rows.size();
        for (size_t i = 0; i < s->rows.size(); ++i) if (!s->rows[i].empty() && s->rows[i][0] == first) { h = i; break; }
        if (h == s->rows.size()) return;
        present = true; head = s->rows[h];
        for (size_t i = h + 1; i < s->rows.size(); ++i) {
            bool any = false; for (auto& c : s->rows[i]) any = any || !c.empty();
            if (!any) break;                                            // таблица кончается на первой пустой строке; ниже — пояснения
            if (s->rows[i][0].empty()) { rep.errors.push_back("лист «" + name + "», строка " + std::to_string(i + 1) + ": не заполнено «" + first + "»"); continue; }
            rows.push_back(s->rows[i]); rowNo.push_back(static_cast<int>(i + 1));
        }
    }
    int col(const std::string& h) const { for (size_t i = 0; i < head.size(); ++i) if (head[i] == h) return static_cast<int>(i); return -1; }
    std::string cell(size_t r, const std::string& h) const { const int c = col(h); return c < 0 || c >= static_cast<int>(rows[r].size()) ? "" : rows[r][c]; }
    std::string where(size_t r, const std::string& h) const { return "лист «" + sheet + "», строка " + std::to_string(rowNo[r]) + " (" + rows[r][0] + "), «" + h + "»"; }
    bool need(size_t r, const std::string& h, double& out, double scale = 1.0) {
        if (col(h) < 0) { rep.errors.push_back("лист «" + sheet + "»: нет столбца «" + h + "» (не меняйте заголовки)"); return false; }
        const std::string s = cell(r, h);
        if (s.empty()) { rep.errors.push_back(where(r, h) + ": не заполнено"); return false; }
        if (!number(s, out)) { rep.errors.push_back(where(r, h) + ": «" + s + "» — не число"); return false; }
        out *= scale; return true;
    }
    bool opt(size_t r, const std::string& h, double& out, double scale, const std::string& defText) {
        const std::string s = cell(r, h);
        if (s.empty()) { if (!defText.empty()) rep.defaults.push_back(where(r, h) + ": не задано, принято " + defText); return false; }
        if (!number(s, out)) { rep.errors.push_back(where(r, h) + ": «" + s + "» — не число"); return false; }
        out *= scale; return true;
    }
};

// запретные сектора: «240-300; 60-120»
inline bool sectors(const std::string& s, std::string& json) {
    json = "["; std::string item; bool first = true;
    auto flush = [&]() {
        std::string t; for (char c : item) if (c != ' ') t += c;
        item.clear(); if (t.empty()) return true;
        const size_t d = t.find('-', 1); if (d == std::string::npos) return false;
        double a, b; if (!number(t.substr(0, d), a) || !number(t.substr(d + 1), b)) return false;
        json += std::string(first ? "" : ",") + "[" + num(a) + "," + num(b) + "]"; first = false; return true;
    };
    for (char c : s) { if (c == ';') { if (!flush()) return false; } else item += c; }
    if (!flush()) return false;
    json += "]"; return true;
}

// Перевод книги в текст настроек ядра. Возвращает false, если есть ошибки заполнения.
inline bool toJson(const xlsx::Book& book, std::string& out, BookReport& rep) {
    for (const char* n : {"Корпус", "Движители", "Датчики"}) if (!book.find(n)) rep.errors.push_back(std::string("в книге нет листа «") + n + "»");
    if (!rep.ok()) return false;
    std::string j = "{";
    // ---------------------------------------------------------------- корпус
    {
        KeyVal K(book, "Корпус", rep);
        const std::string name = K.str("Название судна"); if (name.empty()) rep.errors.push_back(K.where("Название судна") + ": не заполнено");
        double L = 0, B = 0, T = 0, m = 0, af = 0, al = 0, x;
        K.need("Длина по ватерлинии", L); K.need("Ширина", B); K.need("Осадка", T); K.need("Водоизмещение", m, 1000.0);
        K.need("Лобовая площадь парусности", af); K.need("Боковая площадь парусности", al);
        j += "\"vessel\":{\"name\":\"" + esc(name) + "\",\"length\":" + num(L) + ",\"beam\":" + num(B) + ",\"draft\":" + num(T) + ",\"mass\":" + num(m)
           + ",\"frontalArea\":" + num(af) + ",\"lateralArea\":" + num(al);
        j += ",\"lpp\":" + num(K.opt("Длина между перпендикулярами", x, 1, "равной длине по ватерлинии") ? x : L);
        if (K.opt("Центр парусности от миделя", x, 1, "0 м (на миделе)")) j += ",\"xWind\":" + num(x);
        if (K.opt("Поперечная метацентрическая высота", x, 1, "оценка ядра по размерам")) j += ",\"gmT\":" + num(x);
        if (K.opt("Продольная метацентрическая высота", x, 1, "оценка ядра по размерам")) j += ",\"gmL\":" + num(x);
        if (K.opt("Коэффициент полноты ватерлинии", x, 1, "0,8")) j += ",\"waterplaneCoef\":" + num(x);
        if (K.opt("Коэффициент ветра продольный Cx", x, 1, "значение ядра по умолчанию")) j += ",\"cx\":" + num(x);
        if (K.opt("Коэффициент ветра поперечный Cy", x, 1, "значение ядра по умолчанию")) j += ",\"cy\":" + num(x);
        if (K.opt("Коэффициент момента ветра Cn", x, 1, "значение ядра по умолчанию")) j += ",\"cn\":" + num(x);
        if (K.opt("Боковая площадь подводной части", x, 1, "0,85 × длина × осадка")) j += ",\"currentLateralArea\":" + num(x);
        j += "}";
        if (K.opt("Уход за 5 мин без положения", x, 1, "14 м (по имитатору)")) j += ",\"estimator\":{\"drDrift5MinM\":" + num(x) + "}";
        double lat = 0, lon = 0; const bool a = K.need("Широта района работ", lat), b = K.need("Долгота района работ", lon);
        if (a && b) { if (std::fabs(lat) > 90 || std::fabs(lon) > 180) rep.errors.push_back(K.where("Широта района работ") + ": широта до ±90°, долгота до ±180° (в градусах с долями)"); j += ",\"origin\":{\"lat\":" + num(lat) + ",\"lon\":" + num(lon) + "}"; }
    }
    // ---------------------------------------------------------------- движители
    std::vector<std::string> thrIds;
    {
        Table D(book, "Движители", "Обозначение", rep);
        if (!D.present) rep.errors.push_back("лист «Движители»: не найдена строка заголовков (первая ячейка «Обозначение»)");
        else if (D.rows.empty()) rep.errors.push_back("лист «Движители»: не описан ни один движитель");
        j += ",\"thrusters\":[";
        for (size_t r = 0; r < D.rows.size(); ++r) {
            const std::string id = D.rows[r][0], ty = D.cell(r, "Тип"); thrIds.push_back(id);
            std::string jt;
            if (ty == "туннельный") jt = "tunnel"; else if (ty == "поворотный") jt = "azimuth"; else if (ty == "неповоротный") jt = "fixed";
            else { rep.errors.push_back(D.where(r, "Тип") + ": укажите «туннельный», «поворотный» или «неповоротный»"); jt = "tunnel"; }
            bool bad; const bool on = yes(D.cell(r, "В работе"), true, bad); if (bad) rep.errors.push_back(D.where(r, "В работе") + ": «да» или «нет»");
            double x = 0, y = 0, tf = 0, tr = 0, pw = 0, v;
            D.need(r, "X от миделя, м", x); D.need(r, "Y от ДП, м", y); D.need(r, "Упор вперёд, кН", tf, 1000.0); D.need(r, "Упор назад, кН", tr, 1000.0); D.need(r, "Мощность, кВт", pw);
            j += std::string(r ? "," : "") + "{\"id\":\"" + esc(id) + "\",\"type\":\"" + jt + "\",\"enabled\":" + (on ? "true" : "false") + ",\"x\":" + num(x) + ",\"y\":" + num(y)
               + ",\"maxForward\":" + num(tf) + ",\"maxReverse\":" + num(tr) + ",\"powerMaxKW\":" + num(pw);
            if (jt == "fixed") { double a = 0; if (D.need(r, "Угол установки, °", a)) j += ",\"fixedAngleDeg\":" + num(a); }
            if (D.opt(r, "Темп упора, %/с", v, 0.01, "25 %/с")) j += ",\"rateFracPerS\":" + num(v);
            if (jt == "azimuth" && D.opt(r, "Скорость поворота, °/с", v, 1, "10 °/с")) j += ",\"azRateDegS\":" + num(v);
            if (D.opt(r, "Постоянная времени, с", v, 1, "2 с")) j += ",\"timeConstS\":" + num(v);
            if (D.opt(r, "Предел нагрузки, %", v, 1, "")) j += ",\"limitPct\":" + num(v);
            if (D.opt(r, "Порог расхождения, %", v, 1, "")) j += ",\"fbDiffPct\":" + num(v);
            if (D.opt(r, "Порог по углу, °", v, 1, "")) j += ",\"fbDiffAngleDeg\":" + num(v);
            if (D.opt(r, "Выдержка расхождения, с", v, 1, "")) j += ",\"fbDiffDelayS\":" + num(v);
            if (jt == "azimuth") {   // встречный упор (bias): колонка постоянно держит упор вдоль предпочтительного угла, малые изменения силы — без разворота
                double pd = 0, pp = 0; const bool hp = D.opt(r, "Предпочтительный угол, °", pd, 1, ""), hq = D.opt(r, "Встречный упор, %", pp, 1, "");
                if (hp && hq) {
                    if (pp < 5 || pp > 50) rep.errors.push_back(D.where(r, "Встречный упор, %") + ": от 5 до 50 %");
                    else j += ",\"prefDeg\":" + num(pd) + ",\"preloadPct\":" + num(pp);
                }
                else if (hp != hq) rep.errors.push_back(D.where(r, hp ? "Встречный упор, %" : "Предпочтительный угол, °") + ": встречный упор задаётся парой — угол и упор");
            }
            {   // группы наихудшего отказа (анализ последствий): имена через «;»
                const std::string fg = D.cell(r, "Группа отказа");
                if (!fg.empty()) {
                    std::string norm; bool okg = true; size_t p = 0;
                    while (p <= fg.size()) {
                        size_t q = fg.find(';', p); if (q == std::string::npos) q = fg.size();
                        std::string g = fg.substr(p, q - p); while (!g.empty() && g.front() == ' ') g.erase(g.begin()); while (!g.empty() && g.back() == ' ') g.pop_back();
                        if (g.empty()) okg = false; else norm += (norm.empty() ? "" : ";") + g;
                        p = q + 1;
                    }
                    if (!okg) rep.errors.push_back(D.where(r, "Группа отказа") + ": имена групп через «;», без пустых, например «ГРЩ-1; МО-1»");
                    else j += ",\"failGroups\":\"" + esc(norm) + "\"";
                }
            }
            const std::string fs = D.cell(r, "Запретные сектора, °");
            if (!fs.empty()) { std::string sj; if (sectors(fs, sj)) j += ",\"forbidden\":" + sj; else rep.errors.push_back(D.where(r, "Запретные сектора, °") + ": формат «от-до; от-до», например «240-300; 60-120»"); }
            j += "}";
        }
        j += "]";
    }
    // ---------------------------------------------------------------- датчики
    std::string inputs, simGnss; std::vector<std::string> unlinked; int nGyro = 0, nWind = 0, nMru = 0, nGnss = 0; bool laser = false, wire = false;
    {
        Table S(book, "Датчики", "Обозначение", rep);
        if (!S.present) rep.errors.push_back("лист «Датчики»: не найдена строка заголовков (первая ячейка «Обозначение»)");
        for (size_t r = 0; r < S.rows.size(); ++r) {
            const std::string id = S.rows[r][0], kd = S.cell(r, "Вид"); std::string jk; bool isPos = false;
            if (kd == "спутниковый") { jk = "gnss"; ++nGnss; isPos = true; } else if (kd == "гирокомпас") { jk = "gyro"; ++nGyro; } else if (kd == "ветер") { jk = "wind"; ++nWind; }
            else if (kd == "качка") { jk = "mru"; ++nMru; } else if (kd == "лазер") { jk = "laser"; laser = true; isPos = true; } else if (kd == "трос") { jk = "wire"; wire = true; isPos = true; }
            else { rep.errors.push_back(S.where(r, "Вид") + ": укажите «спутниковый», «гирокомпас», «ветер», «качка», «лазер» или «трос»"); continue; }
            double x = 0, y = 0, z = 0, sg = 0; bool hasSg = false;
            if (isPos) { S.need(r, "X от миделя, м", x); S.need(r, "Y от ДП, м", y); S.need(r, "Z вниз от палубы, м", z); hasSg = S.need(r, "СКО", sg); }
            else { S.opt(r, "X от миделя, м", x, 1, ""); S.opt(r, "Y от ДП, м", y, 1, ""); S.opt(r, "Z вниз от палубы, м", z, 1, ""); hasSg = S.opt(r, "СКО", sg, 1, ""); }
            std::string o = "{\"id\":\"" + esc(id) + "\",\"kind\":\"" + jk + "\",\"offset\":[" + num(x) + "," + num(y) + "," + num(z) + "]";
            if (hasSg) o += ",\"sigma\":" + num(sg);
            if (jk == "gnss") simGnss += std::string(simGnss.empty() ? "" : ",") + "{\"id\":\"" + esc(id) + "\",\"offset\":[" + num(x) + "," + num(y) + "," + num(z) + "]" + (hasSg ? ",\"sigma\":" + num(sg) : "") + "}";
            const std::string tr = S.cell(r, "Подключение");
            if (tr.empty()) { unlinked.push_back(id); continue; }        // без подключения датчик работает только в имитаторе
            {
                if (tr != "udp" && tr != "tcp" && tr != "serial") { rep.errors.push_back(S.where(r, "Подключение") + ": укажите udp, tcp или serial"); continue; }
                o += ",\"transport\":{\"type\":\"" + tr + "\"";
                double p = 0;
                if (tr == "serial") { const std::string dv = S.cell(r, "Устройство"); if (dv.empty()) rep.errors.push_back(S.where(r, "Устройство") + ": не заполнено (например COM3)"); o += ",\"device\":\"" + esc(dv) + "\""; if (S.need(r, "Скорость, бод", p)) o += ",\"baud\":" + num(p); }
                else { if (S.need(r, "Порт", p)) o += ",\"port\":" + num(p); const std::string h = S.cell(r, "Адрес"); if (tr == "tcp" && h.empty()) rep.errors.push_back(S.where(r, "Адрес") + ": не заполнено"); if (!h.empty()) o += ",\"host\":\"" + esc(h) + "\""; }
                o += "}";
            }
            inputs += std::string(inputs.empty() ? "" : ",") + o + "}";
        }
        if (nGyro == 0) rep.errors.push_back("лист «Датчики»: нет ни одного гирокомпаса — курс определить нечем");
        if (nGnss == 0 && !laser && !wire) rep.errors.push_back("лист «Датчики»: нет ни одной системы положения (спутниковый, лазер, трос)");
        if (nWind == 0) rep.warnings.push_back("лист «Датчики»: нет датчика ветра — упреждения по ветру не будет");
        if (nGyro == 1) rep.warnings.push_back("лист «Датчики»: один гирокомпас — отказ не будет обнаружен сравнением");
        if (!unlinked.empty() && !inputs.empty()) { std::string s; for (auto& u : unlinked) s += (s.empty() ? "" : ", ") + u; rep.warnings.push_back("лист «Датчики»: не заполнено «Подключение» у " + s + " — с судна эти датчики данных не дадут"); }
        if (!inputs.empty()) j += ",\"inputs\":[" + inputs + "]";
    }
    // ---------------------------------------------------------------- выдача команд (необязательный лист)
    if (book.find("Выдача")) {
        KeyVal O(book, "Выдача", rep); Table R(book, "Выдача", "Движитель", rep);
        const std::string tr = O.str("Подключение");
        if (tr.empty() && R.rows.empty()) rep.warnings.push_back("лист «Выдача» пуст: команды движителям никуда не передаются");
        else {
            if (tr != "tcp" && tr != "serial") rep.errors.push_back(O.where("Подключение") + ": укажите tcp или serial");
            std::string o = "{\"id\":\"CTRL1\",\"transport\":{\"type\":\"" + esc(tr) + "\""; double v = 0;
            if (tr == "serial") { const std::string dv = O.str("Устройство"); if (dv.empty()) rep.errors.push_back(O.where("Устройство") + ": не заполнено"); o += ",\"device\":\"" + esc(dv) + "\""; if (O.need("Скорость, бод", v)) o += ",\"baud\":" + num(v); }
            else { const std::string h = O.str("Адрес"); if (h.empty()) rep.errors.push_back(O.where("Адрес") + ": не заполнено"); o += ",\"host\":\"" + esc(h) + "\""; if (O.need("Порт", v)) o += ",\"port\":" + num(v); }
            o += "}";
            if (O.need("Номер устройства Modbus", v)) o += ",\"unitId\":" + num(v);
            if (O.opt("Период передачи, мс", v, 1, "100 мс")) o += ",\"periodMs\":" + num(v);
            if (O.opt("Регистр счётчика «жив»", v, 1, "")) o += ",\"heartbeatReg\":" + num(v);
            if (O.opt("Регистр режима", v, 1, "")) o += ",\"modeReg\":" + num(v);
            o += ",\"registers\":[";
            if (R.rows.empty()) rep.errors.push_back("лист «Выдача»: не описан ни один регистр (таблица с первой ячейкой «Движитель»)");
            for (size_t r = 0; r < R.rows.size(); ++r) {
                const std::string id = R.rows[r][0], f = R.cell(r, "Величина"); bool known = false; for (auto& t : thrIds) known = known || t == id;
                if (!known) rep.errors.push_back(R.where(r, "Движитель") + ": такого движителя нет на листе «Движители»");
                if (f != "упор" && f != "угол") rep.errors.push_back(R.where(r, "Величина") + ": «упор» или «угол»");
                double ad = 0, sc = 1; R.need(r, "Адрес регистра", ad); R.need(r, "Масштаб", sc);
                bool bad; const bool sg = yes(R.cell(r, "Со знаком"), true, bad); if (bad) rep.errors.push_back(R.where(r, "Со знаком") + ": «да» или «нет»");
                o += std::string(r ? "," : "") + "{\"thruster\":\"" + esc(id) + "\",\"field\":\"" + (f == "угол" ? "angle" : "pct") + "\",\"address\":" + num(ad) + ",\"scale\":" + num(sc) + ",\"signed\":" + (sg ? "true" : "false");
                double mx; if (R.opt(r, "Наибольшее значение", mx, 1, "")) o += ",\"rawMax\":" + num(mx);
                o += "}";
            }
            j += ",\"outputs\":[" + o + "]}]";
        }
    } else rep.warnings.push_back("листа «Выдача» нет: команды движителям никуда не передаются");
    // ---------------------------------------------------------------- состав датчиков для имитатора (сам имитатор выключен)
    j += ",\"app\":{\"startMode\":\"standby\"},\"sim\":{\"enabled\":false,\"sensors\":{\"gyros\":" + num(nGyro) + ",\"anemometers\":" + num(nWind) + ",\"mrus\":" + num(nMru)
       + ",\"laser\":" + (laser ? "true" : "false") + ",\"wire\":" + (wire ? "true" : "false") + ",\"gnss\":[" + simGnss + "]}}";
    out = j + "}";
    return rep.ok();
}

}  // namespace shipbook

// Данные имитатора: среда, начальный курс, ускорение. Лист «Имитатор», «параметр — значение».
struct SimBook { EnvCfg env; double headingDeg = 0, speed = 1.0; unsigned seed = 11; };

inline bool loadSimBook(const std::string& path, SimBook& sb, BookReport& rep) {
    xlsx::Book book; std::string err;
    if (!xlsx::readFile(path, book, err)) { rep.errors.push_back(err); return false; }
    if (!book.find("Имитатор")) { rep.errors.push_back("в книге нет листа «Имитатор»" + std::string(book.find("Корпус") ? " — похоже, это книга судна, а не имитатора" : "")); return false; }
    shipbook::KeyVal K(book, "Имитатор", rep); double v;
    K.need("Ветер, м/с", sb.env.windSpeed); K.need("Ветер откуда, °", sb.env.windFromDeg);
    K.need("Течение, м/с", sb.env.currentSpeed); K.need("Течение куда, °", sb.env.currentToDeg);
    K.need("Высота волны, м", sb.env.hs); K.need("Период волны, с", sb.env.tp); K.need("Волна откуда, °", sb.env.waveFromDeg);
    K.need("Начальный курс, °", sb.headingDeg);
    if (K.opt("Порывистость", v, 1, "1 (обычная)")) sb.env.turbulence = v; else sb.env.turbulence = 1.0;
    if (K.opt("Ускорение", v, 1, "1 (реальное время)")) sb.speed = v;
    if (K.opt("Начальное число случайных", v, 1, "")) sb.seed = static_cast<unsigned>(v);
    auto range = [&](const char* k, double x, double lo, double hi) { if (x < lo || x > hi) rep.errors.push_back(K.where(k) + ": допустимо от " + shipbook::num(lo) + " до " + shipbook::num(hi)); };
    range("Ветер, м/с", sb.env.windSpeed, 0, 60); range("Течение, м/с", sb.env.currentSpeed, 0, 5); range("Высота волны, м", sb.env.hs, 0, 15);
    range("Период волны, с", sb.env.tp, 2, 20); range("Порывистость", sb.env.turbulence, 0, 2); range("Ускорение", sb.speed, 0.1, 100);
    return rep.ok();
}

// Чтение книги с данными судна: заполненность, перевод в настройки, проверка физической согласованности и оценки.
inline bool loadShipBook(const std::string& path, AppCfg& cfg, BookReport& rep) {
    xlsx::Book book; std::string err;
    if (!xlsx::readFile(path, book, err)) { rep.errors.push_back(err); return false; }
    std::string text;
    if (!shipbook::toJson(book, text, rep)) return false;
    json::Report jr;
    const bool ok = loadConfigText(text, cfg, jr);
    for (auto& e : jr.errors) rep.errors.push_back(e);
    for (auto& w : jr.warnings) rep.warnings.push_back(w);
    if (!ok) return false;
    VesselProfile p; p.vessel = cfg.vessel; p.thrusters = cfg.thrusters;
    for (const auto& i : validateProfile(p)) (i.error ? rep.errors : rep.warnings).push_back(i.field + ": " + i.text);
    for (const auto& e : estimateProfile(p)) rep.estimates.push_back(e.label + ": " + e.value + (e.note.empty() ? "" : " (" + e.note + ")"));
    return rep.ok();
}

}  // namespace dp
