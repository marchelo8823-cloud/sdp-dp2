// Чтение таблиц Excel (.xlsx) без внешних библиотек: листы → строки → ячейки как текст.
// Поддержано: общие строки (sharedStrings), строки в ячейке (inlineStr), числа, логические значения, результаты формул.
// Не поддержано: старый формат .xls, защищённые паролем книги, даты как даты (читаются числом).
#pragma once
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "dp/zipread.hpp"

namespace dp {
namespace xlsx {

using Row = std::vector<std::string>;
struct Sheet { std::string name; std::vector<Row> rows; };          // rows[r][c], r и c с нуля; пустые ячейки — пустые строки
struct Book {
    std::vector<Sheet> sheets;
    const Sheet* find(const std::string& name) const { for (auto& s : sheets) if (s.name == name) return &s; return nullptr; }
};

namespace detail {
inline std::string unescape(const std::string& s) {
    std::string o; o.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] != '&') { o += s[i]; continue; }
        const size_t e = s.find(';', i); if (e == std::string::npos) { o += s[i]; continue; }
        const std::string k = s.substr(i + 1, e - i - 1);
        if (k == "amp") o += '&'; else if (k == "lt") o += '<'; else if (k == "gt") o += '>'; else if (k == "quot") o += '"'; else if (k == "apos") o += '\'';
        else if (k.size() > 1 && k[0] == '#') {
            unsigned long cp = 0; try { cp = k[1] == 'x' ? std::stoul(k.substr(2), nullptr, 16) : std::stoul(k.substr(1)); } catch (...) { cp = '?'; }
            if (cp < 0x80) o += char(cp);
            else if (cp < 0x800) { o += char(0xC0 | (cp >> 6)); o += char(0x80 | (cp & 63)); }
            else if (cp < 0x10000) { o += char(0xE0 | (cp >> 12)); o += char(0x80 | ((cp >> 6) & 63)); o += char(0x80 | (cp & 63)); }
            else { o += char(0xF0 | (cp >> 18)); o += char(0x80 | ((cp >> 12) & 63)); o += char(0x80 | ((cp >> 6) & 63)); o += char(0x80 | (cp & 63)); }
        } else { o += s.substr(i, e - i + 1); }
        i = e;
    }
    return o;
}
// значение атрибута name в открывающем теге tag («<c r="B2" t="s">»)
inline std::string attr(const std::string& tag, const std::string& name) {
    size_t p = 0; const std::string key = name + "=\"";
    while ((p = tag.find(key, p)) != std::string::npos) {
        if (p > 0 && (tag[p - 1] == ' ' || tag[p - 1] == '\t' || tag[p - 1] == '\n')) { const size_t b = p + key.size(), e = tag.find('"', b); return e == std::string::npos ? "" : unescape(tag.substr(b, e - b)); }
        p += key.size();
    }
    return "";
}
// весь текст внутри тегов <t>…</t> фрагмента (для строк с форматированием их несколько)
inline std::string textOf(const std::string& x) {
    std::string o; size_t p = 0;
    for (;;) {
        size_t a = x.find("<t", p); if (a == std::string::npos) break;
        if (a + 2 < x.size() && x[a + 2] != '>' && x[a + 2] != ' ' && x[a + 2] != '/') { p = a + 2; continue; }   // <tab…>, <table…> и т. п.
        const size_t ge = x.find('>', a); if (ge == std::string::npos) break;
        if (x[ge - 1] == '/') { p = ge + 1; continue; }
        const size_t e = x.find("</t>", ge); if (e == std::string::npos) break;
        o += unescape(x.substr(ge + 1, e - ge - 1)); p = e + 4;
    }
    return o;
}
inline int colIndex(const std::string& ref) { int c = 0; for (char ch : ref) { if (ch >= 'A' && ch <= 'Z') c = c * 26 + (ch - 'A' + 1); else break; } return c - 1; }
inline int rowIndex(const std::string& ref) { size_t i = 0; while (i < ref.size() && (ref[i] < '0' || ref[i] > '9')) ++i; try { return std::stoi(ref.substr(i)) - 1; } catch (...) { return -1; } }
inline std::string trim(std::string s) {
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r' || s.back() == '\n')) s.pop_back();
    size_t i = 0; while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) ++i;
    return s.substr(i);
}
}  // namespace detail

inline bool readBytes(const std::vector<uint8_t>& data, Book& book, std::string& err) {
    using namespace detail;
    zip::Archive ar;
    if (!ar.open(data, err)) { err = "файл не является книгой Excel (.xlsx): " + err; return false; }
    auto get = [&](const std::string& name, std::string& out) {
        const zip::Entry* e = ar.find(name); if (!e) return false;
        std::vector<uint8_t> b; std::string er; if (!ar.read(*e, b, er, 64u << 20)) { err = er; return false; }
        out.assign(b.begin(), b.end()); return true;
    };
    std::string wb, rels, sst;
    if (!get("xl/workbook.xml", wb)) { if (err.empty()) err = "в файле нет книги Excel (xl/workbook.xml) — возможно, это старый формат .xls или файл защищён паролем"; return false; }
    get("xl/_rels/workbook.xml.rels", rels);
    get("xl/sharedStrings.xml", sst);
    std::vector<std::string> shared;
    for (size_t p = 0;;) {
        const size_t a = sst.find("<si", p); if (a == std::string::npos) break;
        const size_t ge = sst.find('>', a); if (ge == std::string::npos) break;
        if (sst[ge - 1] == '/') { shared.push_back(""); p = ge + 1; continue; }
        const size_t e = sst.find("</si>", ge); if (e == std::string::npos) break;
        shared.push_back(textOf(sst.substr(ge + 1, e - ge - 1))); p = e + 5;
    }
    std::map<std::string, std::string> target;                      // rId → путь листа
    for (size_t p = 0;;) {
        const size_t a = rels.find("<Relationship ", p); if (a == std::string::npos) break;
        const size_t e = rels.find('>', a); if (e == std::string::npos) break;
        const std::string tag = rels.substr(a, e - a + 1); std::string t = attr(tag, "Target");
        if (!t.empty() && t[0] == '/') t = t.substr(1); else t = "xl/" + t;
        target[attr(tag, "Id")] = t; p = e + 1;
    }
    for (size_t p = 0;;) {
        const size_t a = wb.find("<sheet ", p); if (a == std::string::npos) break;
        const size_t e = wb.find('>', a); if (e == std::string::npos) break;
        const std::string tag = wb.substr(a, e - a + 1); p = e + 1;
        Sheet sh; sh.name = trim(attr(tag, "name"));
        const std::string path = target[attr(tag, "r:id")];
        std::string x; if (path.empty() || !get(path, x)) { if (err.empty()) err = "не найден лист «" + sh.name + "» внутри файла"; return false; }
        for (size_t q = 0;;) {
            const size_t ca = x.find("<c ", q); if (ca == std::string::npos) break;
            const size_t ce = x.find('>', ca); if (ce == std::string::npos) break;
            const std::string ctag = x.substr(ca, ce - ca + 1); q = ce + 1;
            if (ctag[ctag.size() - 2] == '/') continue;                      // пустая ячейка только с оформлением
            const size_t cend = x.find("</c>", ce); if (cend == std::string::npos) break;
            const std::string body = x.substr(ce + 1, cend - ce - 1); q = cend + 4;
            const std::string ref = attr(ctag, "r"), ty = attr(ctag, "t");
            const int r = rowIndex(ref), c = colIndex(ref); if (r < 0 || c < 0 || r > 100000 || c > 500) continue;
            std::string val;
            if (ty == "inlineStr") val = textOf(body);
            else {
                const size_t va = body.find("<v"); if (va != std::string::npos) { const size_t vg = body.find('>', va), ve = body.find("</v>", va); if (vg != std::string::npos && ve != std::string::npos && vg < ve) val = unescape(body.substr(vg + 1, ve - vg - 1)); }
                if (ty == "s") { size_t k = 0; try { k = std::stoul(val); } catch (...) { k = shared.size(); } val = k < shared.size() ? shared[k] : ""; }
                else if (ty == "b") val = val == "1" ? "да" : "нет";
            }
            val = trim(val); if (val.empty()) continue;
            if (static_cast<int>(sh.rows.size()) <= r) sh.rows.resize(r + 1);
            if (static_cast<int>(sh.rows[r].size()) <= c) sh.rows[r].resize(c + 1);
            sh.rows[r][c] = val;
        }
        book.sheets.push_back(std::move(sh));
    }
    if (book.sheets.empty()) { err = "в книге нет листов"; return false; }
    return true;
}

inline bool readFile(const std::string& path, Book& book, std::string& err) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { err = "не удалось открыть файл: " + path; return false; }
    std::vector<uint8_t> data((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return readBytes(data, book, err);
}

}  // namespace xlsx
}  // namespace dp
