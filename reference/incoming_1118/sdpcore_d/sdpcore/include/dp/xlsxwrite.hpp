// Запись книги Excel (.xlsx) без внешних библиотек: строки и числа, четыре стиля ячеек, ширина столбцов,
// выпадающие списки, скрытые листы. Архив пишется без сжатия (метод «хранение») — Excel и LibreOffice такие открывают.
#pragma once
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "dp/zipread.hpp"

namespace dp {
namespace xlsxw {

enum Style { Plain = 0, Head = 1, Required = 2, Optional = 3, Bold = 4, Title = 5 };
struct Cell { std::string text; bool number = false; int style = Plain; };
struct ListCheck { std::string range; std::vector<std::string> items; };        // выпадающий список: «B2:B13», значения
struct Sheet {
    std::string name; bool hidden = false;
    std::vector<std::vector<Cell>> rows;
    std::vector<double> widths;                                                  // ширина столбцов A, B, …
    std::vector<ListCheck> lists;
    std::string freeze;                                                          // «B2» — закрепить строку и столбец
    void set(size_t r, size_t c, const std::string& t, int st = Plain, bool num = false) {
        if (rows.size() <= r) rows.resize(r + 1);
        if (rows[r].size() <= c) rows[r].resize(c + 1);
        rows[r][c] = {t, num, st};
    }
    // значение: число, если строка читается как число (с точкой), иначе текст
    void value(size_t r, size_t c, const std::string& t, int st) {
        bool num = !t.empty();
        if (num) { try { size_t k = 0; (void)std::stod(t, &k); num = k == t.size() && t.find(',') == std::string::npos; } catch (...) { num = false; } }
        set(r, c, t, st, num);
    }
};

namespace detail {
inline std::string esc(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c == '&') o += "&amp;"; else if (c == '<') o += "&lt;"; else if (c == '>') o += "&gt;"; else if (c == '"') o += "&quot;";
        else if (static_cast<unsigned char>(c) < 32 && c != '\n' && c != '\t') continue; else o += c;
    }
    return o;
}
inline std::string col(size_t c) { std::string s; ++c; while (c) { s.insert(s.begin(), char('A' + (c - 1) % 26)); c = (c - 1) / 26; } return s; }

struct Zip {
    std::vector<uint8_t> out; std::string dir; uint16_t count = 0;
    static void u16(std::vector<uint8_t>& v, uint32_t x) { v.push_back(x & 255); v.push_back((x >> 8) & 255); }
    static void u32(std::vector<uint8_t>& v, uint32_t x) { for (int i = 0; i < 4; ++i) v.push_back((x >> (8 * i)) & 255); }
    void add(const std::string& name, const std::string& data) {
        const uint32_t crc = zip::crc32(reinterpret_cast<const uint8_t*>(data.data()), data.size());
        const uint32_t off = static_cast<uint32_t>(out.size());
        u32(out, 0x04034b50); u16(out, 20); u16(out, 0x0800); u16(out, 0); u16(out, 0); u16(out, 0x21);
        u32(out, crc); u32(out, static_cast<uint32_t>(data.size())); u32(out, static_cast<uint32_t>(data.size()));
        u16(out, static_cast<uint32_t>(name.size())); u16(out, 0);
        out.insert(out.end(), name.begin(), name.end()); out.insert(out.end(), data.begin(), data.end());
        std::vector<uint8_t> d;
        u32(d, 0x02014b50); u16(d, 20); u16(d, 20); u16(d, 0x0800); u16(d, 0); u16(d, 0); u16(d, 0x21);
        u32(d, crc); u32(d, static_cast<uint32_t>(data.size())); u32(d, static_cast<uint32_t>(data.size()));
        u16(d, static_cast<uint32_t>(name.size())); u16(d, 0); u16(d, 0); u16(d, 0); u16(d, 0); u32(d, 0); u32(d, off);
        dir.append(d.begin(), d.end()); dir += name; ++count;
    }
    std::vector<uint8_t> finish() {
        const uint32_t off = static_cast<uint32_t>(out.size());
        out.insert(out.end(), dir.begin(), dir.end());
        u32(out, 0x06054b50); u16(out, 0); u16(out, 0); u16(out, count); u16(out, count); u32(out, static_cast<uint32_t>(dir.size())); u32(out, off); u16(out, 0);
        return out;
    }
};
}  // namespace detail

inline std::vector<uint8_t> build(const std::vector<Sheet>& sheets) {
    using namespace detail;
    Zip z;
    std::string ct = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">"
                     "<Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/><Default Extension=\"xml\" ContentType=\"application/xml\"/>"
                     "<Override PartName=\"/xl/workbook.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml\"/>"
                     "<Override PartName=\"/xl/styles.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml\"/>";
    for (size_t i = 0; i < sheets.size(); ++i) ct += "<Override PartName=\"/xl/worksheets/sheet" + std::to_string(i + 1) + ".xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml\"/>";
    ct += "</Types>";
    z.add("[Content_Types].xml", ct);
    z.add("_rels/.rels", "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
                         "<Relationship Id=\"rId1\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument\" Target=\"xl/workbook.xml\"/></Relationships>");
    std::string wb = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n<workbook xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\" xmlns:r=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships\"><sheets>";
    std::string rels = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">";
    for (size_t i = 0; i < sheets.size(); ++i) {
        const std::string n = std::to_string(i + 1);
        wb += "<sheet name=\"" + esc(sheets[i].name) + "\" sheetId=\"" + n + "\"" + (sheets[i].hidden ? " state=\"hidden\"" : "") + " r:id=\"rId" + n + "\"/>";
        rels += "<Relationship Id=\"rId" + n + "\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet\" Target=\"worksheets/sheet" + n + ".xml\"/>";
    }
    const size_t sid = sheets.size() + 1;
    rels += "<Relationship Id=\"rId" + std::to_string(sid) + "\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles\" Target=\"styles.xml\"/></Relationships>";
    wb += "</sheets></workbook>";
    z.add("xl/workbook.xml", wb);
    z.add("xl/_rels/workbook.xml.rels", rels);
    z.add("xl/styles.xml", "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n<styleSheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\">"
        "<fonts count=\"4\"><font><sz val=\"10\"/><name val=\"Arial\"/></font><font><b/><sz val=\"10\"/><color rgb=\"FFFFFFFF\"/><name val=\"Arial\"/></font><font><b/><sz val=\"10\"/><name val=\"Arial\"/></font><font><b/><sz val=\"12\"/><name val=\"Arial\"/></font></fonts>"
        "<fills count=\"5\"><fill><patternFill patternType=\"none\"/></fill><fill><patternFill patternType=\"gray125\"/></fill>"
        "<fill><patternFill patternType=\"solid\"><fgColor rgb=\"FF1F4E79\"/></patternFill></fill><fill><patternFill patternType=\"solid\"><fgColor rgb=\"FFFFF2CC\"/></patternFill></fill>"
        "<fill><patternFill patternType=\"solid\"><fgColor rgb=\"FFE2EFDA\"/></patternFill></fill></fills>"
        "<borders count=\"2\"><border/><border><left style=\"thin\"><color rgb=\"FF999999\"/></left><right style=\"thin\"><color rgb=\"FF999999\"/></right><top style=\"thin\"><color rgb=\"FF999999\"/></top><bottom style=\"thin\"><color rgb=\"FF999999\"/></bottom></border></borders>"
        "<cellStyleXfs count=\"1\"><xf/></cellStyleXfs><cellXfs count=\"6\">"
        "<xf fontId=\"0\" applyAlignment=\"1\"><alignment vertical=\"top\" wrapText=\"1\"/></xf>"
        "<xf fontId=\"1\" fillId=\"2\" borderId=\"1\" applyFont=\"1\" applyFill=\"1\" applyBorder=\"1\" applyAlignment=\"1\"><alignment vertical=\"top\" wrapText=\"1\"/></xf>"
        "<xf fontId=\"0\" fillId=\"3\" borderId=\"1\" applyFill=\"1\" applyBorder=\"1\" applyAlignment=\"1\"><alignment vertical=\"top\" wrapText=\"1\"/></xf>"
        "<xf fontId=\"0\" fillId=\"4\" borderId=\"1\" applyFill=\"1\" applyBorder=\"1\" applyAlignment=\"1\"><alignment vertical=\"top\" wrapText=\"1\"/></xf>"
        "<xf fontId=\"2\" borderId=\"1\" applyFont=\"1\" applyBorder=\"1\" applyAlignment=\"1\"><alignment vertical=\"top\" wrapText=\"1\"/></xf>"
        "<xf fontId=\"3\" applyFont=\"1\"/></cellXfs><cellStyles count=\"1\"><cellStyle name=\"Normal\" xfId=\"0\" builtinId=\"0\"/></cellStyles></styleSheet>");
    for (size_t i = 0; i < sheets.size(); ++i) {
        const Sheet& s = sheets[i];
        std::string x = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n<worksheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\">";
        if (!s.freeze.empty()) {
            const int fr = std::atoi(s.freeze.c_str() + 1) - 1, fc = s.freeze[0] - 'A';
            x += "<sheetViews><sheetView workbookViewId=\"0\"><pane" + (fc > 0 ? " xSplit=\"" + std::to_string(fc) + "\"" : std::string()) + (fr > 0 ? " ySplit=\"" + std::to_string(fr) + "\"" : std::string())
               + " topLeftCell=\"" + s.freeze + "\" activePane=\"bottomRight\" state=\"frozen\"/></sheetView></sheetViews>";
        }
        if (!s.widths.empty()) { x += "<cols>"; for (size_t c = 0; c < s.widths.size(); ++c) { char b[96]; std::snprintf(b, sizeof b, "<col min=\"%zu\" max=\"%zu\" width=\"%.1f\" customWidth=\"1\"/>", c + 1, c + 1, s.widths[c]); x += b; } x += "</cols>"; }
        x += "<sheetData>";
        for (size_t r = 0; r < s.rows.size(); ++r) {
            x += "<row r=\"" + std::to_string(r + 1) + "\">";
            for (size_t c = 0; c < s.rows[r].size(); ++c) {
                const Cell& e = s.rows[r][c];
                if (e.text.empty() && e.style == Plain) continue;
                const std::string ref = col(c) + std::to_string(r + 1), st = e.style ? " s=\"" + std::to_string(e.style) + "\"" : "";
                if (e.text.empty()) x += "<c r=\"" + ref + "\"" + st + "/>";
                else if (e.number) x += "<c r=\"" + ref + "\"" + st + "><v>" + esc(e.text) + "</v></c>";
                else x += "<c r=\"" + ref + "\"" + st + " t=\"inlineStr\"><is><t xml:space=\"preserve\">" + esc(e.text) + "</t></is></c>";
            }
            x += "</row>";
        }
        x += "</sheetData>";
        if (!s.lists.empty()) {
            x += "<dataValidations count=\"" + std::to_string(s.lists.size()) + "\">";
            for (const auto& l : s.lists) { std::string f; for (auto& it : l.items) f += (f.empty() ? "" : ",") + it; x += "<dataValidation type=\"list\" allowBlank=\"1\" showErrorMessage=\"1\" sqref=\"" + l.range + "\"><formula1>\"" + esc(f) + "\"</formula1></dataValidation>"; }
            x += "</dataValidations>";
        }
        x += "</worksheet>";
        z.add("xl/worksheets/sheet" + std::to_string(i + 1) + ".xml", x);
    }
    return z.finish();
}

inline bool writeFile(const std::string& path, const std::vector<Sheet>& sheets, std::string& err) {
    const std::vector<uint8_t> d = build(sheets);
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) { err = "не удалось записать файл: " + path; return false; }
    f.write(reinterpret_cast<const char*>(d.data()), static_cast<std::streamsize>(d.size()));
    if (!f) { err = "ошибка записи: " + path; return false; }
    return true;
}

}  // namespace xlsxw
}  // namespace dp
