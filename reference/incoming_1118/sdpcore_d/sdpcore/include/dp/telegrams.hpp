// Разбор не-NMEA телеграмм относительных систем положения (v0.22).
// Источники форматов: Fanbeam 5 «Installation and commissioning guide», гл. 3.1 (MDL, CEGELEC, MDL+контрольная сумма);
// Artemis Mk IV, гл. 11.2.3 (ASCII-16/17/22). Только разбор строки — привязка к координатам делается в SensorHub.
// ⚠ Двоичные форматы (KA-BCD, Kongsberg/Nautronix «legacy») не разбираются: без образца с прибора ошибка неизбежна.
#pragma once
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

namespace dp {
namespace tele {

// Результат разбора одной телеграммы «дальность — пеленг» (Fanbeam) или «дальность — азимут» (Artemis).
struct RangeBearing {
    bool ok = false;           // строка разобрана
    std::string why;           // причина отказа (если ok == false)
    bool valid = true;         // прибор отметил данные как достоверные
    int target = 0;            // номер мишени (Fanbeam)
    double rangeM = 0;
    double bearingDeg = 0;     // Fanbeam: пеленг на мишень от носа; Artemis ASCII-22: пеленг мобильной антенны (только показ)
    bool hasAzimuth = false;   // Artemis: азимут передан (символы «*» — азимута нет)
    double azimuthDeg = 0;     // Artemis: азимут мобильной станции от севера (направление от Fix-станции на мобильную)
    int status = -1;           // Artemis ASCII-22: статус 0…3
    std::string type;          // MDL, MDL+CS, CEG17, CEG16, ART16, ART17, ART22
};

inline std::string trimEol(std::string s) {
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n')) s.pop_back();
    return s;
}
inline std::vector<std::string> tokens(const std::string& s) {
    std::vector<std::string> r; std::string t;
    for (char c : s) { if (c == ' ' || c == '\t') { if (!t.empty()) r.push_back(t); t.clear(); } else t += c; }
    if (!t.empty()) r.push_back(t);
    return r;
}
inline bool allDigits(const std::string& s) { if (s.empty()) return false; for (char c : s) if (!std::isdigit(static_cast<unsigned char>(c))) return false; return true; }
inline bool toNum(const std::string& s, double& v) {
    if (s.empty()) return false;
    char* e = nullptr; v = std::strtod(s.c_str(), &e);
    return e && *e == '\0' && std::isfinite(v);
}
inline RangeBearing fail(const std::string& type, const std::string& why) { RangeBearing r; r.type = type; r.why = why; return r; }

// XOR символов 1…18 (формат «MDL Standard c/w Checksum»), результат — число 0…255.
inline unsigned xor18(const std::string& line) {
    unsigned x = 0; for (size_t i = 0; i < 18 && i < line.size(); ++i) x ^= static_cast<unsigned char>(line[i]);
    return x;
}

// MDL Standard: «NN RRRR.RR BBB.BB»; с контрольной суммой — добавляется «CC» (две шестнадцатеричные цифры).
inline RangeBearing parseMdl(const std::string& raw) {
    const std::string line = trimEol(raw);
    const auto t = tokens(line);
    if (t.size() != 3 && t.size() != 4) return fail("MDL", "MDL: ожидается «NN RRRR.RR BBB.BB» [CC]");
    RangeBearing r; r.type = t.size() == 4 ? "MDL+CS" : "MDL";
    double R, B;
    if (!allDigits(t[0]) || t[0].size() > 2 || !toNum(t[1], R) || !toNum(t[2], B) || t[1].find('.') == std::string::npos || t[2].find('.') == std::string::npos) return fail(r.type, "MDL: неверные числа");
    if (R < 0 || R > 99999.0 || B < 0 || B >= 360.001) return fail(r.type, "MDL: дальность или пеленг вне диапазона");
    if (t.size() == 4) {
        if (t[3].size() != 2 || !std::isxdigit(static_cast<unsigned char>(t[3][0])) || !std::isxdigit(static_cast<unsigned char>(t[3][1]))) return fail(r.type, "MDL: контрольная сумма — две шестнадцатеричные цифры");
        if (line.size() < 18) return fail(r.type, "MDL: строка короче 18 символов, контрольную сумму проверить нельзя");
        const unsigned cs = static_cast<unsigned>(std::strtoul(t[3].c_str(), nullptr, 16));
        if (cs != xor18(line)) return fail(r.type, "MDL: контрольная сумма не совпала");
    }
    r.ok = true; r.target = std::atoi(t[0].c_str()); r.rangeM = R; r.bearingDeg = B;
    return r;
}

// CEGELEC Standard (ASCII 17): «RRRRRR BBBBBB N», дальность ×10, пеленг ×1000. Legacy (ASCII 16): пеленг ×100, пять цифр.
inline RangeBearing parseCegelec(const std::string& raw, bool legacy) {
    const char* nm = legacy ? "CEG16" : "CEG17";
    const auto t = tokens(trimEol(raw));
    if (t.size() != 3) return fail(nm, std::string(nm) + ": ожидается «RRRRRR BBBBBB N»");
    const size_t bl = legacy ? 5 : 6;
    if (t[0].size() != 6 || t[1].size() != bl || t[2].size() != 1 || !allDigits(t[0]) || !allDigits(t[1]) || !allDigits(t[2])) return fail(nm, std::string(nm) + ": неверная длина или не цифры");
    RangeBearing r; r.type = nm; r.ok = true;
    r.rangeM = std::atof(t[0].c_str()) / 10.0;
    r.bearingDeg = std::atof(t[1].c_str()) / (legacy ? 100.0 : 1000.0);
    r.target = std::atoi(t[2].c_str());
    if (r.bearingDeg >= 360.001) return fail(nm, std::string(nm) + ": пеленг вне диапазона");
    return r;
}

// Artemis Mk IV, периферийный порт. ASCII-16: «DDDDDD AAAAA R», ASCII-17: «DDDDDD AAAAAA R», ASCII-22: «DDDDDD BBBB AAAAAA S».
// D — дальность ×10 (6 цифр), A — азимут (×100 или ×1000; «*» — азимута нет), B — пеленг мобильной антенны ×10, R — READY, S — статус.
// Все нули при потере связи со встречной станцией: READY = 0 (для ASCII-22 статус 0) — данные недостоверны.
inline RangeBearing parseArtemis(const std::string& raw, int kind /*16, 17, 22*/) {
    const std::string nm = "ART" + std::to_string(kind);
    const auto t = tokens(trimEol(raw));
    const size_t want = kind == 22 ? 4 : 3;
    if (t.size() != want) return fail(nm, nm + ": неверное число полей");
    const size_t azLen = kind == 16 ? 5 : 6;
    const std::string& az = t[kind == 22 ? 2 : 1];
    const std::string& tail = t[want - 1];
    if (t[0].size() != 6 || !allDigits(t[0])) return fail(nm, nm + ": дальность — шесть цифр");
    if (az.size() != azLen) return fail(nm, nm + ": неверная длина азимута");
    if (tail.size() != 1 || !allDigits(tail)) return fail(nm, nm + ": неверный признак READY/статус");
    RangeBearing r; r.type = nm; r.ok = true;
    r.rangeM = std::atof(t[0].c_str()) / 10.0;
    if (kind == 22) {
        if (t[1].size() != 4 || !allDigits(t[1])) return fail(nm, nm + ": неверный пеленг мобильной антенны");
        r.bearingDeg = std::atof(t[1].c_str()) / 10.0;
        r.status = tail[0] - '0';
        r.valid = r.status == 1 || r.status == 2 || r.status == 3;
    } else {
        r.valid = tail[0] == '1';
    }
    if (allDigits(az)) { r.hasAzimuth = true; r.azimuthDeg = std::atof(az.c_str()) / (kind == 16 ? 100.0 : 1000.0); if (r.azimuthDeg >= 360.001) return fail(nm, nm + ": азимут вне диапазона"); }
    return r;
}

// Имена «сырых» (не NMEA) типов, которые можно указать в списке принимаемых посылок.
inline bool isRawType(const std::string& t) { return t == "MDL" || t == "CEG17" || t == "CEG16" || t == "ART16" || t == "ART17" || t == "ART22"; }
inline RangeBearing parseRaw(const std::string& type, const std::string& line) {
    if (type == "MDL") return parseMdl(line);
    if (type == "CEG17") return parseCegelec(line, false);
    if (type == "CEG16") return parseCegelec(line, true);
    if (type == "ART16") return parseArtemis(line, 16);
    if (type == "ART17") return parseArtemis(line, 17);
    if (type == "ART22") return parseArtemis(line, 22);
    return fail(type, "неизвестный формат");
}

}  // namespace tele
}  // namespace dp
