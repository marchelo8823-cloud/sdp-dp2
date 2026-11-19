// Разбор предложений NMEA 0183 и формирование их для испытаний.
#pragma once
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#include "dp/math.hpp"

namespace dp {
namespace nmea {

struct Sentence {
    std::string talker;           // два символа, например "GP", "HE"; для проприетарных — "P"
    std::string type;             // тип: "GGA", "HDT"; для проприетарных — всё после 'P' ("SXN", "HTRO")
    std::vector<std::string> f;   // поля после типа
    bool proprietary = false;
    bool hadChecksum = false;
};

inline unsigned char checksum(const std::string& body) {
    unsigned char c = 0;
    for (char ch : body) c ^= static_cast<unsigned char>(ch);
    return c;
}

// Формирует строку "$body*CS\r\n".
inline std::string make(const std::string& body) {
    char cs[8];
    std::snprintf(cs, sizeof cs, "*%02X", checksum(body));
    return "$" + body + cs + "\r\n";
}

// Разбор одной строки. requireChecksum — отбрасывать предложения без контрольной суммы.
inline std::optional<Sentence> parse(std::string line, bool requireChecksum = true) {
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n' || line.back() == ' ')) line.pop_back();
    size_t s = line.find('$');
    if (s == std::string::npos) return std::nullopt;
    line = line.substr(s + 1);
    Sentence r;
    const size_t star = line.find('*');
    if (star != std::string::npos) {
        if (line.size() < star + 3) return std::nullopt;
        const unsigned long given = std::strtoul(line.substr(star + 1, 2).c_str(), nullptr, 16);
        if (given != checksum(line.substr(0, star))) return std::nullopt;
        r.hadChecksum = true;
        line = line.substr(0, star);
    } else if (requireChecksum) {
        return std::nullopt;
    }
    std::vector<std::string> parts;
    size_t p = 0;
    while (true) {
        const size_t q = line.find(',', p);
        if (q == std::string::npos) { parts.push_back(line.substr(p)); break; }
        parts.push_back(line.substr(p, q - p));
        p = q + 1;
    }
    const std::string& head = parts[0];
    if (head.size() < 3) return std::nullopt;
    if (head[0] == 'P' && head.size() >= 4) { r.proprietary = true; r.talker = "P"; r.type = head.substr(1); }
    else { r.talker = head.substr(0, 2); r.type = head.substr(2); }
    r.f.assign(parts.begin() + 1, parts.end());
    return r;
}

inline bool toDouble(const std::string& s, double& v) {
    if (s.empty()) return false;
    char* e = nullptr;
    v = std::strtod(s.c_str(), &e);
    return e && *e == '\0';
}

// Широта/долгота "ddmm.mmmm"/"dddmm.mmmm" + полушарие → градусы.
inline bool parseAngle(const std::string& v, const std::string& hemi, bool isLat, double& deg) {
    double x;
    if (!toDouble(v, x) || hemi.empty()) return false;
    const double d = std::floor(x / 100.0);
    const double m = x - d * 100.0;
    if (m < 0 || m >= 60.0) return false;
    deg = d + m / 60.0;
    if (isLat && deg > 90.0) return false;
    if (!isLat && deg > 180.0) return false;
    const char h = hemi[0];
    if (h == 'S' || h == 'W') deg = -deg;
    else if (h != 'N' && h != 'E') return false;
    return true;
}

// Для испытаний: градусы → поля NMEA.
inline std::string fmtLat(double deg) {
    const double a = std::fabs(deg); const int d = static_cast<int>(a); const double m = (a - d) * 60.0;
    char b[32]; std::snprintf(b, sizeof b, "%02d%09.6f,%c", d, m, deg >= 0 ? 'N' : 'S'); return b;
}
inline std::string fmtLon(double deg) {
    const double a = std::fabs(deg); const int d = static_cast<int>(a); const double m = (a - d) * 60.0;
    char b[32]; std::snprintf(b, sizeof b, "%03d%09.6f,%c", d, m, deg >= 0 ? 'E' : 'W'); return b;
}

}  // namespace nmea

// ---------------------------------------------------------------- локальная система координат
class LocalFrame {
public:
    bool valid() const { return valid_; }
    double lat0() const { return lat0_; }
    double lon0() const { return lon0_; }
    void setOrigin(double latDeg, double lonDeg) {
        lat0_ = latDeg; lon0_ = lonDeg; valid_ = true;
        const double a = 6378137.0, e2 = 0.00669437999014;
        const double s = std::sin(deg2rad(latDeg));
        const double w = std::sqrt(1.0 - e2 * s * s);
        mPerDegLat_ = deg2rad(1.0) * a * (1.0 - e2) / (w * w * w);
        mPerDegLon_ = deg2rad(1.0) * a * std::cos(deg2rad(latDeg)) / w;
    }
    void toLocal(double latDeg, double lonDeg, double& n, double& e) const {
        n = (latDeg - lat0_) * mPerDegLat_;
        e = (lonDeg - lon0_) * mPerDegLon_;
    }
    void toGeo(double n, double e, double& latDeg, double& lonDeg) const {
        latDeg = lat0_ + n / mPerDegLat_;
        lonDeg = lon0_ + e / mPerDegLon_;
    }
private:
    bool valid_ = false;
    double lat0_ = 0, lon0_ = 0, mPerDegLat_ = 111000, mPerDegLon_ = 111000;
};

}  // namespace dp
