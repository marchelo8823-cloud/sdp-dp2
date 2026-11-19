// Преобразование измерений имитатора в предложения NMEA (чтобы в имитации данные шли тем же путём, что и с судна).
#pragma once
#include "dp/hub.hpp"
#include "dp/sim.hpp"

namespace dp {

struct NmeaLine { std::string channel, sentence; };

inline std::vector<NmeaLine> encodeNmea(const SensorFrame& fr, const LocalFrame& lf) {
    std::vector<NmeaLine> out;
    for (auto& p : fr.pos) if (p.fresh) {
        if (p.type == PosType::Gnss) {
            double la, lo; lf.toGeo(p.n, p.e, la, lo);
            out.push_back({p.id, nmea::make("GPGGA,000000," + nmea::fmtLat(la) + "," + nmea::fmtLon(lo) + ",2,10,0.8,0,M,0,M,,")});
        } else {
            char b[96]; std::snprintf(b, sizeof b, "PDPREL,%.3f,%.3f", p.n, p.e);
            out.push_back({p.id, nmea::make(b)});
        }
    }
    for (auto& h : fr.hdg) if (h.fresh) {
        char b[64]; std::snprintf(b, sizeof b, "HEHDT,%.2f,T", std::fmod(rad2deg(h.psi) + 360.0, 360.0));
        out.push_back({h.id, nmea::make(b)});
    }
    for (auto& w : fr.wind) if (w.fresh) {
        char b[64]; std::snprintf(b, sizeof b, "WIMWV,%.1f,R,%.2f,M,A", std::fmod(rad2deg(w.fromRel) + 360.0, 360.0), w.speed);
        out.push_back({w.id, nmea::make(b)});
    }
    for (auto& m : fr.mru) if (m.fresh) {
        char b[96]; std::snprintf(b, sizeof b, "PSXN,23,%.3f,%.3f,0.0,%.3f", rad2deg(m.roll), rad2deg(m.pitch), m.heave);
        out.push_back({m.id, nmea::make(b)});
    }
    return out;
}

}  // namespace dp
