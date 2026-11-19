// Чтение и запись описания судна и движителей в JSON. Одни и те же функции используются для файла настроек
// и для профиля судна администратора, чтобы имена полей и проверки не расходились.
#pragma once
#include <cstdio>
#include <string>

#include "dp/json.hpp"
#include "dp/types.hpp"

namespace dp {

inline void readVessel(json::Obj& o, VesselCfg& v) {
    v.name = o.str("name", v.name); v.length = o.num("length", v.length); v.beam = o.num("beam", v.beam); v.draft = o.num("draft", v.draft);
    v.mass = o.num("mass", v.mass); v.Ixx = o.num("Ixx", v.Ixx); v.Iyy = o.num("Iyy", v.Iyy); v.Izz = o.num("Izz", v.Izz);
    v.addSurge = o.num("addSurge", v.addSurge); v.addSway = o.num("addSway", v.addSway); v.addHeave = o.num("addHeave", v.addHeave);
    v.addRoll = o.num("addRoll", v.addRoll); v.addPitch = o.num("addPitch", v.addPitch); v.addYaw = o.num("addYaw", v.addYaw);
    v.Xu = o.num("Xu", v.Xu); v.Yv = o.num("Yv", v.Yv); v.Zw = o.num("Zw", v.Zw); v.Kp = o.num("Kp", v.Kp); v.Mq = o.num("Mq", v.Mq); v.Nr = o.num("Nr", v.Nr);
    v.Xuu = o.num("Xuu", v.Xuu); v.Yvv = o.num("Yvv", v.Yvv); v.Nrr = o.num("Nrr", v.Nrr);
    v.gmT = o.num("gmT", v.gmT); v.gmL = o.num("gmL", v.gmL); v.waterplaneCoef = o.num("waterplaneCoef", v.waterplaneCoef);
    v.frontalArea = o.num("frontalArea", v.frontalArea); v.lateralArea = o.num("lateralArea", v.lateralArea);
    v.cx = o.num("cx", v.cx); v.cy = o.num("cy", v.cy); v.cn = o.num("cn", v.cn);
    v.waveReflection = o.num("waveReflection", v.waveReflection);
    v.lpp = o.num("lpp", v.lpp); v.los = o.num("los", v.los); v.xLos = o.num("xLos", v.xLos); v.waveBeta = o.num("waveBeta", v.waveBeta); v.waveCwla = o.num("waveCwla", v.waveCwla);
    v.waveModel = static_cast<int>(o.num("waveModel", v.waveModel)); v.xWind = o.num("xWind", v.xWind); v.windMomentRS = o.boolean("windMomentRS", v.windMomentRS);
    v.currentLateralArea = o.num("currentLateralArea", v.currentLateralArea); v.xCurrent = o.num("xCurrent", v.xCurrent); v.currentRS = o.boolean("currentRS", v.currentRS);
    v.raoHeave = o.num("raoHeave", v.raoHeave); v.raoRoll = o.num("raoRoll", v.raoRoll); v.raoPitch = o.num("raoPitch", v.raoPitch);
    if (o.has("windTable")) {
        auto w = o.obj("windTable");
        v.windTable.stepDeg = w.num("stepDeg", 10); v.windTable.cx = w.nums("cx"); v.windTable.cy = w.nums("cy"); v.windTable.cn = w.nums("cn");
        w.finish();
    }
    o.finish();
}

inline void readThruster(json::Obj& o, ThrusterCfg& t) {
    t.id = o.str("id", "");
    const std::string ty = o.str("type", "fixed");
    if (ty == "fixed") t.type = ThrusterType::Fixed; else if (ty == "tunnel") t.type = ThrusterType::Tunnel;
    else if (ty == "azimuth") t.type = ThrusterType::Azimuth; else o.error("тип движителя «" + ty + "» неизвестен (fixed, tunnel, azimuth)");
    t.enabled = o.boolean("enabled", true);
    t.x = o.num("x", 0); t.y = o.num("y", 0); t.z = o.num("z", 0);
    t.fixedAngleDeg = o.num("fixedAngleDeg", 0); t.maxForward = o.num("maxForward", 0); t.maxReverse = o.num("maxReverse", t.maxForward);
    t.timeConstS = o.num("timeConstS", 2.0); t.rateFracPerS = o.num("rateFracPerS", 0.25); t.azRateDegS = o.num("azRateDegS", 10.0);
    t.fbDiffPct = o.num("fbDiffPct", 25.0); t.fbDiffAngleDeg = o.num("fbDiffAngleDeg", 30.0); t.fbDiffDelayS = o.num("fbDiffDelayS", 30.0);
    t.weight = o.num("weight", 1.0); t.powerMaxKW = o.num("powerMaxKW", 0.0);
    t.limitPct = o.num("limitPct", 100.0); t.idlePct = o.num("idlePct", 0.0); t.fadeKn = o.num("fadeKn", 0.0); t.fadeFloorPct = o.num("fadeFloorPct", 50.0); t.bus = o.integer("bus", -1); t.parkDeg = o.num("parkDeg", 1e9); t.failGroups = o.str("failGroups", "");
    t.prefDeg = o.num("prefDeg", 1e9); t.preloadPct = o.num("preloadPct", 0.0); t.azDeadDeg = o.num("azDeadDeg", 0.0); t.pctDeadPct = o.num("pctDeadPct", 0.0); t.hardCoreDeg = o.num("hardCoreDeg", 360.0);
    for (auto& pr : o.numLists("curve")) { if (pr.size() == 3) t.curve.push_back({pr[0], pr[1], pr[2]}); else o.error("точка кривой задаётся тройкой [обороты %, упор %, мощность %]"); }
    for (auto& pr : o.numLists("forbidden")) { if (pr.size() == 2) t.forbidden.push_back({pr[0], pr[1]}); else o.error("запретный сектор задаётся парой [от, до]"); }
    for (auto& pr : o.numLists("effAngle")) { if (pr.size() == 2 && pr[1] >= 0 && pr[1] <= 100) t.effAngle.push_back({pr[0], pr[1]}); else o.error("точка КПД по углу задаётся парой [угол, %] (КПД 0…100)"); }
    t.effIsLimit = o.boolean("effIsLimit", false);
    if (t.id.empty()) o.error("не задан id движителя");
    if (t.maxForward <= 0) o.error("maxForward должен быть больше нуля");
    o.finish();
}

namespace vio {
inline std::string esc(const std::string& s) {
    std::string r;
    for (unsigned char c : s) {
        if (c == '"') r += "\\\""; else if (c == '\\') r += "\\\\"; else if (c == '\n') r += "\\n"; else if (c == '\r') r += "\\r"; else if (c == '\t') r += "\\t";
        else if (c < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", c); r += b; } else r += static_cast<char>(c);
    }
    return r;
}
inline std::string num(double v) { if (!(v == v) || v > 1e300 || v < -1e300) v = 0; char b[40]; std::snprintf(b, sizeof b, "%.12g", v); return b; }
inline std::string arr(const std::vector<double>& a) { std::string r = "["; for (size_t i = 0; i < a.size(); ++i) r += (i ? "," : "") + num(a[i]); return r + "]"; }
}  // namespace vio

inline std::string writeVessel(const VesselCfg& v) {
    using namespace vio;
    std::string s = "{";
    auto n = [&](const char* k, double x) { s += "\"" + std::string(k) + "\":" + num(x) + ","; };
    s += "\"name\":\"" + esc(v.name) + "\",";
    n("length", v.length); n("beam", v.beam); n("draft", v.draft); n("mass", v.mass); n("Ixx", v.Ixx); n("Iyy", v.Iyy); n("Izz", v.Izz);
    n("addSurge", v.addSurge); n("addSway", v.addSway); n("addHeave", v.addHeave); n("addRoll", v.addRoll); n("addPitch", v.addPitch); n("addYaw", v.addYaw);
    n("Xu", v.Xu); n("Yv", v.Yv); n("Zw", v.Zw); n("Kp", v.Kp); n("Mq", v.Mq); n("Nr", v.Nr); n("Xuu", v.Xuu); n("Yvv", v.Yvv); n("Nrr", v.Nrr);
    n("gmT", v.gmT); n("gmL", v.gmL); n("waterplaneCoef", v.waterplaneCoef);
    n("frontalArea", v.frontalArea); n("lateralArea", v.lateralArea); n("cx", v.cx); n("cy", v.cy); n("cn", v.cn);
    n("lpp", v.lpp); n("los", v.los); n("xLos", v.xLos); n("waveBeta", v.waveBeta); n("waveCwla", v.waveCwla); n("waveModel", v.waveModel); n("xWind", v.xWind);
    s += "\"windMomentRS\":" + std::string(v.windMomentRS ? "true" : "false") + ",";
    n("currentLateralArea", v.currentLateralArea); n("xCurrent", v.xCurrent);
    s += "\"currentRS\":" + std::string(v.currentRS ? "true" : "false") + ",";
    n("waveReflection", v.waveReflection); n("raoHeave", v.raoHeave); n("raoRoll", v.raoRoll); n("raoPitch", v.raoPitch);
    s += "\"windTable\":{\"stepDeg\":" + num(v.windTable.stepDeg) + ",\"cx\":" + arr(v.windTable.cx) + ",\"cy\":" + arr(v.windTable.cy) + ",\"cn\":" + arr(v.windTable.cn) + "}}";
    return s;
}

inline std::string writeThruster(const ThrusterCfg& t) {
    using namespace vio;
    std::string s = "{\"id\":\"" + esc(t.id) + "\",\"type\":\"" + (t.type == ThrusterType::Fixed ? "fixed" : t.type == ThrusterType::Tunnel ? "tunnel" : "azimuth") + "\"";
    s += ",\"enabled\":" + std::string(t.enabled ? "true" : "false");
    s += ",\"x\":" + num(t.x) + ",\"y\":" + num(t.y) + ",\"z\":" + num(t.z) + ",\"fixedAngleDeg\":" + num(t.fixedAngleDeg);
    s += ",\"maxForward\":" + num(t.maxForward) + ",\"maxReverse\":" + num(t.maxReverse) + ",\"timeConstS\":" + num(t.timeConstS);
    s += ",\"rateFracPerS\":" + num(t.rateFracPerS) + ",\"azRateDegS\":" + num(t.azRateDegS) + ",\"weight\":" + num(t.weight) + ",\"powerMaxKW\":" + num(t.powerMaxKW);
    if (t.fbDiffPct != 25.0) s += ",\"fbDiffPct\":" + num(t.fbDiffPct);
    if (t.fbDiffAngleDeg != 30.0) s += ",\"fbDiffAngleDeg\":" + num(t.fbDiffAngleDeg);
    if (t.fbDiffDelayS != 30.0) s += ",\"fbDiffDelayS\":" + num(t.fbDiffDelayS);
    if (t.parkDeg < 1e8) s += ",\"parkDeg\":" + num(t.parkDeg);
    if (t.prefDeg < 1e8) s += ",\"prefDeg\":" + num(t.prefDeg);
    if (t.preloadPct != 0.0) s += ",\"preloadPct\":" + num(t.preloadPct);
    if (t.azDeadDeg != 0.0) s += ",\"azDeadDeg\":" + num(t.azDeadDeg);
    if (t.pctDeadPct != 0.0) s += ",\"pctDeadPct\":" + num(t.pctDeadPct);
    if (t.bus >= 0) s += ",\"bus\":" + std::to_string(t.bus);
    if (!t.failGroups.empty()) s += ",\"failGroups\":\"" + vio::esc(t.failGroups) + "\"";
    if (t.limitPct != 100.0) s += ",\"limitPct\":" + num(t.limitPct);
    if (t.idlePct != 0.0) s += ",\"idlePct\":" + num(t.idlePct);
    if (t.fadeKn != 0.0) s += ",\"fadeKn\":" + num(t.fadeKn) + ",\"fadeFloorPct\":" + num(t.fadeFloorPct);
    if (!t.curve.empty()) {
        s += ",\"curve\":[";
        for (size_t i = 0; i < t.curve.size(); ++i) s += (i ? "," : "") + std::string("[") + num(t.curve[i].rpm) + "," + num(t.curve[i].thrust) + "," + num(t.curve[i].power) + "]";
        s += "]";
    }
    if (!t.effAngle.empty()) {
        s += ",\"effAngle\":[";
        for (size_t i = 0; i < t.effAngle.size(); ++i) s += (i ? "," : "") + std::string("[") + num(t.effAngle[i].angleDeg) + "," + num(t.effAngle[i].pct) + "]";
        s += "]";
        if (t.effIsLimit) s += ",\"effIsLimit\":true";
    }
    s += ",\"hardCoreDeg\":" + num(t.hardCoreDeg);
    s += ",\"forbidden\":[";
    for (size_t i = 0; i < t.forbidden.size(); ++i) s += (i ? "," : "") + std::string("[") + num(t.forbidden[i].fromDeg) + "," + num(t.forbidden[i].toDeg) + "]";
    return s + "]}";
}

}  // namespace dp
