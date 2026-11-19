// Профиль судна: данные, которые вводит администратор и которые питают математическую модель.
// Состав: корпус и посадка (осадка, масса), парусность, динамика (инерция, присоединённые массы, демпфирование),
// пропульсивный комплекс (движители) и сведения об условиях, влияющих на судно.
// Здесь же проверка введённых значений на физическую согласованность и оценки, которые помогают модели
// (масса по осадке, парусность при данной осадке, предел удержания по ветру).
#pragma once
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

#include "dp/thrust.hpp"
#include "dp/vesselio.hpp"

namespace dp {

// Условия загрузки и внешние факторы. Поле «влияет на модель» указано в комментарии.
struct LoadingInfo {
    std::string condition = "Рабочее";   // название состояния загрузки (для журнала и экрана)
    double draftFwd = 0, draftAft = 0;   // осадка носом и кормой, м; 0 — взять vessel.draft. Среднее идёт в модель (draft) и в оценку массы
    bool massFromDraft = false;          // true — массу вычислить по осадке: ρ·Cb·L·B·T
    double blockCoef = 0.65;             // коэффициент общей полноты Cb (для оценки массы)
    double waterDensity = 1025;          // плотность воды, кг/м³ (участвует в оценке массы; в динамике модели постоянна — 1025)
    double refDraft = 0;                 // осадка, при которой измерены площади парусности; 0 — поправки по осадке нет
    double deckFrontal = 0, deckLateral = 0;   // добавка парусности от палубного груза/надстроек, м² (идёт в модель)
    double yawGyrationFrac = 0;          // радиус инерции по рысканью, доля L; 0 — оставить Izz как задан (идёт в модель)
    // --- только справочные сведения: в модели не используются (⚠ требуют доработки модели)
    double waterDepth = 0;               // глубина места, м: мелководье увеличивает присоединённые массы и демпфирование (пока не учтено)
    double freeboard = 0;                // надводный борт, м
    double windageHeight = 0;            // высота центра парусности над ватерлинией, м (нужна для крена, в 3 степенях свободы не используется)
    double iceThicknessCm = 0;           // лёд, см (не учтён)
};

struct VesselProfile {
    int version = 1;
    VesselCfg vessel;
    LoadingInfo loading;
    std::vector<ThrusterCfg> thrusters;
    std::string notes;
};

struct Issue { bool error = false; std::string field, text; };
struct Estimate { std::string label, value, note; };

inline VesselProfile defaultProfile() {
    VesselProfile p;
    p.vessel = VesselCfg{};
    return p;
}

// ---------------------------------------------------------------- JSON
inline void readLoading(json::Obj& o, LoadingInfo& l) {
    l.condition = o.str("condition", l.condition); l.draftFwd = o.num("draftFwd", l.draftFwd); l.draftAft = o.num("draftAft", l.draftAft);
    l.massFromDraft = o.boolean("massFromDraft", l.massFromDraft); l.blockCoef = o.num("blockCoef", l.blockCoef); l.waterDensity = o.num("waterDensity", l.waterDensity);
    l.refDraft = o.num("refDraft", l.refDraft); l.deckFrontal = o.num("deckFrontal", l.deckFrontal); l.deckLateral = o.num("deckLateral", l.deckLateral);
    l.yawGyrationFrac = o.num("yawGyrationFrac", l.yawGyrationFrac);
    l.waterDepth = o.num("waterDepth", l.waterDepth); l.freeboard = o.num("freeboard", l.freeboard); l.windageHeight = o.num("windageHeight", l.windageHeight);
    l.iceThicknessCm = o.num("iceThicknessCm", l.iceThicknessCm);
    o.finish();
}

// Разбор значения json::Value в профиль. Ошибки формата — в rep.errors.
inline bool parseProfile(const json::Value& root, VesselProfile& p, json::Report& rep) {
    json::Obj R(&root, "профиль", &rep);
    p = VesselProfile{};
    p.version = R.integer("version", 1);
    { auto o = R.obj("vessel"); readVessel(o, p.vessel); }
    { auto o = R.obj("loading"); readLoading(o, p.loading); }
    for (auto o : R.objs("thrusters")) { ThrusterCfg t; readThruster(o, t); p.thrusters.push_back(t); }
    p.notes = R.str("notes", "");
    R.finish();
    if (p.version != 1) rep.errors.push_back("профиль: версия формата " + std::to_string(p.version) + " не поддерживается");
    return rep.errors.empty();
}
inline bool parseProfileText(const std::string& text, VesselProfile& p, json::Report& rep) {
    json::Value v; std::string err;
    if (!json::parse(text, v, err)) { rep.errors.push_back("профиль: " + err); return false; }
    return parseProfile(v, p, rep);
}

inline std::string writeLoading(const LoadingInfo& l) {
    using namespace vio;
    std::string s = "{\"condition\":\"" + esc(l.condition) + "\"";
    auto n = [&](const char* k, double x) { s += ",\"" + std::string(k) + "\":" + num(x); };
    n("draftFwd", l.draftFwd); n("draftAft", l.draftAft);
    s += ",\"massFromDraft\":" + std::string(l.massFromDraft ? "true" : "false");
    n("blockCoef", l.blockCoef); n("waterDensity", l.waterDensity); n("refDraft", l.refDraft); n("deckFrontal", l.deckFrontal); n("deckLateral", l.deckLateral);
    n("yawGyrationFrac", l.yawGyrationFrac); n("waterDepth", l.waterDepth); n("freeboard", l.freeboard); n("windageHeight", l.windageHeight); n("iceThicknessCm", l.iceThicknessCm);
    return s + "}";
}
inline std::string writeProfile(const VesselProfile& p) {
    std::string s = "{\"version\":" + std::to_string(p.version) + ",\"vessel\":" + writeVessel(p.vessel) + ",\"loading\":" + writeLoading(p.loading) + ",\"thrusters\":[";
    for (size_t i = 0; i < p.thrusters.size(); ++i) s += (i ? "," : "") + writeThruster(p.thrusters[i]);
    s += "]";
    return s + ",\"notes\":\"" + vio::esc(p.notes) + "\"}";
}

// Плоское представление «поле → значение» для сравнения двух профилей (журнал изменений).
inline std::map<std::string, std::string> flatten(const VesselProfile& p) {
    std::map<std::string, std::string> m;
    json::Value v; std::string e; json::parse(writeProfile(p), v, e);
    struct R { static void go(const json::Value& x, const std::string& path, std::map<std::string, std::string>& m) {
        if (x.t == json::Value::Obj) { for (auto& kv : x.o) go(kv.second, path.empty() ? kv.first : path + "." + kv.first, m); }
        else if (x.t == json::Value::Arr) {
            const bool simple = x.a.empty() || x.a[0].t == json::Value::Num;
            if (simple) { std::string s = "["; for (size_t i = 0; i < x.a.size(); ++i) s += (i ? "," : "") + vio::num(x.a[i].n); m[path] = s + "]"; }
            else for (size_t i = 0; i < x.a.size(); ++i) {
                std::string key = path + "[" + std::to_string(i) + "]";
                if (x.a[i].t == json::Value::Obj) { const json::Value* id = x.a[i].find("id"); if (id && id->t == json::Value::Str) key = path + "[" + id->s + "]"; }
                go(x.a[i], key, m);
            }
        }
        else if (x.t == json::Value::Num) m[path] = vio::num(x.n);
        else if (x.t == json::Value::Bool) m[path] = x.b ? "да" : "нет";
        else if (x.t == json::Value::Str) m[path] = x.s;
    } };
    R::go(v, "", m);
    return m;
}
inline std::vector<std::string> diffProfiles(const VesselProfile& a, const VesselProfile& b) {
    const auto fa = flatten(a), fb = flatten(b);
    std::vector<std::string> d;
    for (auto& kv : fb) { auto it = fa.find(kv.first); if (it == fa.end()) d.push_back(kv.first + ": (нет) → " + kv.second); else if (it->second != kv.second) d.push_back(kv.first + ": " + it->second + " → " + kv.second); }
    for (auto& kv : fa) if (!fb.count(kv.first)) d.push_back(kv.first + ": " + kv.second + " → (нет)");
    return d;
}

// ---------------------------------------------------------------- расчёты для модели
inline double meanDraft(const VesselProfile& p) {
    const LoadingInfo& l = p.loading;
    if (l.draftFwd > 0 && l.draftAft > 0) return 0.5 * (l.draftFwd + l.draftAft);
    return p.vessel.draft;
}

// Параметры судна, которые реально получает модель: с учётом осадки, условий загрузки и добавок парусности.
inline VesselCfg effectiveVessel(const VesselProfile& p) {
    VesselCfg v = p.vessel; const LoadingInfo& l = p.loading;
    const double T = meanDraft(p);
    v.draft = T;
    if (l.massFromDraft) v.mass = l.waterDensity * l.blockCoef * v.length * v.beam * T;
    double fr = v.frontalArea, la = v.lateralArea;
    if (l.refDraft > 0) {   // чем глубже осадка, тем меньше надводная часть: площадь меняется на L·ΔT и B·ΔT
        const double dT = T - l.refDraft;
        la = std::max(0.2 * la, la - dT * v.length);
        fr = std::max(0.2 * fr, fr - dT * v.beam);
    }
    v.frontalArea = fr + l.deckFrontal; v.lateralArea = la + l.deckLateral;
    if (l.yawGyrationFrac > 0) { const double k = l.yawGyrationFrac * v.length; v.Izz = v.mass * k * k; }
    return v;
}

inline std::string fmt(double x, int prec = 3) { char b[48]; std::snprintf(b, sizeof b, "%.*g", prec, x); return b; }

inline std::vector<Issue> validateProfile(const VesselProfile& p) {
    std::vector<Issue> r;
    auto err = [&](const std::string& f, const std::string& t) { r.push_back({true, f, t}); };
    auto warn = [&](const std::string& f, const std::string& t) { r.push_back({false, f, t}); };
    const VesselCfg& v = p.vessel; const LoadingInfo& l = p.loading;
    auto isnum = [](double x) { return std::isfinite(x); };
    // --- корпус
    if (!(v.length >= 5 && v.length <= 400)) err("vessel.length", "длина должна быть от 5 до 400 м");
    if (!(v.beam >= 1 && v.beam <= 80)) err("vessel.beam", "ширина должна быть от 1 до 80 м");
    if (!(v.draft >= 0.2 && v.draft <= 30)) err("vessel.draft", "осадка должна быть от 0,2 до 30 м");
    if (!(v.mass >= 1e4 && v.mass <= 1e9)) err("vessel.mass", "масса (водоизмещение) должна быть от 10 т до 1 000 000 т, в килограммах");
    if (isnum(v.length) && isnum(v.beam) && v.length < v.beam) warn("vessel.length", "длина меньше ширины: проверьте, что x — вдоль судна");
    if (l.draftFwd < 0 || l.draftAft < 0 || l.draftFwd > 30 || l.draftAft > 30) err("loading.draftFwd", "осадка носом/кормой должна быть от 0 до 30 м (0 — не задана)");
    if ((l.draftFwd > 0) != (l.draftAft > 0)) err("loading.draftFwd", "задайте осадку носом и кормой вместе, либо обе равными нулю");
    const double T = meanDraft(p);
    if (l.draftFwd > 0 && l.draftAft > 0 && std::fabs(l.draftFwd - l.draftAft) > 0.05 * v.length) warn("loading.draftAft", "дифферент больше 5 % длины: модель его не учитывает, берётся средняя осадка");
    if (!(l.blockCoef >= 0.2 && l.blockCoef <= 1.0)) err("loading.blockCoef", "коэффициент полноты Cb должен быть от 0,2 до 1,0");
    if (!(l.waterDensity >= 990 && l.waterDensity <= 1040)) err("loading.waterDensity", "плотность воды должна быть от 990 до 1040 кг/м³");
    const double mEff = effectiveVessel(p).mass;
    if (v.length > 0 && v.beam > 0 && T > 0) {
        const double cb = mEff / (l.waterDensity * v.length * v.beam * T);
        if (cb > 1.05 || cb < 0.15) err("vessel.mass", "масса не согласуется с размерами и осадкой: получается Cb = " + fmt(cb) + " (допустимо 0,15…1,05)");
        else if (cb > 0.9 || cb < 0.4) warn("vessel.mass", "необычная полнота корпуса: Cb = " + fmt(cb) + " (обычно 0,4…0,9). Проверьте массу или осадку");
    }
    if (!(v.gmT > 0 && v.gmT <= 30)) err("vessel.gmT", "поперечная метацентрическая высота должна быть больше 0 и не больше 30 м");
    else if (v.gmT < 0.3) warn("vessel.gmT", "очень малая остойчивость (GM < 0,3 м)");
    if (!(v.gmL > 0)) err("vessel.gmL", "продольная метацентрическая высота должна быть больше 0");
    if (!(v.waterplaneCoef >= 0.5 && v.waterplaneCoef <= 1.0)) err("vessel.waterplaneCoef", "коэффициент полноты ватерлинии должен быть от 0,5 до 1,0");
    // --- парусность
    if (!(v.frontalArea >= 1 && v.frontalArea <= 1e5)) err("vessel.frontalArea", "площадь парусности в носовой проекции должна быть от 1 до 100 000 м²");
    if (!(v.lateralArea >= 1 && v.lateralArea <= 1e5)) err("vessel.lateralArea", "площадь парусности в боковой проекции должна быть от 1 до 100 000 м²");
    if (v.frontalArea > v.lateralArea) warn("vessel.frontalArea", "носовая проекция больше боковой — для судна это необычно");
    if (!(v.cx >= 0 && v.cx <= 3) || !(v.cy >= 0 && v.cy <= 3)) err("vessel.cx", "коэффициенты ветра cx, cy должны быть от 0 до 3");
    if (!(std::fabs(v.cn) <= 1)) err("vessel.cn", "коэффициент момента cn должен быть по модулю не больше 1");
    if (l.deckFrontal < 0 || l.deckLateral < 0) err("loading.deckFrontal", "добавка парусности не может быть отрицательной");
    if (l.refDraft < 0 || l.refDraft > 30) err("loading.refDraft", "осадка отсчёта парусности должна быть от 0 до 30 м");
    if (l.refDraft > 0 && T > 0 && std::fabs(T - l.refDraft) > 0.5 * l.refDraft) warn("loading.refDraft", "осадка сильно отличается от осадки отсчёта парусности (более 50 %): поправка по осадке грубая");
    {
        const auto& w = v.windTable;
        if (!w.cx.empty()) {
            if (!(w.stepDeg > 0 && w.stepDeg <= 90)) err("vessel.windTable.stepDeg", "шаг таблицы ветра должен быть от 0 до 90°");
            else if (w.cx.size() < 2) err("vessel.windTable.cx", "в таблице ветра должно быть минимум две точки");
            else {
                if ((!w.cy.empty() && w.cy.size() != w.cx.size()) || (!w.cn.empty() && w.cn.size() != w.cx.size())) err("vessel.windTable.cy", "столбцы cx, cy, cn таблицы ветра должны иметь одинаковую длину");
                if ((w.cx.size() - 1) * w.stepDeg < 179.9) warn("vessel.windTable.cx", "таблица ветра не доходит до 180°: за пределами используется последнее значение");
            }
        }
    }
    // --- динамика
    struct F { const char* name; double val; };
    const F dyn[] = {{"Ixx", v.Ixx}, {"Iyy", v.Iyy}, {"Izz", v.Izz}, {"addSurge", v.addSurge}, {"addSway", v.addSway}, {"addHeave", v.addHeave}, {"addRoll", v.addRoll}, {"addPitch", v.addPitch}, {"addYaw", v.addYaw},
                     {"Xu", v.Xu}, {"Yv", v.Yv}, {"Zw", v.Zw}, {"Kp", v.Kp}, {"Mq", v.Mq}, {"Nr", v.Nr}, {"Xuu", v.Xuu}, {"Yvv", v.Yvv}, {"Nrr", v.Nrr}};
    for (auto& f : dyn) if (!(f.val >= 0) || !isnum(f.val)) err(std::string("vessel.") + f.name, "значение не может быть отрицательным (0 — оценить автоматически)");
    if (v.addSurge > 0 && v.addSurge > 3 * mEff) warn("vessel.addSurge", "присоединённая масса вдоль больше 3 масс судна — проверьте");
    if (v.addSway > 0 && v.addSway > 3 * mEff) warn("vessel.addSway", "присоединённая масса поперёк больше 3 масс судна — проверьте");
    if (l.yawGyrationFrac != 0 && !(l.yawGyrationFrac >= 0.15 && l.yawGyrationFrac <= 0.35)) err("loading.yawGyrationFrac", "радиус инерции по рысканью обычно 0,2…0,3 L; допустимо 0,15…0,35 (0 — не задан)");
    if (!(v.waveReflection >= 0 && v.waveReflection <= 1)) err("vessel.waveReflection", "коэффициент отражения волн должен быть от 0 до 1");
    // --- справочные
    if (l.waterDepth < 0 || l.freeboard < 0 || l.windageHeight < 0 || l.iceThicknessCm < 0) err("loading.waterDepth", "справочные величины не могут быть отрицательными");
    if (l.waterDepth > 0 && T > 0 && l.waterDepth < 3 * T) warn("loading.waterDepth", "глубина менее 3 осадок: мелководье заметно меняет динамику судна, а модель этого пока не учитывает");
    if (l.iceThicknessCm > 0) warn("loading.iceThicknessCm", "лёд в модели не учитывается");
    // --- движители
    if (p.thrusters.empty()) err("thrusters", "не описан ни один движитель");
    std::set<std::string> ids;
    for (size_t i = 0; i < p.thrusters.size(); ++i) {
        const ThrusterCfg& t = p.thrusters[i]; const std::string f = "thrusters[" + (t.id.empty() ? std::to_string(i) : t.id) + "].";
        if (t.id.empty()) err(f + "id", "не задан идентификатор");
        else if (!ids.insert(t.id).second) err(f + "id", "идентификатор повторяется");
        if (t.id.size() > 16) err(f + "id", "идентификатор длиннее 16 символов");
        if (!(t.maxForward > 0 && t.maxForward <= 5e7)) err(f + "maxForward", "максимальный упор вперёд должен быть от 0 до 50 МН (в ньютонах)");
        if (!(t.maxReverse >= 0 && t.maxReverse <= 5e7)) err(f + "maxReverse", "упор назад должен быть от 0 до 50 МН");
        if (t.maxReverse > 1.05 * t.maxForward && t.maxForward > 0) warn(f + "maxReverse", "упор назад больше упора вперёд — для винтов необычно");
        if (std::fabs(t.x) > 0.55 * v.length || std::fabs(t.y) > 0.6 * v.beam) warn(f + "x", "движитель вне габаритов корпуса — проверьте x (к носу) и y (на правый борт), в метрах от точки управления");
        if (!(t.timeConstS >= 0.1 && t.timeConstS <= 60)) err(f + "timeConstS", "постоянная времени упора должна быть от 0,1 до 60 с");
        if (!(t.rateFracPerS >= 0.01 && t.rateFracPerS <= 5)) err(f + "rateFracPerS", "скорость изменения упора должна быть от 0,01 до 5 максимума в секунду");
        if (t.type == ThrusterType::Azimuth && !(t.azRateDegS >= 0.5 && t.azRateDegS <= 180)) err(f + "azRateDegS", "скорость поворота колонки должна быть от 0,5 до 180 °/с");
        if (!(t.fbDiffPct >= 1 && t.fbDiffPct <= 100)) err(f + "fbDiffPct", "порог расхождения упора должен быть от 1 до 100 %");
        if (!(t.fbDiffAngleDeg >= 1 && t.fbDiffAngleDeg <= 180)) err(f + "fbDiffAngleDeg", "порог расхождения угла должен быть от 1 до 180°");
        if (!(t.fbDiffDelayS >= 1 && t.fbDiffDelayS <= 300)) err(f + "fbDiffDelayS", "выдержка расхождения должна быть от 1 до 300 с");
        if (!(t.powerMaxKW >= 0 && t.powerMaxKW <= 1e5)) err(f + "powerMaxKW", "мощность должна быть от 0 до 100 000 кВт");
        if (t.parkDeg < 1e8) {
            if (t.type != ThrusterType::Azimuth) warn(f + "parkDeg", "стояночный угол задан не для азимутального движителя — игнорируется");
            else if (!(std::fabs(t.parkDeg) <= 360)) err(f + "parkDeg", "стояночный угол должен быть в пределах ±360°");
            else if (angleForbidden(t, t.parkDeg)) warn(f + "parkDeg", "стояночный угол попадает в запретный сектор");
        }
        if (t.bus < -1 || t.bus > 1) err(f + "bus", "шина должна быть 0 (A) или 1 (B)");
        if (!(t.limitPct >= 5 && t.limitPct <= 100)) err(f + "limitPct", "предел нагрузки должен быть от 5 до 100 % максимального упора");
        if (!t.curve.empty()) {
            if (t.curve.size() < 2) err(f + "curve", "кривая должна содержать не менее двух точек");
            for (size_t k = 0; k < t.curve.size(); ++k) {
                const CurvePt& c = t.curve[k];
                if (!(c.rpm >= 0 && c.rpm <= 100 && c.thrust >= 0 && c.thrust <= 100 && c.power >= 0 && c.power <= 100)) err(f + "curve", "значения кривой должны быть от 0 до 100 %");
                if (k && (c.rpm < t.curve[k - 1].rpm || c.thrust < t.curve[k - 1].thrust || c.power < t.curve[k - 1].power)) err(f + "curve", "значения кривой не должны убывать");
            }
            if (!t.curve.empty() && std::fabs(t.curve.back().thrust - 100.0) > 1.0) warn(f + "curve", "последняя точка кривой должна давать упор около 100 %");
        }
        if (!(t.weight > 0 && t.weight <= 100)) err(f + "weight", "вес распределения должен быть больше 0 и не больше 100");
        for (auto& s : t.forbidden) if (!(std::fabs(s.fromDeg) <= 360 && std::fabs(s.toDeg) <= 360)) err(f + "forbidden", "углы запретного сектора должны быть в пределах ±360°");
        if (!t.effAngle.empty()) {
            if (t.type != ThrusterType::Azimuth) warn(f + "effAngle", "таблица КПД по углу имеет смысл только для азимутальных колонок — игнорируется");
            else if (t.effAngle.size() < 2) warn(f + "effAngle", "в таблице КПД по углу меньше двух точек");
            for (auto& q : t.effAngle) if (q.pct < 20) { warn(f + "effAngle", "КПД ниже 20 % на каком-то угле — проверьте таблицу"); break; }
        }
        if (t.type == ThrusterType::Azimuth && !t.forbidden.empty()) {
            double span = 0; for (auto& s : t.forbidden) { double d = std::fmod(s.toDeg - s.fromDeg + 720.0, 360.0); span += d; }
            if (span > 270) warn(f + "forbidden", "запретные сектора занимают более 270° — колонка почти не сможет поворачиваться");
        }
        if (t.powerMaxKW > 0 && t.maxForward > 0) {   // грубая проверка: упор на единицу мощности (на швартовах порядка 100–250 Н на кВт)
            const double nPerKw = t.maxForward / t.powerMaxKW;
            if (nPerKw > 350 || nPerKw < 40) warn(f + "powerMaxKW", "упор " + fmt(nPerKw) + " Н на кВт выглядит нетипично (обычно 100…250): проверьте упор и мощность");
        }
    }
    return r;
}

// Оценки, помогающие модели и оператору администратора. Не заменяют данные модельных испытаний.
inline std::vector<Estimate> estimateProfile(const VesselProfile& p) {
    std::vector<Estimate> e;
    const VesselCfg v = effectiveVessel(p);
    const LoadingInfo& l = p.loading;
    const double T = v.draft;
    if (v.length > 0 && v.beam > 0 && T > 0) {
        const double mCb = l.waterDensity * l.blockCoef * v.length * v.beam * T;
        e.push_back({"Масса по размерам, осадке и Cb", fmt(mCb / 1000.0, 5) + " т", "ρ·Cb·L·B·T; сравните с введённой массой " + fmt(v.mass / 1000.0, 5) + " т"});
        e.push_back({"Полнота корпуса по введённой массе", fmt(v.mass / (l.waterDensity * v.length * v.beam * T), 3), "Cb = m / (ρ·L·B·T)"});
    }
    e.push_back({"Средняя осадка в модели", fmt(T, 3) + " м", l.draftFwd > 0 ? "по осадкам носом и кормой" : "по полю «осадка»"});
    e.push_back({"Масса в модели", fmt(v.mass / 1000.0, 5) + " т", l.massFromDraft ? "вычислена по осадке" : "введена вручную"});
    e.push_back({"Парусность в модели: носовая / боковая", fmt(v.frontalArea, 4) + " / " + fmt(v.lateralArea, 4) + " м²", l.refDraft > 0 ? "с поправкой на осадку и добавками палубного груза" : "с добавками палубного груза"});
    const Inertia in = completeInertia(v);
    e.push_back({"Момент инерции по рысканью Izz", fmt((v.Izz > 0 ? v.Izz : v.mass * 0.0625 * v.length * v.length) / 1e6, 4) + " т·м²·10³", v.Izz > 0 ? "задан" : "по умолчанию: радиус инерции 0,25 L"});
    e.push_back({"Присоединённая масса поперёк / вдоль", fmt((in.m22 - v.mass) / 1000.0, 4) + " / " + fmt((in.m11 - v.mass) / 1000.0, 4) + " т", (v.addSway > 0 && v.addSurge > 0) ? "заданы" : "частично по умолчанию (0,6 m и 0,1 m)"});
    // возможности комплекса и предел удержания по ветру
    std::vector<ThrusterCfg> th; for (auto& t : p.thrusters) if (t.maxForward > 0) th.push_back(t);
    if (!th.empty()) {
        Allocator al(th); const Capability c = al.capability(1.0);
        e.push_back({"Предельные усилия комплекса: вперёд-назад / поперёк", fmt(c.X / 1000.0, 4) + " / " + fmt(c.Y / 1000.0, 4) + " кН", "по модулю, при всех движителях в строю"});
        e.push_back({"Предельный момент по рысканью", fmt(c.N / 1000.0, 4) + " кН·м", ""});
        const double q1 = 0.5 * RHO_AIR * v.lateralArea * v.cy, q2 = 0.5 * RHO_AIR * v.frontalArea * v.cx;
        if (q1 > 0 && c.Y > 0) e.push_back({"Предельный ветер, удержание лагом (80 % усилия)", fmt(std::sqrt(0.8 * c.Y / q1), 3) + " м/с", "только ветер и только силы (без момента, течения и волн); ориентир, а не расчёт класса DP"});
        if (q2 > 0 && c.X > 0) e.push_back({"Предельный ветер, удержание носом (80 % усилия)", fmt(std::sqrt(0.8 * c.X / q2), 3) + " м/с", "только ветер и только силы (без момента, течения и волн); ориентир"});
    }
    return e;
}

inline std::string writeIssues(const std::vector<Issue>& v) {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); ++i) s += (i ? "," : "") + std::string("{\"error\":") + (v[i].error ? "true" : "false") + ",\"field\":\"" + vio::esc(v[i].field) + "\",\"text\":\"" + vio::esc(v[i].text) + "\"}";
    return s + "]";
}
inline std::string writeEstimates(const std::vector<Estimate>& v) {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); ++i) s += (i ? "," : "") + std::string("{\"label\":\"") + vio::esc(v[i].label) + "\",\"value\":\"" + vio::esc(v[i].value) + "\",\"note\":\"" + vio::esc(v[i].note) + "\"}";
    return s + "]";
}
inline bool hasErrors(const std::vector<Issue>& v) { for (auto& i : v) if (i.error) return true; return false; }

}  // namespace dp
