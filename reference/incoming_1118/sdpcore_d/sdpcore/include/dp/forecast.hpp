// Прогнозирование внешних сил по накопленным данным («матрица прогнозирования»).
// 1. Таблица поправок «направление и сила ветра → остаток силы». Пока судно удерживается, в ячейке, соответствующей
//    нынешним условиям, накапливается средняя разница между действительной силой (оценка неизвестной силы + уже
//    внесённая поправка) и расчётной силой ветра и волн. Поправка добавляется к силе ветра в ядре (если включено «применять»).
//    Ячейки учитываются постепенно: от minSec накопления с нарастающим доверием, полностью с fullSec.
// 2. Медленная составляющая («течение, приливы»): среднее остатка после таблицы за slowS с и его тренд (в земных осях).
// 3. Прогноз силы и загрузки комплекса на 1, 2, 5 и 10 минут: ветер и волны экстраполируются по среднему и тренду,
//    плюс поправка таблицы и медленная составляющая. Тревога FORECAST_LOAD, если загрузка на 5 мин превысит порог.
// Таблица сохраняется в файл (папка данных) и восстанавливается при запуске для того же судна.
// ⚠ Только имитация, не сертифицировано. Порывы (секунды) не предсказываются. Поправка ограничена долей возможностей комплекса.
#pragma once
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "dp/json.hpp"
#include "dp/model.hpp"
#include "dp/thrust.hpp"

namespace dp {

struct ForecastCfg {
    bool enabled = false;          // накопление и прогноз
    bool apply = false;            // добавлять поправку таблицы к силе ветра в ядре
    int dirBins = 36;              // направлений ветра (относительно носа)
    double speedStep = 3.0;        // шаг по скорости ветра, м/с
    int speedBins = 10;            // ячеек по скорости (последняя — «и выше»)
    double minSec = 300.0;         // накопление в ячейке до начала использования, с
    double fullSec = 900.0;        // накопление до полного доверия, с
    double memSec = 3600.0;        // память ячейки (старые данные забываются), с
    double maxCorrFrac = 0.25;     // предел поправки от возможностей комплекса
    double settleS = 120.0;        // после включения режима «позиция» данные не берутся, с
    double slowS = 600.0;          // постоянная времени медленной составляющей, с
    double warnRatio = 0.9, clearRatio = 0.8;   // порог тревоги прогноза нагрузки и снятие
    double warnHorizonS = 300.0, warnHoldS = 30.0;
};

struct FcIn {
    double dt = 0.1;
    bool learn = false;            // судно удерживается спокойно: данные годятся для обучения
    bool windValid = false, hdgValid = false;
    double windSpeed = 0, windRel = 0;       // кажущийся ветер: скорость, м/с; откуда относительно носа, рад
    double psi = 0;                          // курс, рад
    double biasN = 0, biasE = 0, biasM = 0;  // неизвестная сила (оценка), земные оси, Н, Н·м
    Wrench applied;                          // поправка таблицы, фактически внесённая в модель на этом шаге (связанные оси)
    bool waveValid = false; double hs = 0, waveFrom = 0, tz = 0, waveGain = 1;   // волны по оценке МИО
    Capability cap;
};

struct FcOut {
    bool on = false, apply = false, valid = false;    // valid — прогноз рассчитан
    double dataSec = 0;                    // всего накоплено секунд
    int filled = 0, total = 0;             // ячеек с данными для использования / всего
    double conf = 0;                       // доверие к ячейке нынешних условий, 0..1
    int cellDir = -1, cellSpd = -1;
    Wrench cellCorr;                       // поправка нынешней ячейки (связанные оси), Н
    Wrench slow;                           // медленная составляющая, связанные оси, Н
    std::vector<double> horizonS, fx, fy, fn, load;   // прогноз на горизонты: сила и загрузка (1 = предел)
    bool warn = false;
};

class Forecaster {
public:
    struct Cell { double x = 0, y = 0, n = 0, w = 0; };

    ForecastCfg& cfg() { return c_; }
    const ForecastCfg& cfg() const { return c_; }
    void configure(const ForecastCfg& c) {
        const bool shape = c.dirBins != c_.dirBins || c.speedBins != c_.speedBins;
        c_ = c; if (shape || cells_.size() != size_t(c_.dirBins) * c_.speedBins) { cells_.assign(size_t(c_.dirBins) * c_.speedBins, Cell{}); }
    }
    Forecaster() { cells_.assign(size_t(c_.dirBins) * c_.speedBins, Cell{}); }
    void reset() { cells_.assign(size_t(c_.dirBins) * c_.speedBins, Cell{}); dataSec_ = 0; slowW_[0] = slowW_[1] = slowW_[2] = 0; trendW_[0] = trendW_[1] = trendW_[2] = 0; init_ = false; dirty_ = true; }
    const std::vector<Cell>& cells() const { return cells_; }
    double dataSec() const { return dataSec_; }
    bool dirty() const { return dirty_; }
    void clearDirty() { dirty_ = false; }

    // Поправка к силе ветра (связанные оси): интерполяция по направлению между ближайшими ячейками, учёт доверия, предел по возможностям.
    Wrench correction(double speed, double relRad, const Capability& cap) const {
        const int si = spdIdx(speed);
        const double pos = dirPos(relRad);
        const int d0 = static_cast<int>(std::floor(pos)) % c_.dirBins, d1 = (d0 + 1) % c_.dirBins; const double f = pos - std::floor(pos);
        Wrench a = cellUse(d0, si), b = cellUse(d1, si), r;
        r.X = a.X + f * (b.X - a.X); r.Y = a.Y + f * (b.Y - a.Y); r.N = a.N + f * (b.N - a.N);
        const double mf = c_.maxCorrFrac;
        r.X = clampd(r.X, -mf * cap.X, mf * cap.X); r.Y = clampd(r.Y, -mf * cap.Y, mf * cap.Y); r.N = clampd(r.N, -mf * cap.N, mf * cap.N);
        return r;
    }

    FcOut step(const FcIn& in, const VesselCfg& v) {
        FcOut o; o.on = c_.enabled; o.apply = c_.apply; o.total = static_cast<int>(cells_.size());
        for (const auto& c : cells_) if (c.w >= c_.minSec) ++o.filled;
        o.dataSec = dataSec_;
        if (!c_.enabled) { init_ = false; learnT_ = 0; return o; }
        const double dt = std::max(in.dt, 1e-6);
        // --- ветер и волны: среднее и тренд
        if (in.windValid) {
            const double s = in.windSpeed, cs = std::cos(in.windRel), sn = std::sin(in.windRel);
            if (!init_) { wMean_ = s; wTrend_ = 0; dc_ = cs; ds_ = sn; hsMean_ = in.hs; hsTrend_ = 0; init_ = true; }
            else {
                const double am = dt / (120.0 + dt), at = dt / (600.0 + dt), prev = wMean_;
                wMean_ += am * (s - wMean_); wTrend_ += at * ((wMean_ - prev) / dt - wTrend_);
                dc_ += am * (cs - dc_); ds_ += am * (sn - ds_);
                if (in.waveValid) { const double ph = hsMean_; hsMean_ += am * (in.hs - hsMean_); hsTrend_ += at * ((hsMean_ - ph) / dt - hsTrend_); }
            }
        }
        const double cp = std::cos(in.psi), sp = std::sin(in.psi);
        // --- неизвестная сила: сглаживание, связанные оси
        if (!bInit_) { bw_[0] = in.biasN; bw_[1] = in.biasE; bw_[2] = in.biasM; bInit_ = true; }
        else { const double ab = dt / (30.0 + dt); bw_[0] += ab * (in.biasN - bw_[0]); bw_[1] += ab * (in.biasE - bw_[1]); bw_[2] += ab * (in.biasM - bw_[2]); }
        const double bx = bw_[0] * cp + bw_[1] * sp, by = -bw_[0] * sp + bw_[1] * cp;
        if (in.windValid) { const int di = dirIdx(in.windRel), si = spdIdx(in.windSpeed); o.cellDir = di; o.cellSpd = si; o.conf = confOf(cells_[idx(di, si)].w);
                            o.cellCorr = correction(in.windSpeed, in.windRel, in.cap); }
        // --- обучение
        learnT_ = in.learn && in.windValid && in.hdgValid ? learnT_ + dt : 0;
        if (learnT_ > c_.settleS) {
            const Wrench R{bx + in.applied.X, by + in.applied.Y, bw_[2] + in.applied.N};     // сила, которую модель ветра должна была дать сверх расчёта
            Cell& c = cells_[idx(dirIdx(in.windRel), spdIdx(in.windSpeed))];
            c.w = std::min(c.w + dt, c_.memSec);
            const double k = dt / c.w, mf = c_.maxCorrFrac;
            c.x = clampd(c.x + k * (R.X - c.x), -mf * in.cap.X, mf * in.cap.X);
            c.y = clampd(c.y + k * (R.Y - c.y), -mf * in.cap.Y, mf * in.cap.Y);
            c.n = clampd(c.n + k * (R.N - c.n), -mf * in.cap.N, mf * in.cap.N);
            dataSec_ += dt; dirty_ = true;
            // медленная составляющая: остаток после таблицы (земные оси)
            const Wrench used = o.cellCorr;                                    // с учётом доверия
            const double rx = R.X - used.X, ry = R.Y - used.Y, rm = R.N - used.N;
            const double rN = rx * cp - ry * sp, rE = rx * sp + ry * cp;
            const double r3[3] = {rN, rE, rm};
            const double as = dt / (c_.slowS + dt), at = dt / (900.0 + dt);
            for (int i = 0; i < 3; ++i) { const double pv = slowW_[i]; slowW_[i] += as * (r3[i] - slowW_[i]); trendW_[i] += at * ((slowW_[i] - pv) / dt - trendW_[i]); }
        }
        o.slow.X = slowW_[0] * cp + slowW_[1] * sp; o.slow.Y = -slowW_[0] * sp + slowW_[1] * cp; o.slow.N = slowW_[2];
        // --- прогноз
        if (init_ && in.windValid && in.hdgValid && dataSec_ > 0) {
            const double dir = std::atan2(ds_, dc_);
            const double hz[4] = {60, 120, 300, 600};
            for (double T : hz) {
                const double sMax = std::max(wMean_, 3.0);
                const double s = std::max(0.0, wMean_ + clampd(wTrend_ * T, -0.5 * sMax, 0.5 * sMax));
                Wrench F = windForce(v, s, dir);
                if (in.waveValid && in.hs > 0) {
                    const double hMax = std::max(hsMean_, 1.0);
                    const double hs = std::max(0.0, hsMean_ + clampd(hsTrend_ * T, -0.5 * hMax, 0.5 * hMax));
                    Wrench w = waveDriftForce(v, hs, in.waveFrom, in.psi, in.tz);
                    F.X += in.waveGain * w.X; F.Y += in.waveGain * w.Y; F.N += in.waveGain * w.N;
                }
                // остаток силы сверх модели: где в таблице достаточно данных — поправка ячейки и медленная составляющая;
                // иначе (мало данных) — нынешний остаток, масштабированный по ожидаемой силе ветра (ошибка модели считается пропорциональной силе)
                const double cf = confOf(cells_[idx(dirIdx(dir), spdIdx(s))].w);
                const Wrench cc = correction(s, dir, in.cap);
                const double fl = 5000.0;
                double tN[3];
                for (int i = 0; i < 3; ++i) tN[i] = clampd(trendW_[i] * T, -(0.5 * std::fabs(slowW_[i]) + fl), 0.5 * std::fabs(slowW_[i]) + fl);
                const double sN = slowW_[0] + tN[0], sE = slowW_[1] + tN[1];
                const Wrench w0 = windForce(v, in.windSpeed, dir), w1 = windForce(v, s, dir);
                const double m0 = std::hypot(w0.X, w0.Y), m1 = std::hypot(w1.X, w1.Y);
                const double ratio = m0 > 1000.0 ? clampd(m1 / m0, 0.2, 5.0) : 1.0;
                F.X += cc.X + cf * (sN * cp + sE * sp) + (1.0 - cf) * ratio * (bx + in.applied.X);
                F.Y += cc.Y + cf * (-sN * sp + sE * cp) + (1.0 - cf) * ratio * (by + in.applied.Y);
                F.N += cc.N + cf * (slowW_[2] + tN[2]) + (1.0 - cf) * ratio * (bw_[2] + in.applied.N);
                auto need = [](double req, double c) { return std::fabs(req) < 1.0 ? 0.0 : (c > 1e-6 ? std::fabs(req) / c : 1e9); };
                o.horizonS.push_back(T); o.fx.push_back(F.X); o.fy.push_back(F.Y); o.fn.push_back(F.N);
                o.load.push_back(std::max({need(F.X, in.cap.X), need(F.Y, in.cap.Y), need(F.N, in.cap.N)}));
            }
            o.valid = true;
        }
        // --- тревога прогноза нагрузки (с выдержкой и гистерезисом)
        double lw = 0;
        if (o.valid) { for (size_t i = 0; i < o.horizonS.size(); ++i) if (std::fabs(o.horizonS[i] - c_.warnHorizonS) < 1.0) lw = o.load[i]; }
        if (!warn_) { wOn_ = (o.valid && lw >= c_.warnRatio) ? wOn_ + dt : 0; if (wOn_ >= c_.warnHoldS) { warn_ = true; wOff_ = 0; } }
        else { wOff_ = (!o.valid || lw < c_.clearRatio) ? wOff_ + dt : 0; if (wOff_ >= c_.warnHoldS) { warn_ = false; wOn_ = 0; } }
        o.warn = warn_;
        return o;
    }

    // Компактная карта для окна: модуль поправки ячейки, кН (округлено); −1 — данных для использования нет.
    std::vector<int> mapKN() const {
        std::vector<int> r(cells_.size(), -1);
        for (size_t i = 0; i < cells_.size(); ++i) if (cells_[i].w >= c_.minSec) r[i] = static_cast<int>(std::lround(std::hypot(cells_[i].x, cells_[i].y) / 1000.0));
        return r;
    }

    // ---- сохранение: json; ship — признак судна (имя и масса), при несовпадении файл не применяется
    std::string toJson(const std::string& ship, double mass) const {
        std::ostringstream s; s.precision(9);
        s << "{\"version\":1,\"ship\":\"" << esc(ship) << "\",\"mass\":" << mass << ",\"dirBins\":" << c_.dirBins << ",\"speedBins\":" << c_.speedBins << ",\"speedStep\":" << c_.speedStep
          << ",\"dataSec\":" << dataSec_ << ",\"enabled\":" << (c_.enabled ? "true" : "false") << ",\"apply\":" << (c_.apply ? "true" : "false") << ",\"cells\":[";
        for (size_t i = 0; i < cells_.size(); ++i) s << (i ? "," : "") << "[" << cells_[i].x << "," << cells_[i].y << "," << cells_[i].n << "," << cells_[i].w << "]";
        s << "]}";
        return s.str();
    }
    bool fromJson(const std::string& text, const std::string& ship, double mass, std::string& why) {
        json::Value v; std::string err;
        if (!json::parse(text, v, err) || v.t != json::Value::Obj) { why = "файл прогноза повреждён"; return false; }
        auto num = [&](const char* k, double d) { const json::Value* p = v.find(k); return p && p->t == json::Value::Num ? p->n : d; };
        const json::Value* sp = v.find("ship");
        if (!sp || sp->t != json::Value::Str || sp->s != ship || std::fabs(num("mass", 0) - mass) > 0.02 * std::max(mass, 1.0)) { why = "таблица накоплена для другого судна или другой массы — не применена"; return false; }
        if (static_cast<int>(num("dirBins", 0)) != c_.dirBins || static_cast<int>(num("speedBins", 0)) != c_.speedBins || std::fabs(num("speedStep", 0) - c_.speedStep) > 1e-9) { why = "другая сетка таблицы — не применена"; return false; }
        const json::Value* cs = v.find("cells");
        if (!cs || cs->t != json::Value::Arr || cs->a.size() != cells_.size()) { why = "неверное число ячеек — не применена"; return false; }
        std::vector<Cell> t(cells_.size());
        for (size_t i = 0; i < t.size(); ++i) {
            const auto& e = cs->a[i];
            if (e.t != json::Value::Arr || e.a.size() != 4) { why = "ячейка повреждена"; return false; }
            for (const auto& q : e.a) if (q.t != json::Value::Num || !std::isfinite(q.n)) { why = "ячейка повреждена"; return false; }
            t[i] = {e.a[0].n, e.a[1].n, e.a[2].n, std::max(0.0, std::min(e.a[3].n, c_.memSec))};
        }
        cells_ = t; dataSec_ = num("dataSec", 0); dirty_ = false;
        const json::Value* en = v.find("enabled"); const json::Value* ap = v.find("apply");
        if (en && en->t == json::Value::Bool) c_.enabled = en->b;
        if (ap && ap->t == json::Value::Bool) c_.apply = ap->b;
        return true;
    }

private:
    static std::string esc(const std::string& s) { std::string r; for (char c : s) { if (c == '"' || c == '\\') r += '\\'; if (static_cast<unsigned char>(c) >= 0x20) r += c; } return r; }
    size_t idx(int d, int s) const { return static_cast<size_t>(s) * c_.dirBins + d; }
    double dirPos(double rad) const { double d = std::fmod(rad2deg(rad), 360.0); if (d < 0) d += 360.0; return d / (360.0 / c_.dirBins); }
    int dirIdx(double rad) const { return static_cast<int>(std::lround(dirPos(rad))) % c_.dirBins; }
    int spdIdx(double s) const { return std::min(c_.speedBins - 1, std::max(0, static_cast<int>(s / c_.speedStep))); }
    double confOf(double w) const { return clampd((w - c_.minSec) / std::max(1.0, c_.fullSec - c_.minSec), 0.0, 1.0); }
    Wrench cellUse(int d, int s) const { const Cell& c = cells_[idx(d, s)]; const double k = confOf(c.w); return Wrench{c.x * k, c.y * k, c.n * k}; }

    ForecastCfg c_;
    std::vector<Cell> cells_;
    double dataSec_ = 0; bool dirty_ = false;
    bool init_ = false, bInit_ = false;
    double wMean_ = 0, wTrend_ = 0, dc_ = 1, ds_ = 0, hsMean_ = 0, hsTrend_ = 0;
    double bw_[3] = {0, 0, 0};
    double slowW_[3] = {0, 0, 0}, trendW_[3] = {0, 0, 0};
    double learnT_ = 0, wOn_ = 0, wOff_ = 0; bool warn_ = false;
};

}  // namespace dp
