// Калибровка систем положения: постоянное смещение каждой системы и ошибка места установки антенны.
// Невязка системы i = её позиция, приведённая к точке управления (с уже применёнными поправками), минус эталон.
// Эталон — медиана остальных систем (по умолчанию) или система, выбранная оператором.
//   r_n = bn + cosψ·dx − sinψ·dy,   r_e = be + sinψ·dx + cosψ·dy
// bn, be — постоянное смещение в осях север/восток, dx, dy — ошибка места установки антенны (нос, правый борт) в связанных осях.
// Метод наименьших квадратов; dx, dy определяются только после поворота курса не менее kMinSpanDeg, иначе только смещение.
// ⚠ Калибровка выравнивает системы друг относительно друга. Общая для всех систем ошибка (например, одна и та же поправка DGPS)
//   не видна; при эталоне-медиане ошибка одной из систем влияет на остальные. Только оценка, применение — по команде оператора.
#pragma once
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>
#include "dp/math.hpp"

namespace dp {

struct CalSensor {
    std::string id;
    bool isRef = false;          // эталон (ручной выбор): его невязка равна нулю
    int n = 0;                   // отсчётов
    double rn = 0, re = 0;       // нынешняя невязка, м
    double bn = 0, be = 0;       // смещение по северу и востоку, м
    double sigma = 0;            // СКО невязки после подгонки, м
    bool fitAnt = false;         // ошибка антенны определена (был поворот курса)
    double dx = 0, dy = 0;       // поправка к месту установки антенны, м
    bool ready = false;
};

struct CalView {
    bool on = false;
    std::string refId;           // пусто — медиана остальных
    double hdgSpanDeg = 0;
    int minSamples = 300;
    bool enough = false;         // есть из чего строить эталон (три системы и больше, либо ручной эталон)
    std::vector<CalSensor> s;
};

class Calibrator {
public:
    static constexpr double kMinSpanDeg = 30.0;
    static constexpr int kMinSamples = 300;
    struct Item { std::string id; double n, e; };

    void start() { reset(); on_ = true; }
    void stop() { on_ = false; }
    void reset() { acc_.clear(); span0_ = 0; spanMin_ = spanMax_ = 0; haveHdg_ = false; lastT_ = -1e9; cur_.clear(); }
    void setRef(const std::string& id) { ref_ = (id == "median" || id == "MEDIAN" || id == "медиана") ? "" : id; reset(); }
    bool on() const { return on_; }
    const std::string& ref() const { return ref_; }

    // Вызывается каждый шаг оценивателя с действующими (приведёнными, с поправками) позициями всех пригодных систем.
    void tick(double time, double psi, const std::vector<Item>& items) {
        if (!on_) return;
        if (time - lastT_ < 1.0) return;
        lastT_ = time;
        if (!haveHdg_) { span0_ = psi; haveHdg_ = true; spanMin_ = spanMax_ = 0; }
        const double d = rad2deg(wrapPi(psi - span0_)); spanMin_ = std::min(spanMin_, d); spanMax_ = std::max(spanMax_, d);
        cur_.clear();
        for (const auto& it : items) {
            double rn = 0, re = 0;
            if (!ref_.empty()) {
                const Item* r = nullptr; for (const auto& o : items) if (o.id == ref_) r = &o;
                if (!r) continue;
                if (it.id == ref_) { cur_[it.id] = {0, 0}; continue; }
                rn = it.n - r->n; re = it.e - r->e;
            } else {
                std::vector<double> vn, ve; for (const auto& o : items) if (o.id != it.id) { vn.push_back(o.n); ve.push_back(o.e); }
                if (vn.empty()) continue;
                rn = it.n - med(vn); re = it.e - med(ve);
            }
            cur_[it.id] = {rn, re};
            Acc& a = acc_[it.id];
            const double c = std::cos(psi), s = std::sin(psi);
            const double row[2][4] = {{1, 0, c, -s}, {0, 1, s, c}};
            const double y[2] = {rn, re};
            for (int k = 0; k < 2; ++k) {
                for (int i = 0; i < 4; ++i) { a.b[i] += row[k][i] * y[k]; for (int j = 0; j < 4; ++j) a.A[i][j] += row[k][i] * row[k][j]; }
                a.yy += y[k] * y[k];
            }
            ++a.n;
        }
    }

    CalView view(const std::vector<std::string>& allIds) const {
        CalView v; v.on = on_; v.refId = ref_; v.minSamples = kMinSamples; v.hdgSpanDeg = spanMax_ - spanMin_;
        v.enough = !ref_.empty() ? allIds.size() >= 2 : allIds.size() >= 3;
        for (const auto& id : allIds) {
            CalSensor c; c.id = id; c.isRef = (id == ref_);
            auto ca = cur_.find(id); if (ca != cur_.end()) { c.rn = ca->second.first; c.re = ca->second.second; }
            auto it = acc_.find(id);
            if (it != acc_.end() && !c.isRef) {
                const Acc& a = it->second; c.n = a.n;
                const bool ant = (spanMax_ - spanMin_) >= kMinSpanDeg;
                double x[4] = {0, 0, 0, 0};
                if (a.n >= 5 && solve(a, ant ? 4 : 2, x)) {
                    c.bn = x[0]; c.be = x[1]; c.fitAnt = ant; if (ant) { c.dx = x[2]; c.dy = x[3]; }
                    const int k = ant ? 4 : 2; double dot = 0; for (int i = 0; i < k; ++i) dot += x[i] * a.b[i];
                    const double sse = std::max(0.0, a.yy - dot); c.sigma = std::sqrt(sse / std::max(1, 2 * a.n - k));
                    c.ready = a.n >= kMinSamples && v.enough;
                }
            } else if (it != acc_.end()) c.n = it->second.n;
            v.s.push_back(c);
        }
        return v;
    }

private:
    struct Acc { double A[4][4] = {}, b[4] = {}, yy = 0; int n = 0; };
    static double med(std::vector<double> v) { std::sort(v.begin(), v.end()); const size_t m = v.size(); return m % 2 ? v[m / 2] : 0.5 * (v[m / 2 - 1] + v[m / 2]); }
    static bool solve(const Acc& a, int k, double* x) {
        double M[4][5];
        for (int i = 0; i < k; ++i) { for (int j = 0; j < k; ++j) M[i][j] = a.A[i][j]; M[i][k] = a.b[i]; }
        for (int i = 0; i < k; ++i) {
            int p = i; for (int r = i + 1; r < k; ++r) if (std::fabs(M[r][i]) > std::fabs(M[p][i])) p = r;
            if (std::fabs(M[p][i]) < 1e-9) return false;
            if (p != i) for (int c = 0; c <= k; ++c) std::swap(M[i][c], M[p][c]);
            for (int r = 0; r < k; ++r) if (r != i) { const double f = M[r][i] / M[i][i]; for (int c = i; c <= k; ++c) M[r][c] -= f * M[i][c]; }
        }
        for (int i = 0; i < k; ++i) x[i] = M[i][k] / M[i][i];
        return true;
    }
    bool on_ = false, haveHdg_ = false;
    std::string ref_;
    double lastT_ = -1e9, span0_ = 0, spanMin_ = 0, spanMax_ = 0;
    std::map<std::string, Acc> acc_;
    std::map<std::string, std::pair<double, double>> cur_;
};

}  // namespace dp
