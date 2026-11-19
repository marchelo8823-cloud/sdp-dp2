// Оценка волнения по данным МИО (крен, дифферент, вертикальная качка) с учётом передаточных функций судна.
// Радара нет: высота, период и направление волн вычисляются из движения самого судна, как у волновой буи-станции
// (heave-pitch-roll). Измеренные движения пересчитываются в характеристики волны делением на комплексный отклик судна
// H(ω) = RAO·k / (k − m·ω² + i·d·ω) по параметрам из профиля судна (Inertia, RAO).
// Ограничения: модель отклика упрощённая (линейная, 2-го порядка), МИО считается стоящим в точке управления, волны —
// одно направление; для коротких волн, которые крупное судно почти не «чувствует», оценка недостоверна (минимальное
// усиление |H| задано в WaveCfg::minGain).
#pragma once
#include <algorithm>
#include <complex>
#include <deque>

#include "dp/model.hpp"

namespace dp {

struct WaveCfg {
    double windowS = 300.0;       // длина окна анализа, с (короткое окно даёт заметный разброс высоты)
    double minS = 120.0;          // минимальная длина окна для первой оценки, с
    double updateS = 5.0;         // период пересчёта, с
    double decimHz = 2.0;         // частота выборки для анализа (усреднением)
    double bandLoRadS = 0.35;     // полоса волн: нижняя граница, рад/с (период ≈ 18 с)
    double bandHiRadS = 2.2;      // верхняя граница, рад/с (период ≈ 2,9 с)
    double minGain = 0.15;        // минимальный |H|: где судно почти не реагирует, частоту не используем
    double minHs = 0.15;          // ниже этой высоты направление не определяем (уровень шума МИО)
    double smooth = 0.06;         // сглаживание новых оценок (пересчёт раз в updateS, постоянная времени ≈ updateS/smooth ≈ 80 с)
};

struct WaveEstimate {
    bool valid = false;           // высота и период оценены
    bool dirValid = false;        // направление определено
    double hs = 0;                // высота значительная, м
    double tz = 0;                // средний период по нулям, с
    double tp = 0;                // период пика спектра, с
    double fromDeg = 0;           // откуда идут волны, истинный пеленг, град
    double conf = 0;              // уверенность в направлении, 0..1 (1 — волны с одной стороны)
    double seconds = 0;           // длина накопленного окна, с
    double ffGain = 0;            // доля дрейфовой силы, которую разрешено применять в упреждении, 0..1
};

class WaveEstimator {
public:
    WaveEstimator(const VesselCfg& v, WaveCfg wc = {}) : vc_(v), in_(completeInertia(v)), wc_(wc) {}

    // Один отсчёт слитых данных МИО; вызывается на каждом шаге оценки, пока данные достоверны.
    void push(double time, double psi, double roll, double pitch, double heave) {
        const double step = lastT_ > -1e8 ? time - lastT_ : 0.0;
        lastT_ = time;
        if (accN_ == 0) accT0_ = time;
        accH_ += heave; accP_ += pitch; accR_ += roll; accS_ += std::sin(psi); accC_ += std::cos(psi); accT_ += time; ++accN_;
        // окно усреднения закрывается, когда следующий отсчёт уже вышел бы за период выборки (иначе период получается длиннее заданного)
        if (time - accT0_ + step + 1e-9 >= 1.0 / wc_.decimHz) {
            Sample s{accH_ / accN_, accP_ / accN_, accR_ / accN_, accS_ / accN_, accC_ / accN_, accT_ / accN_};
            buf_.push_back(s);
            while (buf_.size() > static_cast<size_t>(wc_.windowS * wc_.decimHz)) buf_.pop_front();
            accH_ = accP_ = accR_ = accS_ = accC_ = accT_ = 0; accN_ = 0;
            if (time - lastUpdate_ >= wc_.updateS) { lastUpdate_ = time; recompute(); }
        }
    }
    // Данных МИО нет: накопленное окно устаревает.
    void reset() { buf_.clear(); accN_ = 0; lastT_ = -1e9; est_ = {}; init_ = false; }
    const WaveEstimate& state() const { return est_; }

    // Отклик судна на волну единичной амплитуды (комплексный), для проверки и документации.
    std::complex<double> hHeave(double w) const { return resp(vc_.raoHeave, in_.kz, in_.m33, in_.d33, w); }
    std::complex<double> hPitch(double w) const { return resp(vc_.raoPitch, in_.ktheta, in_.m55, in_.d55, w); }
    std::complex<double> hRoll(double w) const { return resp(vc_.raoRoll, in_.kphi, in_.m44, in_.d44, w); }

private:
    struct Sample { double h, p, r, sinPsi, cosPsi, t; };
    static std::complex<double> resp(double rao, double k, double m, double d, double w) {
        return rao * k / std::complex<double>(k - m * w * w, d * w);
    }

    void recompute() {
        using C = std::complex<double>;
        const size_t N = buf_.size();
        if (N < 8) return;
        const double dt = (buf_.back().t - buf_.front().t) / (N - 1), T = N * dt;      // фактический шаг по меткам времени
        est_.seconds = T;
        if (T < wc_.minS) return;
        // средние и окно Ханна
        double mh = 0, mp = 0, mr = 0, ss = 0, sc = 0;
        for (const auto& s : buf_) { mh += s.h; mp += s.p; mr += s.r; ss += s.sinPsi; sc += s.cosPsi; }
        mh /= N; mp /= N; mr /= N;
        const double psiMean = std::atan2(ss, sc);
        std::vector<double> w(N), xh(N), xp(N), xr(N);
        double W2 = 0;
        for (size_t n = 0; n < N; ++n) {
            w[n] = 0.5 * (1.0 - std::cos(2.0 * PI * n / (N - 1)));
            W2 += w[n] * w[n];
            xh[n] = w[n] * (buf_[n].h - mh); xp[n] = w[n] * (buf_[n].p - mp); xr[n] = w[n] * (buf_[n].r - mr);
        }
        // Спектр считается на сетке частот в L раз чаще, чем 2π/T: у судна отклик резко меняется около резонанса,
        // и деление на H(ω) на «чужой» частоте даёт заметную ошибку по фазе.
        const int L = 4;
        const double dw = 2.0 * PI / T / L;
        double m0 = 0, m2 = 0, pkP = 0, pkW = 0, dx = 0, dy = 0, nrm = 0;
        int bins = 0, dirBins = 0;
        std::vector<double> ps, wk;                     // спектр для поиска пика
        for (int k = std::max(1, static_cast<int>(std::ceil(wc_.bandLoRadS / dw))); k * dw <= wc_.bandHiRadS; ++k) {
            const double om = k * dw;
            const C rot = std::polar(1.0, -om * dt);
            C e = 1.0, Xh = 0, Xp = 0, Xr = 0;
            for (size_t n = 0; n < N; ++n) {
                Xh += xh[n] * e; Xp += xp[n] * e; Xr += xr[n] * e;
                e *= rot;
            }
            const C Hh = hHeave(om), Hp = hPitch(om), Hr = hRoll(om);
            if (std::abs(Hh) < wc_.minGain) continue;
            const C Z = Xh / Hh;
            const double pw = 2.0 * std::norm(Z) / (N * W2 * L);      // вклад в дисперсию ординаты волны
            m0 += pw; m2 += om * om * pw; ++bins;
            ps.push_back(pw); wk.push_back(om);
            if (std::abs(Hp) >= wc_.minGain && std::abs(Hr) >= wc_.minGain) {
                const C Sx = Xp / Hp, Sy = -Xr / Hr;                  // уклоны поверхности вдоль и поперёк судна
                const C iZc = C(0, 1) * std::conj(Z);
                dx += std::real(Sx * iZc); dy += std::real(Sy * iZc);
                nrm += std::sqrt(std::norm(Sx) + std::norm(Sy)) * std::abs(Z);
                ++dirBins;
            }
        }
        // период пика по сглаженному спектру
        for (size_t i = 0; i < ps.size(); ++i) {
            double v = ps[i];
            for (int j = 1; j <= L; ++j) { if (i >= static_cast<size_t>(j)) v += ps[i - j] * (1.0 - double(j) / (L + 1)); if (i + j < ps.size()) v += ps[i + j] * (1.0 - double(j) / (L + 1)); }
            if (v > pkP) { pkP = v; pkW = wk[i]; }
        }
        if (bins < 3 * L || m0 <= 0) { est_.valid = false; est_.dirValid = false; est_.ffGain = 0; return; }
        const double hs = 4.0 * std::sqrt(m0), tz = 2.0 * PI * std::sqrt(m0 / std::max(1e-12, m2)), tp = pkW > 0 ? 2.0 * PI / pkW : tz;
        bool dirOk = false; double fromDeg = est_.fromDeg, conf = 0;
        if (dirBins >= 3 * L && hs >= wc_.minHs && nrm > 0) {
            conf = std::min(1.0, std::hypot(dx, dy) / nrm);
            const double alpha = std::atan2(dy, dx);                   // направление распространения относительно носа
            fromDeg = std::fmod(rad2deg(psiMean + alpha + PI) + 720.0, 360.0);
            dirOk = true;
        }
        const double a = init_ ? wc_.smooth : 1.0;
        est_.hs += a * (hs - est_.hs); est_.tz += a * (tz - est_.tz); est_.tp += a * (tp - est_.tp);
        est_.valid = true;
        if (dirOk) {
            if (!est_.dirValid) { est_.fromDeg = fromDeg; est_.conf = conf; }
            else {
                const double ux = (1 - a) * std::cos(deg2rad(est_.fromDeg)) + a * std::cos(deg2rad(fromDeg));
                const double uy = (1 - a) * std::sin(deg2rad(est_.fromDeg)) + a * std::sin(deg2rad(fromDeg));
                est_.fromDeg = std::fmod(rad2deg(std::atan2(uy, ux)) + 360.0, 360.0);
                est_.conf += a * (conf - est_.conf);
            }
            est_.dirValid = true;
        } else { est_.dirValid = false; est_.conf = 0; }
        est_.ffGain = est_.dirValid ? clampd((est_.conf - 0.35) / 0.35, 0.0, 1.0) * clampd((T - wc_.minS) / 120.0, 0.0, 1.0) : 0.0;
        init_ = true;
    }

    VesselCfg vc_;
    Inertia in_;
    WaveCfg wc_;
    std::deque<Sample> buf_;
    double accH_ = 0, accP_ = 0, accR_ = 0, accS_ = 0, accC_ = 0, accT_ = 0, accT0_ = 0, lastUpdate_ = -1e9, lastT_ = -1e9;
    int accN_ = 0;
    WaveEstimate est_;
    bool init_ = false;
};

}  // namespace dp
