// Модуль предсказания внешних сил (ветер, волны): медленное среднее + тренд + прогноз на заданный горизонт.
// Порывы короче постоянной времени среднего не отрабатываются: упор следует за средним значением, а не за мгновенным.
// Только имитация, не сертифицировано. Параметры по умолчанию отключают модуль (поведение прежнее).
#pragma once
#include <cmath>

namespace dp {

struct PredictCfg {
    bool enabled = false;
    double meanS = 40.0;          // постоянная времени среднего, с: порывы короче этого значения почти не проходят
    double trendS = 120.0;        // постоянная времени оценки тренда (скорости изменения среднего), с
    double horizonS = 20.0;       // на сколько секунд вперёд экстраполируется тренд
    double trendLimitFrac = 0.3;  // тренд не добавляет больше этой доли от среднего (по модулю) + trendFloor
    double trendFloor = 0.0;      // нижний порог, Н (Н·м): чтобы тренд работал и при малом среднем
};

// Один канал: возвращает прогнозируемое значение. Первое обращение инициализирует среднее текущим значением (без скачка).
class Predictor {
public:
    void reset() { init_ = false; mean_ = trend_ = 0; }
    double step(double dt, double x, const PredictCfg& c) {
        if (!c.enabled) { init_ = false; return x; }
        if (!init_) { mean_ = x; trend_ = 0; init_ = true; return x; }
        const double am = dt / (c.meanS + dt), at = dt / (c.trendS + dt);
        const double prev = mean_;
        mean_ += am * (x - mean_);
        trend_ += at * ((mean_ - prev) / std::max(dt, 1e-6) - trend_);
        double add = trend_ * c.horizonS;
        const double lim = c.trendLimitFrac * std::fabs(mean_) + c.trendFloor;
        if (add > lim) add = lim; else if (add < -lim) add = -lim;
        return mean_ + add;
    }
    double mean() const { return mean_; }
    double trend() const { return trend_; }
private:
    bool init_ = false; double mean_ = 0, trend_ = 0;
};

}  // namespace dp
