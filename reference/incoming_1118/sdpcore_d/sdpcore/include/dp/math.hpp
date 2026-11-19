// Базовая математика ядра СДП: углы, малые матрицы, решение линейных систем.
// Система координат: NED (север, восток, вниз). Курс отсчитывается от севера по часовой стрелке.
// Связанные оси судна: x — к носу, y — на правый борт, z — вниз.
#pragma once
#include <random>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace dp {

constexpr double PI = 3.14159265358979323846;
inline double deg2rad(double d) { return d * PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / PI; }
inline double wrapPi(double a) {
    a = std::fmod(a + PI, 2 * PI);
    if (a < 0) a += 2 * PI;
    return a - PI;
}
inline double wrap2Pi(double a) {
    a = std::fmod(a, 2 * PI);
    if (a < 0) a += 2 * PI;
    return a;
}
inline double clampd(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline double sgn(double x) { return x < 0 ? -1.0 : 1.0; }

struct V3 {
    double x = 0, y = 0, z = 0;
};

// Поворот из связанных осей в NED (последовательность Z-Y-X: курс, дифферент, крен).
inline V3 rotateBodyToNed(double phi, double theta, double psi, const V3& b) {
    const double cf = std::cos(phi), sf = std::sin(phi);
    const double ct = std::cos(theta), st = std::sin(theta);
    const double cp = std::cos(psi), sp = std::sin(psi);
    V3 r;
    r.x = cp * ct * b.x + (cp * st * sf - sp * cf) * b.y + (sp * sf + cp * st * cf) * b.z;
    r.y = sp * ct * b.x + (cp * cf + sf * st * sp) * b.y + (st * sp * cf - cp * sf) * b.z;
    r.z = -st * b.x + ct * sf * b.y + ct * cf * b.z;
    return r;
}

// Плотная матрица небольшого размера.
struct Mx {
    int r = 0, c = 0;
    std::vector<double> a;
    Mx() = default;
    Mx(int rows, int cols, double v = 0.0) : r(rows), c(cols), a(static_cast<size_t>(rows) * cols, v) {}
    double& operator()(int i, int j) { return a[static_cast<size_t>(i) * c + j]; }
    double operator()(int i, int j) const { return a[static_cast<size_t>(i) * c + j]; }
    static Mx eye(int n) {
        Mx m(n, n);
        for (int i = 0; i < n; ++i) m(i, i) = 1.0;
        return m;
    }
    Mx T() const {
        Mx t(c, r);
        for (int i = 0; i < r; ++i)
            for (int j = 0; j < c; ++j) t(j, i) = (*this)(i, j);
        return t;
    }
};
inline Mx operator*(const Mx& A, const Mx& B) {
    if (A.c != B.r) throw std::runtime_error("Mx: размеры не согласованы");
    Mx R(A.r, B.c);
    for (int i = 0; i < A.r; ++i)
        for (int k = 0; k < A.c; ++k) {
            const double v = A(i, k);
            if (v == 0.0) continue;
            for (int j = 0; j < B.c; ++j) R(i, j) += v * B(k, j);
        }
    return R;
}
inline Mx operator+(const Mx& A, const Mx& B) {
    Mx R = A;
    for (size_t i = 0; i < R.a.size(); ++i) R.a[i] += B.a[i];
    return R;
}
inline Mx operator-(const Mx& A, const Mx& B) {
    Mx R = A;
    for (size_t i = 0; i < R.a.size(); ++i) R.a[i] -= B.a[i];
    return R;
}
inline Mx operator*(double s, const Mx& A) {
    Mx R = A;
    for (double& v : R.a) v *= s;
    return R;
}

// Решение A·X = B методом Гаусса с выбором главного элемента. Возвращает false, если матрица вырождена.
inline bool solveLinear(Mx A, Mx B, Mx& X) {
    const int n = A.r;
    for (int col = 0; col < n; ++col) {
        int piv = col;
        for (int i = col + 1; i < n; ++i)
            if (std::fabs(A(i, col)) > std::fabs(A(piv, col))) piv = i;
        if (std::fabs(A(piv, col)) < 1e-12) return false;
        if (piv != col) {
            for (int j = 0; j < A.c; ++j) std::swap(A(piv, j), A(col, j));
            for (int j = 0; j < B.c; ++j) std::swap(B(piv, j), B(col, j));
        }
        for (int i = col + 1; i < n; ++i) {
            const double f = A(i, col) / A(col, col);
            if (f == 0.0) continue;
            for (int j = col; j < A.c; ++j) A(i, j) -= f * A(col, j);
            for (int j = 0; j < B.c; ++j) B(i, j) -= f * B(col, j);
        }
    }
    X = Mx(n, B.c);
    for (int j = 0; j < B.c; ++j)
        for (int i = n - 1; i >= 0; --i) {
            double s = B(i, j);
            for (int k = i + 1; k < n; ++k) s -= A(i, k) * X(k, j);
            X(i, j) = s / A(i, i);
        }
    return true;
}

inline double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}


// Генератор случайных чисел с одинаковым результатом на всех компьютерах. std::mt19937 стандартизован, а
// std::normal_distribution и std::uniform_real_distribution — нет: на Mac (libc++) и Linux (libstdc++) они дают разные
// числа при одном начальном числе, и имитатор вёл себя по-разному. Нормальное — по Бокса — Мюллеру.
class PortableRng {
public:
    explicit PortableRng(unsigned seed = 5489u) : g_(seed) {}
    double uniform() { return (static_cast<double>(g_()) + 0.5) / 4294967296.0; }            // (0; 1)
    double uniform(double a, double b) { return a + (b - a) * uniform(); }
    double normal() {
        if (have_) { have_ = false; return spare_; }
        const double u1 = uniform(), u2 = uniform(), r = std::sqrt(-2.0 * std::log(u1)), t = 2.0 * 3.14159265358979323846 * u2;
        spare_ = r * std::sin(t); have_ = true; return r * std::cos(t);
    }
    double normal(double mean, double sigma) { return mean + sigma * normal(); }
private:
    std::mt19937 g_; double spare_ = 0; bool have_ = false;
};

}  // namespace dp
