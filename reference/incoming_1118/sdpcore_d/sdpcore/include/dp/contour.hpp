// Подсказка контура по картинке: в выделенной области ищется самый крупный предмет, отличающийся от фона,
// и обводится многоугольником с небольшим числом вершин. Это подсказка: на фото с тенями и перспективой
// контур нужно поправить вручную.
// Порядок: яркость → порог Оцу → фон определяется по краю области → удаление тонких линий → крупнейшая связная область →
// заполнение дыр → обход внешней границы → упрощение (Дуглас — Пекер).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace dp {
namespace contour {

struct P { double x = 0, y = 0; };
struct Result { bool ok = false; std::vector<P> pts; double fillFrac = 0; const char* why = ""; };

// gray — яркость 0…255 построчно (w × h); область поиска [x0, x1) × [y0, y1)
// thinFrac — какие линии считать тонкими: тоньше 2·thinFrac диагонали области (по умолчанию около 1 % диагонали)
inline Result find(const std::vector<uint8_t>& gray, int w, int h, int x0, int y0, int x1, int y1, double simplifyFrac = 0.006, int maxPts = 40, double thinFrac = 0.004) {
    Result r;
    x0 = std::max(0, x0); y0 = std::max(0, y0); x1 = std::min(w, x1); y1 = std::min(h, y1);
    const int W = x1 - x0, H = y1 - y0;
    if (W < 8 || H < 8) { r.why = "область слишком мала"; return r; }
    // порог Оцу
    long hist[256] = {0}; for (int y = y0; y < y1; ++y) for (int x = x0; x < x1; ++x) ++hist[gray[static_cast<size_t>(y) * w + x]];
    const double total = double(W) * H; double sum = 0; for (int i = 0; i < 256; ++i) sum += i * double(hist[i]);
    double sB = 0, wB = 0, best = -1; int thr = 128;
    for (int t = 0; t < 256; ++t) {
        wB += hist[t]; if (wB == 0) continue; const double wF = total - wB; if (wF == 0) break;
        sB += t * double(hist[t]); const double mB = sB / wB, mF = (sum - sB) / wF, v = wB * wF * (mB - mF) * (mB - mF);
        if (v > best) { best = v; thr = t; }
    }
    std::vector<uint8_t> m(static_cast<size_t>(W) * H);
    for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) m[static_cast<size_t>(y) * W + x] = gray[static_cast<size_t>(y + y0) * w + x + x0] > thr ? 1 : 0;
    // фон — то, чего больше по краю области
    long edge1 = 0, edgeN = 0;
    for (int x = 0; x < W; ++x) { edge1 += m[x] + m[static_cast<size_t>(H - 1) * W + x]; edgeN += 2; }
    for (int y = 0; y < H; ++y) { edge1 += m[static_cast<size_t>(y) * W] + m[static_cast<size_t>(y) * W + W - 1]; edgeN += 2; }
    const uint8_t bg = edge1 * 2 > edgeN ? 1 : 0;
    for (auto& v : m) v = v == bg ? 0 : 1;                                   // 1 — предмет
    // размыкание: тонкие линии (ватерлиния, размерные и выносные линии, надписи) убираются, крупные части остаются
    const int rad = std::max(1, static_cast<int>(std::lround(thinFrac * std::hypot(W, H))));
    auto pass = [&](bool erode, bool horiz) {
        std::vector<uint8_t> o(m.size());
        const int n1 = horiz ? H : W, n2 = horiz ? W : H;
        for (int i = 0; i < n1; ++i) {
            int cnt = 0;                                                     // число единиц в окне [j − rad, j + rad]
            auto at = [&](int j) -> int { return horiz ? m[static_cast<size_t>(i) * W + j] : m[static_cast<size_t>(j) * W + i]; };
            for (int j = 0; j <= std::min(rad, n2 - 1); ++j) cnt += at(j);
            for (int j = 0; j < n2; ++j) {
                const int lo = std::max(0, j - rad), hi = std::min(n2 - 1, j + rad), len = hi - lo + 1;
                const uint8_t v = erode ? (cnt == len ? 1 : 0) : (cnt > 0 ? 1 : 0);
                (horiz ? o[static_cast<size_t>(i) * W + j] : o[static_cast<size_t>(j) * W + i]) = v;
                if (j - rad >= 0) cnt -= at(j - rad);
                if (j + rad + 1 < n2) cnt += at(j + rad + 1);
            }
        }
        m.swap(o);
    };
    pass(true, true); pass(true, false); pass(false, true); pass(false, false);
    // крупнейшая связная область (4-связность)
    std::vector<int> lab(m.size(), 0); int best_l = 0; long bestN = 0, nl = 0; std::vector<int> st;
    for (size_t i = 0; i < m.size(); ++i) {
        if (!m[i] || lab[i]) continue;
        ++nl; long n = 0; st.assign(1, static_cast<int>(i)); lab[i] = static_cast<int>(nl);
        while (!st.empty()) {
            const int k = st.back(); st.pop_back(); ++n; const int x = k % W, y = k / W;
            const int nb[4] = {x > 0 ? k - 1 : -1, x < W - 1 ? k + 1 : -1, y > 0 ? k - W : -1, y < H - 1 ? k + W : -1};
            for (int q : nb) if (q >= 0 && m[q] && !lab[q]) { lab[q] = static_cast<int>(nl); st.push_back(q); }
        }
        if (n > bestN) { bestN = n; best_l = static_cast<int>(nl); }
    }
    if (bestN < 30) { r.why = "предмет на фоне не найден"; return r; }
    // предмет без дыр: всё, до чего нельзя дойти от края области, не проходя через предмет
    std::vector<uint8_t> out(m.size(), 0);
    st.clear();
    auto seed = [&](int k) { if (lab[k] != best_l && !out[k]) { out[k] = 1; st.push_back(k); } };
    for (int x = 0; x < W; ++x) { seed(x); seed((H - 1) * W + x); }
    for (int y = 0; y < H; ++y) { seed(y * W); seed(y * W + W - 1); }
    while (!st.empty()) {
        const int k = st.back(); st.pop_back(); const int x = k % W, y = k / W;
        const int nb[4] = {x > 0 ? k - 1 : -1, x < W - 1 ? k + 1 : -1, y > 0 ? k - W : -1, y < H - 1 ? k + W : -1};
        for (int q : nb) if (q >= 0) seed(q);
    }
    auto inObj = [&](int x, int y) { return x >= 0 && y >= 0 && x < W && y < H && !out[static_cast<size_t>(y) * W + x]; };
    long objN = 0; for (auto v : out) objN += v ? 0 : 1;
    r.fillFrac = objN / total;
    // обход внешней границы (Мур): начинаем с верхней левой точки предмета
    int sx = -1, sy = -1;
    for (int y = 0; y < H && sx < 0; ++y) for (int x = 0; x < W; ++x) if (inObj(x, y)) { sx = x; sy = y; break; }
    static const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1}, dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    std::vector<P> b; int cx = sx, cy = sy, dir = 7; const size_t limit = static_cast<size_t>(4) * (W + H) * 8;
    do {
        b.push_back({double(cx), double(cy)});
        int k = (dir + 6) % 8, i = 0;
        for (; i < 8; ++i) { const int d = (k + i) % 8; if (inObj(cx + dx[d], cy + dy[d])) { cx += dx[d]; cy += dy[d]; dir = d; break; } }
        if (i == 8) break;
    } while ((cx != sx || cy != sy) && b.size() < limit);
    if (b.size() < 4) { r.why = "контур не обойдён"; return r; }
    // упрощение
    const double eps = std::max(1.0, simplifyFrac * std::hypot(W, H));
    std::vector<char> keep(b.size(), 0);
    auto rdp = [&](auto&& self, size_t a, size_t c) -> void {
        double dmax = 0; size_t idx = a;
        const double ax = b[a].x, ay = b[a].y, cx2 = b[c].x, cy2 = b[c].y, L = std::hypot(cx2 - ax, cy2 - ay);
        for (size_t i = a + 1; i < c; ++i) {
            const double d = L > 1e-9 ? std::fabs((cx2 - ax) * (ay - b[i].y) - (ax - b[i].x) * (cy2 - ay)) / L : std::hypot(b[i].x - ax, b[i].y - ay);
            if (d > dmax) { dmax = d; idx = i; }
        }
        if (dmax > eps) { keep[idx] = 1; self(self, a, idx); self(self, idx, c); }
    };
    // замкнутый контур: делим на две половины по самой дальней от начала точке
    size_t far = 0; double fd = 0; for (size_t i = 0; i < b.size(); ++i) { const double d = std::hypot(b[i].x - b[0].x, b[i].y - b[0].y); if (d > fd) { fd = d; far = i; } }
    keep[0] = 1; keep[far] = 1; rdp(rdp, 0, far);
    b.push_back(b[0]); keep.push_back(0); rdp(rdp, far, b.size() - 1); b.pop_back(); keep.pop_back();
    for (size_t i = 0; i < b.size(); ++i) if (keep[i]) r.pts.push_back({b[i].x + x0 + 0.5, b[i].y + y0 + 0.5});
    if (static_cast<int>(r.pts.size()) > maxPts) return find(gray, w, h, x0, y0, x1, y1, simplifyFrac * 1.6, maxPts, thinFrac);
    r.ok = r.pts.size() >= 3; if (!r.ok) r.why = "контур вырожден";
    return r;
}

}  // namespace contour
}  // namespace dp
