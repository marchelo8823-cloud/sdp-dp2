// Анализ последствий отказа (Consequence Analysis): удержится ли судно после наихудшего отказа при нынешней погоде.
// Основание: РС ч. XV 8.9.5–8.9.6 (классы DYNPOS-2, -3), MSC.1/Circ.1580 3.4.2.4; по образцу K-Pos 7.0, разд. 18 и Mega-Guard, 5.4.1.
// Раз в periodS (60 с) берётся среднее за это время усилие регулятора (в нём ветер, течение, волна) и для каждой группы наихудшего
// отказа проверяется, наберут ли его оставшиеся движители полным упором (распределитель — тот же, что в работе; колонки доведены
// до установившихся углов). Группы — колонка «Группа отказа» листа «Движители»; если её нет — одиночный отказ каждого движителя.
// Анализ идёт только в режиме «Позиция» при неподвижной опорной точке (как PRESENT у K-Pos), без возврата и с измерениями положения;
// иначе — пауза без сообщений. По умолчанию выключен, включается командой оператора (`class 2|3`).
// ⚠ Только по упору: электростанции в ядре нет, недостаток мощности («power critical» у K-Pos) не определяется.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>
#include "dp/thrust.hpp"

namespace dp {

// Распределение с колонками, доведёнными до установившихся углов (как capHolds в capability.hpp).
inline AllocResult caSettle(const Allocator& al, const Wrench& tau, double scale) {
    const size_t nt = al.thrusters().size();
    std::vector<double> act(nt, 0.0);
    AllocResult r;
    for (int it = 0; it < 6; ++it) {
        r = al.allocate(tau, act, scale);
        bool moved = false;
        for (size_t i = 0; i < nt && i < r.cmds.size(); ++i) {
            if (al.thrusters()[i].type != ThrusterType::Azimuth) continue;
            if (std::fabs(rad2deg(wrapPi(deg2rad(r.cmds[i].angleDeg - act[i])))) > 1.0) moved = true;
            act[i] = r.cmds[i].angleDeg;
        }
        if (!moved) break;
    }
    return r;
}

struct CaGroup {
    std::string name;                 // имя группы («ГРЩ-1») или обозначение движителя при одиночных отказах
    std::vector<std::string> ids;     // движители группы, которые сейчас в работе
    double forcePct = 100, momentPct = 100;   // сколько требуемой силы и момента наберут оставшиеся, %
    bool holds = true;                // набирают требуемое
    int fails = 0;                    // подряд расчётов с недостатком
    bool alarm = false;               // тревога CA_DRIFT_OFF стоит
};

struct CaMsg { std::string code, text; bool critical; };

class Consequence {
public:
    double periodS = 60.0;
    int dpClass() const { return cls_; }
    // класс 2 или 3 — включить, 0 — выключить; возвращает сообщение оператору
    std::string setClass(int c) {
        if (c != 0 && c != 2 && c != 3) return "Класс ДП для анализа последствий: 2, 3 или off.";
        const bool was = cls_ != 0; cls_ = c; reset(true);
        if (c == 0) return was ? "Анализ последствий остановлен." : "Анализ последствий и так выключен.";
        return "Анализ последствий: класс " + std::to_string(c) + ", расчёт раз в " + std::to_string(static_cast<int>(periodS)) + " с в режиме «Позиция» при неизменных точке и курсе.";
    }
    const std::vector<CaGroup>& groups() const { return gr_; }
    bool singleFailures() const { return single_; }
    double lastRunT() const { return lastT_; }
    bool ran() const { return ran_; }
    const Wrench& lastTau() const { return lastTau_; }

    // Шаг: active — условия анализа выполнены (режим, неподвижная опорная точка, есть положение); inMode — режим «Позиция»
    // (при выходе из него результаты и тревоги снимаются). tau — требуемое усилие регулятора, th — движители распределителя.
    void step(double t, double dt, bool inMode, bool active, const Wrench& tau, const std::vector<ThrusterCfg>& th, std::vector<CaMsg>& out) {
        if (cls_ == 0) return;
        if (!inMode) { if (!gr_.empty() || ran_) reset(false); return; }
        if (!active) { winT_ = 0; sum_ = Wrench{}; }
        else {
            winT_ += dt; sum_.X += tau.X * dt; sum_.Y += tau.Y * dt; sum_.N += tau.N * dt;
            if (winT_ >= periodS) {
                const Wrench mean{sum_.X / winT_, sum_.Y / winT_, sum_.N / winT_};
                analyze(mean, th); lastT_ = t; ran_ = true;
                winT_ = 0; sum_ = Wrench{};
            }
        }
        // сообщения — каждый шаг, пока состояние держится (как прочие тревоги ядра)
        if (single_) out.push_back({"CA_NO_GROUPS", "Анализ последствий: группы наихудшего отказа не заданы (колонка «Группа отказа» листа «Движители») — проверяется отказ каждого движителя по отдельности", false});
        for (const auto& g : gr_) if (g.alarm) {
            char b[400];
            std::snprintf(b, sizeof b, "Анализ последствий (класс %d): при отказе «%s» (%s) не хватит упора — наберётся %.0f %% силы и %.0f %% момента. Включите движители или поверните судно к равнодействующей среды",
                          cls_, g.name.c_str(), join(g.ids).c_str(), g.forcePct, g.momentPct);
            out.push_back({"CA_DRIFT_OFF", b, true});
            break;   // одна тревога; остальные группы — в пояснении команды ca
        }
    }

    // Расчёт сейчас по заданному усилию (команда ca); результаты и тревоги не меняет.
    std::vector<CaGroup> evaluate(const Wrench& tau, const std::vector<ThrusterCfg>& th) const {
        std::vector<CaGroup> g = makeGroups(th, nullptr);
        for (auto& x : g) check(x, tau, th, 1.0);
        return g;
    }
    static std::string join(const std::vector<std::string>& v) { std::string s; for (const auto& x : v) s += (s.empty() ? "" : ", ") + x; return s; }

    // Группы отказа из данных судна: имя → движители в работе. Если ни у одного движителя группа не задана — одиночные отказы.
    static std::vector<CaGroup> makeGroups(const std::vector<ThrusterCfg>& th, bool* single) {
        std::map<std::string, std::vector<std::string>> m; std::vector<std::string> order; bool any = false;
        for (const auto& t : th) {
            if (t.failGroups.empty()) continue;
            any = true;
            size_t p = 0;
            while (p <= t.failGroups.size()) {
                size_t q = t.failGroups.find(';', p); if (q == std::string::npos) q = t.failGroups.size();
                std::string n = t.failGroups.substr(p, q - p);
                while (!n.empty() && n.front() == ' ') n.erase(n.begin());
                while (!n.empty() && n.back() == ' ') n.pop_back();
                if (!n.empty()) { if (!m.count(n)) order.push_back(n); auto& v = m[n]; if (t.enabled) v.push_back(t.id); }
                p = q + 1;
            }
        }
        std::vector<CaGroup> g;
        if (single) *single = !any;
        if (!any) { for (const auto& t : th) if (t.enabled && t.maxForward > 0) { CaGroup x; x.name = t.id; x.ids = {t.id}; g.push_back(x); } }
        else for (const auto& n : order) { CaGroup x; x.name = n; x.ids = m[n]; g.push_back(x); }
        return g;
    }

private:
    int cls_ = 0;
    double winT_ = 0, lastT_ = -1; Wrench sum_, lastTau_; bool ran_ = false, single_ = false;
    std::vector<CaGroup> gr_;

    void reset(bool full) { winT_ = 0; sum_ = Wrench{}; gr_.clear(); ran_ = false; single_ = false; if (full) lastT_ = -1; }

    // Сколько требуемого усилия наберут движители без группы g; holds — с допуском capHolds (2 % силы + 500 Н, 3 % момента + 5 кН·м).
    static void check(CaGroup& g, const Wrench& tau, const std::vector<ThrusterCfg>& th, double k) {
        std::vector<ThrusterCfg> rest = th;
        for (auto& t : rest) if (std::find(g.ids.begin(), g.ids.end(), t.id) != g.ids.end()) t.enabled = false;
        const Wrench need{tau.X * k, tau.Y * k, tau.N * k};
        bool anyOn = false; for (const auto& t : rest) anyOn = anyOn || (t.enabled && t.maxForward > 0);
        Wrench ach;
        if (anyOn) { Allocator al(rest); ach = caSettle(al, need, 1.0).achieved; }
        const double fn = std::hypot(need.X, need.Y), fe = std::hypot(need.X - ach.X, need.Y - ach.Y), ne = std::fabs(need.N - ach.N);
        g.forcePct = fn < 500.0 ? 100.0 : clampd(100.0 * (1.0 - fe / fn), 0.0, 100.0);
        g.momentPct = std::fabs(need.N) < 5000.0 ? 100.0 : clampd(100.0 * (1.0 - ne / std::fabs(need.N)), 0.0, 100.0);
        g.holds = fe <= 0.02 * fn + 500.0 && ne <= 0.03 * std::fabs(need.N) + 5000.0;
    }

    void analyze(const Wrench& tau, const std::vector<ThrusterCfg>& th) {
        lastTau_ = tau;
        std::vector<CaGroup> ng = makeGroups(th, &single_);
        for (auto& g : ng) {
            for (const auto& o : gr_) if (o.name == g.name) { g.fails = o.fails; g.alarm = o.alarm; }
            check(g, tau, th, 1.0);
            if (!g.holds) { ++g.fails; if (g.fails >= 2) g.alarm = true; }
            else {
                g.fails = 0;
                if (g.alarm) { CaGroup m = g; check(m, tau, th, 1.05); if (m.holds) g.alarm = false; }   // снимается только с запасом 5 %
            }
        }
        gr_ = ng;
    }
};

}  // namespace dp
