// Система сообщений по образцу K-Pos (глава «Message system»): четыре приоритета, квитирование, отключение звука,
// состояния Active / Normal, динамическая страница тревог и журнал событий (история).
// Только логика; звук и мигание — дело окна оператора.
#pragma once
#include <algorithm>
#include <deque>
#include <string>
#include <vector>

#include "dp/engine.hpp"

namespace dp {

enum AlarmPrio { PrioEmergency = 0, PrioAlarm = 1, PrioWarning = 2, PrioInfo = 3 };
enum AlarmState { AlarmActive = 0, AlarmNormal = 1 };       // Normal — условие исчезло, но тревога ещё не квитирована

// Строка динамической страницы тревог (одна на каждое сообщение, пока оно не квитировано и не исчезло).
struct AlarmRec {
    long long id = 0;
    std::string code, text;
    int prio = PrioWarning, state = AlarmActive;
    bool acked = false;
    double tFirst = 0, tLast = 0;      // первое появление, последнее подтверждение условия
    int count = 1;                     // сколько раз возникало
};

// Строка журнала событий: появление, исчезновение, квитирование, информационное сообщение.
enum EventKind { EvActive = 0, EvNormal = 1, EvAck = 2, EvInfo = 3 };
struct EventRec {
    long long seq = 0;
    double t = 0;
    std::string code, text;
    int prio = PrioInfo, kind = EvInfo;
};

class AlarmMgr {
public:
    // holdS — сколько секунд условие должно отсутствовать, чтобы тревога стала Normal (в K-Pos обычно 20 с ⚠ у нас 5 с: условия считаются каждый цикл)
    explicit AlarmMgr(double holdS = 5.0) : hold_(holdS) {}
    void setHold(double s) { hold_ = s; }

    // Один шаг: cur — тревоги, действующие сейчас.
    void update(double t, const std::vector<Alarm>& cur) {
        seen_.assign(recs_.size(), false);
        for (const auto& a : cur) {
            const std::string key = keyOf_(a.code, a.text);
            size_t i = 0; for (; i < recs_.size(); ++i) if (keyOf_(recs_[i].code, recs_[i].text) == key) break;
            if (i == recs_.size()) {
                AlarmRec r; r.id = ++nextId_; r.code = a.code; r.text = a.text; r.prio = a.level(); r.tFirst = r.tLast = t;
                recs_.push_back(r); seen_.push_back(true);
                event_(t, r, EvActive);
                if (r.prio <= PrioAlarm) silenced_ = false;      // новая тревога снова включает звук
            } else {
                seen_[i] = true;
                AlarmRec& r = recs_[i];
                r.text = a.text;                                  // числа в тексте обновляются, запись остаётся той же
                if (r.state == AlarmNormal) {                    // условие вернулось
                    r.state = AlarmActive; r.acked = false; ++r.count; r.tFirst = t;
                    event_(t, r, EvActive);
                    if (r.prio <= PrioAlarm) silenced_ = false;
                }
                r.tLast = t;
            }
        }
        for (size_t i = 0; i < recs_.size();) {
            AlarmRec& r = recs_[i];
            if (r.state == AlarmActive && !seen_[i] && t - r.tLast >= hold_) { r.state = AlarmNormal; event_(t, r, EvNormal); }
            if (r.state == AlarmNormal && (r.acked || r.prio >= PrioInfo)) { recs_.erase(recs_.begin() + static_cast<long>(i)); seen_.erase(seen_.begin() + static_cast<long>(i)); continue; }
            ++i;
        }
    }

    // Квитирование: code пустой — все; иначе все записи с таким кодом. Возвращает число квитированных.
    int ack(double t, const std::string& code = "") {
        int n = 0;
        for (size_t i = 0; i < recs_.size(); ++i) {
            AlarmRec& r = recs_[i];
            if (!code.empty() && r.code != code) continue;
            if (r.acked) continue;
            r.acked = true; ++n; event_(t, r, EvAck);
        }
        for (size_t i = 0; i < recs_.size();) {                  // квитированные исчезнувшие уходят со страницы
            if (recs_[i].state == AlarmNormal && recs_[i].acked) { recs_.erase(recs_.begin() + static_cast<long>(i)); continue; }
            ++i;
        }
        seen_.assign(recs_.size(), false);
        silenced_ = true;
        return n;
    }

    void silence() { silenced_ = true; }                        // звук можно отключить без квитирования

    void info(double t, const std::string& text, const std::string& code = "INFO") {
        EventRec e; e.seq = ++evSeq_; e.t = t; e.code = code; e.text = text; e.prio = PrioInfo; e.kind = EvInfo; push_(e);
    }

    const std::vector<AlarmRec>& records() const { return recs_; }
    const std::deque<EventRec>& events() const { return ev_; }
    long long lastSeq() const { return evSeq_; }
    // Звук включён, пока есть неквитированная тревога или аварийное сообщение и звук не отключён.
    bool audible() const {
        if (silenced_) return false;
        for (auto& r : recs_) if (!r.acked && r.prio <= PrioAlarm) return true;
        return false;
    }
    // Мигание: любая неквитированная тревога, аварийное сообщение или предупреждение.
    int unacked() const { int n = 0; for (auto& r : recs_) if (!r.acked && r.prio <= PrioWarning) ++n; return n; }
    int active() const { int n = 0; for (auto& r : recs_) if (r.state == AlarmActive && r.prio <= PrioWarning) ++n; return n; }
    // Последние события с номерами больше after (для передачи постам).
    std::vector<EventRec> eventsSince(long long after, size_t maxN = 40) const {
        std::vector<EventRec> r; for (auto& e : ev_) if (e.seq > after) r.push_back(e);
        if (r.size() > maxN) r.erase(r.begin(), r.end() - static_cast<long>(maxN));
        return r;
    }

private:
    // Запись определяется кодом и, если в тексте есть « — », его началом (там имя системы): «Ошибка предсказания положения: GNSS1 — …».
    // Всё, что после тире, может меняться (числа, секунды) и на идентичность не влияет.
    static std::string keyOf_(const std::string& code, const std::string& text) {
        const size_t p = text.find(" — ");
        return code + "|" + (p == std::string::npos ? std::string() : text.substr(0, p));
    }
    void event_(double t, const AlarmRec& r, int kind) {
        EventRec e; e.seq = ++evSeq_; e.t = t; e.code = r.code; e.text = r.text; e.prio = r.prio; e.kind = kind; push_(e);
    }
    void push_(const EventRec& e) { ev_.push_back(e); while (ev_.size() > 2000) ev_.pop_front(); }

    double hold_;
    std::vector<AlarmRec> recs_;
    std::vector<bool> seen_;
    std::deque<EventRec> ev_;
    long long nextId_ = 0, evSeq_ = 0;
    bool silenced_ = false;
};

}  // namespace dp
