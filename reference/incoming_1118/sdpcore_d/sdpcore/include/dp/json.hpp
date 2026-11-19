// Небольшой разборщик JSON без внешних зависимостей. Допускает комментарии (// и /* */) и лишние запятые,
// чтобы файл настроек было удобно править вручную.
#pragma once
#include <cstdlib>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace dp {
namespace json {

struct Value {
    enum Type { Null, Bool, Num, Str, Arr, Obj } t = Null;
    bool b = false;
    double n = 0;
    std::string s;
    std::vector<Value> a;
    std::vector<std::pair<std::string, Value>> o;
    const Value* find(const std::string& k) const {
        for (const auto& kv : o) if (kv.first == k) return &kv.second;
        return nullptr;
    }
};

class Parser {
public:
    explicit Parser(const std::string& text) : s_(text) {}
    bool parse(Value& out, std::string& err) {
        try { ws(); out = value(); ws(); if (i_ != s_.size()) fail("лишние данные после конца документа"); }
        catch (const std::string& e) { err = e; return false; }
        return true;
    }
private:
    [[noreturn]] void fail(const std::string& m) {
        int line = 1, col = 1;
        for (size_t k = 0; k < i_ && k < s_.size(); ++k) { if (s_[k] == '\n') { ++line; col = 1; } else ++col; }
        throw "строка " + std::to_string(line) + ", столбец " + std::to_string(col) + ": " + m;
    }
    void ws() {
        while (i_ < s_.size()) {
            const char c = s_[i_];
            if (c == ' ' || c == '\t' || c == '\r' || c == '\n') ++i_;
            else if (c == '/' && i_ + 1 < s_.size() && s_[i_ + 1] == '/') { while (i_ < s_.size() && s_[i_] != '\n') ++i_; }
            else if (c == '/' && i_ + 1 < s_.size() && s_[i_ + 1] == '*') {
                i_ += 2; while (i_ + 1 < s_.size() && !(s_[i_] == '*' && s_[i_ + 1] == '/')) ++i_;
                if (i_ + 1 >= s_.size()) fail("не закрыт комментарий");
                i_ += 2;
            } else break;
        }
    }
    Value value() {
        if (i_ >= s_.size()) fail("неожиданный конец документа");
        const char c = s_[i_];
        Value v;
        if (c == '{') {
            ++i_; v.t = Value::Obj; ws();
            while (true) {
                ws();
                if (i_ < s_.size() && s_[i_] == '}') { ++i_; break; }
                if (i_ >= s_.size() || s_[i_] != '"') fail("ожидалось имя поля в кавычках");
                std::string k = str(); ws();
                if (i_ >= s_.size() || s_[i_] != ':') fail("ожидалось ':'");
                ++i_; ws();
                v.o.emplace_back(k, value()); ws();
                if (i_ < s_.size() && s_[i_] == ',') { ++i_; continue; }
                if (i_ < s_.size() && s_[i_] == '}') { ++i_; break; }
                fail("ожидалось ',' или '}'");
            }
        } else if (c == '[') {
            ++i_; v.t = Value::Arr;
            while (true) {
                ws();
                if (i_ < s_.size() && s_[i_] == ']') { ++i_; break; }
                v.a.push_back(value()); ws();
                if (i_ < s_.size() && s_[i_] == ',') { ++i_; continue; }
                if (i_ < s_.size() && s_[i_] == ']') { ++i_; break; }
                fail("ожидалось ',' или ']'");
            }
        } else if (c == '"') { v.t = Value::Str; v.s = str(); }
        else if (s_.compare(i_, 4, "true") == 0) { v.t = Value::Bool; v.b = true; i_ += 4; }
        else if (s_.compare(i_, 5, "false") == 0) { v.t = Value::Bool; v.b = false; i_ += 5; }
        else if (s_.compare(i_, 4, "null") == 0) { i_ += 4; }
        else {
            char* e = nullptr;
            v.n = std::strtod(s_.c_str() + i_, &e);
            if (e == s_.c_str() + i_) fail("неожиданный символ");
            i_ = static_cast<size_t>(e - s_.c_str()); v.t = Value::Num;
        }
        return v;
    }
    std::string str() {
        std::string r; ++i_;
        while (i_ < s_.size() && s_[i_] != '"') {
            char c = s_[i_++];
            if (c == '\\' && i_ < s_.size()) {
                const char e = s_[i_++];
                switch (e) {
                    case 'n': r += '\n'; break; case 't': r += '\t'; break; case 'r': r += '\r'; break;
                    case 'u': {   // только базовая плоскость, в UTF-8
                        unsigned cp = static_cast<unsigned>(std::strtoul(s_.substr(i_, 4).c_str(), nullptr, 16)); i_ += 4;
                        if (cp < 0x80) r += static_cast<char>(cp);
                        else if (cp < 0x800) { r += static_cast<char>(0xC0 | (cp >> 6)); r += static_cast<char>(0x80 | (cp & 0x3F)); }
                        else { r += static_cast<char>(0xE0 | (cp >> 12)); r += static_cast<char>(0x80 | ((cp >> 6) & 0x3F)); r += static_cast<char>(0x80 | (cp & 0x3F)); }
                        break;
                    }
                    default: r += e;
                }
            } else r += c;
        }
        if (i_ >= s_.size()) fail("не закрыта строка");
        ++i_;
        return r;
    }
    const std::string& s_;
    size_t i_ = 0;
};

inline bool parse(const std::string& text, Value& out, std::string& err) { return Parser(text).parse(out, err); }

// Чтение объекта с проверкой типов, значениями по умолчанию и обнаружением неизвестных полей.
struct Report { std::vector<std::string> errors, warnings; };

class Obj {
public:
    Obj(const Value* v, std::string path, Report* rep) : v_(v && v->t == Value::Obj ? v : nullptr), path_(std::move(path)), rep_(rep) {
        if (v && v->t != Value::Obj) rep_->errors.push_back(path_ + ": ожидался объект");
    }
    bool present() const { return v_ != nullptr; }
    double num(const std::string& k, double def) { const Value* x = get(k); if (!x) return def; if (x->t != Value::Num) { err(k, "ожидалось число"); return def; } return x->n; }
    int integer(const std::string& k, int def) { return static_cast<int>(num(k, def)); }
    bool boolean(const std::string& k, bool def) { const Value* x = get(k); if (!x) return def; if (x->t != Value::Bool) { err(k, "ожидалось true или false"); return def; } return x->b; }
    std::string str(const std::string& k, const std::string& def) { const Value* x = get(k); if (!x) return def; if (x->t != Value::Str) { err(k, "ожидалась строка"); return def; } return x->s; }
    Obj obj(const std::string& k) { const Value* x = get(k); return Obj(x, path_ + "." + k, rep_); }
    // Массив объектов
    std::vector<Obj> objs(const std::string& k) {
        std::vector<Obj> r; const Value* x = get(k); if (!x) return r;
        if (x->t != Value::Arr) { err(k, "ожидался массив"); return r; }
        for (size_t i = 0; i < x->a.size(); ++i) r.emplace_back(&x->a[i], path_ + "." + k + "[" + std::to_string(i) + "]", rep_);
        return r;
    }
    std::vector<double> nums(const std::string& k) {
        std::vector<double> r; const Value* x = get(k); if (!x) return r;
        if (x->t != Value::Arr) { err(k, "ожидался массив чисел"); return r; }
        for (const auto& e : x->a) { if (e.t != Value::Num) { err(k, "в массиве должны быть только числа"); return {}; } r.push_back(e.n); }
        return r;
    }
    // Массив массивов чисел
    std::vector<std::vector<double>> numLists(const std::string& k) {
        std::vector<std::vector<double>> r; const Value* x = get(k); if (!x) return r;
        if (x->t != Value::Arr) { err(k, "ожидался массив"); return r; }
        for (const auto& e : x->a) {
            std::vector<double> row;
            if (e.t != Value::Arr) { err(k, "ожидался массив массивов чисел"); return {}; }
            for (const auto& q : e.a) { if (q.t != Value::Num) { err(k, "только числа"); return {}; } row.push_back(q.n); }
            r.push_back(row);
        }
        return r;
    }
    bool has(const std::string& k) const { return v_ && v_->find(k); }
    void finish() {   // сообщает об опечатках в именах полей
        if (!v_) return;
        for (const auto& kv : v_->o) if (!used_.count(kv.first)) rep_->warnings.push_back(path_ + ": неизвестное поле «" + kv.first + "» (опечатка?)");
    }
    const std::string& path() const { return path_; }
    void error(const std::string& m) { rep_->errors.push_back(path_ + ": " + m); }
private:
    const Value* get(const std::string& k) { if (!v_) return nullptr; used_.insert(k); return v_->find(k); }
    void err(const std::string& k, const std::string& m) { rep_->errors.push_back(path_ + "." + k + ": " + m); }
    const Value* v_; std::string path_; Report* rep_; std::set<std::string> used_;
};

}  // namespace json
}  // namespace dp
