// Чтение ZIP-архивов без внешних библиотек: хранение (0) и Deflate (8), проверка CRC-32.
// Нужно для KMZ-карт. Не поддержано: Zip64, шифрование, многотомные архивы.
#pragma once
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace dp {
namespace zip {

inline uint32_t crc32(const uint8_t* p, size_t n) {
    static uint32_t table[256]; static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; ++i) { uint32_t c = i; for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1; table[i] = c; }
        init = true;
    }
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i) c = table[(c ^ p[i]) & 255] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

namespace detail {
// Распаковка Deflate по схеме из puff.c (М. Адлер): простая и проверяемая.
struct Huff { uint16_t count[16]; uint16_t symbol[288]; };

struct Inflater {
    const uint8_t* in; size_t inlen, incnt = 0; uint32_t bitbuf = 0; int bitcnt = 0;
    std::vector<uint8_t>& out; size_t limit;
    Inflater(const uint8_t* p, size_t n, std::vector<uint8_t>& o, size_t lim) : in(p), inlen(n), out(o), limit(lim) {}

    int bits(int need) {
        uint32_t val = bitbuf;
        while (bitcnt < need) {
            if (incnt >= inlen) throw std::runtime_error("неожиданный конец данных");
            val |= uint32_t(in[incnt++]) << bitcnt; bitcnt += 8;
        }
        bitbuf = val >> need; bitcnt -= need;
        return int(val & ((1u << need) - 1));
    }
    static int construct(Huff& h, const uint16_t* length, int n) {
        for (int l = 0; l <= 15; ++l) h.count[l] = 0;
        for (int s = 0; s < n; ++s) ++h.count[length[s]];
        if (h.count[0] == n) return 0;
        int left = 1;
        for (int l = 1; l <= 15; ++l) { left <<= 1; left -= h.count[l]; if (left < 0) return left; }
        uint16_t offs[16]; offs[1] = 0;
        for (int l = 1; l < 15; ++l) offs[l + 1] = uint16_t(offs[l] + h.count[l]);
        for (int s = 0; s < n; ++s) if (length[s] != 0) h.symbol[offs[length[s]]++] = uint16_t(s);
        return left;
    }
    int decode(const Huff& h) {
        int code = 0, first = 0, index = 0;
        for (int len = 1; len <= 15; ++len) {
            code |= bits(1);
            const int count = h.count[len];
            if (code - count < first) return h.symbol[index + (code - first)];
            index += count; first += count; first <<= 1; code <<= 1;
        }
        throw std::runtime_error("неверный код Хаффмана");
    }
    void codes(const Huff& lencode, const Huff& distcode) {
        static const uint16_t lens[29] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
        static const uint16_t lext[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
        static const uint16_t dists[30] = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
        static const uint16_t dext[30] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};
        for (;;) {
            int sym = decode(lencode);
            if (sym < 256) { if (out.size() >= limit) throw std::runtime_error("слишком большой размер"); out.push_back(uint8_t(sym)); }
            else if (sym == 256) return;
            else {
                sym -= 257; if (sym >= 29) throw std::runtime_error("неверный код длины");
                const int len = lens[sym] + bits(lext[sym]);
                const int ds = decode(distcode); if (ds >= 30) throw std::runtime_error("неверный код расстояния");
                const size_t dist = size_t(dists[ds]) + size_t(bits(dext[ds]));
                if (dist > out.size()) throw std::runtime_error("расстояние за пределами данных");
                if (out.size() + len > limit) throw std::runtime_error("слишком большой размер");
                for (int k = 0; k < len; ++k) out.push_back(out[out.size() - dist]);
            }
        }
    }
    void run() {
        int last;
        do {
            last = bits(1);
            const int type = bits(2);
            if (type == 0) {
                bitbuf = 0; bitcnt = 0;
                if (incnt + 4 > inlen) throw std::runtime_error("обрезанный блок");
                const unsigned len = in[incnt] | (in[incnt + 1] << 8), nlen = in[incnt + 2] | (in[incnt + 3] << 8);
                incnt += 4;
                if (len != (~nlen & 0xFFFF)) throw std::runtime_error("повреждён блок без сжатия");
                if (incnt + len > inlen) throw std::runtime_error("обрезанный блок");
                if (out.size() + len > limit) throw std::runtime_error("слишком большой размер");
                out.insert(out.end(), in + incnt, in + incnt + len); incnt += len;
            } else if (type == 1) {
                Huff lc, dc; uint16_t lengths[288];
                int s = 0;
                for (; s < 144; ++s) lengths[s] = 8;
                for (; s < 256; ++s) lengths[s] = 9;
                for (; s < 280; ++s) lengths[s] = 7;
                for (; s < 288; ++s) lengths[s] = 8;
                construct(lc, lengths, 288);
                for (s = 0; s < 30; ++s) lengths[s] = 5;
                construct(dc, lengths, 30);
                codes(lc, dc);
            } else if (type == 2) {
                static const uint8_t order[19] = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
                uint16_t lengths[320];
                const int nlen = bits(5) + 257, ndist = bits(5) + 1, ncode = bits(4) + 4;
                if (nlen > 286 || ndist > 30) throw std::runtime_error("неверные таблицы");
                int index = 0;
                for (; index < ncode; ++index) lengths[order[index]] = uint16_t(bits(3));
                for (; index < 19; ++index) lengths[order[index]] = 0;
                Huff lc, dc;
                if (construct(lc, lengths, 19) != 0) throw std::runtime_error("неполная таблица длин");
                index = 0;
                while (index < nlen + ndist) {
                    int sym = decode(lc);
                    if (sym < 16) lengths[index++] = uint16_t(sym);
                    else {
                        int len = 0, rep;
                        if (sym == 16) { if (index == 0) throw std::runtime_error("повтор без предыдущего"); len = lengths[index - 1]; rep = 3 + bits(2); }
                        else if (sym == 17) rep = 3 + bits(3); else rep = 11 + bits(7);
                        if (index + rep > nlen + ndist) throw std::runtime_error("слишком длинный повтор");
                        while (rep--) lengths[index++] = uint16_t(len);
                    }
                }
                if (lengths[256] == 0) throw std::runtime_error("нет конца блока");
                const int e1 = construct(lc, lengths, nlen);
                if (e1 < 0 || (e1 > 0 && nlen - lc.count[0] != 1)) throw std::runtime_error("неверная таблица литералов");
                const int e2 = construct(dc, lengths + nlen, ndist);
                if (e2 < 0 || (e2 > 0 && ndist - dc.count[0] != 1)) throw std::runtime_error("неверная таблица расстояний");
                codes(lc, dc);
            } else throw std::runtime_error("неверный тип блока");
        } while (!last);
    }
};
}  // namespace detail

struct Entry { std::string name; uint32_t method = 0, crc = 0, csize = 0, usize = 0, localOffset = 0; };

class Archive {
public:
    // Открывает архив из памяти; в случае ошибки возвращает false и заполняет err.
    bool open(const std::vector<uint8_t>& data, std::string& err) {
        data_ = &data; entries_.clear();
        const size_t n = data.size();
        if (n < 22) { err = "Файл слишком короткий для ZIP."; return false; }
        size_t eocd = std::string::npos;
        const size_t from = n > 22 + 65535 ? n - 22 - 65535 : 0;
        for (size_t i = n - 22 + 1; i-- > from;) if (u32(i) == 0x06054b50u) { eocd = i; break; }
        if (eocd == std::string::npos) { err = "Это не ZIP-архив (нет оглавления)."; return false; }
        const unsigned count = u16(eocd + 10); size_t cd = u32(eocd + 16);
        if (count == 0xFFFF || cd == 0xFFFFFFFFu) { err = "Zip64 не поддерживается."; return false; }
        for (unsigned i = 0; i < count; ++i) {
            if (cd + 46 > n || u32(cd) != 0x02014b50u) { err = "Повреждено оглавление ZIP."; return false; }
            Entry e; e.method = u16(cd + 10); e.crc = u32(cd + 16); e.csize = u32(cd + 20); e.usize = u32(cd + 24); e.localOffset = u32(cd + 42);
            const size_t nl = u16(cd + 28), xl = u16(cd + 30), cl = u16(cd + 32);
            if (cd + 46 + nl > n) { err = "Повреждено оглавление ZIP."; return false; }
            e.name.assign(reinterpret_cast<const char*>(&data[cd + 46]), nl);
            if (u16(cd + 8) & 1) { err = "Архив зашифрован."; return false; }
            entries_.push_back(e);
            cd += 46 + nl + xl + cl;
        }
        return true;
    }
    const std::vector<Entry>& entries() const { return entries_; }
    const Entry* find(const std::string& name) const { for (auto& e : entries_) if (e.name == name) return &e; return nullptr; }

    // Содержимое файла из архива; limit — предельный размер распакованных данных.
    bool read(const Entry& e, std::vector<uint8_t>& out, std::string& err, size_t limit = 512u << 20) const {
        out.clear(); const auto& d = *data_;
        const size_t lo = e.localOffset;
        if (lo + 30 > d.size() || u32(lo) != 0x04034b50u) { err = "Повреждён заголовок файла «" + e.name + "»."; return false; }
        const size_t start = lo + 30 + u16(lo + 26) + u16(lo + 28);
        if (start + e.csize > d.size()) { err = "Файл «" + e.name + "» обрезан."; return false; }
        if (e.usize > limit) { err = "Файл «" + e.name + "» слишком велик."; return false; }
        try {
            if (e.method == 0) out.assign(d.begin() + start, d.begin() + start + e.csize);
            else if (e.method == 8) { out.reserve(e.usize); detail::Inflater inf(d.data() + start, e.csize, out, limit); inf.run(); }
            else { err = "Метод сжатия " + std::to_string(e.method) + " в файле «" + e.name + "» не поддерживается."; return false; }
        } catch (const std::exception& ex) { err = "Ошибка распаковки «" + e.name + "»: " + ex.what(); out.clear(); return false; }
        if (out.size() != e.usize || crc32(out.data(), out.size()) != e.crc) { err = "Файл «" + e.name + "» повреждён (не совпала контрольная сумма)."; out.clear(); return false; }
        return true;
    }

private:
    unsigned u16(size_t o) const { const auto& d = *data_; return o + 2 <= d.size() ? unsigned(d[o] | d[o + 1] << 8) : 0u; }
    uint32_t u32(size_t o) const { const auto& d = *data_; return o + 4 <= d.size() ? uint32_t(d[o]) | uint32_t(d[o + 1]) << 8 | uint32_t(d[o + 2]) << 16 | uint32_t(d[o + 3]) << 24 : 0u; }
    const std::vector<uint8_t>* data_ = nullptr;
    std::vector<Entry> entries_;
};

}  // namespace zip
}  // namespace dp
