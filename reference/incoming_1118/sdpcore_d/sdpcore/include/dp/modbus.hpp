// Modbus-мастер: кадры Modbus TCP и Modbus RTU, транспорты TCP и последовательный порт.
// Функции: 0x03 (чтение регистров хранения), 0x06 (запись одного), 0x10 (запись нескольких).
#pragma once
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "dp/io.hpp"

namespace dp {
namespace mb {

inline uint16_t crc16(const uint8_t* d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        crc ^= d[i];
        for (int b = 0; b < 8; ++b) crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
    }
    return crc;
}

// ---------------------------------------------------------------- транспорт
class Transport {
public:
    virtual ~Transport() = default;
    virtual bool open(std::string& err) = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
    virtual bool writeAll(const uint8_t* d, size_t n) = 0;
    virtual bool readExact(uint8_t* d, size_t n, int timeoutMs) = 0;
    virtual void flushInput() = 0;
    virtual bool isRtu() const = 0;
    virtual int baud() const { return 0; }
};

class TcpTransport : public Transport {
public:
    TcpTransport(std::string host, int port) : host_(std::move(host)), port_(port) {}
    ~TcpTransport() override { close(); }
    bool isRtu() const override { return false; }
    bool isOpen() const override { return s_ != BAD_SOCK; }
    bool open(std::string& err) override {
        close(); netInit();
        addrinfo hints{}, *res = nullptr;
        hints.ai_family = AF_INET; hints.ai_socktype = SOCK_STREAM;
        if (::getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res) != 0 || !res) { err = "адрес не найден: " + host_; return false; }
        s_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (s_ == BAD_SOCK) { ::freeaddrinfo(res); err = "не удалось создать сокет"; return false; }
#ifdef _WIN32
        DWORD to = 2000; ::setsockopt(s_, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&to), sizeof to);
#else
        timeval tv{2, 0}; ::setsockopt(s_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
#endif
        if (::connect(s_, res->ai_addr, static_cast<int>(res->ai_addrlen)) != 0) { ::freeaddrinfo(res); close(); err = "нет соединения с " + host_ + ":" + std::to_string(port_); return false; }
        ::freeaddrinfo(res);
        int one = 1; ::setsockopt(s_, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof one);
        return true;
    }
    void close() override { if (s_ != BAD_SOCK) { sockClose(s_); s_ = BAD_SOCK; } }
    bool writeAll(const uint8_t* d, size_t n) override {
        size_t off = 0;
        while (off < n) {
#ifdef _WIN32
            int r = ::send(s_, reinterpret_cast<const char*>(d + off), static_cast<int>(n - off), 0);
#else
            ssize_t r = ::send(s_, d + off, n - off, MSG_NOSIGNAL_FLAG);
#endif
            if (r <= 0) return false;
            off += static_cast<size_t>(r);
        }
        return true;
    }
    bool readExact(uint8_t* d, size_t n, int timeoutMs) override {
        size_t off = 0;
        const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
        while (off < n) {
            const int left = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(end - std::chrono::steady_clock::now()).count());
            if (left <= 0) return false;
#ifdef _WIN32
            fd_set fds; FD_ZERO(&fds); FD_SET(s_, &fds); timeval tv{left / 1000, (left % 1000) * 1000};
            if (::select(0, &fds, nullptr, nullptr, &tv) <= 0) return false;
            int r = ::recv(s_, reinterpret_cast<char*>(d + off), static_cast<int>(n - off), 0);
#else
            pollfd p{s_, POLLIN, 0};
            if (::poll(&p, 1, left) <= 0) return false;
            ssize_t r = ::recv(s_, d + off, n - off, 0);
#endif
            if (r <= 0) return false;
            off += static_cast<size_t>(r);
        }
        return true;
    }
    void flushInput() override {
        uint8_t b[256];
        while (true) {
#ifdef _WIN32
            u_long avail = 0; if (ioctlsocket(s_, FIONREAD, &avail) != 0 || avail == 0) return;
            if (::recv(s_, reinterpret_cast<char*>(b), sizeof b, 0) <= 0) return;
#else
            pollfd p{s_, POLLIN, 0};
            if (::poll(&p, 1, 0) <= 0) return;
            if (::recv(s_, b, sizeof b, MSG_DONTWAIT) <= 0) return;
#endif
        }
    }
private:
#ifdef MSG_NOSIGNAL
    static constexpr int MSG_NOSIGNAL_FLAG = MSG_NOSIGNAL;
#else
    static constexpr int MSG_NOSIGNAL_FLAG = 0;
#endif
    std::string host_; int port_; SockT s_ = BAD_SOCK;
};

class SerialTransport : public Transport {
public:
    SerialTransport(std::string dev, int baud, char parity = 'N', int stopBits = 1) : dev_(std::move(dev)), baud_(baud), parity_(parity), stop_(stopBits) {}
    ~SerialTransport() override { close(); }
    bool isRtu() const override { return true; }
    int baud() const override { return baud_; }
#ifdef _WIN32
    bool isOpen() const override { return h_ != INVALID_HANDLE_VALUE; }
    bool open(std::string& err) override {
        close();
        std::string name = dev_.rfind("COM", 0) == 0 ? "\\\\.\\" + dev_ : dev_;
        h_ = CreateFileA(name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
        if (h_ == INVALID_HANDLE_VALUE) { err = "не удалось открыть " + dev_; return false; }
        DCB dcb{}; dcb.DCBlength = sizeof dcb; GetCommState(h_, &dcb);
        dcb.BaudRate = static_cast<DWORD>(baud_); dcb.ByteSize = 8; dcb.fBinary = TRUE;
        dcb.Parity = parity_ == 'E' ? EVENPARITY : parity_ == 'O' ? ODDPARITY : NOPARITY; dcb.fParity = parity_ != 'N';
        dcb.StopBits = stop_ == 2 ? TWOSTOPBITS : ONESTOPBIT;
        dcb.fOutxCtsFlow = dcb.fOutxDsrFlow = FALSE; dcb.fOutX = dcb.fInX = FALSE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE; dcb.fRtsControl = RTS_CONTROL_ENABLE;
        SetCommState(h_, &dcb);
        COMMTIMEOUTS to{}; to.ReadIntervalTimeout = MAXDWORD; SetCommTimeouts(h_, &to);   // чтение без ожидания
        return true;
    }
    void close() override { if (h_ != INVALID_HANDLE_VALUE) { CloseHandle(h_); h_ = INVALID_HANDLE_VALUE; } }
    bool writeAll(const uint8_t* d, size_t n) override { DWORD w = 0; return WriteFile(h_, d, static_cast<DWORD>(n), &w, nullptr) && w == n; }
    bool readExact(uint8_t* d, size_t n, int timeoutMs) override {
        size_t off = 0; const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
        while (off < n) {
            DWORD r = 0;
            if (!ReadFile(h_, d + off, static_cast<DWORD>(n - off), &r, nullptr)) return false;
            if (r > 0) { off += r; continue; }
            if (std::chrono::steady_clock::now() > end) return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return true;
    }
    void flushInput() override { PurgeComm(h_, PURGE_RXCLEAR); }
private:
    HANDLE h_ = INVALID_HANDLE_VALUE;
#else
    bool isOpen() const override { return fd_ >= 0; }
    bool open(std::string& err) override {
        close();
        fd_ = ::open(dev_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) { err = "не удалось открыть " + dev_; return false; }
        termios t{};
        if (::tcgetattr(fd_, &t) != 0) { close(); err = "не удалось прочитать параметры порта"; return false; }
        cfmakeraw(&t);
        const speed_t sp = speedOf(baud_);
        cfsetispeed(&t, sp); cfsetospeed(&t, sp);
        t.c_cflag |= (CLOCAL | CREAD); t.c_cflag &= ~(PARENB | PARODD | CSTOPB | CSIZE | CRTSCTS); t.c_cflag |= CS8;
        if (parity_ == 'E') t.c_cflag |= PARENB; else if (parity_ == 'O') t.c_cflag |= (PARENB | PARODD);
        if (stop_ == 2) t.c_cflag |= CSTOPB;
        t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
        ::tcsetattr(fd_, TCSANOW, &t);
        return true;
    }
    void close() override { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
    bool writeAll(const uint8_t* d, size_t n) override {
        size_t off = 0;
        while (off < n) {
            ssize_t r = ::write(fd_, d + off, n - off);
            if (r > 0) { off += static_cast<size_t>(r); continue; }
            if (r < 0 && (errno == EAGAIN || errno == EINTR)) { pollfd p{fd_, POLLOUT, 0}; ::poll(&p, 1, 100); continue; }
            return false;
        }
        ::tcdrain(fd_);
        return true;
    }
    bool readExact(uint8_t* d, size_t n, int timeoutMs) override {
        size_t off = 0; const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
        while (off < n) {
            const int left = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(end - std::chrono::steady_clock::now()).count());
            if (left <= 0) return false;
            pollfd p{fd_, POLLIN, 0};
            if (::poll(&p, 1, left) <= 0) return false;
            ssize_t r = ::read(fd_, d + off, n - off);
            if (r > 0) off += static_cast<size_t>(r);
            else if (r == 0 || (errno != EAGAIN && errno != EINTR)) return false;
        }
        return true;
    }
    void flushInput() override { if (fd_ >= 0) ::tcflush(fd_, TCIFLUSH); }
private:
    static speed_t speedOf(int b) {
        switch (b) {
            case 1200: return B1200; case 2400: return B2400; case 4800: return B4800; case 9600: return B9600;
            case 19200: return B19200; case 38400: return B38400; case 57600: return B57600; case 115200: return B115200;
            default: return B9600;
        }
    }
    int fd_ = -1;
#endif
    std::string dev_; int baud_; char parity_; int stop_;
};

// ---------------------------------------------------------------- мастер
class Master {
public:
    Master(std::unique_ptr<Transport> t, int unitId, int timeoutMs = 500) : t_(std::move(t)), unit_(unitId), timeout_(timeoutMs) {}
    Transport& transport() { return *t_; }
    bool open(std::string& err) { return t_->open(err); }
    void close() { t_->close(); }
    bool isOpen() const { return t_->isOpen(); }

    bool writeRegisters(int addr, const std::vector<uint16_t>& v, std::string& err) {
        if (v.empty() || v.size() > 123) { err = "недопустимое число регистров"; return false; }
        std::vector<uint8_t> pdu = {0x10, hi(addr), lo(addr), hi(static_cast<int>(v.size())), lo(static_cast<int>(v.size())), static_cast<uint8_t>(v.size() * 2)};
        for (uint16_t x : v) { pdu.push_back(static_cast<uint8_t>(x >> 8)); pdu.push_back(static_cast<uint8_t>(x & 0xFF)); }
        std::vector<uint8_t> rsp;
        if (!transact(pdu, 5, rsp, err)) return false;
        if (rsp.size() < 5 || rsp[0] != 0x10 || ((rsp[1] << 8) | rsp[2]) != addr || ((rsp[3] << 8) | rsp[4]) != static_cast<int>(v.size())) { err = "неверный ответ на запись"; return false; }
        return true;
    }
    bool writeRegister(int addr, uint16_t value, std::string& err) {
        std::vector<uint8_t> pdu = {0x06, hi(addr), lo(addr), static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
        std::vector<uint8_t> rsp;
        if (!transact(pdu, 5, rsp, err)) return false;
        if (rsp.size() < 5 || rsp[0] != 0x06) { err = "неверный ответ на запись"; return false; }
        return true;
    }
    bool readRegisters(int addr, int count, std::vector<uint16_t>& out, std::string& err) {
        if (count < 1 || count > 125) { err = "недопустимое число регистров"; return false; }
        std::vector<uint8_t> pdu = {0x03, hi(addr), lo(addr), hi(count), lo(count)};
        std::vector<uint8_t> rsp;
        if (!transact(pdu, 2 + count * 2, rsp, err)) return false;
        if (rsp.size() < static_cast<size_t>(2 + count * 2) || rsp[0] != 0x03 || rsp[1] != count * 2) { err = "неверный ответ на чтение"; return false; }
        out.clear();
        for (int i = 0; i < count; ++i) out.push_back(static_cast<uint16_t>((rsp[2 + 2 * i] << 8) | rsp[3 + 2 * i]));
        return true;
    }

private:
    static uint8_t hi(int x) { return static_cast<uint8_t>((x >> 8) & 0xFF); }
    static uint8_t lo(int x) { return static_cast<uint8_t>(x & 0xFF); }

    // Отправляет PDU и принимает ответ. okLen — ожидаемая длина PDU нормального ответа.
    bool transact(const std::vector<uint8_t>& pdu, int okLen, std::vector<uint8_t>& rsp, std::string& err) {
        if (!t_->isOpen()) { err = "канал не открыт"; return false; }
        t_->flushInput();
        if (t_->isRtu()) return transactRtu(pdu, okLen, rsp, err);
        return transactTcp(pdu, rsp, err);
    }
    bool transactTcp(const std::vector<uint8_t>& pdu, std::vector<uint8_t>& rsp, std::string& err) {
        ++tid_;
        std::vector<uint8_t> f = {hi(tid_), lo(tid_), 0, 0, hi(static_cast<int>(pdu.size()) + 1), lo(static_cast<int>(pdu.size()) + 1), static_cast<uint8_t>(unit_)};
        f.insert(f.end(), pdu.begin(), pdu.end());
        if (!t_->writeAll(f.data(), f.size())) { err = "ошибка записи в канал"; return false; }
        uint8_t h[7];
        if (!t_->readExact(h, 7, timeout_)) { err = "нет ответа (таймаут)"; return false; }
        const int len = (h[4] << 8) | h[5];
        if (((h[0] << 8) | h[1]) != (tid_ & 0xFFFF) || h[2] != 0 || h[3] != 0 || len < 2 || len > 254) { err = "неверный заголовок ответа"; return false; }
        rsp.assign(static_cast<size_t>(len - 1), 0);
        if (!t_->readExact(rsp.data(), rsp.size(), timeout_)) { err = "ответ неполный"; return false; }
        return checkException(rsp, err);
    }
    bool transactRtu(const std::vector<uint8_t>& pdu, int okLen, std::vector<uint8_t>& rsp, std::string& err) {
        gap();
        std::vector<uint8_t> f = {static_cast<uint8_t>(unit_)};
        f.insert(f.end(), pdu.begin(), pdu.end());
        const uint16_t c = crc16(f.data(), f.size());
        f.push_back(static_cast<uint8_t>(c & 0xFF)); f.push_back(static_cast<uint8_t>(c >> 8));
        if (!t_->writeAll(f.data(), f.size())) { err = "ошибка записи в канал"; return false; }
        uint8_t h[2];
        if (!t_->readExact(h, 2, timeout_)) { err = "нет ответа (таймаут)"; return false; }
        std::vector<uint8_t> all = {h[0], h[1]};
        const size_t rest = (h[1] & 0x80) ? 3 : static_cast<size_t>(okLen - 1 + 2);   // тело после кода функции + CRC
        std::vector<uint8_t> tail(rest);
        if (!t_->readExact(tail.data(), rest, timeout_)) { err = "ответ неполный"; return false; }
        all.insert(all.end(), tail.begin(), tail.end());
        const uint16_t got = static_cast<uint16_t>(all[all.size() - 2] | (all[all.size() - 1] << 8));
        if (crc16(all.data(), all.size() - 2) != got) { err = "ошибка CRC в ответе"; return false; }
        if (all[0] != unit_) { err = "ответ от другого устройства"; return false; }
        rsp.assign(all.begin() + 1, all.end() - 2);
        return checkException(rsp, err);
    }
    bool checkException(const std::vector<uint8_t>& rsp, std::string& err) {
        if (!rsp.empty() && (rsp[0] & 0x80)) {
            const int code = rsp.size() > 1 ? rsp[1] : 0;
            err = "исключение Modbus, код " + std::to_string(code);
            return false;
        }
        return true;
    }
    void gap() {   // пауза между кадрами RTU не менее 3,5 символа
        const int b = t_->baud() > 0 ? t_->baud() : 9600;
        const double ms = std::max(2.0, 3.5 * 11.0 * 1000.0 / b);
        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long long>(ms * 1000)));
    }
    std::unique_ptr<Transport> t_; int unit_; int timeout_; uint16_t tid_ = 0;
};

}  // namespace mb
}  // namespace dp
