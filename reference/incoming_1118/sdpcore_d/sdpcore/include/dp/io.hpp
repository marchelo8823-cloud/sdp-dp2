// Каналы ввода: последовательный порт (COM), UDP, TCP-клиент. Каждый канал работает в своём потоке,
// собирает строки и передаёт их в обработчик.
#pragma once
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  include <windows.h>
#else
#  include <arpa/inet.h>
#  include <fcntl.h>
#  include <netdb.h>
#  include <netinet/in.h>
#  include <netinet/tcp.h>
#  include <poll.h>
#  include <sys/socket.h>
#  include <termios.h>
#  include <unistd.h>
#endif

namespace dp {

using LineHandler = std::function<void(const std::string& line)>;

inline double monotonicSeconds() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
}

// Собирает строки из потока байт (разделители \r и \n).
class LineAssembler {
public:
    explicit LineAssembler(LineHandler h, size_t maxLen = 512) : h_(std::move(h)), max_(maxLen) {}
    void feed(const char* data, size_t n) {
        for (size_t i = 0; i < n; ++i) {
            const char c = data[i];
            if (c == '\n' || c == '\r') { if (!buf_.empty()) { h_(buf_); buf_.clear(); } }
            else if (buf_.size() < max_) buf_.push_back(c);
            else buf_.clear();     // слишком длинная строка — мусор
        }
    }
private:
    LineHandler h_; size_t max_; std::string buf_;
};

struct IoStatus { bool connected = false; long long bytes = 0; std::string lastError; };

class Channel {
public:
    virtual ~Channel() { stop(); }
    void start(LineHandler h) {
        if (running_) return;
        handler_ = std::move(h);
        running_ = true;
        th_ = std::thread([this] { run(); });
    }
    void stop() { running_ = false; if (th_.joinable()) th_.join(); }
    IoStatus status() const { return {connected_.load(), bytes_.load(), lastErrorCopy()}; }
protected:
    virtual void run() = 0;
    void setError(const std::string& e) { std::lock_guard<std::mutex> lk(em_); err_ = e; }
    std::string lastErrorCopy() const { std::lock_guard<std::mutex> lk(em_); return err_; }
    std::atomic<bool> running_{false}, connected_{false};
    std::atomic<long long> bytes_{0};
    LineHandler handler_;
    std::thread th_;
    mutable std::mutex em_;
    std::string err_;
};

#ifdef _WIN32
inline void netInit() { static bool done = [] { WSADATA w; WSAStartup(MAKEWORD(2, 2), &w); return true; }(); (void)done; }
using SockT = SOCKET;
inline void sockClose(SockT s) { closesocket(s); }
constexpr SockT BAD_SOCK = INVALID_SOCKET;
#else
inline void netInit() {}
using SockT = int;
inline void sockClose(SockT s) { ::close(s); }
constexpr SockT BAD_SOCK = -1;
#endif

// ---------------------------------------------------------------- UDP
class UdpChannel : public Channel {
public:
    UdpChannel(int port, std::string bindAddr = "0.0.0.0") : port_(port), addr_(std::move(bindAddr)) {}
protected:
    void run() override {
        netInit();
        SockT s = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (s == BAD_SOCK) { setError("не удалось создать сокет UDP"); return; }
        int one = 1;
        ::setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof one);
        sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons(static_cast<uint16_t>(port_));
        if (::inet_pton(AF_INET, addr_.c_str(), &a.sin_addr) != 1) { setError("неверный адрес"); sockClose(s); return; }
        if (::bind(s, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0) { setError("не удалось занять порт UDP " + std::to_string(port_)); sockClose(s); return; }
        connected_ = true;
        LineAssembler la(handler_);
        char buf[2048];
        while (running_) {
#ifdef _WIN32
            fd_set fds; FD_ZERO(&fds); FD_SET(s, &fds); timeval tv{0, 200000};
            if (::select(0, &fds, nullptr, nullptr, &tv) <= 0) continue;
            int n = ::recv(s, buf, sizeof buf, 0);
#else
            pollfd p{s, POLLIN, 0};
            if (::poll(&p, 1, 200) <= 0) continue;
            ssize_t n = ::recv(s, buf, sizeof buf, 0);
#endif
            if (n > 0) { bytes_ += n; la.feed(buf, static_cast<size_t>(n)); la.feed("\n", 1); }   // датаграмма — целое предложение
        }
        connected_ = false;
        sockClose(s);
    }
private:
    int port_; std::string addr_;
};

// ---------------------------------------------------------------- TCP-клиент с переподключением
class TcpChannel : public Channel {
public:
    TcpChannel(std::string host, int port, double retryS = 2.0) : host_(std::move(host)), port_(port), retry_(retryS) {}
protected:
    void run() override {
        netInit();
        while (running_) {
            SockT s = connectOnce();
            if (s == BAD_SOCK) { sleepInterruptible(retry_); continue; }
            connected_ = true; setError("");
            LineAssembler la(handler_);
            char buf[2048];
            while (running_) {
#ifdef _WIN32
                fd_set fds; FD_ZERO(&fds); FD_SET(s, &fds); timeval tv{0, 200000};
                int r = ::select(0, &fds, nullptr, nullptr, &tv);
                if (r <= 0) continue;
                int n = ::recv(s, buf, sizeof buf, 0);
#else
                pollfd p{s, POLLIN, 0};
                int r = ::poll(&p, 1, 200);
                if (r <= 0) continue;
                ssize_t n = ::recv(s, buf, sizeof buf, 0);
#endif
                if (n <= 0) { setError("соединение закрыто"); break; }
                bytes_ += n; la.feed(buf, static_cast<size_t>(n));
            }
            connected_ = false;
            sockClose(s);
        }
    }
private:
    SockT connectOnce() {
        addrinfo hints{}, *res = nullptr;
        hints.ai_family = AF_INET; hints.ai_socktype = SOCK_STREAM;
        if (::getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res) != 0 || !res) { setError("адрес не найден: " + host_); return BAD_SOCK; }
        SockT s = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (s == BAD_SOCK) { ::freeaddrinfo(res); setError("не удалось создать сокет"); return BAD_SOCK; }
#ifdef _WIN32
        DWORD to = 3000; ::setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&to), sizeof to);
#else
        timeval tv{3, 0}; ::setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
#endif
        if (::connect(s, res->ai_addr, static_cast<int>(res->ai_addrlen)) != 0) { ::freeaddrinfo(res); sockClose(s); setError("нет соединения с " + host_ + ":" + std::to_string(port_)); return BAD_SOCK; }
        ::freeaddrinfo(res);
        return s;
    }
    void sleepInterruptible(double sec) {
        for (double t = 0; t < sec && running_; t += 0.1) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::string host_; int port_; double retry_;
};

// ---------------------------------------------------------------- последовательный порт
class SerialChannel : public Channel {
public:
    // device: "COM3" (Windows) или "/dev/cu.usbserial-XXXX" (macOS/Linux)
    SerialChannel(std::string device, int baud = 4800, double retryS = 2.0) : dev_(std::move(device)), baud_(baud), retry_(retryS) {}
protected:
#ifdef _WIN32
    void run() override {
        while (running_) {
            std::string name = dev_.rfind("COM", 0) == 0 ? "\\\\.\\" + dev_ : dev_;
            HANDLE h = CreateFileA(name.c_str(), GENERIC_READ, 0, nullptr, OPEN_EXISTING, 0, nullptr);
            if (h == INVALID_HANDLE_VALUE) { setError("не удалось открыть " + dev_); wait(); continue; }
            DCB dcb{}; dcb.DCBlength = sizeof dcb; GetCommState(h, &dcb);
            dcb.BaudRate = static_cast<DWORD>(baud_); dcb.ByteSize = 8; dcb.Parity = NOPARITY; dcb.StopBits = ONESTOPBIT;
            dcb.fBinary = TRUE; dcb.fOutxCtsFlow = FALSE; dcb.fOutxDsrFlow = FALSE; dcb.fOutX = FALSE; dcb.fInX = FALSE;
            dcb.fDtrControl = DTR_CONTROL_ENABLE; dcb.fRtsControl = RTS_CONTROL_ENABLE;
            SetCommState(h, &dcb);
            COMMTIMEOUTS to{}; to.ReadIntervalTimeout = 50; to.ReadTotalTimeoutConstant = 200; SetCommTimeouts(h, &to);
            connected_ = true; setError("");
            LineAssembler la(handler_);
            char buf[1024];
            while (running_) {
                DWORD n = 0;
                if (!ReadFile(h, buf, sizeof buf, &n, nullptr)) { setError("ошибка чтения порта"); break; }
                if (n > 0) { bytes_ += n; la.feed(buf, n); }
            }
            connected_ = false; CloseHandle(h);
        }
    }
#else
    void run() override {
        while (running_) {
            int fd = ::open(dev_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
            if (fd < 0) { setError("не удалось открыть " + dev_); wait(); continue; }
            termios t{};
            if (::tcgetattr(fd, &t) == 0) {
                cfmakeraw(&t);
                speed_t sp = speedOf(baud_);
                cfsetispeed(&t, sp); cfsetospeed(&t, sp);
                t.c_cflag |= (CLOCAL | CREAD); t.c_cflag &= ~(PARENB | CSTOPB | CSIZE); t.c_cflag |= CS8;
                t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
                ::tcsetattr(fd, TCSANOW, &t);
            }
            connected_ = true; setError("");
            LineAssembler la(handler_);
            char buf[1024];
            while (running_) {
                pollfd p{fd, POLLIN, 0};
                int r = ::poll(&p, 1, 200);
                if (r < 0) { setError("ошибка опроса порта"); break; }
                if (r == 0) continue;
                if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) { setError("порт отключён"); break; }
                ssize_t n = ::read(fd, buf, sizeof buf);
                if (n > 0) { bytes_ += n; la.feed(buf, static_cast<size_t>(n)); }
                else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EINTR)) { setError("порт отключён"); break; }
            }
            connected_ = false; ::close(fd);
        }
    }
    static speed_t speedOf(int b) {
        switch (b) {
            case 1200: return B1200; case 2400: return B2400; case 4800: return B4800; case 9600: return B9600;
            case 19200: return B19200; case 38400: return B38400; case 57600: return B57600; case 115200: return B115200;
            default: return B4800;
        }
    }
#endif
private:
    void wait() { for (double t = 0; t < retry_ && running_; t += 0.1) std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
    std::string dev_; int baud_; double retry_;
};

}  // namespace dp
