// Проверка блока приёма: разбор NMEA, сопоставление датчикам, транспорты UDP/TCP/COM и замкнутый контур
// «имитатор → строки NMEA → приёмник → движок → движители».
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <thread>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "dp/hub.hpp"
#include "dp/io.hpp"
#include "dp/sim.hpp"

using namespace dp;
static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { std::printf("  [ПРОВАЛ] %s\n", msg); ++fails; } else std::printf("  [ок] %s\n", msg); } while (0)

static void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

static void testParser() {
    std::printf("Разбор NMEA\n");
    const std::string gga = nmea::make("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    auto s = nmea::parse(gga);
    CHECK(s && s->type == "GGA" && s->talker == "GP" && s->f.size() >= 8, "GGA разобрано");
    // эталон из описания NMEA: контрольная сумма известного предложения
    auto s2 = nmea::parse("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
    CHECK(s2.has_value(), "контрольная сумма эталонного GGA верна (*47)");
    auto bad = nmea::parse("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48");
    CHECK(!bad.has_value(), "неверная контрольная сумма отвергается");
    CHECK(!nmea::parse("$GPGGA,1,2,3").has_value(), "без контрольной суммы отвергается по умолчанию");
    double lat = 0, lon = 0;
    nmea::parseAngle("4807.038", "N", true, lat); nmea::parseAngle("01131.000", "E", false, lon);
    CHECK(std::fabs(lat - 48.1173) < 1e-4 && std::fabs(lon - 11.516667) < 1e-5, "широта и долгота");
    CHECK(!nmea::parseAngle("9961.0", "N", true, lat), "недопустимые минуты отвергаются");
    auto p = nmea::parse(nmea::make("PSXN,23,1.5,-0.5,90.0,0.3"));
    CHECK(p && p->proprietary && p->type == "SXN", "проприетарное предложение");
    LineAssembler* dummy = nullptr; (void)dummy;
    std::vector<std::string> got; LineAssembler la([&](const std::string& l) { got.push_back(l); });
    const char* part1 = "$GPHDT,10.0,T*"; const char* part2 = "00\r\n$GPHDT,2"; const char* part3 = "0.0,T*0A\r\n";
    la.feed(part1, strlen(part1)); la.feed(part2, strlen(part2)); la.feed(part3, strlen(part3));
    CHECK(got.size() == 2, "сборка строк из фрагментов потока");
}

static void testHub() {
    std::printf("Сопоставление датчикам\n");
    SensorHub hub;
    ChannelCfg g; g.id = "GNSS1"; g.kind = ChannelKind::Gnss; g.offset = {20, 3, -15}; g.sigma = 0.5; g.minGnssQuality = 1;
    hub.addChannel(g);
    ChannelCfg gy; gy.id = "GYRO1"; gy.kind = ChannelKind::Gyro; hub.addChannel(gy);
    ChannelCfg w; w.id = "WIND1"; w.kind = ChannelKind::Wind; hub.addChannel(w);
    ChannelCfg m; m.id = "MRU"; m.kind = ChannelKind::Mru; hub.addChannel(m);
    hub.setOrigin(60.0, 30.0);
    hub.ingest("GNSS1", nmea::make("GPGGA,000000,6000.0060,N,03000.0000,E,1,08,0.9,0,M,0,M,,"), 1.0);
    hub.ingest("GYRO1", nmea::make("HEHDT,45.5,T"), 1.0);
    hub.ingest("WIND1", nmea::make("WIMWV,30.0,R,10.0,M,A"), 1.0);
    hub.ingest("MRU", nmea::make("PSXN,23,2.0,-1.0,0.0,0.4"), 1.0);
    auto fr = hub.buildFrame(1.1);
    CHECK(fr.pos.size() == 1 && fr.pos[0].fresh && std::fabs(fr.pos[0].n - 11.12) < 0.1 && std::fabs(fr.pos[0].e) < 0.01, "положение: 0,006' широты ≈ 11,1 м к северу");
    CHECK(fr.hdg.size() == 1 && fr.hdg[0].fresh && std::fabs(rad2deg(fr.hdg[0].psi) - 45.5) < 1e-6, "курс HDT");
    CHECK(fr.wind.size() == 1 && std::fabs(fr.wind[0].speed - 10.0) < 1e-9 && std::fabs(rad2deg(fr.wind[0].fromRel) - 30.0) < 1e-6, "кажущийся ветер MWV");
    CHECK(fr.mru.size() == 1 && fr.mru[0].fresh && std::fabs(rad2deg(fr.mru[0].roll) - 2.0) < 1e-6 && std::fabs(fr.mru[0].heave - 0.4) < 1e-9, "крен/дифферент/вертикальная качка PSXN,23");
    auto fr2 = hub.buildFrame(1.2);
    CHECK(!fr2.pos[0].fresh && !fr2.hdg[0].fresh, "признак «свежее» выставляется один раз");
    hub.ingest("GNSS1", nmea::make("GPGGA,000000,6000.0060,N,03000.0000,E,0,00,99.9,0,M,0,M,,"), 2.0);
    hub.ingest("WIND1", nmea::make("WIMWV,30.0,T,10.0,M,A"), 2.0);
    hub.ingest("GYRO1", nmea::make("HETHS,45.5,V"), 2.0);
    auto fr3 = hub.buildFrame(2.1);
    CHECK(!fr3.pos[0].fresh && !fr3.wind[0].fresh && !fr3.hdg[0].fresh, "решение без фиксации, истинный ветер и недостоверный курс отбрасываются");
    long long rej = 0; for (auto& s : hub.stats()) rej += s.rejected;
    CHECK(rej == 3, "отбракованные сообщения учитываются в статистике");
}

// --- транспорты
static void testUdpTcp() {
    std::printf("Транспорты UDP и TCP\n");
    std::vector<std::string> gotU; std::mutex mu;
    UdpChannel u(34567, "127.0.0.1");
    u.start([&](const std::string& l) { std::lock_guard<std::mutex> lk(mu); gotU.push_back(l); });
    sleepMs(300);
    int s = socket(AF_INET, SOCK_DGRAM, 0); sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons(34567); inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
    std::string m = nmea::make("HEHDT,10.0,T");
    sendto(s, m.c_str(), m.size(), 0, reinterpret_cast<sockaddr*>(&a), sizeof a);
    sendto(s, "$HEHDT,11.0,T*XX", 17, 0, reinterpret_cast<sockaddr*>(&a), sizeof a);   // датаграмма без перевода строки
    close(s); sleepMs(400);
    { std::lock_guard<std::mutex> lk(mu); CHECK(gotU.size() == 2, "UDP: получены две датаграммы"); }
    u.stop();

    // TCP: сервер на 34568, канал подключается и принимает; затем сервер перезапускается — канал переподключается
    std::vector<std::string> gotT;
    auto serve = [&](const char* text) {
        int ls = socket(AF_INET, SOCK_STREAM, 0); int one = 1; setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        sockaddr_in b{}; b.sin_family = AF_INET; b.sin_port = htons(34568); inet_pton(AF_INET, "127.0.0.1", &b.sin_addr);
        bind(ls, reinterpret_cast<sockaddr*>(&b), sizeof b); listen(ls, 1);
        int c = accept(ls, nullptr, nullptr);
        send(c, text, strlen(text), 0); sleepMs(300); close(c); close(ls);
    };
    TcpChannel t("127.0.0.1", 34568, 0.3);
    std::thread srv1([&] { serve("$HEHDT,20.0,T*0A\r\n$HEHDT,21"); });
    sleepMs(100);
    t.start([&](const std::string& l) { std::lock_guard<std::mutex> lk(mu); gotT.push_back(l); });
    srv1.join(); sleepMs(100);
    std::thread srv2([&] { serve(".0,T*0B\r\n"); });
    srv2.join(); sleepMs(500);
    t.stop();
    { std::lock_guard<std::mutex> lk(mu); std::printf("  принято строк TCP: %zu\n", gotT.size()); for (auto& l : gotT) std::printf("    [%s]\n", l.c_str()); CHECK(gotT.size() == 2 && gotT[1] == ".0,T*0B", "TCP: после обрыва канал переподключился, неполная строка отброшена"); }
}

static void testSerialPty() {
    std::printf("Последовательный порт (виртуальная пара)\n");
    int master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || grantpt(master) != 0 || unlockpt(master) != 0) { std::printf("  [пропуск] нет pty\n"); return; }
    std::string slave = ptsname(master);
    std::vector<std::string> got; std::mutex mu;
    SerialChannel sc(slave, 4800, 0.3);
    sc.start([&](const std::string& l) { std::lock_guard<std::mutex> lk(mu); got.push_back(l); });
    sleepMs(300);
    std::string m = nmea::make("HEHDT,33.0,T") + nmea::make("HEHDT,34.0,T");
    for (size_t i = 0; i < m.size(); i += 7) { (void)!write(master, m.data() + i, std::min<size_t>(7, m.size() - i)); sleepMs(20); }
    sleepMs(400);
    { std::lock_guard<std::mutex> lk(mu); CHECK(got.size() == 2 && nmea::parse(got[0]).has_value(), "COM: строки собраны из фрагментов, суммы верны"); }
    sc.stop(); close(master);
}

// --- замкнутый контур через NMEA
static std::vector<std::string> encode(const Simulator& sim, const LocalFrame& lf, const SensorFrame& fr) {
    std::vector<std::string> out;
    for (auto& p : fr.pos) if (p.fresh) {
        if (p.id.rfind("GNSS", 0) == 0) {
            double la, lo; lf.toGeo(p.n, p.e, la, lo);
            out.push_back(p.id + "|" + nmea::make("GPGGA,000000," + nmea::fmtLat(la) + "," + nmea::fmtLon(lo) + ",2,10,0.8,0,M,0,M,,"));
        } else {
            char b[96]; std::snprintf(b, sizeof b, "PDPREL,%.3f,%.3f", p.n, p.e);
            out.push_back(p.id + "|" + nmea::make(b));
        }
    }
    for (auto& h : fr.hdg) if (h.fresh) { char b[64]; std::snprintf(b, sizeof b, "HEHDT,%.2f,T", std::fmod(rad2deg(h.psi) + 360.0, 360.0)); out.push_back(h.id + "|" + nmea::make(b)); }
    for (auto& w : fr.wind) if (w.fresh) { char b[64]; std::snprintf(b, sizeof b, "WIMWV,%.1f,R,%.2f,M,A", std::fmod(rad2deg(w.fromRel) + 360.0, 360.0), w.speed); out.push_back(w.id + "|" + nmea::make(b)); }
    for (auto& m : fr.mru) if (m.fresh) { char b[96]; std::snprintf(b, sizeof b, "PSXN,23,%.3f,%.3f,0.0,%.3f", rad2deg(m.roll), rad2deg(m.pitch), m.heave); out.push_back(m.id + "|" + nmea::make(b)); }
    (void)sim;
    return out;
}

static void testLoop() {
    std::printf("Замкнутый контур через NMEA\n");
    EnvCfg env; env.windSpeed = 10; env.windFromDeg = 60; env.currentSpeed = 0.5; env.currentToDeg = 200; env.hs = 1.5; env.tp = 7; env.waveFromDeg = 60;
    VesselCfg v = demoVessel(); auto th = demoThrusters();
    SimSensorsCfg sc; sc.gnss[0].sigma = sc.gnss[1].sigma = sc.gnss[2].sigma = 0.5;
    Simulator sim(v, th, env, sc);
    DpEngine eng(v, th);
    SensorHub hub;
    for (auto& g : sc.gnss) { ChannelCfg c; c.id = g.id; c.kind = ChannelKind::Gnss; c.offset = g.offset; c.sigma = g.sigma; hub.addChannel(c); }
    { ChannelCfg c; c.id = "LASER"; c.kind = ChannelKind::LaserRadar; c.offset = {22, 0, -8}; c.sigma = sc.laserSigma; hub.addChannel(c); }
    { ChannelCfg c; c.id = "WIRE"; c.kind = ChannelKind::TautWire; c.offset = {25, 0, 2}; c.sigma = sc.wireSigma; hub.addChannel(c); }
    for (int i = 1; i <= 3; ++i) { ChannelCfg c; c.id = "GYRO" + std::to_string(i); c.kind = ChannelKind::Gyro; hub.addChannel(c); }
    for (int i = 1; i <= 3; ++i) { ChannelCfg c; c.id = "WIND" + std::to_string(i); c.kind = ChannelKind::Wind; hub.addChannel(c); }
    for (int i = 1; i <= 3; ++i) { ChannelCfg c; c.id = "MRU" + std::to_string(i); c.kind = ChannelKind::Mru; hub.addChannel(c); }
    LocalFrame lf; lf.setOrigin(59.9, 30.3); hub.setOrigin(59.9, 30.3);
    const double dt = 0.1;
    double t = 0, maxErr = 0;
    auto cycle = [&]() {
        sim.step(dt); t += dt;
        SensorFrame truthFr = sim.measure();
        for (auto& l : encode(sim, lf, truthFr)) { const size_t bar = l.find('|'); hub.ingest(l.substr(0, bar), l.substr(bar + 1), t); }
        hub.setPower(1e9);
        for (auto& f : truthFr.fb) hub.setThrusterFb(f);
        auto fr = hub.buildFrame(t);
        auto o = eng.step(dt, fr);
        sim.applyCommands(o.cmds);
        return o;
    };
    for (int i = 0; i < 100; ++i) cycle();
    eng.setMode(DpMode::AutoPosition);
    eng.controller().setSetpoint({sim.truth().n, sim.truth().e, rad2deg(sim.truth().psi)});
    for (int i = 0; i < 9000; ++i) {
        auto o = cycle();
        if (t > 400) maxErr = std::max(maxErr, std::hypot(sim.truth().n - o.ctl.refN, sim.truth().e - o.ctl.refE));
    }
    std::printf("  максимальное отклонение при работе через NMEA: %.2f м\n", maxErr);
    CHECK(maxErr < 1.0, "удержание позиции по данным, пришедшим как строки NMEA");
}

int main() {
    testParser();
    testHub();
    testUdpTcp();
    testSerialPty();
    testLoop();
    std::printf(fails ? "ИТОГ: есть провалы (%d)\n" : "ИТОГ: всё в порядке\n", fails);
    return fails ? 1 : 0;
}
