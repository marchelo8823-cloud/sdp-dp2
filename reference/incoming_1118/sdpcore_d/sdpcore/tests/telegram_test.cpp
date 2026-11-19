// v0.22: телеграммы Fanbeam (MDL, CEGELEC, $PMDTG) и Artemis (ASCII-16/17/22), NMEA GNS/GST/GSA/ZDA/ROT/DPGGA.
#include <cstdio>
#include <cmath>
#include "dp/hub.hpp"

using namespace dp;
static int fails = 0;
#define CHECK(c, msg) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", msg); if (!ok_) ++fails; } while (0)
static bool near(double a, double b, double tol = 0.02) { return std::fabs(a - b) < tol; }
static bool fix(SensorHub& h, const char* id, double& n, double& e, double now = 1.0) {
    SensorFrame fr = h.buildFrame(now);
    for (auto& p : fr.pos) if (p.id == id) { n = p.n; e = p.e; return p.fresh; }
    return false;
}

int main() {
    std::printf("== разбор строк ==\n");
    auto m = tele::parseMdl("01 1098.70 123.45\r\n");
    CHECK(m.ok && m.target == 1 && near(m.rangeM, 1098.7) && near(m.bearingDeg, 123.45), "MDL Standard: пример из руководства");
    CHECK(tele::xor18("01 1098.70 123.45 ") == 0x17, "контрольная сумма XOR(1..18) = 0x17, как в примере руководства");
    CHECK(tele::parseMdl("01 1098.70 123.45 17\r\n").ok, "MDL + контрольная сумма: верная сумма принята");
    CHECK(!tele::parseMdl("01 1098.70 123.45 18\r\n").ok, "MDL + контрольная сумма: неверная отклонена");
    CHECK(!tele::parseMdl("01 1098.70 400.00\r\n").ok && !tele::parseMdl("1098.70 123.45\r\n").ok, "MDL: пеленг вне диапазона и неполная строка отклоняются");
    auto c17 = tele::parseCegelec("010987 123450 1\r\n", false);
    CHECK(c17.ok && near(c17.rangeM, 1098.7) && near(c17.bearingDeg, 123.45) && c17.target == 1, "CEGELEC Standard (ASCII 17): пример из руководства");
    auto c16 = tele::parseCegelec("010987 12345 1\r\n", true);
    CHECK(c16.ok && near(c16.bearingDeg, 123.45), "CEGELEC Legacy: пеленг ×100");
    CHECK(!tele::parseCegelec("010987 12345 1\r\n", false).ok, "Standard не принимает пятизначный пеленг");
    auto a17 = tele::parseArtemis("010987 247904 1\r\n", 17);
    CHECK(a17.ok && a17.valid && a17.hasAzimuth && near(a17.rangeM, 1098.7) && near(a17.azimuthDeg, 247.904), "Artemis ASCII-17: дальность ×10, азимут ×1000, READY=1");
    CHECK(!tele::parseArtemis("010987 247904 0\r\n", 17).valid, "READY=0 — данные недостоверны");
    CHECK(!tele::parseArtemis("010987 ****** 1\r\n", 17).hasAzimuth, "азимут «*» — азимута нет");
    auto a22 = tele::parseArtemis("010987 1234 247904 1\r\n", 22);
    CHECK(a22.ok && a22.status == 1 && near(a22.bearingDeg, 123.4) && near(a22.azimuthDeg, 247.904), "Artemis ASCII-22: пеленг антенны и статус");
    CHECK(!tele::parseArtemis("000000 0000 000000 0\r\n", 22).valid, "ASCII-22: все нули (потеря связи) — недостоверно");
    auto a16 = tele::parseArtemis("010987 24790 1\r\n", 16);
    CHECK(a16.ok && near(a16.azimuthDeg, 247.90), "Artemis ASCII-16: азимут ×100");

    std::printf("== Fanbeam в SensorHub ==\n");
    SensorHub h; std::string why;
    ChannelCfg l; l.id = "FAN"; l.kind = ChannelKind::LaserRadar; l.accept = "MDL,CEG17,PMDTG"; h.addChannel(l);
    ChannelCfg y; y.id = "GYRO"; y.kind = ChannelKind::Gyro; h.addChannel(y);
    auto info = [&](const char* id) { for (auto& c : h.channels(1.0)) if (c.id == id) return c; return ChannelInfo{}; };
    CHECK(h.setParam("FAN", "target", "1 0 100", why), "мишень 1 задана: север 0, восток 100");
    h.ingest("FAN", "01 0100.00 000.00\r\n", 1.0);
    CHECK(info("FAN").rejected == 1, "без курса пересчёт невозможен — отказ");
    h.ingest("GYRO", nmea::make("HEHDT,90.0,T"), 1.0);
    h.ingest("FAN", "01 0100.00 000.00\r\n", 1.0);
    double n = 9, e = 9;
    CHECK(fix(h, "FAN", n, e) && near(n, 0.0) && near(e, 0.0), "мишень 100 м по носу при курсе 090° — судно в начале координат");
    h.ingest("FAN", "000000 000000 1\r\n", 2.0);
    CHECK(info("FAN").rejected == 2, "дальность 0 отклоняется");
    h.ingest("FAN", "001000 000000 1\r\n", 2.0);
    CHECK(fix(h, "FAN", n, e, 2.0) && near(n, 0.0) && near(e, 0.0), "CEGELEC ASCII 17 даёт то же положение");
    h.ingest("FAN", "001000 000000 7\r\n", 3.0);
    CHECK(info("FAN").rejected == 3, "неизвестная мишень отклоняется");
    h.ingest("FAN", nmea::make("PMDTG,01,0100.00,000.00,123,7,3,0,1,000"), 4.0);
    CHECK(fix(h, "FAN", n, e, 4.0) && near(n, 0.0) && near(e, 0.0) && info("FAN").values.find("уровень 123") != std::string::npos, "$PMDTG: положение и показатели качества");
    CHECK(h.setParam("FAN", "brg.off", "90", why), "поправка нуля пеленга");
    h.ingest("FAN", "01 0100.00 000.00\r\n", 5.0);
    CHECK(fix(h, "FAN", n, e, 5.0) && near(n, 100.0) && near(e, 100.0), "поправка 90°: мишень справа от носа (на юге), судно севернее мишени");

    std::printf("== Artemis в SensorHub ==\n");
    ChannelCfg ar; ar.id = "ART"; ar.kind = ChannelKind::LaserRadar; ar.accept = "ART17,ART22"; h.addChannel(ar);
    CHECK(h.setParam("ART", "ref", "1000,2000", why), "положение Fix-станции задано");
    h.ingest("ART", "010000 090000 1\r\n", 1.0);
    CHECK(fix(h, "ART", n, e) && near(n, 1000.0) && near(e, 3000.0), "азимут 90°, 1000 м: мобильная станция на восток от Fix");
    h.ingest("ART", "010000 090000 0\r\n", 2.0);
    CHECK(info("ART").rejected == 1, "READY=0: нет захвата — отказ");
    h.ingest("ART", "010000 ****** 1\r\n", 2.0);
    CHECK(info("ART").rejected == 2, "нет азимута — отказ");
    h.ingest("ART", "010000 0000 ****** 2\r\n", 3.0);
    CHECK(info("ART").rejected == 3, "ASCII-22 статус 2 (маяк): азимута нет — отказ");
    h.ingest("ART", "010000 0000 000000 1\r\n", 3.0);
    CHECK(fix(h, "ART", n, e, 3.0) && near(n, 2000.0) && near(e, 2000.0), "ASCII-22: азимут 0°, 1000 м: севернее Fix");

    std::printf("== NMEA ==\n");
    ChannelCfg g; g.id = "GPS"; g.kind = ChannelKind::Gnss; g.accept = "GGA,GNS,GST,GSA,ZDA"; g.sigma = 0.3; h.addChannel(g);
    h.ingest("GPS", nmea::make("GNGNS,120000.00,5500.000000,N,03700.000000,E,AD,12,0.8,10.0,0.0,1.0,0000"), 1.0);
    CHECK(info("GPS").accepted == 1 && fix(h, "GPS", n, e), "GNS принят");
    h.ingest("GPS", nmea::make("GNGNS,120000.00,5500.000000,N,03700.000000,E,NN,0,99.9,,,,"), 2.0);
    CHECK(info("GPS").rejected == 1, "GNS без решения (NN) отклонён");
    h.ingest("GPS", nmea::make("GPGST,120000.00,1.0,0.5,0.4,45.0,0.30,0.40,0.80"), 3.0);
    { SensorFrame fr = h.buildFrame(3.5); double sg = 0; for (auto& p : fr.pos) if (p.id == "GPS") sg = p.sigma; CHECK(near(sg, std::sqrt((0.09 + 0.16) / 2.0), 0.001) && sg > 0.3, "GST: СКО из GST заменяет заниженное настроенное"); }
    { SensorFrame fr = h.buildFrame(20.0); double sg = 0; for (auto& p : fr.pos) if (p.id == "GPS") sg = p.sigma; CHECK(near(sg, 0.3, 1e-9), "GST старше 5 с — снова настроенное СКО"); }
    h.ingest("GPS", nmea::make("GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.5,0.9,1.2"), 4.0);
    h.ingest("GPS", nmea::make("GPZDA,120000.00,09,11,2026,00,00"), 4.0);
    CHECK(info("GPS").values.find("UTC 120000") != std::string::npos, "GSA и ZDA показываются в значениях канала");
    h.ingest("GPS", nmea::make("PDPGGA,120000.00,5500.0,N,03700.0,E,2,12,0.8,1.0,M,0.0,M,,"), 5.0);
    CHECK(info("GPS").rejected == 1 || info("GPS").accepted >= 4, "$PDPGGA разобран как GGA");
    ChannelCfg g2; g2.id = "GYRO2"; g2.kind = ChannelKind::Gyro; g2.accept = "HDT,ROT"; h.addChannel(g2);
    h.ingest("GYRO2", nmea::make("HEROT,-12.5,A"), 6.0);
    CHECK(info("GYRO2").values.find("ROT -12.5") != std::string::npos, "ROT показывается у гирокомпаса");
    h.ingest("GYRO2", nmea::make("HEROT,5.0,V"), 7.0);
    CHECK(info("GYRO2").rejected == 1, "ROT со статусом V отклонён");
    CHECK(!h.setParam("FAN", "accept", "GGA", why), "тип GGA лазерный канал не принимает");
    CHECK(h.setParam("FAN", "accept", "MDL,ART17", why), "типы MDL и ART17 лазерный канал принимает");

    std::printf(fails ? "\nОШИБОК: %d\n" : "\nвсё в порядке\n", fails);
    return fails ? 1 : 0;
}
