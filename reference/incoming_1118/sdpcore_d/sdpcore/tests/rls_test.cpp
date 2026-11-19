// Лазерный радар CyScan: посылка $RLS → положение судна по известному положению отражателя и курсу гирокомпаса.
#include <cstdio>
#include <cmath>
#include "dp/hub.hpp"

using namespace dp;
static int fails = 0;
#define CHECK(c, msg) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", msg); if (!ok_) ++fails; } while (0)

static bool near(double a, double b, double tol = 0.02) { return std::fabs(a - b) < tol; }
static bool fix(SensorHub& h, const char* id, double& n, double& e) {
    SensorFrame fr = h.buildFrame(1.0);
    for (auto& p : fr.pos) if (p.id == id) { n = p.n; e = p.e; return p.fresh; }
    return false;
}

int main() {
    SensorHub h;
    ChannelCfg l; l.id = "RLS1"; l.kind = ChannelKind::LaserRadar; h.addChannel(l);
    ChannelCfg y; y.id = "GYRO"; y.kind = ChannelKind::Gyro; y.accept = ""; h.addChannel(y);
    std::string why;
    const std::string L1 = nmea::make("RLS,+100.00,+000.00,A,000.000,V,900F");
    const std::string LV = nmea::make("RLS,+000.00,+000.00,V,000.000,V,0002");
    const std::string H090 = nmea::make("HEHDT,90.0,T");
    const std::string H000 = nmea::make("HEHDT,0.0,T");
    auto info = [&](const char* id) { for (auto& c : h.channels(1.0)) if (c.id == id) return c; return ChannelInfo{}; };

    CHECK(h.setParam("RLS1", "ref", "0,100", why), "положение начала осей (главной мишени) задано");
    h.ingest("RLS1", L1, 1.0);
    CHECK(info("RLS1").rejected == 1, "режим «судно»: без курса пересчёт невозможен — отказ");
    h.ingest("GYRO", H090, 1.0);
    double n = 9, e = 9;
    h.ingest("RLS1", nmea::make("RLS,-100.00,+000.00,A,000.000,V,900F"), 1.0);
    CHECK(fix(h, "RLS1", n, e) && near(n, 0.0) && near(e, 0.0), "режим «судно», курс 090°: положение = начало + R(курс)·(A,B)");
    h.ingest("GYRO", H000, 2.0); h.ingest("RLS1", nmea::make("RLS,+010.00,+005.00,A,000.000,V,900F"), 2.0);
    CHECK(fix(h, "RLS1", n, e) && near(n, 10.0) && near(e, 105.0), "курс 000°: A вперёд, B вправо от начала");
    const long long acc = info("RLS1").accepted;
    h.ingest("RLS1", LV, 3.0);
    CHECK(info("RLS1").accepted == acc && info("RLS1").rejected == 2, "статус V — посылка отклонена");
    // неподвижная платформа: A — север, B — восток, курс не нужен
    CHECK(h.setParam("RLS1", "rls.mode", "ne", why) && h.setParam("RLS1", "ref", "0,0", why), "режим «ne»");
    h.ingest("RLS1", nmea::make("RLS,+100.00,-020.00,A,123.000,A,C000"), 4.0);
    CHECK(fix(h, "RLS1", n, e) && near(n, 100.0) && near(e, -20.0), "режим «ne»: A — север, B — восток");
    // оси, привязанные к конструкции: A по азимуту 90°
    CHECK(h.setParam("RLS1", "rls.mode", "frame", why) && h.setParam("RLS1", "rls.az", "90", why), "режим «frame», A по азимуту 90°");
    h.ingest("RLS1", nmea::make("RLS,+010.00,+005.00,A,000.000,V,900F"), 5.0);
    CHECK(fix(h, "RLS1", n, e) && near(n, -5.0) && near(e, 10.0), "режим «frame»: A на восток, B на юг (90° по часовой от A)");
    CHECK(h.setParam("RLS1", "rls.swap", "1", why), "поменять оси");
    h.ingest("RLS1", nmea::make("RLS,+010.00,+005.00,A,000.000,V,900F"), 6.0);
    CHECK(fix(h, "RLS1", n, e) && near(n, -10.0) && near(e, 5.0), "после swap A и B меняются местами");
    CHECK(h.setParam("RLS1", "rls.swap", "0", why) && h.setParam("RLS1", "rls.signa", "-1", why) && h.setParam("RLS1", "rls.signa", "1", why), "знак оси A");
    CHECK(!h.setParam("RLS1", "ref", "abc", why) && !h.setParam("RLS1", "rls.signa", "2", why) && !h.setParam("RLS1", "rls.mode", "x", why), "неверные значения отклоняются");

    // $PGNMT: дальность и пеленг до мишени, положение мишени задано
    std::printf("== PGNMT ==\n");
    CHECK(h.setParam("RLS1", "target", "3 0 100", why) && h.setParam("RLS1", "target", "4 50 0", why), "мишени 3 и 4 заданы");
    CHECK(!h.setParam("RLS1", "target", "x", why) && !h.setParam("RLS1", "target", "70000 1 1", why), "неверная мишень отклоняется");
    h.ingest("GYRO", H090, 7.0);
    h.ingest("RLS1", nmea::make("PGNMT,R,12 00 00.000,100,1,3,N,100.00,0.05,0.00,0.10,9,120,0000"), 7.0);
    CHECK(fix(h, "RLS1", n, e) && near(n, 0.0) && near(e, 0.0), "мишень 100 м по носу при курсе 090° — судно в начале координат");
    SensorFrame fr = h.buildFrame(7.5); double sg = 0; for (auto& p : fr.pos) if (p.id == "RLS1") sg = p.sigma;
    h.ingest("RLS1", nmea::make("PGNMT,R,12 00 00.000,101,1,3,N,100.00,0.50,0.00,0.20,9,120,0000"), 8.0);
    fr = h.buildFrame(8.5); for (auto& p : fr.pos) if (p.id == "RLS1") sg = p.sigma;
    CHECK(sg > 0.55 && sg < 0.67, "СКО измерения берётся из посылки (дальность 0,5 м, пеленг 0,2° на 100 м → ≈0,61 м)");
    CHECK(info("RLS1").values.find("мишень 3") != std::string::npos && info("RLS1").values.find("120 мс") != std::string::npos, "качество и возраст показаны");
    h.ingest("GYRO", H000, 9.0);
    h.ingest("RLS1", nmea::make("PGNMT,R,12 00 00.000,102,1,3,A,50.00,0.05,90.00,0.10,9,100,0000"), 9.0);
    CHECK(fix(h, "RLS1", n, e) && near(n, 0.0) && near(e, 50.0), "мишень справа в 50 м при курсе 000° — судно на 50 м западнее мишени");
    const long long r0 = info("RLS1").rejected;
    h.ingest("RLS1", nmea::make("PGNMT,R,12 00 00.000,103,1,3,V,50.00,0.05,90.00,0.10,9,100,0000"), 10.0);
    h.ingest("RLS1", nmea::make("PGNMT,R,12 00 00.000,104,1,3,A,50.00,0.05,90.00,0.10,9,3000,0000"), 10.0);
    h.ingest("RLS1", nmea::make("PGNMT,R,12 00 00.000,105,1,9,A,50.00,0.05,90.00,0.10,9,100,0000"), 10.0);
    CHECK(info("RLS1").rejected == r0 + 3, "статус V, возраст более 2 с и неизвестная мишень отклоняются");
    CHECK(h.setParam("RLS1", "accept", "RLS,PGNMT", why), "accept PGNMT допустим");
    std::printf(fails ? "ПРОВАЛ: %d\n" : "Все проверки пройдены\n", fails);
    return fails ? 1 : 0;
}
