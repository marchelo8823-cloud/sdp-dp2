// Настройка посылок датчиков: типы предложений, talker ID, проверки, новые разборщики VTG, HDM, VHW.
#include <cstdio>
#include <cmath>
#include "dp/hub.hpp"

using namespace dp;
static int fails = 0;
#define CHECK(c, msg) do { const bool ok_ = (c); std::printf("%s %s\n", ok_ ? "  ok " : "FAIL ", msg); if (!ok_) ++fails; } while (0)

static const ChannelInfo* find(const std::vector<ChannelInfo>& v, const std::string& id) { for (auto& c : v) if (c.id == id) return &c; return nullptr; }

int main() {
    std::printf("== Настройка посылок ==\n");
    SensorHub h; h.setOrigin(60.0, 30.0);
    ChannelCfg g; g.id = "G"; g.kind = ChannelKind::Gnss; h.addChannel(g);
    ChannelCfg y; y.id = "Y"; y.kind = ChannelKind::Gyro; h.addChannel(y);
    std::string why;
    const std::string GGA = nmea::make("GPGGA,000000,6000.0000,N,03000.0000,E,1,10,0.8,0,M,0,M,,");
    const std::string RMC = nmea::make("GPRMC,000000,A,6000.0010,N,03000.0000,E,0.1,10.0,010126,,");
    const std::string GGA_GN = nmea::make("GNGGA,000000,6000.0000,N,03000.0000,E,1,10,0.8,0,M,0,M,,");
    const std::string VTG = nmea::make("GPVTG,45.0,T,,M,3.5,N,6.5,K,A");
    auto info = [&](const char* id) { return *find(h.channels(1.0), id); };

    h.ingest("G", GGA, 1.0); h.ingest("G", RMC, 1.0); h.ingest("G", VTG, 1.0);
    CHECK(info("G").accepted == 2 && info("G").ignored == 1, "по умолчанию: GGA и RMC принимаются, VTG игнорируется");
    CHECK(info("G").accept == "GGA,GLL,RMC", "по умолчанию — прежний набор");

    CHECK(h.setParam("G", "accept", "gga,vtg", why), "accept GGA,VTG задан");
    h.ingest("G", GGA, 2.0); h.ingest("G", RMC, 2.0); h.ingest("G", VTG, 2.0);
    CHECK(info("G").accepted == 4 && info("G").ignored == 2, "теперь RMC игнорируется, VTG принимается");
    CHECK(info("G").values.find("3.5 уз") != std::string::npos && info("G").values.find("45°") != std::string::npos, "VTG: скорость и курс над грунтом показаны");
    CHECK(!h.setParam("G", "accept", "GGA,HDT", why) && !why.empty(), "чужое для GNSS предложение отклоняется");

    CHECK(h.setParam("G", "talkers", "GP", why), "talkers GP задан");
    h.ingest("G", GGA_GN, 3.0);
    CHECK(info("G").ignored == 3, "talker GN не пропущен фильтром");
    CHECK(h.setParam("G", "talkers", "any", why), "talkers any");
    h.ingest("G", GGA_GN, 4.0);
    CHECK(info("G").accepted == 5, "talker GN принят после снятия фильтра");

    CHECK(h.setParam("G", "check", "off", why), "проверка контрольной суммы выключена");
    h.ingest("G", "$GPGGA,000000,6000.0000,N,03000.0000,E,1,10,0.8,0,M,0,M,,", 5.0);
    CHECK(info("G").accepted == 6, "строка без контрольной суммы принята");
    CHECK(h.setParam("G", "check", "on", why), "проверка включена");
    h.ingest("G", "$GPGGA,000000,6000.0000,N,03000.0000,E,1,10,0.8,0,M,0,M,,", 6.0);
    CHECK(info("G").rejected >= 1, "без контрольной суммы теперь отклоняется");
    CHECK(h.setParam("G", "quality", "2", why), "минимальное качество 2");
    const long long a0 = info("G").accepted; h.ingest("G", GGA, 7.0);
    CHECK(info("G").accepted == a0, "GGA с качеством 1 отклонена при минимуме 2");

    // гирокомпас
    const std::string HDT = nmea::make("HEHDT,100.0,T"), HDM = nmea::make("HEHDM,90.0,M"), VHW = nmea::make("HEVHW,,T,95.0,M,5.0,N,9.3,K");
    h.ingest("Y", HDM, 8.0);
    CHECK(info("Y").accepted == 0 && info("Y").ignored == 1, "HDM по умолчанию не принимается");
    CHECK(h.setParam("Y", "accept", "HDM,VHW", why), "accept HDM,VHW");
    CHECK(h.setParam("Y", "magvar", "10", why), "склонение 10°");
    h.ingest("Y", HDM, 9.0);
    SensorFrame fr = h.buildFrame(9.0);
    CHECK(!fr.hdg.empty() && fr.hdg[0].fresh && std::fabs(rad2deg(fr.hdg[0].psi) - 100.0) < 0.01, "HDM 90° + склонение 10° = 100°");
    h.ingest("Y", VHW, 10.0); fr = h.buildFrame(10.0);
    CHECK(!fr.hdg.empty() && std::fabs(rad2deg(fr.hdg[0].psi) - 105.0) < 0.01, "VHW: магнитный курс 95° + склонение = 105°");
    h.ingest("Y", HDT, 11.0);
    CHECK(info("Y").ignored == 2, "HDT не принимается, когда его нет в списке");
    std::printf(fails ? "ИТОГ: ошибок %d\n" : "ИТОГ: всё в порядке\n", fails);
    return fails ? 1 : 0;
}
