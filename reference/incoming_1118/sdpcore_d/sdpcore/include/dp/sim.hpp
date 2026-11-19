// Имитатор: «истинное» судно, движители и датчики с шумами и отказами. Служит для проверки ядра.
#pragma once
#include "dp/engine.hpp"

#include <array>
#include <set>

namespace dp {

struct SimSensorsCfg {
    struct Gnss { std::string id; V3 offset; double sigma = 0.5; double rateHz = 1.0; };
    std::vector<Gnss> gnss = {{"GNSS1", {20, 3, -15}, 0.5, 1.0}, {"GNSS2", {-18, -3, -15}, 0.6, 1.0}, {"GNSS3", {0, 5, -15}, 0.8, 1.0}};
    bool laser = true;  double laserSigma = 0.3, laserRate = 5.0;
    bool wire = true;   double wireSigma = 0.15, wireRate = 5.0;
    int gyros = 3;      double gyroSigmaDeg = 0.1, gyroRate = 10.0;
    int anemometers = 3; double windSigma = 0.3, windRate = 1.0;
    int mrus = 3;       double mruAngSigmaDeg = 0.03, mruHeaveSigma = 0.03, mruRate = 10.0;
    double thrusterFbRate = 10.0;
    double availableKW = 1e9;
};

class Simulator {
public:
    Simulator(const VesselCfg& v, const std::vector<ThrusterCfg>& th, const EnvCfg& env, SimSensorsCfg sc = {}, unsigned seed = 11)
        : vcfg_(v), dyn_(v, env), target_(env), sc_(std::move(sc)), rng_(seed) {
        for (const auto& t : th) ths_.emplace_back(t);
        gyroBias_.resize(static_cast<size_t>(std::max(0, sc_.gyros)));
        for (auto& b : gyroBias_) b = deg2rad(rng_.normal(0.0, 0.05));
        mruBias_.resize(static_cast<size_t>(std::max(0, sc_.mrus)));
        for (auto& b : mruBias_) { const double r = deg2rad(rng_.normal(0.0, 0.02)); b = {r, deg2rad(rng_.normal(0.0, 0.02))}; }
        nextT_.assign(32, 0.0);
    }

    VesselDynamics& dyn() { return dyn_; }
    // Заданная оператором среда. Фактическая (dyn().env()) приближается к ней с ограниченной скоростью,
    // как в природе: ветер меняется порядка 0,4 м/с за секунду, течение и волны — медленнее.
    EnvCfg& target() { return target_; }
    const VesselState& truth() const { return dyn_.state(); }
    std::vector<ThrusterSim>& thrusters() { return ths_; }
    double time() const { return t_; }
    SimSensorsCfg& sensors() { return sc_; }

    // Отказы: датчики, помеченные идентификатором, перестают выдавать свежие данные.
    void failSensor(const std::string& id, bool failed) { if (failed) failed_.insert(id); else failed_.erase(id); }
    // Скачок показаний датчика (м): имитация сбоя GNSS.
    void jumpSensor(const std::string& id, double dn, double de) { jump_[id] = {dn, de}; }
    void clearJump(const std::string& id) { jump_.erase(id); }
    void freezeSensor(const std::string& id, bool on) { if (on) frozenPos_[id] = {1e300, 1e300}; else frozenPos_.erase(id); }   // датчик «завис»: повторяет одно и то же положение
    // Сбой МИО: постоянное смещение показаний (рад, рад, м); снимается нулями.
    void biasMru(const std::string& id, double roll, double pitch, double heave) { mruJump_[id] = {roll, pitch, heave}; }
    bool isFailed(const std::string& id) const { return failed_.count(id) > 0; }
    std::vector<std::string> failedSensors() const { return {failed_.begin(), failed_.end()}; }
    // Оператор имитации: отказ/восстановление движителя по имени, шум и сдвиг курса гирокомпаса
    bool failThrusterById(const std::string& id, bool f) { for (auto& t : ths_) if (t.cfg().id == id) { t.setFailed(f); return true; } return false; }
    std::vector<std::string> failedThrusters() const { std::vector<std::string> r; for (auto& t : ths_) if (t.failed()) r.push_back(t.cfg().id); return r; }
    void noiseScale(const std::string& id, double k) { if (k <= 1.0001) noise_.erase(id); else noise_[id] = k; }
    double noiseScaleOf(const std::string& id) const { auto i = noise_.find(id); return i == noise_.end() ? 1.0 : i->second; }
    void gyroOffset(const std::string& id, double deg) { if (std::fabs(deg) < 1e-9) gyroJump_.erase(id); else gyroJump_[id] = deg2rad(deg); }
    std::map<std::string, std::pair<double, double>> jumps() const { return jump_; }
    std::map<std::string, double> gyroOffsets() const { return gyroJump_; }
    std::map<std::string, std::array<double, 3>> mruOffsets() const { return mruJump_; }
    void failThruster(size_t i) { ths_[i] = ThrusterSim([&] { ThrusterCfg c = ths_[i].cfg(); c.enabled = false; return c; }()); }

    // Замена параметров судна и движителей «на ходу» (только когда судно в ожидании): положение и скорости сохраняются.
    void setVessel(const VesselCfg& v, const std::vector<ThrusterCfg>& th) {
        const VesselState st = dyn_.state(); const EnvCfg env = dyn_.env();
        vcfg_ = v; dyn_ = VesselDynamics(v, env); dyn_.state() = st;
        ths_.clear(); for (const auto& t : th) ths_.emplace_back(t);
    }

    void applyCommands(const std::vector<ThrusterCmd>& cmds) {
        for (auto& th : ths_) for (const auto& c : cmds) if (c.id == th.cfg().id) th.command(c);
    }

    void step(double dt) {
        rampEnv(dt);
        double ps = 1.0;
        Wrench tau{};
        const double kn = std::hypot(dyn_.state().u, dyn_.state().v) / 0.514444;
        for (auto& th : ths_) { th.setSpeedKn(kn); th.step(dt, ps); const Wrench w = th.wrench(); tau.X += w.X; tau.Y += w.Y; tau.N += w.N; }
        dyn_.step(dt, tau, t_);
        t_ += dt;
    }

    SensorFrame measure() {
        SensorFrame fr;
        fr.time = t_;
        const VesselState& s = dyn_.state();
        auto g = [this](PortableRng&) { return rng_.normal(); };
        auto due = [&](int slot, double rate) {
            if (rate <= 0) return false;
            if (t_ + 1e-9 >= nextT_[static_cast<size_t>(slot)]) { nextT_[static_cast<size_t>(slot)] = t_ + 1.0 / rate; return true; }
            return false;
        };
        auto addPos = [&](const std::string& id, PosType ty, const V3& off, double sigma, bool fresh) {
            PosMeas p; p.id = id; p.type = ty; p.sigma = sigma; p.offset = off;
            const V3 o = rotateBodyToNed(s.phi, s.theta, s.psi, off);
            const double ns = noiseScaleOf(id);
            p.n = s.n + o.x + sigma * ns * g(rng_);
            p.e = s.e + o.y + sigma * ns * g(rng_);
            auto j = jump_.find(id);
            if (j != jump_.end()) { p.n += j->second.first; p.e += j->second.second; }
            { auto z = frozenPos_.find(id); if (z != frozenPos_.end()) { if (z->second.first > 1e299) z->second = {p.n, p.e}; p.n = z->second.first; p.e = z->second.second; } }
            p.fresh = fresh && !failed_.count(id);
            fr.pos.push_back(p);
        };
        int slot = 0;
        for (const auto& gn : sc_.gnss) { const bool f = due(slot++, gn.rateHz); addPos(gn.id, PosType::Gnss, gn.offset, gn.sigma, f); }
        if (sc_.laser) { const bool f = due(slot++, sc_.laserRate); addPos("LASER", PosType::LaserRadar, V3{22, 0, -8}, sc_.laserSigma, f); } else ++slot;
        if (sc_.wire) { const bool f = due(slot++, sc_.wireRate); addPos("WIRE", PosType::TautWire, V3{25, 0, 2}, sc_.wireSigma, f); } else ++slot;
        {
            const bool f = due(slot++, sc_.gyroRate);
            for (size_t i = 0; i < gyroBias_.size(); ++i) {
                HdgMeas h; h.id = "GYRO" + std::to_string(i + 1);
                h.psi = wrapPi(s.psi + gyroBias_[i] + deg2rad(sc_.gyroSigmaDeg) * g(rng_));
                { auto gj = gyroJump_.find(h.id); if (gj != gyroJump_.end()) h.psi = wrapPi(h.psi + gj->second); }
                h.fresh = f && !failed_.count(h.id);
                fr.hdg.push_back(h);
            }
        }
        {
            const bool f = due(slot++, sc_.windRate);
            double sp, fromRel; dyn_.apparentWind(sp, fromRel);
            for (int i = 0; i < sc_.anemometers; ++i) {
                WindMeas w; w.id = "WIND" + std::to_string(i + 1);
                w.speed = std::max(0.0, sp + sc_.windSigma * g(rng_));
                w.fromRel = fromRel + (sp > 0.5 ? 0.03 * g(rng_) : 0.5 * g(rng_));
                w.fresh = f && !failed_.count(w.id);
                fr.wind.push_back(w);
            }
        }
        {
            const bool f = due(slot++, sc_.mruRate);
            for (size_t i = 0; i < mruBias_.size(); ++i) {
                MruMeas m; m.id = "MRU" + std::to_string(i + 1);
                m.roll = s.phi + mruBias_[i].first + deg2rad(sc_.mruAngSigmaDeg) * g(rng_);
                m.pitch = s.theta + mruBias_[i].second + deg2rad(sc_.mruAngSigmaDeg) * g(rng_);
                m.heave = s.d + sc_.mruHeaveSigma * g(rng_);                      // в имитаторе d — вниз, на выходе heave — вниз (как K-Pos)
                auto j = mruJump_.find(m.id);
                if (j != mruJump_.end()) { m.roll += j->second[0]; m.pitch += j->second[1]; m.heave += j->second[2]; }
                m.fresh = f && !failed_.count(m.id);
                fr.mru.push_back(m);
            }
        }
        fr.power.fresh = true; fr.power.availableKW = sc_.availableKW;
        {
            const bool f = due(slot++, sc_.thrusterFbRate);
            for (const auto& th : ths_) {
                ThrusterFb fb; fb.id = th.cfg().id; fb.valid = f && th.cfg().enabled && !th.failed();
                fb.pct = th.pct(); fb.angleDeg = th.angleDeg();
                fr.fb.push_back(fb);
            }
        }
        return fr;
    }

private:
    static double toward(double x, double tgt, double maxStep) { const double d = tgt - x; return std::fabs(d) <= maxStep ? tgt : x + (d > 0 ? maxStep : -maxStep); }
    static double towardAngle(double x, double tgt, double maxStep) {
        const double d = std::fmod(tgt - x + 540.0, 360.0) - 180.0;
        return std::fabs(d) <= maxStep ? tgt : x + (d > 0 ? maxStep : -maxStep);
    }
    void rampEnv(double dt) {
        EnvCfg& e = dyn_.env(); const EnvCfg& t = target_;
        // направление ветра при почти нулевой скорости не важно — переходим сразу, чтобы штиль не «крутил» стрелку
        e.windSpeed = toward(e.windSpeed, t.windSpeed, 0.4 * dt);
        e.windFromDeg = e.windSpeed < 0.3 ? t.windFromDeg : towardAngle(e.windFromDeg, t.windFromDeg, 2.0 * dt);
        e.currentSpeed = toward(e.currentSpeed, t.currentSpeed, 0.02 * dt);
        e.currentToDeg = e.currentSpeed < 0.05 ? t.currentToDeg : towardAngle(e.currentToDeg, t.currentToDeg, 1.0 * dt);
        e.hs = toward(e.hs, t.hs, 0.05 * dt);
        e.tp = toward(e.tp, t.tp, 0.1 * dt);
        e.waveFromDeg = e.hs < 0.1 ? t.waveFromDeg : towardAngle(e.waveFromDeg, t.waveFromDeg, 1.0 * dt);
        e.turbulence = t.turbulence;
    }
    VesselCfg vcfg_;
    VesselDynamics dyn_;
    EnvCfg target_;
    SimSensorsCfg sc_;
    PortableRng rng_;
    std::vector<ThrusterSim> ths_;
    std::vector<double> gyroBias_, nextT_;
    std::vector<std::pair<double, double>> mruBias_;
    std::map<std::string, std::array<double, 3>> mruJump_;
    std::set<std::string> failed_;
    std::map<std::string, double> noise_, gyroJump_;
    std::map<std::string, std::pair<double, double>> jump_;
    std::map<std::string, std::pair<double, double>> frozenPos_;
    double t_ = 0;
};

// Типовое судно для проверки: 60 м, два азимутальных подруливающих + носовое туннельное.
inline VesselCfg demoVessel() { return VesselCfg{}; }

inline std::vector<ThrusterCfg> demoThrusters() {
    std::vector<ThrusterCfg> v;
    ThrusterCfg a; a.id = "AZ1"; a.type = ThrusterType::Azimuth; a.x = -22; a.y = -4; a.maxForward = 180e3; a.maxReverse = 150e3;
    a.forbidden = {};
    ThrusterCfg b = a; b.id = "AZ2"; b.y = 4;
    ThrusterCfg c; c.id = "BOW1"; c.type = ThrusterType::Tunnel; c.x = 22; c.y = 0; c.maxForward = 100e3; c.maxReverse = 100e3;
    ThrusterCfg d = c; d.id = "BOW2"; d.x = 18;
    v = {a, b, c, d};
    return v;
}

}  // namespace dp
