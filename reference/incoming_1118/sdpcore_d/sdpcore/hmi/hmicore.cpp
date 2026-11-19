#include "hmicore.h"
#include <QDateTime>
#include <QUrl>
#include <cmath>

using namespace dp;

HmiCore::HmiCore(QObject* parent) : QObject(parent), core_({}, [this](const std::string& m) { say(m); }) {
    connect(&timer_, &QTimer::timeout, this, &HmiCore::tick);
    timer_.start(100);
    refresh();
}

std::string HmiCore::localPath(const QString& p) {
    const QUrl u(p);
    return (u.isLocalFile() ? u.toLocalFile() : p).toStdString();
}

void HmiCore::say(const std::string& m) {
    const QString t = QString::fromStdString(m);
    for (const QString& line : t.split('\n', Qt::SkipEmptyParts))
        log_.prepend(QString("%1  %2").arg(core_.clock(), 7, 'f', 0).arg(line));
    while (log_.size() > 300) log_.removeLast();
    emit logChanged();
}

void HmiCore::command(const QString& line) {
    const QString l = line.trimmed();
    if (l.isEmpty()) return;
    say(("> " + l).toStdString());
    const bool hadShip = core_.hasShip();
    core_.command(l.toStdString());
    if (core_.hasShip() != hadShip || l.startsWith("ship load")) { trail_.clear(); emit shipChanged(); }
    refresh();
}
void HmiCore::loadShip(const QString& p) { command("ship load " + QString::fromStdString(localPath(p))); }
void HmiCore::simStart(const QString& p) { command("sim start " + QString::fromStdString(localPath(p))); }
void HmiCore::ack() { for (const QVariant& a : alm_) acked_.insert(a.toMap().value("code").toString()); refresh(); }

void HmiCore::tick() {
    if (paused_) return;
    const int n = std::max(1, static_cast<int>(std::lround(core_.simSpeed())));   // ускорение имитатора: столько циклов за 0,1 с
    for (int i = 0; i < n; ++i) core_.cycle();
    refresh();
}

void HmiCore::runFor(double seconds) {
    const double until = core_.clock() + seconds;
    while (core_.clock() < until) {
        core_.cycle();
        const EngineOutput& o = core_.out();
        if (core_.hasShip() && o.est.posValid && core_.clock() - lastTrail_ >= 2.0) { trail_.push_back({o.est.n, o.est.e}); lastTrail_ = core_.clock(); if (trail_.size() > 450) trail_.pop_front(); }
    }
    refresh();
}

void HmiCore::refresh() {
    const EngineOutput& o = core_.out();
    const EstimatorState& es = o.est;
    QVariantMap m;
    const CoreState st = core_.state();
    m["state"] = coreStateName(st);
    m["stateCode"] = static_cast<int>(st);
    m["mode"] = modeName(core_.mode());
    m["modeCode"] = static_cast<int>(core_.mode());
    m["sim"] = core_.simRunning();
    m["simSpeed"] = core_.simSpeed();
    m["clock"] = core_.clock();
    m["hasShip"] = core_.hasShip();
    m["shipName"] = core_.ship() ? QString::fromStdString(core_.ship()->vessel.name) : QString();
    m["n"] = es.n; m["e"] = es.e; m["hdg"] = std::fmod(rad2deg(es.psi) + 360.0, 360.0);
    m["rot"] = rad2deg(es.r) * 60.0;                              // °/мин
    m["u"] = es.u; m["v"] = es.v;
    m["posValid"] = es.posValid; m["hdgValid"] = es.hdgValid; m["hdgDR"] = es.hdgDeadReckon; m["posDR"] = es.refLost;
    m["posSource"] = QString::fromStdString(es.posSource);
    m["refN"] = o.ctl.refN; m["refE"] = o.ctl.refE; m["refHdg"] = std::fmod(o.ctl.refPsiDeg + 360.0, 360.0);
    m["errDist"] = o.ctl.errDist; m["errHdg"] = o.ctl.errHdgDeg; m["errAlong"] = o.ctl.errAlong; m["errCross"] = o.ctl.errCross;
    m["windSpeed"] = es.windSpeed; m["windFromRel"] = std::fmod(rad2deg(es.windFromRel) + 360.0, 360.0); m["windValid"] = es.windValid;
    // неизвестная сила (течение, волны и всё, что не измерено) в мировых осях — направление «куда» и величина
    m["distN"] = es.biasN; m["distE"] = es.biasE;
    m["load"] = o.alloc.utilization; m["powerKW"] = o.powerKW;
    m["tauX"] = o.ctl.tau.X; m["tauY"] = o.ctl.tau.Y; m["tauN"] = o.ctl.tau.N;
    m["capX"] = o.cap.X; m["capY"] = o.cap.Y; m["capN"] = o.cap.N;
    m["roll"] = rad2deg(es.roll); m["pitch"] = rad2deg(es.pitch); m["heave"] = es.heave;
    m["waveHs"] = es.wave.valid ? es.wave.hs : -1.0;
    if (core_.simRunning()) { auto* sim = core_.sim(); const auto& tr = sim->truth(); m["trueN"] = tr.n; m["trueE"] = tr.e; }
    // анализ последствий отказа: класс, был ли расчёт, запас по группам наихудшего отказа
    if (DpEngine* eng = core_.engine()) {
        const Consequence& ca = eng->consequence();
        m["caClass"] = ca.dpClass(); m["caRan"] = ca.ran(); m["caAge"] = ca.ran() ? core_.clock() - ca.lastRunT() : -1.0; m["caSingle"] = ca.singleFailures();
        QVariantList gl; bool bad = false;
        for (const auto& g : ca.groups()) {
            QVariantMap x; x["name"] = QString::fromStdString(g.name); x["ids"] = QString::fromStdString(Consequence::join(g.ids));
            x["force"] = g.forcePct; x["moment"] = g.momentPct; x["holds"] = g.holds; x["alarm"] = g.alarm; bad = bad || g.alarm;
            gl.push_back(x);
        }
        m["caGroups"] = gl; m["caAlarm"] = bad;
    } else { m["caClass"] = 0; m["caRan"] = false; m["caGroups"] = QVariantList(); m["caAlarm"] = false; }
    s_ = m;

    QVariantList th;
    if (const AppCfg* cfg = core_.ship()) {
        for (size_t i = 0; i < cfg->thrusters.size(); ++i) {
            const ThrusterCfg& t = cfg->thrusters[i];
            QVariantMap x;
            x["id"] = QString::fromStdString(t.id);
            x["type"] = t.type == ThrusterType::Azimuth ? "поворотный" : t.type == ThrusterType::Tunnel ? "туннельный" : "неповоротный";
            x["azimuth"] = t.type == ThrusterType::Azimuth;
            x["x"] = t.x; x["y"] = t.y;
            x["enabled"] = t.enabled; x["taken"] = core_.taken(i);
            x["maxKN"] = t.maxForward / 1000.0;
            double cmdPct = 0, cmdAng = 0, fbPct = 0, fbAng = 0; bool fb = false, failed = false;
            if (i < o.cmds.size()) { cmdPct = o.cmds[i].pct; cmdAng = o.cmds[i].angleDeg; }
            if (core_.simRunning() && i < core_.sim()->thrusters().size()) {
                const auto& ts = core_.sim()->thrusters()[i]; fbPct = ts.pct(); fbAng = ts.angleDeg(); fb = true; failed = ts.failed() || !ts.cfg().enabled;
            }
            x["cmdPct"] = cmdPct; x["cmdAng"] = std::fmod(cmdAng + 360.0, 360.0);
            x["fbPct"] = fbPct; x["fbAng"] = std::fmod(fbAng + 360.0, 360.0); x["fb"] = fb; x["failed"] = failed;
            x["thrustKN"] = (fbPct >= 0 ? fbPct / 100.0 * t.maxForward : fbPct / 100.0 * t.maxReverse) / 1000.0;
            th.push_back(x);
        }
    }
    thr_ = th;

    QVariantList sn;
    for (const auto& ss : es.sensors) {
        QVariantMap x; const QString id = QString::fromStdString(ss.id);
        const QString k = id.left(1);
        x["kind"] = k == "P" ? "положение" : k == "H" ? "курс" : k == "W" ? "ветер" : k == "M" ? "качка" : "?";
        x["id"] = id.mid(2); x["valid"] = ss.valid; x["age"] = ss.ageS < 1e8 ? ss.ageS : -1.0;
        x["state"] = QString::fromStdString(ss.state); x["rejected"] = ss.rejected;
        sn.push_back(x);
    }
    sen_ = sn;

    QVariantList al; QSet<QString> now;
    for (const auto& a : o.alarms) {
        QVariantMap x; const QString code = QString::fromStdString(a.code);
        x["code"] = code; x["text"] = QString::fromStdString(a.text); x["level"] = a.level();   // 1 — аварийная, 2 — предупреждение, 3 — сообщение
        x["acked"] = acked_.contains(code); now.insert(code);
        al.push_back(x);
    }
    for (auto it = acked_.begin(); it != acked_.end();) { if (!now.contains(*it)) it = acked_.erase(it); else ++it; }   // снятая тревога при повторе снова «новая»
    std::stable_sort(al.begin(), al.end(), [](const QVariant& a, const QVariant& b) { return a.toMap()["level"].toInt() < b.toMap()["level"].toInt(); });
    alm_ = al;

    if (core_.hasShip() && es.posValid && core_.clock() - lastTrail_ >= 2.0) { trail_.push_back({es.n, es.e}); lastTrail_ = core_.clock(); if (trail_.size() > 450) trail_.pop_front(); }
    QVariantList tk; for (const auto& p : trail_) { tk.push_back(p.first); tk.push_back(p.second); }
    trk_ = tk;

    QVariantMap h;
    if (const AppCfg* cfg = core_.ship()) {
        h["length"] = cfg->vessel.length; h["beam"] = cfg->vessel.beam;
        QVariantList hs; for (const auto& t : cfg->thrusters) { QVariantMap x; x["id"] = QString::fromStdString(t.id); x["x"] = t.x; x["y"] = t.y; x["azimuth"] = t.type == ThrusterType::Azimuth; hs.push_back(x); }
        h["thrusters"] = hs;
    }
    if (h != hull_) { hull_ = h; emit shipChanged(); }
    emit changed();
}
