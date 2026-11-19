// Связь пульта (QML) с ядром СДП: ядро работает в том же процессе, пульт раз в 0,1 с забирает его состояние
// и передаёт команды оператора теми же текстовыми командами, что и sdp_run.
#pragma once
#include <QObject>
#include <QTimer>
#include <QVariant>
#include <QStringList>
#include <deque>
#include <memory>
#include "dp/core.hpp"

class HmiCore : public QObject {
    Q_OBJECT
    Q_PROPERTY(QVariantMap s READ s NOTIFY changed)              // сводка: состояние, режим, положение, курс, ошибки, ветер, оценка
    Q_PROPERTY(QVariantList thrusters READ thrusters NOTIFY changed)
    Q_PROPERTY(QVariantList sensors READ sensors NOTIFY changed)
    Q_PROPERTY(QVariantList alarms READ alarms NOTIFY changed)
    Q_PROPERTY(QVariantList track READ track NOTIFY changed)       // след судна: [n, e, n, e, …] за последние минуты
    Q_PROPERTY(QVariantMap hull READ hull NOTIFY shipChanged)     // корпус для рисования: длина, ширина, движители и датчики на корпусе
    Q_PROPERTY(QStringList log READ log NOTIFY logChanged)
public:
    explicit HmiCore(QObject* parent = nullptr);
    QVariantMap s() const { return s_; }
    QVariantList thrusters() const { return thr_; }
    QVariantList sensors() const { return sen_; }
    QVariantList alarms() const { return alm_; }
    QVariantList track() const { return trk_; }
    QVariantMap hull() const { return hull_; }
    QStringList log() const { return log_; }

    Q_INVOKABLE void command(const QString& line);              // любая команда ядра (как в sdp_run)
    Q_INVOKABLE void loadShip(const QString& pathOrUrl);
    Q_INVOKABLE void simStart(const QString& pathOrUrl);
    Q_INVOKABLE void ack();                                       // квитирование тревог (снимает мигание новых)
    void runFor(double seconds);                                  // прогнать ядро без реального времени (для проверок и снимков)
    void setPaused(bool p) { paused_ = p; }

signals:
    void changed();
    void shipChanged();
    void logChanged();

private:
    void tick();
    void refresh();
    void say(const std::string& m);
    static std::string localPath(const QString& p);

    dp::Core core_;
    QTimer timer_;
    bool paused_ = false;
    QVariantMap s_, hull_;
    QVariantList thr_, sen_, alm_, trk_;
    QStringList log_;
    std::deque<std::pair<double, double>> trail_;
    double lastTrail_ = -1e9;
    QSet<QString> acked_;
};
