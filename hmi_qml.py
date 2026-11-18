"""SDP-DP2 — QML-HMI: современный пульт оператора (Qt Quick, GPU-отрисовка).

Слой отображения нового поколения поверх того же UDP-протокола
(sim_server.py): телеметрия приходит по UDP, сглаживается в мосте
Telemetry (экспоненциальное сглаживание углов 20 Гц — плавные стрелки
и циферблаты), экран — декларативный QML (qml/main.qml), который
дальше можно править визуально в Qt Design Studio.

Запуск:        python hmi_qml.py            (сервер поднимется сам)
Headless-тест: python hmi_qml.py --smoke 200
"""
import math
import os
import subprocess
import sys
import time
from pathlib import Path

from PySide6.QtCore import (QObject, QTimer, Signal, Property, Slot,
                            QCoreApplication)
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine, qmlRegisterSingletonInstance
from PySide6.QtQuickControls2 import QQuickStyle

HERE = Path(__file__).parent
QML = HERE / "qml" / "main.qml"

QQuickStyle.setStyle("Basic")   # нейтивный стиль macOS не даёт кастомизации


def _lerp_angle(a, b, k):
    """Экспоненциальное сближение углов с учётом перехода 0/360."""
    d = (b - a + 180.0) % 360.0 - 180.0
    return (a + k * d) % 360.0


class Telemetry(QObject):
    """UDP → сглаженный QVariantMap для QML + слот команд."""

    tick = Signal()
    stateChanged = Signal()

    def __init__(self, backend):
        super().__init__()
        self.be = backend
        self._state = {}
        self._smooth = {}          # ключ -> текущее сглаженное значение
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll)
        self._timer.start(50)      # 20 Гц — частота анимации

    # ---------------------------------------------------------- сглаживание
    def _smoothed(self, s):
        k = 0.25
        sm = self._smooth

        def ang(key, val):
            sm[key] = _lerp_angle(sm.get(key, val), val, k)
            return sm[key]

        out = dict(s)
        out["hdg_deg"] = ang("hdg", s["hdg_deg"])
        out["hdg_d_deg"] = ang("hdg_d", s["hdg_d_deg"])
        out["wind_dir"] = ang("wind", s["wind_true"]["dir_deg"])
        thr = []
        for i, t_ in enumerate(s["thrusters"]):
            t2 = dict(t_)
            t2["alpha_deg"] = ang(f"alpha{i}", t_["alpha_deg"])
            key = f"u{i}"
            prev = sm.get(key, t_["u_pct"])
            sm[key] = prev + k * (t_["u_pct"] - prev)
            t2["u_pct"] = sm[key]
            thr.append(t2)
        out["thrusters"] = thr
        return out

    def _poll(self):
        s = self.be.state()
        if s is not None:
            self._state = self._smoothed(s)
            self.stateChanged.emit()
        self.tick.emit()

    def get_state(self):
        return self._state

    state = Property("QVariantMap", get_state, notify=stateChanged)

    @Slot(dict)
    def cmd(self, d):
        self.be.cmd(dict(d))


# ------------------------------------------------------------------ запуск

def run_gui():
    app = QGuiApplication(sys.argv)
    app.setApplicationName("SDP-DP2 HMI")
    from hmi import make_backend
    backend = make_backend(sys.argv)
    telem = Telemetry(backend)
    # синглтон вместо context property: байндинги сразу видят объект,
    # без гонки «telem is null» при первой инициализации QML
    qmlRegisterSingletonInstance(Telemetry, "Sdp", 1, 0, "Telem", telem)
    engine = QQmlApplicationEngine()
    engine.load(str(QML))
    if not engine.rootObjects():
        print("ОШИБКА: не загрузился qml/main.qml", file=sys.stderr)
        sys.exit(1)
    sys.exit(app.exec())


def smoke(n_ticks=200):
    """Headless: offscreen QML + UDP-сервер, проверка загрузки и данных."""
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    srv = subprocess.Popen([sys.executable, str(HERE / "sim_server.py")],
                           cwd=str(HERE))
    try:
        time.sleep(1.0)
        app = QGuiApplication(sys.argv)
        from hmi import UdpBackend
        be = UdpBackend()
        deadline = time.monotonic() + 5.0
        while be.state() is None and time.monotonic() < deadline:
            time.sleep(0.1)
        assert be.state() is not None, "сервер не отвечает"
        telem = Telemetry(be)
        qmlRegisterSingletonInstance(Telemetry, "Sdp", 1, 0, "Telem", telem)
        engine = QQmlApplicationEngine()
        engine.load(str(QML))
        assert engine.rootObjects(), "QML не загрузился"
        for k in range(n_ticks):
            if k == 100:
                be.cmd({"action": "fail", "kind": "thruster", "idx": 2})
            app.processEvents()
            time.sleep(0.02)
        st = telem.state
        assert st and math.isfinite(st["pos_err"]), "нет телеметрии в QML-мосте"
        assert any(t_["failed"] for t_ in st["thrusters"]), "отказ не дошёл"
        root = engine.rootObjects()[0]
        print(f"qml smoke OK: t={st['t']:.0f} с, ошибка {st['pos_err']:.2f} м, "
              f"QML root: {root.objectName() or type(root).__name__}, "
              f"ДРУ3 отказ: {st['thrusters'][2]['failed']}")
    finally:
        srv.terminate()


if __name__ == "__main__":
    if "--smoke" in sys.argv:
        i = sys.argv.index("--smoke")
        n = int(sys.argv[i + 1]) if i + 1 < len(sys.argv) else 200
        smoke(n)
    else:
        run_gui()
