"""SDP-DP2 — нативный HMI пульта оператора (Qt, стилистика Kongsberg K-Pos).

Настольное приложение (не браузер). Архитектура «как на судне»:
контур живёт в отдельном процессе (sim_server.py), HMI — чистый
UDP-клиент телеметрии; тот же протокол позже пойдёт к реальным
контроллерам ДП. Режимы запуска:

  python hmi.py            UDP-клиент; сервер поднимается автоматически
  python hmi.py --local    ядро внутри процесса (отладка без сети)
  python hmi.py --smoke N  headless-проверка (offscreen)

Лучшие решения промышленных пультов (K-Pos, NavDP4000, Wärtsilä):
градиентные циферблаты, затухающая трасса, табулярные цифры,
аварийный журнал с квитированием, watch circles, пульт инструктора.
"""
import json
import math
import socket
import subprocess
import sys
import time
from pathlib import Path

from PySide6.QtCore import Qt, QTimer, QPointF, QRectF
from PySide6.QtGui import (QColor, QFont, QPainter, QPainterPath, QPen,
                           QPolygonF, QRadialGradient, QLinearGradient)
from PySide6.QtWidgets import (QApplication, QFrame, QGridLayout, QHBoxLayout,
                               QLabel, QListWidget, QListWidgetItem,
                               QMainWindow, QPushButton, QSlider, QVBoxLayout,
                               QWidget, QDoubleSpinBox)

HERE = Path(__file__).parent

# ------------------------------------------------------------------ палитра
DARK = dict(bg="#0a0f1a", panel="#111a2c", panel2="#0d1524", line="#1e2c47",
            txt="#c9d6ea", dim="#5f7396", disk1="#101a30", disk2="#0a1120")
DAY = dict(bg="#dfe5ee", panel="#f4f7fb", panel2="#e9eef6", line="#c3cede",
           txt="#1b2a41", dim="#5b6b84", disk1="#eef2f9", disk2="#dde5f0")
ACC = dict(green="#2fd07c", yellow="#ffc542", red="#ff5252",
           cyan="#39c2d7", blue="#4d8dff", orange="#ff9f43")

SHIP_N = [38.0, 12.0, -30.0, -38.0, -38.0, -30.0, 12.0]
SHIP_E = [0.0, 9.0, 9.0, 4.0, -4.0, -9.0, -9.0]
THR_POS = [(30.0, 0.0), (18.0, 0.0), (-30.0, 6.0), (-30.0, -6.0)]


def qcol(hexs, alpha=255):
    c = QColor(hexs)
    c.setAlpha(alpha)
    return c


# ------------------------------------------------------------------ бэкенды
class LocalBackend:
    """Ядро внутри процесса (режим --local)."""

    def __init__(self):
        from sdp.hmicore import HMIState
        self._core = HMIState()

    def state(self):
        return self._core.snapshot()

    def cmd(self, d):
        self._core.command(d)


class UdpBackend:
    """Клиент UDP к sim_server.py — схема судовой интеграции."""

    def __init__(self, host="127.0.0.1", port=7200):
        self.server = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.setblocking(False)
        self.tel_port = self.sock.getsockname()[1]
        self._latest = None
        self._last_sub = 0.0
        self._subscribe()

    def _subscribe(self):
        self.sock.sendto(json.dumps(
            {"action": "subscribe", "port": self.tel_port}).encode(),
            self.server)
        self._last_sub = time.monotonic()

    def state(self):
        """Свежайший снимок (или None, если сервер молчит)."""
        if time.monotonic() - self._last_sub > 5.0:
            self._subscribe()
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                self._latest = json.loads(data.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                pass
        return self._latest

    def cmd(self, d):
        try:
            self.sock.sendto(json.dumps(d).encode(), self.server)
        except OSError:
            pass


def make_backend(argv):
    if "--local" in argv:
        return LocalBackend()
    be = UdpBackend()
    deadline = time.monotonic() + 1.2
    while time.monotonic() < deadline:
        if be.state() is not None:
            return be
        time.sleep(0.05)
    # сервер не найден — поднимаем свой (пробрасываем --profile,
    # если задан клиенту: hmi_qml.py --profile sdp2)
    srv_argv = [sys.executable, str(HERE / "sim_server.py")]
    if "--profile" in argv:
        i = argv.index("--profile")
        if i + 1 < len(argv):
            srv_argv += ["--profile", argv[i + 1]]
    subprocess.Popen(srv_argv, cwd=str(HERE),
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        if be.state() is not None:
            break
        time.sleep(0.05)
    return be


# ------------------------------------------------------------- базовый виджет
class Panel(QFrame):
    """Панель с заголовком в стиле K-Pos."""

    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setObjectName("panel")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 8)
        lay.setSpacing(4)
        self.title = QLabel(title.upper())
        self.title.setObjectName("panelTitle")
        lay.addWidget(self.title)


class Led(QLabel):
    def set_on(self, on):
        self.setProperty("on", on)
        self.style().unpolish(self)
        self.style().polish(self)


def _disk_gradient(c, R, pal):
    g = QRadialGradient(c, R)
    g.setColorAt(0.0, qcol(pal["disk1"]))
    g.setColorAt(1.0, qcol(pal["disk2"]))
    return g


# --------------------------------------------------------- position plot
class PositionPlot(QWidget):
    """Компасное кольцо, watch circles, трасса, судно; клик — уставка."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pal = DARK
        self.range_m = 100.0
        self.err_n = self.err_e = 0.0
        self.hdg = 0.0
        self.trail = []
        self.warn_r = 3.0
        self.alarm_r = 5.0
        self.clicked = None          # callback(north_off, east_off)
        self.setMinimumSize(380, 380)

    def set_palette(self, pal):
        self.pal = pal
        self.update()

    def _to_scr(self, n, e, c, R):
        return QPointF(c.x() + e / self.range_m * R,
                       c.y() - n / self.range_m * R)

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        R = min(w, h) / 2 - 14
        c = QPointF(w / 2, h / 2)
        # диск с градиентом
        p.setPen(QPen(qcol(self.pal["line"]), 1))
        p.setBrush(_disk_gradient(c, R, self.pal))
        p.drawEllipse(c, R, R)
        # кольца 1/3, 2/3, 3/3
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(qcol(self.pal["line"]), 1, Qt.DashLine))
        for f in (1, 2, 3):
            p.drawEllipse(c, R * f / 3, R * f / 3)
        # риски и подписи
        fnt = QFont("Menlo", max(7, int(R / 34)))
        p.setFont(fnt)
        for a in range(0, 360, 10):
            r1 = R * (0.95 if a % 30 == 0 else 0.975)
            s, co = math.sin(math.radians(a)), math.cos(math.radians(a))
            p.setPen(QPen(qcol(self.pal["dim"]), 1))
            p.drawLine(QPointF(c.x() + r1 * s, c.y() - r1 * co),
                       QPointF(c.x() + R * s, c.y() - R * co))
            if a % 30 == 0:
                p.drawText(QRectF(c.x() + 0.86 * R * s - 20,
                                  c.y() - 0.86 * R * co - 8, 40, 16),
                           Qt.AlignCenter, str(a))
        for f in (1, 2, 3):
            p.drawText(QPointF(c.x() + 3, c.y() - R * f / 3 - 2),
                       f"{round(self.range_m * f / 3)}")
        p.setPen(qcol(ACC["red"]))
        p.setFont(QFont("Menlo", max(8, int(R / 26)), QFont.Bold))
        p.drawText(QPointF(c.x() - 5, c.y() - R - 3), "N")

        # watch circles (вокруг уставки = центр)
        for rr, col in ((self.warn_r, ACC["yellow"]),
                        (self.alarm_r, ACC["red"])):
            p.setPen(QPen(qcol(col, 200), 1.5, Qt.DashLine))
            p.drawEllipse(c, rr / self.range_m * R, rr / self.range_m * R)
        # уставка: перекрестие
        p.setPen(QPen(qcol(ACC["red"]), 1.6))
        p.drawLine(QPointF(c.x() - 7, c.y()), QPointF(c.x() + 7, c.y()))
        p.drawLine(QPointF(c.x(), c.y() - 7), QPointF(c.x(), c.y() + 7))

        # трасса с затуханием (старые точки прозрачнее)
        n_tr = len(self.trail)
        if n_tr > 1:
            step = 10
            p.setBrush(Qt.NoBrush)
            for i0 in range(0, n_tr - 1, step):
                i1 = min(i0 + step, n_tr - 1)
                alpha = int(30 + 200 * (i0 / max(n_tr - 1, 1)))
                p.setPen(QPen(qcol(ACC["blue"], alpha), 1.4))
                path = QPainterPath()
                path.moveTo(self._to_scr(*self.trail[i0], c, R))
                for i in range(i0 + 1, i1 + 1):
                    path.lineTo(self._to_scr(*self.trail[i], c, R))
                p.drawPath(path)

        # судно: свечение + корпус
        sp = self._to_scr(self.err_n, self.err_e, c, R)
        sc = R / self.range_m
        poly = QPolygonF([QPointF(e * sc, -n * sc)
                          for n, e in zip(SHIP_N, SHIP_E)])
        p.save()
        p.translate(sp)
        p.rotate(self.hdg)
        p.setPen(QPen(qcol(ACC["blue"], 70), 7))
        p.setBrush(Qt.NoBrush)
        p.drawPolygon(poly)
        p.setPen(QPen(qcol(self.pal["bg"]), 1))
        p.setBrush(qcol(ACC["blue"]))
        p.drawPolygon(poly)
        p.restore()

        p.end()

    def mousePressEvent(self, ev):
        if self.clicked is None:
            return
        w, h = self.width(), self.height()
        R = min(w, h) / 2 - 14
        dx = ev.position().x() - w / 2
        dy = ev.position().y() - h / 2
        e = dx / R * self.range_m
        n = -dy / R * self.range_m
        self.clicked(n, e)


# ------------------------------------------------------------- компас
class Compass(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.pal = DARK
        self.hdg = 0.0
        self.hdg_d = 0.0
        self.setMinimumSize(190, 190)

    def set_palette(self, pal):
        self.pal = pal
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        R = min(w, h) / 2 - 10
        c = QPointF(w / 2, h / 2)
        p.setPen(QPen(qcol(self.pal["line"]), 1.5))
        p.setBrush(_disk_gradient(c, R, self.pal))
        p.drawEllipse(c, R, R)
        for a in range(0, 360, 5):
            big = a % 30 == 0
            r1 = R * (0.86 if big else 0.93)
            s, co = math.sin(math.radians(a)), math.cos(math.radians(a))
            p.setPen(QPen(qcol(self.pal["txt"] if big else self.pal["dim"]),
                          2 if big else 1))
            p.drawLine(QPointF(c.x() + r1 * s, c.y() - r1 * co),
                       QPointF(c.x() + 0.99 * R * s, c.y() - 0.99 * R * co))
            if a % 90 == 0:
                col = ACC["red"] if a == 0 else self.pal["txt"]
                p.setPen(qcol(col))
                p.setFont(QFont("Menlo", max(9, int(R / 9)), QFont.Bold))
                p.drawText(QRectF(c.x() + 0.68 * R * s - 20,
                                  c.y() - 0.68 * R * co - 10, 40, 20),
                           Qt.AlignCenter, "NESW"[a // 90])
        # жучок уставки со свечением
        p.save()
        p.translate(c)
        p.rotate(self.hdg_d)
        bug = QPolygonF([QPointF(0, -R - 2), QPointF(6, -R + 12),
                         QPointF(-6, -R + 12)])
        p.setPen(QPen(qcol(ACC["green"], 80), 5))
        p.drawPolyline(bug)
        p.setPen(Qt.NoPen)
        p.setBrush(qcol(ACC["green"]))
        p.drawPolygon(bug)
        p.restore()
        # судно
        p.save()
        p.translate(c)
        p.rotate(self.hdg)
        p.setPen(QPen(qcol(self.pal["bg"]), 1))
        p.setBrush(qcol(ACC["blue"]))
        r6 = R * 0.62
        p.drawPolygon(QPolygonF([QPointF(0, -r6), QPointF(r6 * 0.26, r6 * 0.68),
                                 QPointF(0, r6 * 0.4),
                                 QPointF(-r6 * 0.26, r6 * 0.68)]))
        p.restore()
        # цифра курса
        p.setPen(qcol(self.pal["txt"]))
        p.setFont(QFont("Menlo", max(13, int(R / 6)), QFont.Bold))
        p.drawText(QRectF(c.x() - 60, c.y() - 12, 120, 24), Qt.AlignCenter,
                   f"{self.hdg:.1f}°")
        p.end()


# ------------------------------------------------------------- ветер
class WindRose(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.pal = DARK
        self.dir_deg = 0.0
        self.setMinimumSize(150, 150)

    def set_palette(self, pal):
        self.pal = pal
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        R = min(w, h) / 2 - 8
        c = QPointF(w / 2, h / 2)
        p.setPen(QPen(qcol(self.pal["line"]), 1.2))
        p.setBrush(_disk_gradient(c, R, self.pal))
        p.drawEllipse(c, R, R)
        p.setFont(QFont("Menlo", max(7, int(R / 12))))
        for a in range(0, 360, 30):
            s, co = math.sin(math.radians(a)), math.cos(math.radians(a))
            p.setPen(QPen(qcol(self.pal["dim"]), 1))
            p.drawLine(QPointF(c.x() + 0.9 * R * s, c.y() - 0.9 * R * co),
                       QPointF(c.x() + R * s, c.y() - R * co))
            col = ACC["red"] if a == 0 else self.pal["dim"]
            p.setPen(qcol(col))
            p.drawText(QRectF(c.x() + 0.72 * R * s - 15,
                              c.y() - 0.72 * R * co - 8, 30, 16),
                       Qt.AlignCenter, str(a))
        # стрелка ветра (откуда дует → к центру)
        p.save()
        p.translate(c)
        p.rotate(self.dir_deg)
        p.setPen(QPen(qcol(ACC["cyan"], 70), 6))
        p.drawLine(QPointF(0, 0.75 * R), QPointF(0, 0.12 * R))
        p.setPen(QPen(qcol(ACC["cyan"]), 2))
        p.setBrush(qcol(ACC["cyan"]))
        p.drawLine(QPointF(0, 0.75 * R), QPointF(0, 0.12 * R))
        p.drawPolygon(QPolygonF([QPointF(0, 0), QPointF(0.09 * R, 0.18 * R),
                                 QPointF(-0.09 * R, 0.18 * R)]))
        p.restore()
        p.end()


# ------------------------------------------------------------- mimic ДРУ
class ThrusterMimic(QWidget):
    """Силуэт судна + 4 движителя: стрелка азимута и столбик тяги."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pal = DARK
        self.thr = [(0.0, 0.0, False)] * 4   # (alpha_deg, u_pct, failed)
        self.setMinimumSize(300, 220)

    def set_palette(self, pal):
        self.pal = pal
        self.update()

    @staticmethod
    def _load_col(u_pct, failed):
        if failed:
            return ACC["red"]
        a = min(abs(u_pct) / 100.0, 1.0)
        return ACC["green"] if a < 0.6 else ACC["yellow"] if a < 0.9 \
            else ACC["orange"]

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        sx = (w - 70) / 20.0
        sy = (h - 60) / 88.0
        cx, cy = w / 2 - 30, h / 2

        def scr(lx, ly):
            return QPointF(cx + ly * sx, cy - lx * sy)

        hull = QPolygonF([scr(44, 0), scr(20, 9), scr(-34, 9), scr(-44, 4),
                          scr(-44, -4), scr(-34, -9), scr(20, -9)])
        grad = QLinearGradient(scr(44, 0), scr(-44, 0))
        grad.setColorAt(0.0, qcol(self.pal["disk1"]))
        grad.setColorAt(1.0, qcol(self.pal["disk2"]))
        p.setPen(QPen(qcol(self.pal["dim"]), 1.5))
        p.setBrush(grad)
        p.drawPolygon(hull)

        for (lx, ly), (alpha, u, failed) in zip(THR_POS, self.thr):
            pos = scr(lx, ly)
            col = qcol(self._load_col(u, failed))
            p.setPen(QPen(col, 1.5))
            p.setBrush(qcol(self.pal["panel"]))
            p.drawEllipse(pos, 13, 13)
            # стрелка азимута
            p.save()
            p.translate(pos)
            p.rotate(alpha)
            p.setPen(QPen(col, 2.5))
            p.drawLine(QPointF(0, 0), QPointF(0, -12))
            p.restore()
            # столбик тяги
            bh = min(abs(u) / 100.0, 1.0) * 34
            bx = pos.x() + 17
            p.setPen(Qt.NoPen)
            p.setBrush(col)
            if u >= 0:
                p.drawRect(QRectF(bx, pos.y() + 15 - bh, 7, bh))
            else:
                p.drawRect(QRectF(bx, pos.y() + 15, 7, bh))
            p.setPen(qcol(self.pal["txt"]))
            p.setFont(QFont("Menlo", 9))
            p.drawText(QRectF(pos.x() - 40, pos.y() + 18, 80, 14),
                       Qt.AlignCenter,
                       "ОТКАЗ" if failed else f"{alpha:.0f}° {abs(u):.0f}%")
        p.end()


# ------------------------------------------------------------- тренды
class Trends(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.pal = DARK
        self.data = []           # (t, err, wind_kn, force_kn)
        self.setMinimumHeight(110)

    def set_palette(self, pal):
        self.pal = pal
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, qcol(self.pal["panel2"]))
        p.setPen(QPen(qcol(self.pal["line"]), 1))
        for i in range(1, 4):
            p.drawLine(0, h * i / 4, w, h * i / 4)
        n = len(self.data)
        if n < 2:
            p.end()
            return
        series = ((1, 8.0, ACC["orange"]), (2, 45.0, ACC["cyan"]),
                  (3, 400.0, ACC["green"]))
        for idx, mx, col in series:
            path = QPainterPath()
            for i, row in enumerate(self.data):
                x = i / (n - 1) * w
                y = h - min(row[idx] / mx, 1.0) * h * 0.94 - 3
                if i == 0:
                    path.moveTo(x, y)
                else:
                    path.lineTo(x, y)
            # заливка под кривой
            fill = QPainterPath(path)
            fill.lineTo(w, h)
            fill.lineTo(0, h)
            fill.closeSubpath()
            p.setPen(Qt.NoPen)
            p.setBrush(qcol(col, 28))
            p.drawPath(fill)
            p.setPen(QPen(qcol(col), 1.6))
            p.setBrush(Qt.NoBrush)
            p.drawPath(path)
        p.end()


# ------------------------------------------------------------- главное окно
class HMIWindow(QMainWindow):
    def __init__(self, backend):
        super().__init__()
        self.setWindowTitle("SDP-DP2 — Пульт оператора ДП")
        self.backend = backend
        self.pal = DARK
        self._trail_last = -1.0
        self._alarms_key = None
        self._last_state = None

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 6, 8, 6)
        root.setSpacing(6)

        # ---------------- header
        hdr = QHBoxLayout()
        logo = QLabel("SDP-DP2")
        logo.setObjectName("logo")
        mode = QLabel("AUTO POSITION")
        mode.setObjectName("mode")
        self.clock = QLabel("T+00:00")
        self.clock.setObjectName("clock")
        self.status = QLabel("DP STATUS: NORMAL")
        self.status.setObjectName("lampOk")
        self.conseq = QLabel("…")
        self.conseq.setObjectName("conseqOk")
        self.albadge = QPushButton("0")
        self.albadge.setObjectName("badgeZero")
        self.albadge.setToolTip("Квитировать все")
        self.albadge.clicked.connect(self.ack)
        b_ack = QPushButton("ACK")
        b_ack.clicked.connect(self.ack)
        b_theme = QPushButton("☀/☾")
        b_theme.setFixedWidth(52)
        b_theme.clicked.connect(self.toggle_theme)
        for w_ in (logo, mode, self.clock, self.status):
            hdr.addWidget(w_)
        hdr.addStretch(1)
        hdr.addWidget(self.conseq, 2)
        hdr.addStretch(1)
        for w_ in (self.albadge, b_ack, b_theme):
            hdr.addWidget(w_)
        root.addLayout(hdr)

        # ---------------- основная сетка
        grid = QGridLayout()
        grid.setSpacing(6)
        root.addLayout(grid, 1)

        p_plot = Panel("Position plot")
        self.plot = PositionPlot()
        self.plot.clicked = self.set_setpoint_rel
        self.range_lbl = QLabel("range 100 м")
        self.range_lbl.setObjectName("dimLabel")
        p_plot.layout().addWidget(self.plot, 1)
        p_plot.layout().addWidget(self.range_lbl)
        grid.addWidget(p_plot, 0, 0, 2, 1)

        p_hdg = Panel("Heading")
        self.compass = Compass()
        p_hdg.layout().addWidget(self.compass)
        self.kv_rot = self._kv(p_hdg, "ROT")
        self.kv_hdev = self._kv(p_hdg, "Отклонение курса")
        self.kv_sog = self._kv(p_hdg, "SOG")
        self.kv_gyro = self._kv(p_hdg, "Гирокомпасы")
        grid.addWidget(p_hdg, 0, 1)

        p_thr = Panel("Thruster main · ALLOC: VARIABLE")
        self.mimic = ThrusterMimic()
        p_thr.layout().addWidget(self.mimic, 1)
        self.kv_resf = self._kv(p_thr, "Resultant force")
        self.kv_resd = self._kv(p_thr, "Resultant direction")
        self.kv_resm = self._kv(p_thr, "Resultant moment")
        grid.addWidget(p_thr, 0, 2, 2, 1)

        p_wind = Panel("Wind")
        self.windrose = WindRose()
        p_wind.layout().addWidget(self.windrose)
        self.kv_windm = self._kv(p_wind, "Измеренный")
        self.kv_windt = self._kv(p_wind, "Истинный")
        self.kv_wave = self._kv(p_wind, "Волна Hs / Tp")
        grid.addWidget(p_wind, 0, 3)

        p_sens = Panel("Датчики")
        self.prs_leds = [self._led_row(p_sens, f"PRS {i + 1}") for i in range(3)]
        self.gyro_leds = [self._led_row(p_sens, f"ГИРО {i + 1}")
                          for i in range(2)]
        self.kv_perr = self._kv(p_sens, "Отклонение позиции")
        self.kv_perr_ne = self._kv(p_sens, "N / E")
        self.kv_bias = self._kv(p_sens, "Bias (LF смещение)")
        grid.addWidget(p_sens, 1, 1)
        grid.setColumnStretch(0, 3)
        grid.setColumnStretch(1, 2)
        grid.setColumnStretch(2, 2)
        grid.setColumnStretch(3, 2)

        # ---------------- нижний ряд
        bot = QGridLayout()
        bot.setSpacing(6)
        root.addLayout(bot)

        p_tr = Panel("Тренды: ошибка позиции · ветер · суммарная тяга")
        self.trends = Trends()
        p_tr.layout().addWidget(self.trends)
        bot.addWidget(p_tr, 0, 0)

        p_al = Panel("Аварийный журнал")
        self.alarms = QListWidget()
        p_al.layout().addWidget(self.alarms)
        bot.addWidget(p_al, 0, 1)

        p_in = Panel("Пульт инструктора")
        self._build_instructor(p_in)
        bot.addWidget(p_in, 0, 2)
        bot.setColumnStretch(0, 3)
        bot.setColumnStretch(1, 2)
        bot.setColumnStretch(2, 2)

        self.trend_data = []

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self.apply_theme()
        self.resize(1420, 860)

    # ------------------------------------------------ helpers построения
    def _kv(self, panel, name):
        row = QHBoxLayout()
        k = QLabel(name)
        k.setObjectName("dimLabel")
        v = QLabel("—")
        v.setObjectName("kvValue")
        row.addWidget(k)
        row.addStretch(1)
        row.addWidget(v)
        panel.layout().addLayout(row)
        return v

    def _led_row(self, panel, name):
        row = QHBoxLayout()
        led = Led("●")
        led.setObjectName("led")
        led.set_on(True)
        lbl = QLabel(name)
        st = QLabel("OK")
        st.setObjectName("kvValue")
        row.addWidget(led)
        row.addWidget(lbl)
        row.addStretch(1)
        row.addWidget(st)
        panel.layout().addLayout(row)
        return (led, st)

    def _build_instructor(self, panel):
        lay = panel.layout()
        row = QHBoxLayout()
        for label, kind, idx in (("ДРУ1", "thruster", 0), ("ДРУ2", "thruster", 1),
                                 ("ДРУ3", "thruster", 2), ("ДРУ4", "thruster", 3),
                                 ("PRS3", "prs", 2), ("ГИРО1", "gyro", 0)):
            b = QPushButton(label)
            b.setObjectName("failBtn")
            b.clicked.connect(
                lambda _c, k=kind, i=idx:
                self.backend.cmd({"action": "fail", "kind": k, "idx": i}))
            row.addWidget(b)
        lay.addLayout(row)

        self.sliders = {}
        for key, name, mn, mx, step, val, fmt in (
                ("wind_v", "Ветер", 0, 20, 0.5, 10, "{:.1f} м/с"),
                ("wind_d", "Направление ветра", 0, 360, 5, 30, "{:.0f}°"),
                ("hs", "Hs", 0, 6, 0.5, 2, "{:.1f} м"),
                ("speed", "Ускорение", 1, 20, 1, 5, "{:.0f}×")):
            lbl = QLabel(name + ": " + fmt.format(val))
            lbl.setObjectName("dimLabel")
            sl = QSlider(Qt.Horizontal)
            sl.setRange(int(mn / step), int(mx / step))
            sl.setValue(int(val / step))
            sl.valueChanged.connect(
                lambda v, k=key, l=lbl, n=name, f=fmt, s=step:
                self._slider(k, l, n, f, s, v))
            lay.addWidget(lbl)
            lay.addWidget(sl)
            self.sliders[key] = (sl, lbl, name, fmt, step)

        row2 = QHBoxLayout()
        self.b_pause = QPushButton("Пауза")
        self.b_pause.clicked.connect(
            lambda: self.backend.cmd({"action": "pause"}))
        b_hm = QPushButton("Курс −10°")
        b_hm.clicked.connect(
            lambda: self.backend.cmd({"action": "heading_bump", "d_deg": -10}))
        b_hp = QPushButton("Курс +10°")
        b_hp.clicked.connect(
            lambda: self.backend.cmd({"action": "heading_bump", "d_deg": 10}))
        b_rst = QPushButton("Сброс")
        b_rst.setObjectName("dangerBtn")
        b_rst.clicked.connect(self.reset)
        for b in (self.b_pause, b_hm, b_hp, b_rst):
            row2.addWidget(b)
        lay.addLayout(row2)

        row3 = QHBoxLayout()
        row3.addWidget(QLabel("Watch circle, м:"))
        self.sp_warn = QDoubleSpinBox()
        self.sp_warn.setRange(1, 50)
        self.sp_warn.setValue(3)
        self.sp_alarm = QDoubleSpinBox()
        self.sp_alarm.setRange(2, 80)
        self.sp_alarm.setValue(5)
        b_wc = QPushButton("Задать")
        b_wc.clicked.connect(self._set_watch)
        for w_ in (self.sp_warn, self.sp_alarm, b_wc):
            row3.addWidget(w_)
        lay.addLayout(row3)

    # ------------------------------------------------ команды оператора
    def _slider(self, key, lbl, name, fmt, step, v):
        val = v * step
        lbl.setText(name + ": " + fmt.format(val))
        if key in ("wind_v", "wind_d"):
            self.backend.cmd({"action": "wind",
                              "speed_ms": self.sliders["wind_v"][0].value()
                              * self.sliders["wind_v"][4],
                              "dir_deg": self.sliders["wind_d"][0].value()
                              * self.sliders["wind_d"][4]})
        elif key == "hs":
            self.backend.cmd({"action": "hs", "hs": val})
        elif key == "speed":
            self.backend.cmd({"action": "speed", "value": val})

    def _set_watch(self):
        self.backend.cmd({"action": "limits",
                          "warn": self.sp_warn.value(),
                          "alarm": self.sp_alarm.value()})

    def set_setpoint_rel(self, n_off, e_off):
        s = self._last_state
        if s is None:
            return
        self.backend.cmd({"action": "setpoint",
                          "north": s["eta_d"][0] + n_off,
                          "east": s["eta_d"][1] + e_off})

    def reset(self):
        self.plot.trail.clear()
        self.trend_data.clear()
        self.backend.cmd({"action": "reset"})

    def ack(self):
        self.backend.cmd({"action": "ack"})

    def toggle_theme(self):
        self.pal = DAY if self.pal is DARK else DARK
        self.apply_theme()

    # ------------------------------------------------ стили
    def apply_theme(self):
        pal = self.pal
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ background:{pal['bg']}; color:{pal['txt']};
                font: 13px "SF Pro Text","Segoe UI",Arial; }}
            #panel {{ background:{pal['panel']}; border:1px solid {pal['line']};
                border-radius:6px; }}
            #panelTitle {{ color:{pal['dim']}; font-size:11px;
                letter-spacing:1px; }}
            #logo {{ color:{ACC['cyan']}; font-size:16px; font-weight:bold; }}
            #mode {{ background:{ACC['blue']}; color:white; padding:2px 10px;
                border-radius:3px; font-weight:600; }}
            #clock {{ font-family: Menlo; font-weight:600; }}
            #lampOk {{ color:{ACC['green']}; border:1px solid {ACC['green']};
                padding:2px 10px; border-radius:3px; font-weight:700; }}
            #lampWarn {{ color:{ACC['yellow']}; border:1px solid {ACC['yellow']};
                padding:2px 10px; border-radius:3px; font-weight:700; }}
            #lampAlarm {{ color:{ACC['red']}; border:1px solid {ACC['red']};
                padding:2px 10px; border-radius:3px; font-weight:700; }}
            #conseqOk {{ color:{ACC['green']}; font-weight:600; }}
            #conseqWarn {{ color:{ACC['yellow']}; font-weight:600; }}
            #conseqAlarm {{ color:{ACC['red']}; font-weight:700; }}
            #badgeZero {{ background:{pal['line']}; color:{pal['dim']};
                border-radius:9px; padding:1px 10px; font-weight:700; }}
            #badgeHot {{ background:{ACC['red']}; color:white;
                border-radius:9px; padding:1px 10px; font-weight:700; }}
            QPushButton {{ background:{pal['panel2']}; color:{pal['txt']};
                border:1px solid {pal['line']}; border-radius:4px;
                padding:4px 10px; }}
            QPushButton:hover {{ border-color:{ACC['cyan']}; }}
            #failBtn {{ border-color:{ACC['orange']}; }}
            #dangerBtn {{ border-color:{ACC['red']}; color:{ACC['red']}; }}
            #dimLabel {{ color:{pal['dim']}; }}
            #kvValue {{ font-weight:600; font-family: Menlo; }}
            #led {{ color:{ACC['green']}; }}
            #led[on="false"] {{ color:{ACC['red']}; }}
            QListWidget {{ background:{pal['panel2']};
                border:1px solid {pal['line']}; border-radius:4px; }}
            QSlider::groove:horizontal {{ height:4px;
                background:{pal['line']}; border-radius:2px; }}
            QSlider::handle:horizontal {{ width:14px; margin:-6px 0;
                border-radius:7px; background:{ACC['blue']}; }}
            QDoubleSpinBox {{ background:{pal['panel2']}; color:{pal['txt']};
                border:1px solid {pal['line']}; border-radius:3px;
                padding:2px 4px; }}
        """)
        for w_ in (self.plot, self.compass, self.windrose, self.mimic,
                   self.trends):
            w_.set_palette(pal)

    def _restyle(self, w_):
        self.style().unpolish(w_)
        self.style().polish(w_)

    # ------------------------------------------------ главный цикл
    def tick(self):
        s = self.backend.state()
        self._last_state = s
        if s is None:
            if self.status.objectName() != "lampAlarm":
                self.status.setObjectName("lampAlarm")
                self._restyle(self.status)
            self.status.setText("НЕТ СВЯЗИ С СЕРВЕРОМ")
            return

        # часы
        mm = int(s["t"] // 60)
        ss = int(s["t"] % 60)
        self.clock.setText(
            f"T+{mm:02d}:{ss:02d}{' ⏸' if s['paused'] else ''}"
            f" · {s['speed']:.0f}×")

        # статус
        if s["alarm_worst"] >= 2 or s["conseq"]["level"] >= 2:
            lamp, txt = "lampAlarm", "DP STATUS: ALARM"
        elif s["alarm_worst"] == 1 or s["conseq"]["level"] == 1 \
                or s["pos_err"] > s["warn_pos"]:
            lamp, txt = "lampWarn", "DP STATUS: WARNING"
        else:
            lamp, txt = "lampOk", "DP STATUS: NORMAL"
        if self.status.objectName() != lamp:
            self.status.setObjectName(lamp)
            self._restyle(self.status)
        self.status.setText(txt)
        cq_name = ["conseqOk", "conseqWarn", "conseqAlarm"][s["conseq"]["level"]]
        if self.conseq.objectName() != cq_name:
            self.conseq.setObjectName(cq_name)
            self._restyle(self.conseq)
        self.conseq.setText(s["conseq"]["text"])

        # position plot
        new_range = min(500.0, max(30.0, 1.5 * s["pos_err"],
                                   1.7 * s["alarm_pos"]))
        if abs(new_range - self.plot.range_m) / self.plot.range_m > 0.15:
            self.plot.range_m = new_range
            self.range_lbl.setText(f"range {round(new_range)} м")
        self.plot.err_n = s["err_n"]
        self.plot.err_e = s["err_e"]
        self.plot.hdg = s["hdg_deg"]
        self.plot.warn_r = s["warn_pos"]
        self.plot.alarm_r = s["alarm_pos"]
        if s["t"] - self._trail_last > 1.0:
            if self._trail_last >= 0 and s["t"] < self._trail_last:
                self.plot.trail.clear()          # был сброс на сервере
            self.plot.trail.append((s["err_n"], s["err_e"]))
            if len(self.plot.trail) > 600:
                self.plot.trail.pop(0)
            self._trail_last = s["t"]
        self.plot.update()

        # compass
        self.compass.hdg = s["hdg_deg"]
        self.compass.hdg_d = s["hdg_d_deg"]
        self.compass.update()
        self.kv_rot.setText(f"{s['rot_dpm']:.1f} °/min")
        self.kv_hdev.setText(f"{s['hdg_err_deg']:.1f}°")
        self.kv_sog.setText(f"{s['sog_kn']:.2f} kn")
        self.kv_gyro.setText("  ".join(
            f"G{i + 1}:{'ОТКАЗ' if f else 'OK'}"
            for i, f in enumerate(s["sensors"]["gyro"])))

        # thrusters + resultant
        self.mimic.thr = [(t_["alpha_deg"], t_["u_pct"], t_["failed"])
                          for t_ in s["thrusters"]]
        self.mimic.update()
        r = s["resultant"]
        self.kv_resf.setText(f"{r['force_kn']:.0f} kN")
        self.kv_resd.setText(f"{r['dir_deg']:.1f}°")
        self.kv_resm.setText(f"{r['moment_knm']:.0f} kNm")

        # wind / sensors
        self.windrose.dir_deg = s["wind_true"]["dir_deg"]
        self.windrose.update()
        self.kv_windt.setText(f"{s['wind_true']['speed_kn']:.1f} kn  "
                              f"{s['wind_true']['dir_deg']:.0f}°")
        self.kv_windm.setText(f"{s['wind_meas']['speed_kn']:.1f} kn  "
                              f"{s['wind_meas']['dir_deg']:.0f}°")
        self.kv_wave.setText(f"{s['hs']:.1f} м / {s['tp']:.0f} с")
        for (led, st), failed in zip(self.prs_leds, s["sensors"]["prs"]):
            led.set_on(not failed)
            st.setText("ОТКАЗ" if failed else "OK")
        for (led, st), failed in zip(self.gyro_leds, s["sensors"]["gyro"]):
            led.set_on(not failed)
            st.setText("ОТКАЗ" if failed else "OK")
        self.kv_perr.setText(f"{s['pos_err']:.2f} м")
        self.kv_perr_ne.setText(f"{s['err_n']:.2f} / {s['err_e']:.2f} м")
        self.kv_bias.setText(f"{s['bias_kn']:.1f} kN")

        # alarms
        key = tuple((a["id"], a["acked"]) for a in s["alarms"])
        if key != self._alarms_key:
            self._alarms_key = key
            self.alarms.clear()
            for a in s["alarms"]:
                it = QListWidgetItem(f"{a['t']:6.0f}с  {a['text']}")
                col = [self.pal["dim"], ACC["yellow"], ACC["red"]][a["level"]]
                if a["acked"]:
                    col = self.pal["dim"]
                it.setForeground(qcol(col))
                self.alarms.addItem(it)
        unacked = s["unacked"]
        self.albadge.setText(str(unacked))
        name = "badgeHot" if unacked else "badgeZero"
        if self.albadge.objectName() != name:
            self.albadge.setObjectName(name)
            self._restyle(self.albadge)

        # trends
        self.trend_data.append((s["t"], s["pos_err"],
                                s["wind_meas"]["speed_kn"],
                                abs(r["force_kn"]) * 10))
        if len(self.trend_data) > 2400:
            self.trend_data.pop(0)
        self.trends.data = self.trend_data
        self.trends.update()


# ------------------------------------------------------------------ запуск

def run_gui(argv):
    app = QApplication(argv)
    app.setStyle("Fusion")
    backend = make_backend(argv)
    w = HMIWindow(backend)
    w.show()
    sys.exit(app.exec())


def smoke(n_ticks=300):
    """Headless: offscreen + UDP-сервер в подпроцессе, прогон без окна."""
    import os
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    srv = subprocess.Popen([sys.executable, str(HERE / "sim_server.py")],
                           cwd=str(HERE))
    try:
        time.sleep(1.0)
        app = QApplication(sys.argv)
        be = UdpBackend()
        deadline = time.monotonic() + 5.0
        while be.state() is None and time.monotonic() < deadline:
            time.sleep(0.1)
        assert be.state() is not None, "сервер не отвечает"
        w = HMIWindow(be)
        for k in range(n_ticks):
            w.tick()
            if k == 100:
                be.cmd({"action": "fail", "kind": "thruster", "idx": 3})
            if k == 200:
                be.cmd({"action": "wind", "speed_ms": 15.0, "dir_deg": 120.0})
                w.toggle_theme()
                w.toggle_theme()
            app.processEvents()
            time.sleep(0.01)
        s = be.state()
        assert s is not None and math.isfinite(s["pos_err"])
        assert any(t_["failed"] for t_ in s["thrusters"]), "отказ не дошёл"
        print(f"hmi smoke OK (UDP): t={s['t']:.0f} с, "
              f"ошибка {s['pos_err']:.2f} м, "
              f"banner: {s['conseq']['text']}, алармов: {len(s['alarms'])}")
    finally:
        srv.terminate()


if __name__ == "__main__":
    if "--smoke" in sys.argv:
        i = sys.argv.index("--smoke")
        n = int(sys.argv[i + 1]) if i + 1 < len(sys.argv) else 300
        smoke(n)
    else:
        run_gui(sys.argv)
