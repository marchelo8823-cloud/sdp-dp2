"""Приёмник NMEA 0183 (UDP) — вход реальных датчиков в контур ДП.

Поддерживаемые сообщения:
  GGA — позиция GNSS (lat/lon → локальные N/E, м, от первой точки)
  VTG — курс и скорость над грунтом
  HDT — истинный курс (гирокомпас)
  MWV — ветер (скорость/угол)

Архитектура как на судне: внешний датчик приходит в свой канал
(«PRS 4 EXT», «ГИРО 3 EXT») и участвует в вотировании наравне с
остальными; при пропадании данных (> EXT_TIMEOUT с) канал сам
выпадает из решения без ложных отбраковок.

Запуск слушателя — отдельный поток; разбор — чистый stdlib.
"""
import math
import socket
import threading
import time

EXT_TIMEOUT = 2.0      # с; старше — данные не действительны
M_PER_DEG_LAT = 111320.0


def checksum_ok(sentence):
    """Проверка контрольной суммы NMEA ($...*hh)."""
    s = sentence.strip()
    if not s.startswith("$") or "*" not in s:
        return False
    body, _, tail = s[1:].partition("*")
    try:
        want = int(tail[:2], 16)
    except ValueError:
        return False
    got = 0
    for ch in body:
        got ^= ord(ch)
    return got == want


def make_checksum(body):
    """Собрать предложение с контрольной суммой (для тестов/имитаторов)."""
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"${body}*{cs:02X}"


def _latlon(raw, hemi):
    """NMEA ddmm.mmmm → десятичные градусы."""
    if not raw:
        return None
    dot = raw.find(".")
    deg = float(raw[:dot - 2])
    minutes = float(raw[dot - 2:])
    v = deg + minutes / 60.0
    return -v if hemi in ("S", "W") else v


class NmeaReceiver:
    """UDP-приёмник; хранит последние действительные измерения."""

    def __init__(self, port=7201, host="0.0.0.0"):
        self.port = port
        self.host = host
        self.lock = threading.Lock()
        self.origin = None          # (lat0, lon0) первой точки
        self.pos_ne = None          # (t_mono, n_m, e_m)
        self.heading = None         # (t_mono, deg)
        self.sog_cog = None         # (t_mono, sog_ms, cog_deg)
        self.wind = None            # (t_mono, speed_ms, angle_deg)
        self.sentences = 0
        self.bad = 0
        self.last_src = ""
        self._sock = None
        self._thread = None

    # ------------------------------------------------------------ приём
    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.settimeout(1.0)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while True:
            try:
                data, _ = self._sock.recvfrom(4096)
            except (socket.timeout, OSError):
                continue
            for line in data.decode("ascii", "ignore").splitlines():
                if line.startswith("$"):
                    self.parse(line)

    # ------------------------------------------------------------ разбор
    def parse(self, sentence):
        if not checksum_ok(sentence):
            with self.lock:
                self.bad += 1
            return
        body = sentence.strip()[1:].split("*")[0]
        f = body.split(",")
        typ = f[0][-3:]           # GPGGA/GNGGA → GGA
        now = time.monotonic()
        with self.lock:
            self.sentences += 1
            self.last_src = f[0][:2]
            try:
                if typ == "GGA" and f[6] in ("1", "2", "4", "5"):
                    lat = _latlon(f[2], f[3])
                    lon = _latlon(f[4], f[5])
                    if lat is None or lon is None:
                        return
                    if self.origin is None:
                        self.origin = (lat, lon)
                    lat0, lon0 = self.origin
                    n = (lat - lat0) * M_PER_DEG_LAT
                    e = ((lon - lon0) * M_PER_DEG_LAT
                         * math.cos(math.radians(lat0)))
                    self.pos_ne = (now, n, e)
                elif typ == "HDT" and f[1]:
                    self.heading = (now, float(f[1]))
                elif typ == "VTG" and f[1] and f[7]:
                    self.sog_cog = (now, float(f[7]) / 3.6, float(f[1]))
                elif typ == "MWV" and f[1] and f[3] and f[5] == "A":
                    v = float(f[3])
                    if f[4] == "K":        # км/ч → м/с
                        v /= 3.6
                    elif f[4] == "N":      # узлы → м/с
                        v *= 0.51444
                    self.wind = (now, v, float(f[1]))
            except (ValueError, IndexError):
                self.bad += 1

    # ------------------------------------------------------------ выдача
    def get_position(self):
        with self.lock:
            if self.pos_ne and time.monotonic() - self.pos_ne[0] < EXT_TIMEOUT:
                return self.pos_ne[1], self.pos_ne[2]
        return None

    def get_heading(self):
        with self.lock:
            if self.heading and time.monotonic() - self.heading[0] < EXT_TIMEOUT:
                return self.heading[1]
        return None

    def status(self):
        with self.lock:
            age = (round(time.monotonic() - self.pos_ne[0], 1)
                   if self.pos_ne else None)
            return {"port": self.port, "sentences": self.sentences,
                    "bad": self.bad, "src": self.last_src,
                    "fix_age_s": age,
                    "origin": self.origin}
