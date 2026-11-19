"""Voyage recorder («чёрный ящик») SDP-DP2.

По образцу судовых VDR/журналов DP (IMO MSC/Circ.645 — запись
состояния и событий): телеметрия 1 Гц в CSV + журнал событий
(аварии, команды оператора/инструктора) с метками времени.

Файлы на сессию:
  logs/voyage_YYYYMMDD_HHMMSS.csv   — телеметрия
  logs/events_YYYYMMDD_HHMMSS.csv   — события

Если каталог проекта недоступен на запись (установленный .app),
лог пишется в ~/SDP-DP2-logs.
"""
import csv
import os
import threading
import time

FIELDS = [
    "t", "clock",
    "n", "e", "hdg_deg", "nd", "ed", "hdg_d_deg",
    "pos_err", "hdg_err_deg", "sog_kn", "rot_dpm",
    "wind_meas_kn", "wind_meas_deg", "wind_true_kn", "wind_true_deg",
    "hs", "bias_kn",
    "u1", "u2", "u3", "u4", "az1", "az2", "az3", "az4",
    "failed_mask", "mode", "conseq_level",
]


def _log_dir():
    d = os.path.join(os.getcwd(), "logs")
    try:
        os.makedirs(d, exist_ok=True)
        probe = os.path.join(d, ".probe")
        with open(probe, "w"):
            pass
        os.remove(probe)
        return d
    except OSError:
        d = os.path.expanduser("~/SDP-DP2-logs")
        os.makedirs(d, exist_ok=True)
        return d


class VoyageRecorder:
    """Потокобезопасный писатель CSV; периодический сброс буфера."""

    def __init__(self, flush_s=5.0):
        self.lock = threading.Lock()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        d = _log_dir()
        self.voyage_path = os.path.join(d, f"voyage_{stamp}.csv")
        self.events_path = os.path.join(d, f"events_{stamp}.csv")
        self._fv = open(self.voyage_path, "w", newline="",
                        encoding="utf-8")
        self._fe = open(self.events_path, "w", newline="",
                        encoding="utf-8")
        self._wv = csv.writer(self._fv)
        self._we = csv.writer(self._fe)
        self._wv.writerow(FIELDS)
        self._we.writerow(["t", "clock", "kind", "text"])
        self._last_flush = time.monotonic()
        self._flush_s = flush_s
        self.rows = 0

    # ------------------------------------------------------------- строки
    def log_row(self, row):
        """row: dict по FIELDS (телеметрия, 1 Гц)."""
        with self.lock:
            self._wv.writerow([row.get(f, "") for f in FIELDS])
            self.rows += 1
            self._maybe_flush()

    def event(self, kind, text, t):
        with self.lock:
            self._we.writerow([round(t, 1),
                               time.strftime("%H:%M:%S"), kind, text])
            self._maybe_flush()

    def _maybe_flush(self):
        now = time.monotonic()
        if now - self._last_flush >= self._flush_s:
            self._fv.flush()
            self._fe.flush()
            self._last_flush = now

    def close(self):
        with self.lock:
            self._fv.close()
            self._fe.close()
