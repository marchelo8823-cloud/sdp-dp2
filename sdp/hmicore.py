"""Ядро HMI-сервера SDP-DP2: симуляция + аварийный журнал + команды.

Потокобезопасное состояние, общее для всех фронтендов пульта
(Web-HMI по HTTP, нативный HMI по UDP): контур управления
(console.DPConsole, конфигурация C) шагает в фоновом потоке с
ускорением времени; snapshot() — снимок телеметрии dict/JSON;
command() — команды оператора и инструктора.

Отсюда же живёт аварийный журнал с квитированием и фронтовыми
авариями (пределы позиции, consequence) — как на реальном DP-пульте.
"""
import threading
import time

import numpy as np

from console import DPConsole

DT = 0.1
KN = 1.94384          # м/с → узлы


class HMIState:
    """Симуляция + аварийный журнал + настройки пульта (потокобезопасно)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.con = DPConsole()
        self.speed = 5.0
        self.warn_pos = 3.0     # жёлтый watch circle, м
        self.alarm_pos = 5.0    # красный watch circle, м
        self.alarms = []        # {id, t, text, level, acked}
        self._alarm_id = 0
        self._limit_state = 0   # детектор фронтов по ошибке позиции
        self._conseq_state = 0
        self._last_conseq_t = -10.0
        self._sim_thread = threading.Thread(target=self._sim_loop,
                                            daemon=True)
        self._sim_thread.start()

    # --------------------------------------------------------- аварии
    def _raise(self, text, level):
        self._alarm_id += 1
        self.alarms.insert(0, {"id": self._alarm_id, "t": self.con.t,
                               "text": text, "level": level, "acked": False})
        self.alarms = self.alarms[:50]

    # --------------------------------------------------------- цикл sim
    def _sim_loop(self):
        acc = 0.0
        last = time.monotonic()
        while True:
            time.sleep(0.02)
            now = time.monotonic()
            acc += (now - last) * self.speed
            last = now
            with self.lock:
                n = int(acc / DT)
                acc -= n * DT
                if n > 0 and not self.con.paused:
                    for _ in range(min(n, 200)):
                        self.con.step()
                # consequence — раз в ~5 с модельного времени
                if self.con.t - self._last_conseq_t >= 5.0:
                    self.con.update_consequence()
                    self._last_conseq_t = self.con.t
                self._watch_limits()

    def _watch_limits(self):
        """Фронтовые аварии: ошибка позиции и consequence."""
        err = self.con.pos_err
        state = 2 if err > self.alarm_pos else 1 if err > self.warn_pos else 0
        if state > self._limit_state:
            if state == 2:
                self._raise(f"ПОЗИЦИЯ: выход за аварийный предел "
                            f"({err:.1f} м > {self.alarm_pos:.0f} м)", 2)
            elif state == 1:
                self._raise(f"ПОЗИЦИЯ: выход за предел предупреждения "
                            f"({err:.1f} м > {self.warn_pos:.0f} м)", 1)
        self._limit_state = state
        cs = self.con.conseq_level
        if cs != self._conseq_state:
            if cs == 2:
                self._raise("CONSEQUENCE: " + self.con.conseq_text, 2)
            elif cs == 1:
                self._raise("CONSEQUENCE: " + self.con.conseq_text, 1)
            self._conseq_state = cs

    # --------------------------------------------------------- телеметрия
    def snapshot(self):
        with self.lock:
            c = self.con
            R_err_n = c.eta[0] - c.eta_d[0]
            R_err_e = c.eta[1] - c.eta_d[1]
            unacked = sum(1 for a in self.alarms if not a["acked"])
            worst = max([a["level"] for a in self.alarms if not a["acked"]],
                        default=0)
            thr = [{"u_pct": round(100 * th.u / th.t_max, 1),
                    "alpha_deg": round(float(np.rad2deg(th.alpha)), 1),
                    "failed": bool(th.failed),
                    "tunnel": bool(th.tunnel)} for th in c.thrusters]
            # результирующая сила движителей (body)
            fx = sum(th.u * np.cos(th.alpha) for th in c.thrusters)
            fy = sum(th.u * np.sin(th.alpha) for th in c.thrusters)
            mz = sum(th.lx * th.u * np.sin(th.alpha)
                     - th.ly * th.u * np.cos(th.alpha)
                     for th in c.thrusters)
            return {
                "t": round(c.t, 1),
                "eta": [round(float(v), 3) for v in c.eta],
                "eta_d": [round(float(v), 3) for v in c.eta_d],
                "pos_err": round(c.pos_err, 2),
                "err_n": round(float(R_err_n), 2),
                "err_e": round(float(R_err_e), 2),
                "hdg_deg": round(float(np.rad2deg(c.eta[2])) % 360, 1),
                "hdg_d_deg": round(float(np.rad2deg(c.eta_d[2])) % 360, 1),
                "hdg_err_deg": round(c.hdg_err_deg, 2),
                "sog_kn": round(float(np.hypot(*c.nu[:2])) * KN, 2),
                "rot_dpm": round(float(np.rad2deg(c.nu[2])) * 60, 1),
                "wind_true": {"speed_kn": round(c.wind.V * KN, 1),
                              "dir_deg": round(float(
                                  np.rad2deg(c.wind.gamma)) % 360, 1)},
                "wind_meas": {"speed_kn": round(c.V_m * KN, 1),
                              "dir_deg": round(float(
                                  np.rad2deg(c.g_m)) % 360, 1)},
                "hs": c.hs, "tp": c.tp,
                "bias_kn": round(float(np.hypot(*c.observer.bias_hat[:2]))
                                 / 1e3, 1),
                "thrusters": thr,
                "resultant": {"force_kn": round(float(
                                  np.hypot(fx, fy)) / 1e3, 1),
                              "dir_deg": round(float(
                                  np.rad2deg(np.arctan2(fy, fx))) % 360, 1),
                              "moment_knm": round(float(mz) / 1e3, 1)},
                "sensors": {"prs": [bool(f) for f in c.sensors.prs_failed],
                            "gyro": [bool(f) for f in c.sensors.gyro_failed]},
                "conseq": {"level": c.conseq_level, "text": c.conseq_text},
                "alarms": self.alarms[:12],
                "unacked": unacked,
                "alarm_worst": worst,
                "warn_pos": self.warn_pos,
                "alarm_pos": self.alarm_pos,
                "speed": self.speed,
                "paused": c.paused,
            }

    # --------------------------------------------------------- команды
    def command(self, cmd):
        act = cmd.get("action")
        with self.lock:
            if act == "setpoint":
                self.con.set_setpoint(cmd["north"], cmd["east"])
            elif act == "heading":
                self.con.eta_d[2] = np.deg2rad(float(cmd["deg"]))
            elif act == "heading_bump":
                self.con.bump_heading(cmd["d_deg"])
            elif act == "wind":
                self.con.set_wind(cmd["speed_ms"],
                                  cmd.get("dir_deg",
                                          float(np.rad2deg(
                                              self.con.wind.gamma)) % 360))
            elif act == "hs":
                self.con.set_hs(cmd["hs"])
            elif act == "fail":
                kind, idx = cmd["kind"], int(cmd["idx"])
                self.con.inject(kind, idx)
                name = {"thruster": f"ДРУ {idx + 1}",
                        "prs": f"PRS {idx + 1}",
                        "gyro": f"ГИРО {idx + 1}"}[kind]
                self._raise(f"ИНСТРУКТОР: отказ {name}", 1)
            elif act == "speed":
                self.speed = float(cmd["value"])
            elif act == "pause":
                self.con.paused = not self.con.paused
            elif act == "reset":
                self.con._init_sim()
                self._limit_state = 0
                self._conseq_state = 0
                self._raise("Сброс симуляции", 0)
            elif act == "limits":
                self.warn_pos = float(cmd["warn"])
                self.alarm_pos = float(cmd["alarm"])
            elif act == "ack":
                for a in self.alarms:
                    a["acked"] = True
            return {"ok": True}
