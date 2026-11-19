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
from types import SimpleNamespace

import numpy as np

from console import DPConsole
from sdp.capability_live import live_polar
from sdp.redundancy import GYRO_DISAGREE_DEG, DEV_LIM

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
        self._pos_armed = False  # позиционные аварии активны после первого
                                 # входа в жёлтый круг (нет ложной аварии на старте)
        self._last_conseq_t = -10.0
        # --- деградационные состояния (по образцу промышленных пультов)
        self._gyro_lost_t = None     # старт счисления курса (модельное t)
        self._prs_lost_t = None      # старт счисления положения
        self.gyro_release_s = 4.0    # упор снимается через столько секунд
        self._released = False
        self._sen_prev = None        # снимок состояний датчиков (фронты)
        self._prs_dis_prev = False   # фронт аварии «расхождение PRS»
        # --- онлайн-диаграмма возможностей (K-Pos №10): фоновый пересчёт
        self._cap = {"betas": [], "intact": [], "worst": []}
        self._cap_key = None
        self._sim_thread = threading.Thread(target=self._sim_loop,
                                            daemon=True)
        self._sim_thread.start()
        self._cap_thread = threading.Thread(target=self._cap_loop,
                                            daemon=True)
        self._cap_thread.start()

    # --------------------------------------------------------- capability
    def _cap_loop(self):
        """Пересчёт поляры предельного ветра при смене конфигурации/Hs.

        Тяжёлый статический расчёт (бисекция × QP) идёт вне основного
        контура, чтобы не тормозить шаг симуляции; в snapshot попадает
        последний готовый результат — как отдельный вычислитель
        capability на реальном пульте.
        """
        while True:
            time.sleep(2.0)
            try:
                with self.lock:
                    c = self.con
                    key = (tuple(bool(t.failed) for t in c.thrusters),
                           round(c.hs, 1))
                    if key == self._cap_key:
                        continue
                    ths = [SimpleNamespace(
                        lx=t.lx, ly=t.ly, t_max=t.t_max,
                        tunnel=getattr(t, "tunnel", False),
                        failed=bool(t.failed)) for t in c.thrusters]
                    hs = c.hs
                betas, intact, worst = live_polar(ths, hs)
                cap = {
                    "betas": [round(float(b), 0) for b in betas],
                    "intact": [round(float(v) * KN, 1) for v in intact],
                    "worst": [round(float(v) * KN, 1) for v in worst],
                }
                with self.lock:
                    self._cap = cap
                    self._cap_key = key
            except Exception:                            # noqa: BLE001
                # расчёт capability не должен ронять пульт
                time.sleep(5.0)

    def _cap_margin(self, c):
        """Предел и запас по ветру при текущем направлении среды.

        Интерполяция кривой worst (ещё один отказ) на направление
        измеренного ветра относительно носа.
        """
        cap = self._cap
        if not cap["betas"]:
            return None, None
        beta = (float(np.rad2deg(c.g_m)) - float(np.rad2deg(c.eta[2]))) % 360.0
        step = 360.0 / len(cap["betas"])
        k = int(round(beta / step)) % len(cap["betas"])
        lim = cap["worst"][k]
        margin = lim - c.V_m * KN
        return round(lim, 1), round(margin, 1)

    def _cap_snapshot(self, c):
        lim, margin = self._cap_margin(c)
        return {"betas": self._cap["betas"],
                "intact": self._cap["intact"],
                "worst": self._cap["worst"],
                "limit_kn": lim,
                "margin_kn": margin}

    @staticmethod
    def _prs_view(c):
        """prs_detail + смещение измерения от заданной точки (для меток)."""
        det = c.sensors.prs_detail()
        for d in det:
            if d["xy"] is not None:
                d["dn"] = round(d["xy"][0] - float(c.eta_d[0]), 2)
                d["de"] = round(d["xy"][1] - float(c.eta_d[1]), 2)
            else:
                d["dn"] = d["de"] = None
        return det

    # --------------------------------------------------------- аварии
    # Пояснения к авариям: «почему → что делать» (замечание операторов
    # о нечитаемых тревогах; как help-тексты аварий на K-Pos).
    _HINTS = (
        ("ГИРОКОМПАСЫ: нет данных",
         "Все гирокомпасы отказали или сняты. Курс идёт по модели, "
         "ошибка нарастает. → Вернуть исправный гирокомпас; до снятия "
         "упора приготовиться к ручному управлению (MAN JOY)."),
        ("УПОР СНЯТ — НЕТ КУРСА",
         "Без курса автоматика не держит судно, команды движителям "
         "обнулены, судно дрейфует. → MAN JOY: удерживайте вручную. "
         "После возврата курса восстановите упор."),
        ("СИСТЕМЫ ПОЛОЖЕНИЯ: нет измерений",
         "Ни одна PRS не выдаёт позицию: удержание по модели, уход "
         "растёт со временем и не виден на индикации ошибки. → Вернуть "
         "хотя бы одну систему; следить за судном визуально/по радару."),
        ("отбракована (замирание)",
         "Данные системы не меняются — вероятен отказ канала или "
         "вычислителя. Система исключена из вотирования. → Проверить "
         "оборудование; после устранения — «Вернуть»."),
        ("отбракована (разброс)",
         "Шум измерений выше нормы — помехи, многолучёвость или "
         "неисправность антенны. → Проверить условия приёма; система "
         "вернётся в работу кнопкой «Вернуть»."),
        ("отбракована (скачок)",
         "Ступенчатое изменение данных — типично при потере "
         "диффпоправок GNSS. → Проверить статус поправок."),
        ("расхождение с медианой",
         "Система ушла от остальных — «плывущая» система. Автоматика "
         "исключила её из решения. → Не возвращать, пока причина "
         "не устранена."),
        ("CONSEQUENCE",
         "Анализ последствий: при ЕЩЁ ОДНОМ отказе удержание не "
         "гарантировано. → Развернуть судно к среде для снижения "
         "нагрузки, вернуть резервные агрегаты; при ALARM — прервать "
         "операцию по процедуре."),
        ("ИНСТРУКТОР",
         "Учебная инъекция с пульта инструктора. → Отработать действия; "
         "возврат — кнопками пульта инструктора."),
        ("ОПЕРАТОР",
         "Действие оператора с опорной системой. → Возврат — кнопкой "
         "«Вернуть» в панели опорных систем."),
        ("отказ",  # «ДАТЧИКИ: отказ X — нет данных»
         "Агрегат не выдаёт данные и исключён из решения. → Проверить "
         "питание/канал; резервные системы в работе автоматически."),
        ("расхождение гирокомпасов",
         "Два гирокомпаса разошлись сверх порога — автоматика не знает, "
         "кому верить. → Сравнить с магнитным компасом/визуально и "
         "снять ошибочный («Снять» в панели опорных систем)."),
        ("расхождение систем положения",
         "Две PRS разошлись сверх порога — решение за оператором. "
         "→ Определить ошибочную систему (метки P1–P3 на плане: у кого "
         "метка ушла от судна) и снять её."),
        ("выход за аварийный предел",
         "Судно за красным watch circle. → Проверить тягу, среду, "
         "consequence; по процедуре — прерывание операции/отход."),
        ("выход за предел предупреждения",
         "Судно за жёлтым кругом. → Следить за трендом; проверить "
         "не растёт ли среда и не насыщены ли движители."),
        ("Упор восстановлен",
         "Команды движителям восстановлены, автоматика снова держит "
         "судно. Информационное сообщение."),
    )

    def _raise(self, text, level):
        self._alarm_id += 1
        hint = ""
        for key, h in self._HINTS:
            if key in text:
                hint = h
                break
        self.alarms.insert(0, {"id": self._alarm_id, "t": self.con.t,
                               "clock": time.strftime("%H:%M:%S"),
                               "text": text, "level": level,
                               "hint": hint, "acked": False})
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
                self._watch_degradation()
                self._watch_sensors()

    def _watch_degradation(self):
        """Счисление курса/положения и аварийное снятие упора.

        Потеря всех гирокомпасов → счисление курса; через
        gyro_release_s секунд — снятие упора (автоматика не держит
        судно, нужно ручное управление). Потеря всех PRS → удержание
        по модели, вероятный уход растёт со временем.
        """
        c = self.con
        gyro_lost = c.sensors.gyro_available() == 0
        prs_lost = c.sensors.prs_available() == 0
        # --- гирокомпасы
        if gyro_lost and self._gyro_lost_t is None:
            self._gyro_lost_t = c.t
            self._raise("ГИРОКОМПАСЫ: нет данных — курс по счислению. "
                        f"Упор будет снят через {self.gyro_release_s:.0f} с",
                        2)
        if not gyro_lost:
            self._gyro_lost_t = None
        if (gyro_lost and not self._released
                and c.t - self._gyro_lost_t >= self.gyro_release_s):
            self._released = True
            c.thrust_released = True
            self._raise("УПОР СНЯТ — НЕТ КУРСА: автоматика не держит "
                        "судно, нужно ручное управление", 2)
        # --- системы положения
        if prs_lost and self._prs_lost_t is None:
            self._prs_lost_t = c.t
            self._raise("СИСТЕМЫ ПОЛОЖЕНИЯ: нет измерений — удержание "
                        "по модели, точность падает", 2)
        if not prs_lost:
            self._prs_lost_t = None

    def _watch_sensors(self):
        """Фронтовые аварии по каждому датчику: отказ, отбраковка,
        расхождение гирокомпасов, возврат в работу."""
        s = self.con.sensors
        cur = {}
        for i, ch in enumerate(s.prs):
            cur[f"prs{i}"] = (ch.state, ch.reason, ch.disagree)
        for i, ch in enumerate(s.gyro):
            cur[f"gyro{i}"] = (ch.state, ch.reason, ch.disagree)
        if self._sen_prev is not None:
            for key, val in cur.items():
                old = self._sen_prev.get(key)
                if old is None or old == val:
                    continue
                kind, idx = key[:-1], int(key[-1])
                name = (f"PRS {idx + 1}" if kind == "prs"
                        else f"ГИРО {idx + 1}")
                st, reason = val[0], val[1]
                old_st = old[0]
                if st == "rejected" and old_st != "rejected":
                    self._raise(f"ДАТЧИКИ: {name} отбракована ({reason})",
                                1)
                elif st == "failed" and old_st != "failed":
                    self._raise(f"ДАТЧИКИ: отказ {name} — нет данных", 1)
                elif st == "ok" and old_st in ("rejected", "failed",
                                               "deselected"):
                    self._raise(f"ДАТЧИКИ: {name} в работе", 0)
                if kind == "gyro" and idx == 0:
                    dis_new, dis_old = val[2], old[2]
                    if dis_new and not dis_old:
                        self._raise("ДАТЧИКИ: расхождение гирокомпасов "
                                    f"> {GYRO_DISAGREE_DEG:.0f}°", 1)
        self._sen_prev = cur
        # расхождение систем положения — фронт по любому каналу
        dis = any(ch.disagree for ch in s.prs)
        if dis and not self._prs_dis_prev:
            self._raise("ДАТЧИКИ: расхождение систем положения "
                        f"> {DEV_LIM:.1f} м — проверьте и снимите "
                        "неисправную", 1)
        self._prs_dis_prev = dis

    def degradation(self):
        """Снимок деградационного состояния для HMI."""
        c = self.con
        hdg_el = (c.t - self._gyro_lost_t
                  if self._gyro_lost_t is not None else 0.0)
        pos_el = (c.t - self._prs_lost_t
                  if self._prs_lost_t is not None else 0.0)
        # модель роста неопределённости при счислении положения
        excursion = min(60.0, 0.3 * pos_el ** 1.3) if pos_el > 0 else 0.0
        if self._released:
            state = "thrust_released"
        elif self._gyro_lost_t is not None:
            state = "heading_dr"
        elif self._prs_lost_t is not None:
            state = "position_dr"
        else:
            state = "normal"
        return {
            "state": state,
            "hdg_lost": self._gyro_lost_t is not None,
            "hdg_elapsed": round(hdg_el, 1),
            "release_in": max(0.0, self.gyro_release_s - hdg_el),
            "thrust_released": self._released,
            "pos_lost": self._prs_lost_t is not None,
            "pos_elapsed": round(pos_el, 1),
            "excursion_m": round(excursion, 1),
        }

    def _watch_limits(self):
        """Фронтовые аварии: ошибка позиции и consequence."""
        err = self.con.pos_err
        if not self._pos_armed and err <= self.warn_pos:
            self._pos_armed = True
        if self._pos_armed:
            state = (2 if err > self.alarm_pos
                     else 1 if err > self.warn_pos else 0)
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
                            "gyro": [bool(f) for f in c.sensors.gyro_failed],
                            "prs_detail": self._prs_view(c),
                            "gyro_detail": c.sensors.gyro_detail()},
                "conseq": {"level": c.conseq_level, "text": c.conseq_text},
                # состав контура и состояние анализа последствий —
                # замечание операторов «режим регулятора не виден»
                "ctrl": {"law": "PID + DOB", "est": "EKF",
                         "alloc": "VARIABLE", "ca_on": True},
                "capability": self._cap_snapshot(c),
                "degrad": self.degradation(),
                # ветер относительно судна: от носа / от правого борта
                "wind_rel": {
                    "bow_kn": round(c.wind.V * KN * float(np.cos(
                        c.wind.gamma - c.eta[2])), 1),
                    "stbd_kn": round(c.wind.V * KN * float(np.sin(
                        c.wind.gamma - c.eta[2])), 1),
                },
                "alarms": self.alarms[:12],
                "unacked": unacked,
                "alarm_worst": worst,
                "warn_pos": self.warn_pos,
                "alarm_pos": self.alarm_pos,
                "speed": self.speed,
                "paused": c.paused,
                "mode": c.mode,
                "joy": [round(float(v), 2) for v in c.joy],
            }

    # --------------------------------------------------------- команды
    def command(self, cmd):
        act = cmd.get("action")
        with self.lock:
            if act == "setpoint":
                self.con.set_setpoint(cmd["north"], cmd["east"])
            elif act == "setpoint_jog":
                # сдвиг заданной точки на (dn, de), м — кнопки N/E/S/W
                self.con.set_setpoint(self.con.eta_d[0] + float(cmd["dn"]),
                                      self.con.eta_d[1] + float(cmd["de"]))
            elif act == "heading":
                self.con.eta_d[2] = np.deg2rad(float(cmd["deg"]))
            elif act == "mode":
                m = cmd["mode"]
                self.con.set_mode(m)
                label = {"auto": "АВТО (удержание позиции)",
                         "man_joy": "MAN JOY — ручное управление",
                         "auto_joy": "AUTO JOY — ручное + автокурс"}[m]
                self._raise(f"РЕЖИМ: {label}", 0)
            elif act == "joystick":
                self.con.set_joystick(float(cmd.get("fx", 0.0)),
                                      float(cmd.get("fy", 0.0)),
                                      float(cmd.get("m", 0.0)))
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
                ftype = cmd.get("ftype", "fail")
                if kind == "prs":
                    self.con.sensors.fail_prs(idx, ftype)
                elif kind == "gyro":
                    self.con.sensors.fail_gyro(idx, ftype)
                else:
                    self.con.inject(kind, idx)
                if kind == "thruster":
                    self._raise(f"ИНСТРУКТОР: отказ ДРУ {idx + 1}", 1)
                elif ftype != "fail":
                    self._raise(f"ИНСТРУКТОР: сбой «{ftype}» "
                                f"{'PRS' if kind == 'prs' else 'ГИРО'} "
                                f"{idx + 1}", 1)
                # «нет данных» по датчикам поднимет _watch_sensors
            elif act == "recover":
                # возврат агрегата в работу (инструктор)
                kind, idx = cmd["kind"], int(cmd["idx"])
                if kind == "thruster":
                    self.con.thrusters[idx].failed = False
                    name = f"ДРУ {idx + 1}"
                elif kind == "prs":
                    self.con.sensors.recover_prs(idx)
                    name = f"PRS {idx + 1}"
                else:
                    self.con.sensors.recover_gyro(idx)
                    name = f"ГИРО {idx + 1}"
                self._raise(f"ИНСТРУКТОР: {name} возвращён в работу", 0)
            elif act == "sensor_deselect":
                # оператор снимает/возвращает систему (как в K-Pos Refsys)
                kind, idx = cmd["kind"], int(cmd["idx"])
                on = bool(cmd.get("on", True))
                if kind == "prs":
                    self.con.sensors.deselect_prs(idx, on)
                    name = f"PRS {idx + 1}"
                else:
                    self.con.sensors.deselect_gyro(idx, on)
                    name = f"ГИРО {idx + 1}"
                self._raise(f"ОПЕРАТОР: {name} "
                            + ("снята с вотирования" if on
                               else "возвращена в вотирование"), 0)
            elif act == "sensor_restore":
                # оператор возвращает отбракованную систему
                kind, idx = cmd["kind"], int(cmd["idx"])
                if kind == "prs":
                    self.con.sensors.restore_prs(idx)
                    name = f"PRS {idx + 1}"
                else:
                    self.con.sensors.restore_gyro(idx)
                    name = f"ГИРО {idx + 1}"
                self._raise(f"ОПЕРАТОР: {name} возвращена после "
                            "отбраковки", 0)
            elif act == "thrust_restore":
                # оператор вручную возвращает упор после снятия
                self._released = False
                self.con.thrust_released = False
                self._raise("Упор восстановлен оператором", 0)
            elif act == "speed":
                self.speed = float(cmd["value"])
            elif act == "pause":
                self.con.paused = not self.con.paused
            elif act == "reset":
                self.con._init_sim()
                self._limit_state = 0
                self._conseq_state = 0
                self._pos_armed = False
                self._gyro_lost_t = None
                self._prs_lost_t = None
                self._released = False
                self._sen_prev = None
                self._prs_dis_prev = False
                self._raise("Сброс симуляции", 0)
            elif act == "limits":
                self.warn_pos = float(cmd["warn"])
                self.alarm_pos = float(cmd["alarm"])
            elif act == "ack":
                for a in self.alarms:
                    a["acked"] = True
            elif act == "ack_one":
                # квитирование щелчком по конкретной аварии (MG 6.1.10)
                aid = int(cmd.get("id", 0))
                for a in self.alarms:
                    if a["id"] == aid:
                        a["acked"] = True
            return {"ok": True}
