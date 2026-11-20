"""DP-пульт: интерактивная консоль оператора поверх контура управления.

Одно окно (matplotlib, без сторонних зависимостей):
  - карта позиции: судно, уставка, трасса, стрелки ветра и волнения;
  - панель состояния: ошибки позиции/курса, среда, оценка bias;
  - движители: тяга/азимут по каждой ДРУ, отказы;
  - consequence banner: «переносим ли ещё один отказ прямо сейчас»;
  - органы управления: клик по карте — новая уставка, слайдеры ветра,
    кнопки инъекции отказов (ДРУ 1–4, PRS, гирокомпас), курса ±10°,
    пауза/сброс, ускорение времени.

Контур — рабочая конфигурация C (PID + ветер FF + bias FF + RBF-DOB)
в «несовершенном мире»: рассогласование модели, неизвестное течение,
анемометр с инерцией и шумом.

Запуск GUI:        python console.py
Headless-проверка: python console.py --smoke 400
"""
import sys

import numpy as np

from sdp.vessel import Vessel3DOF
from sdp.environment import WaveForce, Wind
from sdp.observer import DPObserver
from sdp.controller import PIDController
from sdp.allocation import AzimuthAllocator
from sdp.redundancy import SensorSuite
from sdp.consequence import ConsequenceAnalyzer
from sdp.power import PowerPlant
from sdp.simcore import default_thrusters

DT = 0.1
SHIP_N = [38.0, 12.0, -30.0, -38.0, -38.0, -30.0, 12.0]   # обвод, нос +N
SHIP_E = [0.0, 9.0, 9.0, 4.0, -4.0, -9.0, -9.0]


class DPConsole:
    """Состояние контура + логика шага (без GUI — тестируется headless)."""

    def __init__(self, seed=42, hs=2.0, tp=7.0, wind_speed=10.0,
                 wind_dir=30.0, mismatch=1.3,
                 current_force=(1.0e4, 8.0e3, 0.0),
                 anem_lag=5.0, anem_noise=0.3, profile="4dru"):
        self.seed = seed
        self.profile = profile
        self.hs, self.tp = hs, tp
        self.mismatch = mismatch
        self.current_force = np.asarray(current_force, float)
        self.anem_lag, self.anem_noise = anem_lag, anem_noise
        self.wind_speed0, self.wind_dir0 = wind_speed, wind_dir
        self._init_sim()

    def _init_sim(self):
        self.vessel = Vessel3DOF()
        self.vessel.D = self.vessel.D * self.mismatch   # «железо» ≠ модель
        self.waves = WaveForce(hs=self.hs, tp=self.tp, heading_deg=45.0,
                               seed=self.seed)
        self.wind = Wind(speed=self.wind_speed0, dir_deg=self.wind_dir0)
        self.observer = DPObserver(Vessel3DOF(), dt=DT,
                                   omega0=2 * np.pi / self.tp,
                                   q_bias=(1e7, 1e7, 1e8))
        from sdp.disturbance import RBFDisturbanceObserver
        self.dob = RBFDisturbanceObserver()
        self.controller = PIDController()
        if self.profile == "4dru":
            self.thrusters = default_thrusters()
        else:
            from sdp.profiles import build_thrusters
            self.thrusters = build_thrusters(self.profile)
        self.allocator = AzimuthAllocator(self.thrusters)
        self.consequence = ConsequenceAnalyzer(self.thrusters)
        self.plant = PowerPlant(self.profile, self.thrusters)
        self.plant.step(DT)          # начальный баланс мощности
        self.sensors = SensorSuite(seed=self.seed + 1)
        self.rng_anem = np.random.default_rng(self.seed + 7)
        self.V_m, self.g_m = self.wind.V, self.wind.gamma

        self.t = 0.0
        self.eta = np.array([-15.0, 10.0, np.deg2rad(20.0)])
        self.nu = np.zeros(3)
        self.eta_d = np.zeros(3)
        self.tau_cmd = np.zeros(3)
        self.ff = np.zeros(3)
        self.paused = False
        self.thrust_released = False   # аварийное снятие упора (нет курса)
        # --- ручное управление (джойстик) ---
        self.mode = "auto"        # auto | man_joy | auto_joy
        self.joy = np.zeros(3)    # доли от шкалы: [продольная, боковая, момент]
        # шкала полного отклонения: ~60 % суммарной тяги; момент —
        # из расчёта ~1–2 °/с на умеренном отклонении (как реальный
        # джойстик DP: полный ход ≈ 60–90 °/мин установившегося поворота)
        f_sum = sum(th.t_max for th in self.thrusters)
        m_sum = sum(th.t_max * max(abs(th.lx), 1.0) for th in self.thrusters)
        self.joy_scale = np.array([0.6 * f_sum, 0.6 * f_sum, 0.012 * m_sum])
        self.trail = []           # (N, E) для трассы на карте
        self.conseq_text = "…"
        self.conseq_level = 0     # 0 ok, 1 warn, 2 alarm
        self.alarms = []

    # ------------------------------------------------------------------ sim
    def step(self):
        """Один такт контура (как simcore.run, конфигурация C)."""
        t = self.t
        R = Vessel3DOF.R(self.eta[2])
        tau_env = R.T @ (self.waves.force(t) + self.wind.force()
                         + self.current_force)

        # Анемометр: инерция + шум
        a = DT / max(self.anem_lag, DT)
        self.V_m += a * (self.wind.V - self.V_m)
        self.g_m += a * (self.wind.gamma - self.g_m)
        if self.anem_noise > 0.0:
            self.V_m += (self.anem_noise
                         * self.rng_anem.standard_normal() * np.sqrt(DT))
            self.g_m += (self.anem_noise / max(abs(self.wind.V), 1.0)
                         * self.rng_anem.standard_normal() * np.sqrt(DT))
        ff_wind = Wind(speed=max(self.V_m, 0.0),
                       dir_deg=np.rad2deg(self.g_m)).force()

        dob_body = self.dob.estimate(self.observer.nu_hat)
        p = self.sensors.read_position(self.eta, self.t)
        h = self.sensors.read_heading(self.eta[2], self.t)
        if p is not None and h is not None:
            self.observer.step(np.array([p[0], p[1], h]),
                               self.tau_cmd + dob_body)
            R_h = Vessel3DOF.R(self.observer.eta_hat[2])
            self.dob.adapt(self.observer.nu_hat,
                           R_h.T @ self.observer.last_dx[10:13], DT)
        else:
            self.observer.x = (self.observer.x
                               + self.observer._f(self.observer.x,
                                                  self.tau_cmd + dob_body) * DT)
            self.observer.last_dx[:] = 0.0
            self.observer.clamp_bias()

        # FF = измеренный ветер + RBF (NED) + bias EKF
        self.ff = (ff_wind
                   + Vessel3DOF.R(self.observer.eta_hat[2]) @ dob_body
                   + self.observer.bias_hat)
        # --- формирование команды по режиму ---
        if self.mode == "man_joy":
            # MAN JOY: чистое ручное управление, без ветровой компенсации;
            # интегратор PID не копит ошибку, пока автоматика отключена
            self.controller.integral[:] = 0.0
            self.tau_cmd = self.joy * self.joy_scale
        elif self.mode == "auto_joy":
            # AUTO JOY: руки задают продольную/боковую силу,
            # курс держит автоматика (PID только по рысканью)
            tau_pid = self.controller.compute(
                self.eta_d, self.observer.eta_hat, self.observer.nu_hat,
                self.ff, DT)
            self.tau_cmd = np.array([self.joy[0] * self.joy_scale[0],
                                     self.joy[1] * self.joy_scale[1],
                                     tau_pid[2]])
        else:
            self.tau_cmd = self.controller.compute(
                self.eta_d, self.observer.eta_hat, self.observer.nu_hat,
                self.ff, DT)
        if self.thrust_released and self.mode == "auto":
            # упор снят: команды движителям 0 %, судно дрейфует;
            # в ручных режимах джойстик доступен и после снятия упора —
            # это и есть его назначение (как MAN JOY в K-Pos)
            self.tau_cmd = np.zeros(3)
        u, alpha, tau_alloc = self.allocator.allocate(self.tau_cmd, DT)
        self.eta, self.nu = self.vessel.step(self.eta, self.nu,
                                             tau_alloc, tau_env, DT)
        self.plant.step(DT)          # баланс мощности → u_avail на след. шаг
        self.t += DT
        self.trail.append((self.eta[0], self.eta[1]))
        if len(self.trail) > 3000:
            self.trail.pop(0)

    # ------------------------------------------------------------- operator
    def set_wind(self, speed, dir_deg):
        self.wind = Wind(speed=float(speed), dir_deg=float(dir_deg))

    def set_hs(self, hs):
        """Изменить высоту волны на ходу (пересоздаём генератор волн)."""
        self.hs = float(hs)
        self.waves = WaveForce(hs=self.hs, tp=self.tp, heading_deg=45.0,
                               seed=self.seed)

    def set_setpoint(self, north, east):
        self.eta_d[0], self.eta_d[1] = float(north), float(east)

    def set_mode(self, mode):
        """Переключение режима: auto | man_joy | auto_joy."""
        assert mode in ("auto", "man_joy", "auto_joy")
        if mode != self.mode:
            self.controller.reset()
            if mode == "auto":
                self.joy[:] = 0.0
        self.mode = mode

    def set_joystick(self, fx, fy, m):
        """Отклонение джойстика, доли −1..1 (зажимается)."""
        self.joy = np.clip([fx, fy, m], -1.0, 1.0)

    def bump_heading(self, d_deg):
        self.eta_d[2] = (self.eta_d[2] + np.deg2rad(d_deg) + np.pi) \
            % (2 * np.pi) - np.pi

    def inject(self, kind, idx):
        if kind == "thruster" and not self.thrusters[idx].failed:
            self.thrusters[idx].failed = True
            self.alarms.append(f"t={self.t:6.1f}  ОТКАЗ ДРУ {idx + 1}")
        elif kind == "gen" and not self.plant.gens[idx].failed:
            self.plant.fail_gen(idx)
            self.alarms.append(
                f"t={self.t:6.1f}  ОТКАЗ {self.plant.gens[idx].name}")
        elif kind == "bus":
            name = ("A", "B")[idx]
            if not self.plant.bus_dead[name]:
                self.plant.fail_bus(name)
                self.alarms.append(f"t={self.t:6.1f}  ОТКАЗ ШИНЫ {name}")
        elif kind == "prs" and not self.sensors.prs_failed[idx]:
            self.sensors.fail_prs(idx)
            self.alarms.append(f"t={self.t:6.1f}  ОТКАЗ PRS {idx + 1}")
        elif kind == "gyro" and not self.sensors.gyro_failed[idx]:
            self.sensors.fail_gyro(idx)
            self.alarms.append(f"t={self.t:6.1f}  ОТКАЗ ГИРО {idx + 1}")
        self.alarms = self.alarms[-4:]

    def update_consequence(self):
        """Раз в ~5 с: переносим ли ещё один единичный отказ.

        Сценарии: отказ каждого оставшегося движителя + групповые
        отказы по питанию (при разомкнутой перемычке отказ шины
        снимает все её движители — критичный случай DP-2).
        """
        tau_load = Vessel3DOF.R(self.observer.eta_hat[2]).T @ self.ff
        res = self.consequence.check(tau_load)
        worst = max(res.items(), key=lambda kv: kv[1][1])
        bad = [f"ДРУ{j + 1}" for j, (ok, _) in res.items() if not ok]
        worst_u = worst[1][1]
        worst_name = f"ДРУ {worst[0] + 1}"
        # групповые отказы электростанции (шина при разомкнутой перемычке)
        tau = np.asarray(tau_load, float) * (1.0 + self.consequence.margin)
        for gi, grp in enumerate(self.plant.failure_groups()):
            active = [i for i, th in enumerate(self.thrusters)
                      if not th.failed and i not in grp]
            alloc = AzimuthAllocator(self.thrusters)
            ok, f = ConsequenceAnalyzer._feasible(alloc, tau, active)
            util = max((np.hypot(*f[i]) / self.thrusters[i].t_max
                        for i in active), default=1.0)
            name = "шина " + ("A", "B")[gi]
            if not ok:
                bad.append(name)
            if util > worst_u:
                worst_u, worst_name = util, name
        if bad:
            self.conseq_text = ("ALARM: отказ "
                                + ",".join(bad) + " не переносим")
            self.conseq_level = 2
        elif worst_u > 0.9:
            self.conseq_text = (f"ВНИМАНИЕ: после отказа {worst_name} "
                                f"загрузка {worst_u:.0%}")
            self.conseq_level = 1
        else:
            self.conseq_text = ("DP-2 ГОТОВ: ещё один отказ переносим "
                                f"(макс {worst_u:.0%})")
            self.conseq_level = 0

    @property
    def pos_err(self):
        return float(np.hypot(self.eta[0] - self.eta_d[0],
                              self.eta[1] - self.eta_d[1]))

    @property
    def hdg_err_deg(self):
        e = (self.eta[2] - self.eta_d[2] + np.pi) % (2 * np.pi) - np.pi
        return float(np.rad2deg(e))


# --------------------------------------------------------------------- GUI

def run_gui():
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Button, Slider
    from matplotlib.patches import Polygon, FancyArrowPatch

    con = DPConsole()
    speed = {"x": 5}

    fig = plt.figure(figsize=(13.5, 8.2))
    fig.canvas.manager.set_window_title("SDP-DP2 — Пульт оператора ДП")
    ax_map = fig.add_axes([0.03, 0.05, 0.50, 0.92])
    ax_map.set_aspect("equal")
    ax_map.set_xlim(-70, 70)
    ax_map.set_ylim(-70, 70)
    ax_map.set_xlabel("East, м")
    ax_map.set_ylabel("North, м")
    ax_map.grid(True, alpha=0.3)
    ax_map.set_title("Карта позиции (клик — новая уставка)")

    trail_ln, = ax_map.plot([], [], "-", color="tab:blue", lw=0.8,
                            alpha=0.6)
    ship = Polygon(np.zeros((len(SHIP_N), 2)), closed=True,
                   fc="tab:blue", ec="k", alpha=0.85)
    ax_map.add_patch(ship)
    sp_mk, = ax_map.plot([0], [0], "r+", ms=16, mew=2)
    wind_arr = FancyArrowPatch((0, 0), (0, 1), arrowstyle="-|>",
                               mutation_scale=18, color="tab:cyan", lw=2)
    ax_map.add_patch(wind_arr)
    lost_txt = ax_map.text(0, 60, "", color="r", fontsize=13, weight="bold",
                           ha="center")

    ax_st = fig.add_axes([0.56, 0.70, 0.42, 0.27])
    ax_st.axis("off")
    st_txt = ax_st.text(0.0, 1.0, "", va="top", family="monospace",
                        fontsize=10)
    ax_bn = fig.add_axes([0.56, 0.645, 0.42, 0.045])
    ax_bn.axis("off")
    bn_txt = ax_bn.text(0.5, 0.5, "", ha="center", va="center", fontsize=10,
                        weight="bold")

    ax_th = fig.add_axes([0.58, 0.42, 0.38, 0.20])
    ax_th.set_xlim(-1.15, 1.15)
    ax_th.set_ylim(-0.5, 3.5)
    ax_th.set_yticks(range(4), [f"ДРУ {i + 1}" for i in range(4)])
    ax_th.set_xlabel("тяга, доля от макс")
    ax_th.grid(True, axis="x", alpha=0.3)
    bars = ax_th.barh(range(4), np.zeros(4), height=0.55,
                      color=["tab:blue"] * 4)
    az_lbls = [ax_th.text(0, i, "", va="center", fontsize=8)
               for i in range(4)]
    ax_th.invert_yaxis()

    ax_wv = fig.add_axes([0.62, 0.345, 0.30, 0.02])
    ax_wd = fig.add_axes([0.62, 0.305, 0.30, 0.02])
    ax_sp = fig.add_axes([0.62, 0.265, 0.30, 0.02])
    sl_v = Slider(ax_wv, "ветер, м/с", 0.0, 20.0, valinit=10.0, valstep=0.5)
    sl_d = Slider(ax_wd, "напр. ветра, °", 0.0, 360.0, valinit=30.0,
                  valstep=5.0)
    sl_s = Slider(ax_sp, "ускорение ×", 1, 20, valinit=5, valstep=1)

    btns = {}
    labels1 = [("ДРУ1", ("thruster", 0)), ("ДРУ2", ("thruster", 1)),
               ("ДРУ3", ("thruster", 2)), ("ДРУ4", ("thruster", 3)),
               ("PRS", ("prs", 2)), ("ГИРО", ("gyro", 0))]
    for i, (lb, inj) in enumerate(labels1):
        b = Button(fig.add_axes([0.56 + i * 0.071, 0.20, 0.062, 0.035]), lb)
        b.on_clicked(lambda _e, inj=inj: con.inject(*inj))
        btns[lb] = b
    b_hm = Button(fig.add_axes([0.56, 0.15, 0.09, 0.035]), "Курс −10°")
    b_hm.on_clicked(lambda _e: con.bump_heading(-10))
    b_hp = Button(fig.add_axes([0.66, 0.15, 0.09, 0.035]), "Курс +10°")
    b_hp.on_clicked(lambda _e: con.bump_heading(10))
    b_ctr = Button(fig.add_axes([0.76, 0.15, 0.09, 0.035]), "Уставка (0,0)")
    b_ctr.on_clicked(lambda _e: con.set_setpoint(0.0, 0.0))
    b_pau = Button(fig.add_axes([0.56, 0.10, 0.09, 0.035]), "Пауза")
    b_rst = Button(fig.add_axes([0.66, 0.10, 0.09, 0.035]), "Сброс")

    def on_pause(_e):
        con.paused = not con.paused
        b_pau.label.set_text("Пуск" if con.paused else "Пауза")

    def on_reset(_e):
        con._init_sim()
        trail_ln.set_data([], [])
        b_pau.label.set_text("Пауза")

    b_pau.on_clicked(on_pause)
    b_rst.on_clicked(on_reset)

    def on_wind(_v):
        con.set_wind(sl_v.val, sl_d.val)

    sl_v.on_changed(on_wind)
    sl_d.on_changed(on_wind)

    def on_click(ev):
        if ev.inaxes is ax_map and ev.button == 1:
            con.set_setpoint(ev.ydata, ev.xdata)

    fig.canvas.mpl_connect("button_press_event", on_click)

    def ship_xy():
        psi = con.eta[2]
        c, s = np.cos(psi), np.sin(psi)
        pts = []
        for n_b, e_b in zip(SHIP_N, SHIP_E):
            n = c * n_b - s * e_b + con.eta[0]
            e = s * n_b + c * e_b + con.eta[1]
            pts.append((e, n))
        return pts

    level_color = {0: "#c8f7c5", 1: "#ffe9a8", 2: "#ffb3b3"}

    def frame(_i):
        if not con.paused:
            for _ in range(int(speed["x"])):
                con.step()
            if int(con.t / DT) % 50 == 0:
                con.update_consequence()
        # карта
        if con.trail:
            tr = np.array(con.trail)
            trail_ln.set_data(tr[::5, 1], tr[::5, 0])
        ship.set_xy(ship_xy())
        sp_mk.set_data([con.eta_d[1]], [con.eta_d[0]])
        g = con.wind.gamma
        wind_arr.set_positions((-60, 60),
                               (-60 + 12 * np.sin(g), 60 + 12 * np.cos(g)))
        lost_txt.set_text("ПОТЕРЯ ПОЗИЦИИ" if con.pos_err > 5.0 else "")
        # статус
        b = con.observer.bias_hat
        alm = "\n".join(con.alarms) if con.alarms else "—"
        st_txt.set_text(
            f"t = {con.t:7.1f} с        Hs = {con.hs:.1f} м, "
            f"Tp = {con.tp:.0f} с\n"
            f"ошибка позиции: {con.pos_err:6.2f} м\n"
            f"ошибка курса:   {con.hdg_err_deg:6.2f}° "
            f"(уст {np.rad2deg(con.eta_d[2]):5.1f}°)\n"
            f"ветер: {con.wind.V:4.1f} м/с, "
            f"{np.rad2deg(con.wind.gamma):5.1f}°\n"
            f"|bias| = {np.hypot(b[0], b[1]) / 1e3:5.1f} кН   "
            f"скорость: {np.hypot(*con.nu[:2]):4.2f} м/с\n"
            f"события:\n{alm}")
        bn_txt.set_text(con.conseq_text)
        ax_bn.set_facecolor(level_color[con.conseq_level])
        # движители
        for i, (bar, th) in enumerate(zip(bars, con.thrusters)):
            val = th.u / th.t_max
            bar.set_width(val)
            bar.set_color("tab:red" if th.failed else "tab:blue")
            az_lbls[i].set_text(
                "ОТКАЗ" if th.failed
                else f"α={np.rad2deg(th.alpha):6.1f}°  "
                     f"{abs(val):4.0%}")
            az_lbls[i].set_x(0.03 if val >= 0 else -1.12)
        speed["x"] = sl_s.val
        return []

    anim = FuncAnimation(fig, frame, interval=100, blit=False,
                         cache_frame_data=False)
    fig._sdp_anim = anim     # держим ссылку: иначе GC убьёт анимацию
    plt.show()
    return fig, frame, con


# ------------------------------------------------------------------ smoke

def smoke(n_steps=400):
    """Headless-проверка контура пульта: шаги, отказы, смена уставки."""
    con = DPConsole()
    for k in range(n_steps):
        con.step()
        if k == 150:
            con.inject("thruster", 3)
        if k == 250:
            con.inject("prs", 2)
            con.set_setpoint(10.0, -10.0)
            con.bump_heading(15)
            con.set_wind(15.0, 120.0)
        if k % 50 == 0:
            con.update_consequence()
    assert np.all(np.isfinite(con.eta)), "NaN в состоянии судна"
    print(f"smoke OK: t={con.t:.0f} с, ошибка позиции "
          f"{con.pos_err:.2f} м, курс {con.hdg_err_deg:.2f}°, "
          f"banner: {con.conseq_text}")
    print(f"события: {con.alarms}")


if __name__ == "__main__":
    if "--smoke" in sys.argv:
        i = sys.argv.index("--smoke")
        n = int(sys.argv[i + 1]) if i + 1 < len(sys.argv) else 400
        smoke(n)
    else:
        run_gui()
