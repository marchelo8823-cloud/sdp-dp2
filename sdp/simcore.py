"""Ядро симуляции DP-контура для пакетных испытаний (trials).

Вынесено из simulate.py: тот же контур (датчики → EKF → PID → QP-аллокация
→ судно), но параметризованный средой и расписанием отказов.
"""
import numpy as np

from .vessel import Vessel3DOF
from .environment import WaveForce, Wind
from .observer import DPObserver
from .controller import PIDController
from .allocation import AzimuthThruster, AzimuthAllocator
from .redundancy import SensorSuite


def default_thrusters():
    """Конфигурация DP-2: 2 носовых туннельных + 2 кормовых азимутальных.

    Привод по NavDP4000: азимут 12°/с, серво-ошибка ±1–2° (у каждой
    ДРУ своя постоянная), тяга PT1 3 с (0→50 % ≤ 10 с), реверс ≤ 15 с.
    """
    return [
        AzimuthThruster(lx=30.0, ly=0.0, t_max=1.5e5, az0_deg=90.0,
                        tunnel=True),
        AzimuthThruster(lx=18.0, ly=0.0, t_max=1.5e5, az0_deg=90.0,
                        tunnel=True),
        AzimuthThruster(lx=-30.0, ly=6.0, t_max=2.5e5, az0_deg=60.0,
                        az_rate_deg_s=12.0, servo_err_deg=-1.2,
                        forbidden_zones=[(-115.0, -65.0)]),
        AzimuthThruster(lx=-30.0, ly=-6.0, t_max=2.5e5, az0_deg=-60.0,
                        az_rate_deg_s=12.0, servo_err_deg=1.6,
                        forbidden_zones=[(65.0, 115.0)]),
    ]


def run(hs=2.0, tp=7.0, wave_heading=45.0, wind_speed=10.0, wind_dir=30.0,
        failures=(), t_end=400.0, dt=0.1,
        eta0=(-15.0, 10.0, 20.0), eta_d=(0.0, 0.0, 0.0), seed=42,
        pid_kw=None, observer_kw=None, ctrl="pid", mpc_kw=None,
        dob=False, dob_kw=None, wind_schedule=None,
        setpoint_schedule=None, bias_ff=False,
        mismatch=1.0, current_force=None,
        anem_lag=0.0, anem_noise=0.0):
    """Прогон контура с расписанием отказов.

    failures — iterable (t_fail, kind, idx):
        kind: "prs" | "gyro" | "thruster"
    ctrl: "pid" | "mpc". pid_kw / observer_kw / mpc_kw — настройки.
    dob=True — адаптивный RBF-наблюдатель возмущений поверх EKF.
    wind_schedule — iterable (t, speed, dir_deg): ступеньки ветра.
    setpoint_schedule — iterable (t, x, y, psi_deg): смена уставки.
    bias_ff=True — прямая компенсация оценки bias из EKF (классическая
        схема ДП: ветер + bias в feedforward, интеграл PID — остаток).
    mismatch — масштаб демпфирования «железа» против модели САУ
        (рассогласование модели, 1.0 = точная модель).
    current_force — постоянная сила (Fx, Fy, Mz) в NED, контроллеру
        неизвестна (течение/дрейф).
    anem_lag (с) / anem_noise (м/с) — инерционность и шум анемометра;
        при нулях feedforward получает истинный ветер.
    Возвращает dict с временными рядами.
    """
    vessel = Vessel3DOF()
    waves = WaveForce(hs=hs, tp=tp, heading_deg=wave_heading, seed=seed)
    wind = Wind(speed=wind_speed, dir_deg=wind_dir)
    wind_schedule = sorted(wind_schedule or ())
    setpoint_schedule = sorted(setpoint_schedule or ())
    if mismatch != 1.0:
        vessel.D = vessel.D * mismatch   # «железо» ≠ модель САУ
    # САУ работает по НОМИНАЛЬНОЙ модели (свои экземпляры).
    # bias_ff требует подвижной модели bias: иначе random walk слишком
    # жёсткий, оценка возмущения не сходится (типовая настройка ДП).
    observer_kw = dict(observer_kw or {})
    if bias_ff:
        observer_kw.setdefault("q_bias", (1e7, 1e7, 1e8))
    observer = DPObserver(Vessel3DOF(), dt=dt, omega0=2 * np.pi / tp,
                          **observer_kw)
    dob_obs = None
    if dob:
        from .disturbance import RBFDisturbanceObserver
        dob_obs = RBFDisturbanceObserver(**(dob_kw or {}))
    # Анемометр: измеренные скорость/направление (1-й порядок + шум)
    rng_anem = np.random.default_rng(seed + 7)
    V_m, g_m = wind.V, wind.gamma
    if ctrl == "mpc":
        from .mpc import MPCController
        controller = MPCController(Vessel3DOF(), **(mpc_kw or {}))
        mpc_every = max(1, int(controller.dt / dt))
        # Каскад: позиция (x, y) — от MPC, курс (yaw) — от PID.
        # Канал курса требует быстрого цикла 10 Гц против предела цикла
        # аллокатора (мертвая зона + slew rate) — промышленная практика.
        heading_pid = PIDController(**(pid_kw or {}))
    else:
        controller = PIDController(**(pid_kw or {}))
        mpc_every = 1
    thrusters = default_thrusters()
    allocator = AzimuthAllocator(thrusters)
    sensors = SensorSuite(seed=seed + 1)

    failures = sorted(failures)
    eta = np.array([eta0[0], eta0[1], np.deg2rad(eta0[2])])
    nu = np.zeros(3)
    eta_d = np.asarray(eta_d, float)
    eta_d[2] = np.deg2rad(eta_d[2])

    n = int(t_end / dt)
    nt = len(thrusters)
    log = {"t": np.arange(n) * dt,
           "eta": np.zeros((n, 3)), "eta_hat": np.zeros((n, 3)),
           "tau": np.zeros((n, 3)), "tau_alloc": np.zeros((n, 3)),
           "ff": np.zeros((n, 3)), "bias": np.zeros((n, 3)),
           "thrust": np.zeros((n, nt)), "azimuth": np.zeros((n, nt))}

    tau_cmd = np.zeros(3)
    for k in range(n):
        t = k * dt
        while failures and t >= failures[0][0] - dt / 2:
            _, kind, idx = failures.pop(0)
            if kind == "prs":
                sensors.fail_prs(idx)
            elif kind == "gyro":
                sensors.fail_gyro(idx)
            elif kind == "thruster":
                thrusters[idx].failed = True
            else:
                raise ValueError(f"неизвестный тип отказа: {kind}")
        while wind_schedule and t >= wind_schedule[0][0] - dt / 2:
            _, spd, d = wind_schedule.pop(0)
            wind = Wind(speed=spd, dir_deg=d)
        while setpoint_schedule and t >= setpoint_schedule[0][0] - dt / 2:
            _, xd, yd, psid = setpoint_schedule.pop(0)
            eta_d = np.array([xd, yd, np.deg2rad(psid)])

        tau_env = Vessel3DOF.R(eta[2]).T @ (waves.force(t) + wind.force())
        if current_force is not None:
            tau_env = tau_env + Vessel3DOF.R(eta[2]).T @ np.asarray(
                current_force, float)

        # Измерение ветра анемометром (инерция + шум)
        if anem_lag > 0.0 or anem_noise > 0.0:
            a = dt / max(anem_lag, dt)
            V_m += a * (wind.V - V_m)
            g_m += a * (wind.gamma - g_m)
            if anem_noise > 0.0:
                V_m += anem_noise * rng_anem.standard_normal() * np.sqrt(dt)
                g_m += (anem_noise / max(abs(wind.V), 1.0)
                        * rng_anem.standard_normal() * np.sqrt(dt))
            ff_wind = Wind(speed=max(V_m, 0.0),
                           dir_deg=np.rad2deg(g_m)).force()
        else:
            ff_wind = wind.force()

        # Структурированная часть возмущения (RBF), body-кадр
        dob_body = (dob_obs.estimate(observer.nu_hat)
                    if dob_obs is not None else 0.0)
        p = sensors.read_position(eta)
        h = sensors.read_heading(eta[2])
        if p is not None and h is not None:
            # EKF видит tau + RBF: его bias отслеживает только остаток
            observer.step(np.array([p[0], p[1], h]), tau_cmd + dob_body)
            if dob_obs is not None:
                R_h = Vessel3DOF.R(observer.eta_hat[2])
                dob_obs.adapt(observer.nu_hat,
                              R_h.T @ observer.last_dx[10:13], dt)
        else:
            observer.x = observer.x + observer._f(
                observer.x, tau_cmd + dob_body) * dt
            observer.last_dx[:] = 0.0

        # Feedforward: измеренный ветер + RBF-часть (NED) [+ bias из EKF]
        ff = ff_wind
        if dob_obs is not None:
            ff = ff + Vessel3DOF.R(observer.eta_hat[2]) @ dob_body
        if bias_ff:
            ff = ff + observer.bias_hat

        if ctrl == "mpc":
            if k % mpc_every == 0:
                # bias уже внутри ff, если bias_ff=True — не дублируем
                d_hat = ff if bias_ff else ff + observer.bias_hat
                tau_cmd = controller.compute(eta_d, observer.eta_hat,
                                             observer.nu_hat, d_hat)
            # Курс — от PID (быстрый цикл)
            tau_pid = heading_pid.compute(eta_d, observer.eta_hat,
                                          observer.nu_hat, ff, dt)
            tau_cmd[2] = tau_pid[2]
        else:
            tau_cmd = controller.compute(eta_d, observer.eta_hat,
                                         observer.nu_hat, ff, dt)
        u, alpha, tau_alloc = allocator.allocate(tau_cmd, dt)
        eta, nu = vessel.step(eta, nu, tau_alloc, tau_env, dt)

        log["eta"][k] = eta
        log["eta_hat"][k] = observer.eta_hat
        log["tau"][k] = tau_cmd
        log["tau_alloc"][k] = tau_alloc
        log["ff"][k] = ff
        log["bias"][k] = observer.bias_hat
        log["thrust"][k] = u
        log["azimuth"][k] = alpha

    return log


def evaluate(log, t_start, eta_d=(0.0, 0.0), ma_seconds=60.0, dt=0.1):
    """Метрики удержания позиции на окне [t_start, конец].

    pos_mean — средняя ошибка по сырому сигналу.
    pos_max / hdg_max — максимум по скользящему среднему ma_seconds:
    волновая (WF) составляющая не считается потерей позиции, оценивается
    низкочастотное движение — как на реальном DP-пульте.
    """
    mask = log["t"] >= t_start
    err = np.hypot(log["eta"][mask, 0] - eta_d[0],
                   log["eta"][mask, 1] - eta_d[1])
    hdg = np.abs(np.degrees(log["eta"][mask, 2]))
    w = max(1, int(ma_seconds / dt))
    ker = np.ones(w) / w
    err_lf = np.convolve(err, ker, mode="valid")
    hdg_lf = np.convolve(hdg, ker, mode="valid")
    return {"pos_mean": float(err.mean()),
            "pos_max": float(err_lf.max()),
            "hdg_max": float(hdg_lf.max())}
