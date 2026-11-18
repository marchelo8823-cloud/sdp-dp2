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
    """Конфигурация DP-2: 2 носовых туннельных + 2 кормовых азимутальных."""
    return [
        AzimuthThruster(lx=30.0, ly=0.0, t_max=1.5e5, az0_deg=90.0,
                        tunnel=True),
        AzimuthThruster(lx=18.0, ly=0.0, t_max=1.5e5, az0_deg=90.0,
                        tunnel=True),
        AzimuthThruster(lx=-30.0, ly=6.0, t_max=2.5e5, az0_deg=60.0,
                        az_rate_deg_s=10.0, forbidden_zones=[(-115.0, -65.0)]),
        AzimuthThruster(lx=-30.0, ly=-6.0, t_max=2.5e5, az0_deg=-60.0,
                        az_rate_deg_s=10.0, forbidden_zones=[(65.0, 115.0)]),
    ]


def run(hs=2.0, tp=7.0, wave_heading=45.0, wind_speed=10.0, wind_dir=30.0,
        failures=(), t_end=400.0, dt=0.1,
        eta0=(-15.0, 10.0, 20.0), eta_d=(0.0, 0.0, 0.0), seed=42,
        pid_kw=None, observer_kw=None):
    """Прогон контура с расписанием отказов.

    failures — iterable (t_fail, kind, idx):
        kind: "prs" | "gyro" | "thruster"
    pid_kw / observer_kw — переопределение настроек регулятора/наблюдателя.
    Возвращает dict с временными рядами.
    """
    vessel = Vessel3DOF()
    waves = WaveForce(hs=hs, tp=tp, heading_deg=wave_heading, seed=seed)
    wind = Wind(speed=wind_speed, dir_deg=wind_dir)
    observer = DPObserver(vessel, dt=dt, omega0=2 * np.pi / tp,
                          **(observer_kw or {}))
    controller = PIDController(**(pid_kw or {}))
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

        tau_env = Vessel3DOF.R(eta[2]).T @ (waves.force(t) + wind.force())

        p = sensors.read_position(eta)
        h = sensors.read_heading(eta[2])
        if p is not None and h is not None:
            observer.step(np.array([p[0], p[1], h]), tau_cmd)
        else:
            observer.x = observer.x + observer._f(observer.x, tau_cmd) * dt

        tau_cmd = controller.compute(eta_d, observer.eta_hat,
                                     observer.nu_hat, wind.force(), dt)
        u, alpha, tau_alloc = allocator.allocate(tau_cmd, dt)
        eta, nu = vessel.step(eta, nu, tau_alloc, tau_env, dt)

        log["eta"][k] = eta
        log["eta_hat"][k] = observer.eta_hat
        log["tau"][k] = tau_cmd
        log["tau_alloc"][k] = tau_alloc
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
