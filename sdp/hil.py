"""HIL-режим: разделение DP-контура на «железо» и «контроллер».

Plant (симулятор оборудования): судно + среда + датчики + приводы ДРУ.
Работает по таймеру реального времени (с коэффициентом ускорения
time_scale для пакетных прогонов), шлёт телеметрию датчиков и принимает
команды на движители. Отказы (PRS, гиро, ДРУ) инжектируются здесь —
контроллер узнаёт о них только из телеметрии.

Controller: EKF + PID + QP-аллокатор. Ничего не знает о том, что на
другом конце UDP — симуляция: интерфейс тот же, что у реального железа
(измерения в, команды наружу).

Исполнение — два потока в одном процессе (spawn-процессы в среде
встроенного интерпретатора нестабильны). Чтобы GIL не блокировал цикл
управления, пейсинг железа — ТОЛЬКО через time.sleep (отпускает GIL);
busy-wait по perf_counter держал бы GIL весь такт, контроллер голодал,
и контур расходился — проверено экспериментально.

Обмен: JSON-дейтаграммы UDP по 127.0.0.1, телеметрия 10 Гц (sim-время).

Телеметрия (plant -> controller):
    {"t", "pos": [x, y] | None, "hdg": float | None,
     "wind": [Fx, Fy, Mz] (NED), "failed": [bool, ...], "done": bool}
Команда (controller -> plant):
    {"thrust": [Н, ...], "azimuth": [рад, ...]}
"""
import json
import queue
import socket
import threading
import time

import numpy as np

from .vessel import Vessel3DOF
from .environment import WaveForce, Wind
from .observer import DPObserver
from .controller import PIDController
from .allocation import AzimuthAllocator
from .redundancy import SensorSuite
from .simcore import default_thrusters

_PORT_COUNTER = [21000]


def _alloc_ports():
    """Пара портов на прогон (во избежание столкновений между запусками)."""
    _PORT_COUNTER[0] += 10
    return _PORT_COUNTER[0], _PORT_COUNTER[0] + 1


# ---------------------------------------------------------------------------
# Железо (свой цикл реального времени)
# ---------------------------------------------------------------------------

def _plant_main(hs, tp, wave_heading, wind_speed, wind_dir,
                failures, t_end, dt, time_scale, seed, ports, eta0,
                result_q):
    vessel = Vessel3DOF()
    waves = WaveForce(hs=hs, tp=tp, heading_deg=wave_heading, seed=seed)
    wind = Wind(speed=wind_speed, dir_deg=wind_dir)
    sensors = SensorSuite(seed=seed + 1)
    thr = default_thrusters()              # геометрия и статус ДРУ
    nt = len(thr)
    failures = sorted(failures)

    eta = np.array([eta0[0], eta0[1], np.deg2rad(eta0[2])])
    nu = np.zeros(3)

    # Приводы: последняя принятая команда
    cmd_u = np.zeros(nt)
    cmd_a = np.array([t.alpha for t in thr])

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", ports[0]))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("127.0.0.1", ports[1])

    n = int(t_end / dt)
    log_t = np.arange(n) * dt
    log_eta = np.zeros((n, 3))
    log_tau = np.zeros((n, 3))

    overruns = 0
    period = dt / time_scale
    t0 = time.perf_counter()
    for k in range(n):
        t = k * dt
        # Свежие команды от контроллера (берём последнюю)
        while True:
            try:
                data, _ = rx.recvfrom(65535)
            except BlockingIOError:
                break
            c = json.loads(data)
            cmd_u = np.asarray(c["thrust"], float)
            cmd_a = np.asarray(c["azimuth"], float)

        # Расписание отказов — только на стороне железа
        while failures and t >= failures[0][0] - dt / 2:
            _, kind, idx = failures.pop(0)
            if kind == "prs":
                sensors.fail_prs(idx)
            elif kind == "gyro":
                sensors.fail_gyro(idx)
            elif kind == "thruster":
                thr[idx].failed = True

        # Приводы ДРУ: команда -> силы (отказавший — нулевая тяга)
        tau = np.zeros(3)
        for i, th in enumerate(thr):
            if th.failed:
                continue
            T = float(np.clip(cmd_u[i], -th.t_max, th.t_max))
            a = cmd_a[i]
            fx, fy = T * np.cos(a), T * np.sin(a)
            tau += np.array([fx, fy, th.lx * fy - th.ly * fx])

        tau_env = Vessel3DOF.R(eta[2]).T @ (waves.force(t) + wind.force())
        eta, nu = vessel.step(eta, nu, tau, tau_env, dt)

        # Датчики -> телеметрия
        p = sensors.read_position(eta, t)
        h = sensors.read_heading(eta[2], t)
        msg = {"t": t,
               "pos": [float(p[0]), float(p[1])] if p is not None else None,
               "hdg": float(h) if h is not None else None,
               "wind": [float(v) for v in wind.force()],
               "failed": [bool(th.failed) for th in thr],
               "done": False}
        tx.sendto(json.dumps(msg).encode(), dest)

        log_eta[k] = eta
        log_tau[k] = tau

        # Пейсинг реального времени: ТОЛЬКО sleep (отпускает GIL —
        # контроллер работает в своём потоке без голодания)
        target = t0 + (k + 1) * period
        rem = target - time.perf_counter()
        if rem <= 0:
            overruns += 1
        else:
            time.sleep(rem)

    wall = time.perf_counter() - t0
    fin = {"t": t_end, "pos": None, "hdg": None, "wind": [0.0, 0.0, 0.0],
           "failed": [bool(th.failed) for th in thr], "done": True}
    for _ in range(3):                        # дубли на случай потери
        tx.sendto(json.dumps(fin).encode(), dest)
    rx.close()
    tx.close()
    result_q.put({"t": log_t, "eta": log_eta, "tau": log_tau,
                  "wall": wall, "overruns": overruns})


# ---------------------------------------------------------------------------
# Контроллер (свой цикл, видит только телеметрию)
# ---------------------------------------------------------------------------

def _ctrl_main(dt, tp, ports, result_q):
    vessel = Vessel3DOF()                    # модель для EKF
    observer = DPObserver(vessel, dt=dt, omega0=2 * np.pi / tp)
    pid = PIDController()
    thrusters = default_thrusters()
    alloc = AzimuthAllocator(thrusters)
    nt = len(thrusters)
    eta_d = np.zeros(3)
    tau_cmd = np.zeros(3)

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", ports[1]))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("127.0.0.1", ports[0])

    # Начальная команда: нулевая тяга, исходные азимуты
    tx.sendto(json.dumps(
        {"thrust": [0.0] * nt,
         "azimuth": [float(t.alpha) for t in thrusters]}).encode(), dest)

    steps = 0
    ctrl_updates = 0
    deadline = time.perf_counter() + 30.0
    spin = 0
    while True:
        # Забираем ВСЕ накопившиеся дейтаграммы пачкой: EKF шагаем по
        # КАЖДОЙ (временная ось наблюдателя совпадает с железом),
        # управление считаем один раз — по самой свежей. FIFO по одной
        # дейтаграмме накапливал бы задержку в буфере сокета (классика).
        msgs = []
        while True:
            try:
                data, _ = rx.recvfrom(65535)
            except BlockingIOError:
                break
            msgs.append(json.loads(data))
        if not msgs:
            if time.perf_counter() > deadline:
                break
            time.sleep(0.0005)          # отпускаем GIL между опросами
            continue
        deadline = time.perf_counter() + 30.0
        done = any(m.get("done") for m in msgs)
        tele = [m for m in msgs if not m.get("done")]
        # Статусы отказов и шаги EKF — по всем сообщениям пачки
        for m in tele:
            for i, f in enumerate(m["failed"]):
                thrusters[i].failed = f
            p, h = m["pos"], m["hdg"]
            if p is not None and h is not None:
                observer.step(np.array([p[0], p[1], h]), tau_cmd)
            else:
                observer.x = observer.x + observer._f(observer.x,
                                                      tau_cmd) * dt
            steps += 1
        if done:
            break
        if not tele:
            continue
        m = tele[-1]
        tau_cmd = pid.compute(eta_d, observer.eta_hat, observer.nu_hat,
                              np.asarray(m["wind"], float), dt)
        u, alpha, _ = alloc.allocate(tau_cmd, dt)
        tx.sendto(json.dumps(
            {"thrust": [float(v) for v in u],
             "azimuth": [float(v) for v in alpha]}).encode(), dest)
        ctrl_updates += 1

    rx.close()
    tx.close()
    result_q.put({"steps": steps, "ctrl_updates": ctrl_updates})


# ---------------------------------------------------------------------------
# Прогон одного испытания в HIL-режиме
# ---------------------------------------------------------------------------

def run_hil_trial(hs=2.0, tp=7.0, wave_heading=45.0, wind_speed=10.0,
                  wind_dir=30.0, failures=(), t_end=400.0, dt=0.1,
                  time_scale=80.0, seed=42, eta0=(-15.0, 10.0, 20.0)):
    """Один HIL-прогон. Возвращает (log, stats).

    log — временные ряды со стороны железа (истинная траектория);
    stats — {"wall": с, "time_scale_achieved": x, "overruns": n,
             "ctrl_steps": n, "ctrl_updates": n}.
    """
    ports = _alloc_ports()
    q_plant = queue.Queue()
    q_ctrl = queue.Queue()
    plant = threading.Thread(target=_plant_main,
                             args=(hs, tp, wave_heading, wind_speed,
                                   wind_dir, failures, t_end, dt,
                                   time_scale, seed, ports, eta0, q_plant),
                             daemon=True)
    ctrl = threading.Thread(target=_ctrl_main, args=(dt, tp, ports, q_ctrl),
                            daemon=True)
    ctrl.start()
    plant.start()
    plant.join(timeout=t_end / time_scale + 60.0)
    ctrl.join(timeout=15.0)
    try:
        res_p = q_plant.get(timeout=10.0)
    except queue.Empty:
        raise RuntimeError("HIL: железо не вернуло результат "
                           "(прогон превысил отведённое время)")
    try:
        res_c = q_ctrl.get(timeout=10.0)
    except queue.Empty:
        res_c = {}
    log = {"t": res_p["t"], "eta": res_p["eta"], "tau": res_p["tau"]}
    stats = {"wall": res_p["wall"],
             "time_scale_achieved": (t_end / res_p["wall"]
                                     if res_p["wall"] > 0 else 0.0),
             "overruns": res_p["overruns"],
             "ctrl_steps": res_c.get("steps", 0),
             "ctrl_updates": res_c.get("ctrl_updates", 0)}
    return log, stats
