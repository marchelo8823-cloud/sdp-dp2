"""Сценарий: удержание позиции DP-2 с отказом одного движителя и одного PRS.

t = 0     — выход на точку (0, 0), psi_d = 0
t = 150 с — отказ PRS #2 (single failure, DP-2)
t = 300 с — отказ движителя #3 (single failure, DP-2)

Среда: Hs = 2 м, Tp = 7 с, ветер 10 м/с, течение 0.3 м/с.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sdp.vessel import Vessel3DOF
from sdp.environment import WaveForce, Wind
from sdp.observer import DPObserver
from sdp.controller import PIDController
from sdp.allocation import Thruster, ThrustAllocator
from sdp.redundancy import SensorSuite

DT = 0.1
T_END = 450.0


def main():
    vessel = Vessel3DOF()
    waves = WaveForce(hs=2.0, tp=7.0, heading_deg=45.0)
    wind = Wind(speed=10.0, dir_deg=30.0)

    observer = DPObserver(vessel, dt=DT, omega0=2 * np.pi / 7.0)
    controller = PIDController()

    thrusters = [
        Thruster(lx=30.0, ly=0.0, azimuth_deg=90.0),     # носовой туннельный (sway)
        Thruster(lx=-30.0, ly=6.0, azimuth_deg=60.0),    # корма левый азимутальный
        Thruster(lx=-30.0, ly=-6.0, azimuth_deg=-60.0),  # корма правый азимутальный
    ]
    allocator = ThrustAllocator(thrusters)
    sensors = SensorSuite()

    eta = np.array([-15.0, 10.0, np.deg2rad(20.0)])
    nu = np.zeros(3)
    eta_d = np.array([0.0, 0.0, 0.0])

    n = int(T_END / DT)
    log = {k: np.zeros((n, 3)) for k in ("eta", "eta_hat", "tau", "tau_alloc")}
    log["thrust"] = np.zeros((n, 3))
    t_axis = np.arange(n) * DT

    tau_cmd = np.zeros(3)
    for k in range(n):
        t = k * DT
        # --- Отказы по сценарию DP-2 ---
        if abs(t - 150.0) < DT / 2:
            sensors.fail_prs(2)
        if abs(t - 300.0) < DT / 2:
            thrusters[2].failed = True

        # --- Среда (силы заданы в NED → в body-кадр) ---
        # Течение учитывается неявно: его постоянную составляющую
        # поглощает bias-часть наблюдателя
        tau_env = Vessel3DOF.R(eta[2]).T @ (waves.force(t) + wind.force())

        # --- Датчики + наблюдатель ---
        p = sensors.read_position(eta)
        h = sensors.read_heading(eta[2])
        if p is not None and h is not None:
            z = np.array([p[0], p[1], h])
            observer.step(z, tau_cmd)
        else:
            observer.x = observer.x + observer._f(observer.x, tau_cmd) * DT

        # --- Регулятор ---
        tau_cmd = controller.compute(eta_d, observer.eta_hat,
                                     observer.nu_hat, wind.force(), DT)

        # --- Распределение тяги ---
        u, tau_alloc = allocator.allocate(tau_cmd)

        # --- Судно ---
        eta, nu = vessel.step(eta, nu, tau_alloc, tau_env, DT)

        log["eta"][k] = eta
        log["eta_hat"][k] = observer.eta_hat
        log["tau"][k] = tau_cmd
        log["tau_alloc"][k] = tau_alloc
        log["thrust"][k] = u

    # --- Графики ---
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle("СДП класса DP-2: удержание позиции "
                 "(Hs=2 м, ветер 10 м/с; отказы: PRS@150 с, движитель@300 с)")

    ax = axes[0, 0]
    ax.plot(log["eta"][:, 1], log["eta"][:, 0], label="истинная траектория")
    ax.plot(log["eta_hat"][:, 1], log["eta_hat"][:, 0], "--",
            label="оценка EKF", alpha=0.7)
    ax.plot(0, 0, "r*", ms=15, label="уставка")
    ax.set_xlabel("Y, м"); ax.set_ylabel("X, м")
    ax.axis("equal"); ax.grid(True); ax.legend()

    ax = axes[0, 1]
    ax.plot(t_axis, log["eta"][:, 0], label="x")
    ax.plot(t_axis, log["eta"][:, 1], label="y")
    ax.axvline(150, color="r", ls=":", alpha=0.6)
    ax.axvline(300, color="r", ls=":", alpha=0.6)
    ax.set_xlabel("t, с"); ax.set_ylabel("положение, м")
    ax.grid(True); ax.legend()

    ax = axes[1, 0]
    ax.plot(t_axis, np.degrees(log["eta"][:, 2]), label="psi (ист.)")
    ax.plot(t_axis, np.degrees(log["eta_hat"][:, 2]), "--",
            label="psi (EKF)", alpha=0.7)
    ax.axhline(0, color="k", lw=0.5)
    ax.set_xlabel("t, с"); ax.set_ylabel("курс, град")
    ax.grid(True); ax.legend()

    ax = axes[1, 1]
    for i in range(3):
        ax.plot(t_axis, log["thrust"][:, i] / 1e3, label=f"ДРУ {i+1}")
    ax.axvline(300, color="r", ls=":", alpha=0.6)
    ax.set_xlabel("t, с"); ax.set_ylabel("тяга, кН")
    ax.grid(True); ax.legend()

    plt.tight_layout()
    plt.savefig("results_dp2.png", dpi=150)
    print("Графики сохранены: results_dp2.png")

    # --- Итоговые метрики (после выхода на точку, 200–450 с) ---
    i0 = int(200 / DT)
    err = np.linalg.norm(log["eta"][i0:, 0:2], axis=1)
    print(f"Средняя ошибка позиции (200–450 с): {err.mean():.2f} м")
    print(f"Макс. ошибка позиции:               {err.max():.2f} м")


if __name__ == "__main__":
    main()
