"""Сценарий: удержание позиции DP-2 с отказом одного движителя и одного PRS.

t = 0     — выход на точку (0, 0), psi_d = 0
t = 150 с — отказ PRS #2 (single failure, DP-2)
t = 300 с — отказ кормового правого движителя (single failure, DP-2)

Аллокация: динамическая, с QP-распределением, запрещёнными зонами
азимутов и ограничением скорости поворота. После отказа движителя
конфигурация перестраивается автоматически.

Consequence analysis (1 Гц): последствия «ещё одного» отказа —
загрузка оставшихся движителей и предельный ветер после худшего отказа.

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
from sdp.allocation import AzimuthThruster, AzimuthAllocator
from sdp.consequence import ConsequenceAnalyzer
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
        # Два носовых туннельных (резервирование носовой группы — DP-2):
        # азимут фиксирован (90°), реверсивная тяга
        AzimuthThruster(lx=30.0, ly=0.0, t_max=1.5e5, az0_deg=90.0,
                        tunnel=True),
        AzimuthThruster(lx=18.0, ly=0.0, t_max=1.5e5, az0_deg=90.0,
                        tunnel=True),
        # Кормовые азимутальные; запрещённые зоны — направление на соседний
        # движитель (струя не должна бить в соседний винт)
        AzimuthThruster(lx=-30.0, ly=6.0, t_max=2.5e5, az0_deg=60.0,
                        az_rate_deg_s=10.0, forbidden_zones=[(-115.0, -65.0)]),
        AzimuthThruster(lx=-30.0, ly=-6.0, t_max=2.5e5, az0_deg=-60.0,
                        az_rate_deg_s=10.0, forbidden_zones=[(65.0, 115.0)]),
    ]
    allocator = AzimuthAllocator(thrusters)
    sensors = SensorSuite()
    consequence = ConsequenceAnalyzer(thrusters, margin=0.1)

    eta = np.array([-15.0, 10.0, np.deg2rad(20.0)])
    nu = np.zeros(3)
    eta_d = np.array([0.0, 0.0, 0.0])

    n = int(T_END / DT)
    log = {k: np.zeros((n, 3)) for k in ("eta", "eta_hat", "tau", "tau_alloc")}
    NT = len(thrusters)
    log["thrust"] = np.zeros((n, NT))
    log["azimuth"] = np.zeros((n, NT))
    t_axis = np.arange(n) * DT

    tau_cmd = np.zeros(3)
    log["ca_util"] = np.full((n, NT), np.nan)
    log["ca_wind"] = np.full(n, np.nan)
    ca_res, ca_wind, prev_bad = {}, np.nan, set()
    for k in range(n):
        t = k * DT
        # --- Отказы по сценарию DP-2 ---
        if abs(t - 150.0) < DT / 2:
            sensors.fail_prs(2)
        if abs(t - 300.0) < DT / 2:
            thrusters[3].failed = True

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

        # --- Динамическое распределение тяги и азимутов ---
        u, alpha, tau_alloc = allocator.allocate(tau_cmd, DT)

        # --- Судно ---
        eta, nu = vessel.step(eta, nu, tau_alloc, tau_env, DT)

        # --- Consequence analysis (1 Гц): переживём ли ещё один отказ ---
        if k % 10 == 0:
            tau_load = Vessel3DOF.R(eta[2]).T @ (wind.force()
                                                 + observer.bias_hat)
            ca_res = consequence.check(tau_load)
            bad = {j for j, (ok, _) in ca_res.items() if not ok}
            for j in sorted(bad - prev_bad):
                print(f"t={t:6.1f} с  CONSEQUENCE ALARM: отказ ДРУ {j+1} "
                      f"приведёт к потере позиции")
            for j in sorted(prev_bad - bad):
                print(f"t={t:6.1f} с  CONSEQUENCE OK: отказ ДРУ {j+1} "
                      f"снова переносим")
            prev_bad = bad
            # Предельный ветер после худшего оставшегося отказа
            ca_wind = consequence.survivable_wind(30.0, hs=2.0)
        for j, (_, util) in ca_res.items():
            log["ca_util"][k, j] = util
        log["ca_wind"][k] = ca_wind

        log["eta"][k] = eta
        log["eta_hat"][k] = observer.eta_hat
        log["tau"][k] = tau_cmd
        log["tau_alloc"][k] = tau_alloc
        log["thrust"][k] = u
        log["azimuth"][k] = alpha

    # --- Графики ---
    fig, axes = plt.subplots(3, 2, figsize=(13, 12))
    fig.suptitle("СДП класса DP-2: динамическая азимутальная аллокация\n"
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
    for i in range(NT):
        ax.plot(t_axis, log["thrust"][:, i] / 1e3, label=f"ДРУ {i+1}")
    ax.axvline(300, color="r", ls=":", alpha=0.6)
    ax.set_xlabel("t, с"); ax.set_ylabel("тяга, кН")
    ax.grid(True); ax.legend()

    ax = axes[2, 0]
    az_plot = np.degrees(np.unwrap(log["azimuth"], axis=0))
    for i in range(NT):
        ax.plot(t_axis, az_plot[:, i], label=f"ДРУ {i+1}")
    # Запрещённые зоны
    for i, th in enumerate(thrusters):
        for a, b in th.forbidden_zones:
            ax.axhspan(np.degrees(a), np.degrees(b), alpha=0.15,
                       color=f"C{i}")
    ax.axvline(300, color="r", ls=":", alpha=0.6)
    ax.set_xlabel("t, с"); ax.set_ylabel("азимут, град")
    ax.grid(True); ax.legend(loc="lower right")
    ax.set_title("Азимуты (заштриховано — запрещённые зоны)")

    ax = axes[2, 1]
    tau_err = log["tau"] - log["tau_alloc"]
    ax.plot(t_axis, tau_err[:, 0] / 1e3, label="ΔFx")
    ax.plot(t_axis, tau_err[:, 1] / 1e3, label="ΔFy")
    ax.plot(t_axis, tau_err[:, 2] / 1e6, label="ΔN (МН·м)")
    ax.axvline(300, color="r", ls=":", alpha=0.6)
    ax.set_xlabel("t, с"); ax.set_ylabel("невязка аллокации, кН")
    ax.grid(True); ax.legend()
    ax.set_title("Невязка: запрошено vs достигнуто")

    plt.tight_layout()
    plt.savefig("results_dp2.png", dpi=150)
    print("Графики сохранены: results_dp2.png")

    # --- График consequence analysis ---
    fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    fig2.suptitle("Consequence analysis (запас к нагрузке 10 %)")

    for j in range(NT):
        ax1.plot(t_axis, log["ca_util"][:, j], label=f"отказ ДРУ {j+1}")
    ax1.axhline(1.0, color="r", lw=1.5, label="предел 100 %")
    ax1.axhline(0.8, color="orange", ls="--", lw=1, label="предупреждение 80 %")
    ax1.axvline(300, color="r", ls=":", alpha=0.6)
    ax1.set_ylabel("загрузка оставшихся, д.е.")
    ax1.set_title("Загрузка движителей при ещё одном единичном отказе")
    ax1.grid(True); ax1.legend(loc="center right")

    ax2.plot(t_axis, log["ca_wind"], "b-", lw=1.5,
             label="предельный ветер после худшего отказа")
    ax2.axhline(10.0, color="r", ls="--", lw=1.2,
                label="текущий ветер (10 м/с)")
    ax2.axvline(300, color="r", ls=":", alpha=0.6)
    ax2.set_xlabel("t, с"); ax2.set_ylabel("ветер, м/с")
    ax2.set_title("Запас по среде: предельный ветер при ещё одном отказе "
                  "(направление 30°, Hs = 2 м)")
    ax2.grid(True); ax2.legend(loc="center right")

    plt.tight_layout()
    plt.savefig("consequence.png", dpi=150)
    print("График сохранён: consequence.png")

    # --- Итоговые метрики ---
    i0 = int(200 / DT)
    err = np.linalg.norm(log["eta"][i0:, 0:2], axis=1)
    print(f"Средняя ошибка позиции (200–450 с): {err.mean():.2f} м")
    print(f"Макс. ошибка позиции:               {err.max():.2f} м")
    i_fail = int(310 / DT)
    err_f = np.linalg.norm(log["eta"][i_fail:, 0:2], axis=1)
    print(f"После отказа движителя (310–450 с): средняя {err_f.mean():.2f} м, "
          f"макс {err_f.max():.2f} м")


if __name__ == "__main__":
    main()
