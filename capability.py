"""DP capability analysis (статический метод, IMCA M 140 style).

Для каждого направления среды beta (0..360°) ищем максимальную скорость
ветра, при которой статические нагрузки уравновешиваются движителями:

    tau_env(beta, V) = wind(V, beta) + current(beta) + wave_drift(Hs, beta)

Проверка осуществимости — QP-аллокация (векторная параметризация):
tau_env достижим, если найдутся |f_i| <= t_max с A f = -tau_env.

Сценарии:
  1. intact            — все движители
  2. worst single fail — минимум по отказам каждого из движителей (DP-2)

Допущения статического метода:
  - курс судна фиксирован (psi = 0), судно не разворачивается на среду
  - азимуты свободны (динамика и запрещённые зоны не учитываются —
    оценка сверху; динамический запас проверяется в simulate.py)
  - ветер, течение и волновой дрейф коллинеарны (beta)
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sdp.allocation import AzimuthThruster, AzimuthAllocator

# --- Параметры нагрузок -----------------------------------------------------
RHO_AIR = 1.225
RHO_WATER = 1025.0

# Ветер (надводная часть)
AX, AY, AL = 120.0, 600.0, 60.0
CX, CY, CN = 0.5, 0.8, 0.4
# Течение (подводная часть), скорость 0.5 м/с при 10 м/с ветра
ACX, ACY = 25.0, 300.0
CCX, CCY = 0.4, 0.9
# Волновой дрейф ~ Hs² (эмпирический коэффициент, Н/м²)
C_DRIFT = 1.5e4


def env_load(beta_deg, v_wind, hs=3.0):
    """Статическая нагрузка среды в body-кадре (psi = 0)."""
    g = np.deg2rad(beta_deg)
    qw = 0.5 * RHO_AIR * v_wind**2
    vc = 0.05 * v_wind                      # ветровое течение ~5% от ветра
    qc = 0.5 * RHO_WATER * vc**2
    fd = C_DRIFT * hs**2
    return np.array([
        qw * AX * CX * np.cos(g) + qc * ACX * CCX * np.cos(g) + fd * np.cos(g),
        qw * AY * CY * np.sin(g) + qc * ACY * CCY * np.sin(g) + fd * np.sin(g),
        qw * AY * AL * CN * np.sin(g) + 0.1 * fd * 30.0 * np.sin(g),
    ])


# --- Осуществимость ----------------------------------------------------------
def make_allocator(failed_idx=None):
    # Конфигурация DP-2: 2 носовых туннельных + 2 кормовых азимутальных.
    # ВАЖНО: одиночный носовой туннельный — single point of failure:
    # при его отказе два кормовых азимутальных с одинаковым lx не могут
    # дать большую sway-силу без огромного момента yaw → capability ~0.
    thrusters = [
        AzimuthThruster(lx=30.0, ly=0.0, t_max=1.5e5, az0_deg=90.0,
                        tunnel=True),
        AzimuthThruster(lx=18.0, ly=0.0, t_max=1.5e5, az0_deg=90.0,
                        tunnel=True),
        AzimuthThruster(lx=-30.0, ly=6.0, t_max=2.5e5),
        AzimuthThruster(lx=-30.0, ly=-6.0, t_max=2.5e5),
    ]
    if failed_idx is not None:
        thrusters[failed_idx].failed = True
    return AzimuthAllocator(thrusters)


def feasible(alloc, tau, f_tol=1.0e3, n_tol=3.0e4):
    """True, если -tau достижимо активными движителями."""
    active = [i for i, th in enumerate(alloc.thrusters) if not th.failed]
    f = alloc._constrained_forces(-np.asarray(tau, float), active)
    res = -np.asarray(tau, float) - alloc._generalized(f)
    return (np.hypot(res[0], res[1]) < f_tol) and (abs(res[2]) < n_tol)


def max_wind(beta_deg, alloc, hs=3.0, v_hi=60.0, tol=0.1):
    """Бисекция по скорости ветра для направления beta."""
    lo, hi = 0.0, v_hi
    if feasible(alloc, env_load(beta_deg, hi, hs)):
        return hi
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if feasible(alloc, env_load(beta_deg, mid, hs)):
            lo = mid
        else:
            hi = mid
    return lo


def capability_polar(hs=3.0, n_beta=72):
    betas = np.linspace(0.0, 360.0, n_beta, endpoint=False)
    n_th = 4
    out = {"intact": np.zeros(n_beta)}
    for i in range(n_th):
        out[f"fail_T{i+1}"] = np.zeros(n_beta)
    for k, b in enumerate(betas):
        out["intact"][k] = max_wind(b, make_allocator(), hs)
        for i in range(n_th):
            out[f"fail_T{i+1}"][k] = max_wind(b, make_allocator(i), hs)
    worst = np.minimum.reduce([out[f"fail_T{i+1}"] for i in range(n_th)])
    out["worst_single_failure"] = worst
    return betas, out


def main():
    hs = 3.0
    betas, res = capability_polar(hs=hs)

    # --- Полярный график ---
    th = np.deg2rad(np.concatenate([betas, [betas[0]]]))
    fig = plt.figure(figsize=(9, 9))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)

    v = np.concatenate([res["intact"], [res["intact"][0]]])
    ax.plot(th, v, "b-", lw=2, label="все движители (intact)")
    v = np.concatenate([res["worst_single_failure"],
                        [res["worst_single_failure"][0]]])
    ax.plot(th, v, "r-", lw=2, label="худший единичный отказ (DP-2)")
    for i, ls in zip((1, 2, 3, 4), (":", "--", "-.", (0, (3, 1, 1, 1)))):
        v = np.concatenate([res[f"fail_T{i}"], [res[f"fail_T{i}"][0]]])
        ax.plot(th, v, linestyle=ls, lw=1, alpha=0.6,
                label=f"отказ ДРУ {i}")

    ax.set_title(f"DP capability: предельный ветер, Hs = {hs} м\n"
                 f"(курс фиксирован, статический метод)", pad=20)
    ax.set_rlabel_position(22.5)
    ax.annotate("ветер, м/с", xy=(0.02, 0.92), xycoords="axes fraction")
    ax.legend(loc="lower right", bbox_to_anchor=(1.15, -0.08))
    ax.grid(True)
    plt.tight_layout()
    plt.savefig("capability_dp2.png", dpi=150)
    print("График сохранён: capability_dp2.png")

    # --- Численное резюме ---
    for name in ("intact", "worst_single_failure",
                 "fail_T1", "fail_T2", "fail_T3", "fail_T4"):
        v = res[name]
        i_min = int(np.argmin(v))
        print(f"{name:24s}: min {v.min():5.1f} м/с "
              f"(при beta = {betas[i_min]:5.1f}°), mean {v.mean():5.1f} м/с")


if __name__ == "__main__":
    main()
