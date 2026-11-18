"""Валидация оценки возмущений: bias feedforward и RBF-DOB.

Сценарий 1 (повторные переходы): (0,0) -> (10,-10) -> (0,0) при
рассогласовании модели (демпфирование железа x1.5), анемометре с
инерцией/шумом и неизвестном течении. Три конфигурации:
  A — база: feedforward = только измеренный ветер;
  B — +bias FF: feedforward += оценка bias из EKF (стандарт ДП);
  C — +bias FF + RBF-DOB: структурированная часть возмущения учится
      RBF-сетью по скорости, EKF отслеживает остаток.

Сценарий 2 (регресс): DP-2 trials для лучшей конфигурации.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sdp.simcore import run, evaluate
from trials import TRIALS, CRITERIA, T_FAIL, SETTLE, T_END

CURRENT = (1.5e4, 1.0e4, 0.0)
IMPERFECT = {"mismatch": 1.5, "current_force": CURRENT,
             "anem_lag": 8.0, "anem_noise": 0.3}
SP = [(0.0, 0.0, 0.0, 0.0), (200.0, 10.0, -10.0, 0.0), (450.0, 0.0, 0.0, 0.0)]
SEEDS = (42, 43, 44)
CONFIGS = [("A: база (ветер FF)", {}),
           ("B: +bias FF", {"bias_ff": True}),
           ("C: +bias FF + RBF-DOB", {"bias_ff": True, "dob": True})]


def sp_at(t):
    s = SP[0]
    for entry in SP:
        if t >= entry[0]:
            s = entry
    return np.array([s[1], s[2]])


def approach_windows():
    return [(240.0, 380.0), (490.0, 630.0)]


def case(kw, seed):
    log = run(t_end=700.0, seed=seed, setpoint_schedule=SP,
              **IMPERFECT, **kw)
    err = np.array([np.hypot(x - s[0], y - s[1])
                    for (x, y), s in zip(log["eta"][:, :2],
                                         [sp_at(t) for t in log["t"]])])
    means = []
    for a, b in approach_windows():
        mask = (log["t"] >= a) & (log["t"] <= b)
        means.append(float(err[mask].mean()))
    return log, err, means


def main():
    print("=== Переходы (0,0)→(10,−10)→(0,0), mismatch 1.5, анемометр, "
          "течение (среднее по 3 сидам) ===")
    res = {}
    for tag, kw in CONFIGS:
        m2, m3 = [], []
        for s in SEEDS:
            log, err, means = case(kw, s)
            m2.append(means[0])
            m3.append(means[1])
        res[tag] = (np.mean(m2), np.mean(m3), log, err)
        print(f"{tag:22s}  переход 1: ср {np.mean(m2):.2f} м   "
              f"переход 2: ср {np.mean(m3):.2f} м")

    # --- График ---
    fig, ax = plt.subplots(figsize=(11, 4.5))
    colors = {"A: база (ветер FF)": "tab:gray",
              "B: +bias FF": "tab:orange",
              "C: +bias FF + RBF-DOB": "tab:blue"}
    for tag in res:
        _, _, log, err = res[tag]
        ax.plot(log["t"], err, color=colors[tag], lw=0.9, label=tag)
    for sp in SP[1:]:
        ax.axvline(sp[0], color="r", ls=":", alpha=0.6)
    ax.set_xlabel("t, с")
    ax.set_ylabel("ошибка позиции, м")
    ax.set_title("Оценка возмущений: bias FF и RBF-DOB "
                 "(повторные переходы, mismatch 1.5)")
    ax.grid(True)
    ax.legend()
    plt.tight_layout()
    plt.savefig("compare_dob.png", dpi=150)
    print("График сохранён: compare_dob.png")

    # --- Сценарий 2: регресс на trials (конфигурация C) ---
    print("\n=== Регресс: DP-2 trials, конфигурация C "
          "(худшая из 3 реализаций, несовершенный мир) ===")
    print(f"{'ID':5s} {'база макс,м':>11s} {'C макс,м':>10s} "
          f"{'база курс':>9s} {'C курс':>9s} {'вердикт C':>11s}")
    for i, trial in enumerate(TRIALS):
        env = {"hs": 2.0, "tp": 7.0, "wind_speed": 10.0, "wind_dir": 30.0,
               **trial["env"]}
        row = {}
        for tag, kw in (("base", {}), ("C", CONFIGS[2][1])):
            runs = []
            for j in range(3):
                log = run(t_end=T_END, seed=200 + i * 10 + j,
                          failures=trial["failures"], **env, **IMPERFECT,
                          **kw)
                runs.append(evaluate(log, T_FAIL + SETTLE))
            row[tag] = max(runs, key=lambda m: m["pos_max"])
        md = row["C"]
        ok = (md["pos_mean"] <= CRITERIA["pos_mean"]
              and md["pos_max"] <= CRITERIA["pos_max"]
              and md["hdg_max"] <= CRITERIA["hdg_max"])
        print(f"{trial['id']:5s} {row['base']['pos_max']:11.2f} "
              f"{md['pos_max']:10.2f} {row['base']['hdg_max']:8.1f}° "
              f"{md['hdg_max']:8.1f}° {'PASS' if ok else 'FAIL':>11s}")


if __name__ == "__main__":
    main()
