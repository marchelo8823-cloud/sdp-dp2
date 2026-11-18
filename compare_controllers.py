"""Сравнение регуляторов PID и MPC на матрице испытаний DP-2.

MPC — каскадная схема: позиция (surge/sway) от MPC с предсказанием
по модели, курс (yaw) от PID на быстром цикле (промышленная практика:
канал курса требует полосы выше возможностей цикла аллокатора).

Методика: та же, что в trials.py — 5 реализаций шума, вердикт по худшей,
максимумы по скользящему среднему 60 с.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sdp.simcore import run, evaluate
from trials import TRIALS, CRITERIA, T_FAIL, SETTLE, T_END, N_RUNS

CONTROLLERS = ["pid", "mpc"]


def main():
    results = {c: [] for c in CONTROLLERS}
    for i, trial in enumerate(TRIALS):
        env = {"hs": 2.0, "tp": 7.0, "wind_speed": 10.0, "wind_dir": 30.0,
               **trial["env"]}
        for c in CONTROLLERS:
            runs = []
            for j in range(N_RUNS):
                log = run(t_end=T_END, seed=100 + i * 10 + j, ctrl=c,
                          failures=trial["failures"], **env)
                m = evaluate(log, T_FAIL + SETTLE)
                m0 = evaluate(log, 0.0)   # весь прогон (с выходом на точку)
                runs.append((m, m0))
            m_worst = max(runs, key=lambda r: r[0]["pos_max"])
            m0_mean = np.mean([r[1]["pos_mean"] for r in runs])
            results[c].append((trial, m_worst[0], m0_mean))
            print(f"{trial['id']} {c:4s}  после отказа: "
                  f"ср {m_worst[0]['pos_mean']:.2f} м "
                  f"макс {m_worst[0]['pos_max']:.2f} м "
                  f"курс {m_worst[0]['hdg_max']:.1f}° | "
                  f"весь прогон ср {m0_mean:.2f} м")

    # --- Сводная таблица ---
    print("\n=== Сравнение (худшая реализация, окно после отказа) ===")
    print(f"{'ID':5s} {'испытание':42s} {'PID макс,м':>10s} {'MPC макс,м':>10s}"
          f" {'PID курс':>9s} {'MPC курс':>9s}")
    for (trial, mp, _), (_, mm, _) in zip(results["pid"], results["mpc"]):
        print(f"{trial['id']:5s} {trial['name']:42s} "
              f"{mp['pos_max']:10.2f} {mm['pos_max']:10.2f} "
              f"{mp['hdg_max']:8.1f}° {mm['hdg_max']:8.1f}°")

    # --- График ---
    x = np.arange(len(TRIALS))
    w = 0.35
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    ax1.bar(x - w / 2, [m["pos_max"] for _, m, _ in results["pid"]],
            w, label="PID")
    ax1.bar(x + w / 2, [m["pos_max"] for _, m, _ in results["mpc"]],
            w, label="MPC+PID(yaw)")
    ax1.axhline(CRITERIA["pos_max"], color="r", ls="--", lw=1,
                label=f"критерий {CRITERIA['pos_max']} м")
    ax1.set_xticks(x); ax1.set_xticklabels([t["id"] for t, _, _ in results["pid"]])
    ax1.set_ylabel("макс. ошибка позиции, м")
    ax1.set_title("Худшая реализация (окно после отказа)")
    ax1.grid(True, axis="y"); ax1.legend()

    ax2.bar(x - w / 2, [s for _, _, s in results["pid"]], w, label="PID")
    ax2.bar(x + w / 2, [s for _, _, s in results["mpc"]], w,
            label="MPC+PID(yaw)")
    ax2.set_xticks(x); ax2.set_xticklabels([t["id"] for t, _, _ in results["pid"]])
    ax2.set_ylabel("средняя ошибка за весь прогон, м")
    ax2.set_title("Весь прогон (включая выход на точку)")
    ax2.grid(True, axis="y"); ax2.legend()
    plt.tight_layout()
    plt.savefig("compare_controllers.png", dpi=150)
    print("\nГрафик сохранён: compare_controllers.png")


if __name__ == "__main__":
    main()
