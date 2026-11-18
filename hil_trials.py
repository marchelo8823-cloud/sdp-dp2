"""HIL-прогон пакета испытаний DP-2.

Контур разделён на «железо» (судно + среда + датчики, цикл реального
времени) и «контроллер» (EKF + PID + QP-аллокация) с обменом по UDP.
Критерии и методика — те же, что в trials.py (вердикт по худшей
реализации, максимумы по скользящему среднему 60 с).

TIME_SCALE — ускорение относительно реального времени (для пакетных
прогонов); архитектура и тайминги цикла — как в реальном HIL-стенде.
"""
import time

import numpy as np

from sdp.hil import run_hil_trial
from sdp.simcore import evaluate
from trials import TRIALS, CRITERIA, T_FAIL, SETTLE, T_END

# ВНИМАНИЕ: matplotlib.pyplot импортируется внутри main() ПОСЛЕ прогонов —
# на верхнем уровне он не нужен и только тормозит импорт модуля.

N_RUNS = 3          # реализаций шума на испытание
TIME_SCALE = 20.0   # ускорение стенда (sleep-пейсинг, такт 5 мс wall)


def verdict(m):
    return (m["pos_mean"] <= CRITERIA["pos_mean"]
            and m["pos_max"] <= CRITERIA["pos_max"]
            and m["hdg_max"] <= CRITERIA["hdg_max"])


def main():
    t_start = time.perf_counter()
    rows = []
    scales = []
    for i, trial in enumerate(TRIALS):
        env = {"hs": 2.0, "tp": 7.0, "wind_speed": 10.0, "wind_dir": 30.0,
               **trial["env"]}
        runs = []
        for j in range(N_RUNS):
            log, stats = run_hil_trial(t_end=T_END, seed=100 + i * 10 + j,
                                       time_scale=TIME_SCALE,
                                       failures=trial["failures"], **env)
            runs.append(evaluate(log, T_FAIL + SETTLE))
            scales.append(stats["time_scale_achieved"])
            assert stats["ctrl_steps"] > 0.9 * len(log["t"]), \
                "контроллер потерял связь с железом"
        m_worst = max(runs, key=lambda m: m["pos_max"])
        ok = verdict(m_worst)
        rows.append((trial, m_worst, ok))
        print(f"{trial['id']} {trial['name']:38s} "
              f"ср {m_worst['pos_mean']:.2f} м  "
              f"макс {m_worst['pos_max']:.2f} м  "
              f"курс {m_worst['hdg_max']:.1f}°  "
              f"{'PASS' if ok else 'FAIL'}", flush=True)

    n_pass = sum(1 for _, _, ok in rows if ok)
    print(f"\nИТОГ HIL: {n_pass}/{len(rows)} PASS "
          f"(критерии: ср ≤ {CRITERIA['pos_mean']} м, "
          f"макс ≤ {CRITERIA['pos_max']} м, курс ≤ {CRITERIA['hdg_max']}°)")
    print(f"Ускорение стенда: задано {TIME_SCALE:.0f}x, "
          f"достигнуто {np.mean(scales):.1f}x; "
          f"весь пакет за {time.perf_counter() - t_start:.0f} с")

    # --- График ---
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    x = np.arange(len(rows))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    colors = ["#2e9e4f" if ok else "#c0392b" for _, _, ok in rows]
    ax1.bar(x, [m["pos_max"] for _, m, _ in rows], 0.6, color=colors)
    ax1.axhline(CRITERIA["pos_max"], color="r", ls="--", lw=1,
                label=f"критерий {CRITERIA['pos_max']} м")
    ax1.set_xticks(x)
    ax1.set_xticklabels([t["id"] for t, _, _ in rows])
    ax1.set_ylabel("макс. ошибка позиции, м")
    ax1.set_title("HIL: худшая реализация (окно после отказа)")
    ax1.grid(True, axis="y")
    ax1.legend()

    ax2.bar(x, [m["hdg_max"] for _, m, _ in rows], 0.6, color=colors)
    ax2.axhline(CRITERIA["hdg_max"], color="r", ls="--", lw=1,
                label=f"критерий {CRITERIA['hdg_max']}°")
    ax2.set_xticks(x)
    ax2.set_xticklabels([t["id"] for t, _, _ in rows])
    ax2.set_ylabel("макс. ошибка курса, °")
    ax2.set_title("HIL: курс (худшая реализация)")
    ax2.grid(True, axis="y")
    ax2.legend()
    plt.tight_layout()
    plt.savefig("hil_trials.png", dpi=150)
    print("График сохранён: hil_trials.png")


if __name__ == "__main__":
    main()
