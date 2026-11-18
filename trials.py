"""Пакет отказоустойчивых испытаний DP-2 (annual trials style).

Матрица единичных отказов по IMO MSC/Circ.645 / IMCA M 103 для класса DP-2:
каждый агрегат отказывает по одному; после отказа судно обязано удержать
позицию в заданном контуре. Вердикт — автоматический, по критериям:

    средняя ошибка позиции  <= 2.0 м
    макс. ошибка позиции    <= 5.0 м
    макс. ошибка курса      <= 3.0°

Окно оценки: отказ + 90 с (переходный процесс) до конца прогона.
Методика: Монте-Карло — N_RUNS реализаций шума на испытание, вердикт по
худшей; максимумы берутся по скользящему среднему 60 с (WF-фильтрация).
"""
import numpy as np

from sdp.simcore import run, evaluate

# pyplot импортируется внутри main(): модуль входит в цепочку импортов
# HIL-прогонов — на верхнем уровне pyplot не нужен (и его font-cache
# с файловой блокировкой мешает параллельным запускам).

CRITERIA = {"pos_mean": 2.0, "pos_max": 5.0, "hdg_max": 3.0}

T_FAIL = 120.0          # момент отказа, с
SETTLE = 90.0           # время на переходный процесс после отказа, с
T_END = 400.0

TRIALS = [
    {"id": "T-01", "name": "Отказ PRS №3",
     "env": {}, "failures": [(T_FAIL, "prs", 2)]},
    {"id": "T-02", "name": "Отказ кормового азимутального ДРУ 4",
     "env": {}, "failures": [(T_FAIL, "thruster", 3)]},
    {"id": "T-03", "name": "Отказ носового туннельного ДРУ 1",
     "env": {}, "failures": [(T_FAIL, "thruster", 0)]},
    {"id": "T-04", "name": "Отказ гирокомпаса №1",
     "env": {}, "failures": [(T_FAIL, "gyro", 0)]},
    {"id": "T-05", "name": "Отказ ДРУ 4 в усиленной среде (13 м/с, Hs=3 м)",
     "env": {"wind_speed": 13.0, "hs": 3.0},
     "failures": [(T_FAIL, "thruster", 3)]},
    {"id": "T-06", "name": "Отказ ДРУ 1 в усиленной среде (14 м/с, Hs=3 м)",
     "env": {"wind_speed": 14.0, "hs": 3.0},
     "failures": [(T_FAIL, "thruster", 0)]},
]


N_RUNS = 5              # реализаций шума на испытание (Монте-Карло)


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = []
    logs = []
    for i, trial in enumerate(TRIALS):
        env = {"hs": 2.0, "tp": 7.0, "wind_speed": 10.0, "wind_dir": 30.0,
               **trial["env"]}
        runs = []
        for j in range(N_RUNS):
            log = run(t_end=T_END, seed=100 + i * 10 + j,
                      failures=trial["failures"], **env)
            runs.append((log, evaluate(log, T_FAIL + SETTLE)))
        # Худшая реализация — по макс. ошибке позиции
        log_w, m = max(runs, key=lambda r: r[1]["pos_max"])
        ok = all(m[k] <= v for k, v in CRITERIA.items())
        rows.append((trial, m, ok))
        logs.append(log_w)
        print(f"{trial['id']}  {trial['name']:45s}  "
              f"ср {m['pos_mean']:5.2f} м  макс {m['pos_max']:5.2f} м  "
              f"курс {m['hdg_max']:4.1f}°  ->  {'PASS' if ok else 'FAIL'}")

    n_pass = sum(1 for _, _, ok in rows if ok)
    print(f"\nИТОГ: {n_pass}/{len(rows)} испытаний пройдено")

    # --- Сводный график ---
    fig, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
    fig.suptitle("DP-2 trials: ошибка позиции после единичного отказа "
                 f"(отказ @ {T_FAIL:.0f} с, оценка после "
                 f"{T_FAIL + SETTLE:.0f} с)")
    for ax, (trial, m, ok), log in zip(axes.flat, rows, logs):
        err = np.hypot(log["eta"][:, 0], log["eta"][:, 1])
        color = "tab:green" if ok else "tab:red"
        ax.plot(log["t"], err, color=color, lw=1)
        ax.axvline(T_FAIL, color="r", ls=":", alpha=0.6)
        ax.axvline(T_FAIL + SETTLE, color="k", ls=":", alpha=0.4)
        ax.axhline(CRITERIA["pos_max"], color="orange", ls="--", lw=1)
        ax.set_title(f"{trial['id']} {'PASS' if ok else 'FAIL'}: "
                     f"{trial['name']}", fontsize=9)
        ax.set_xlabel("t, с"); ax.set_ylabel("ошибка, м")
        ax.grid(True); ax.set_ylim(bottom=0)
    plt.tight_layout()
    plt.savefig("trials.png", dpi=150)
    print("График сохранён: trials.png")

    # --- Отчёт ---
    with open("trials_report.md", "w") as f:
        f.write("# DP-2 trials — отчёт об отказоустойчивых испытаниях\n\n")
        f.write(f"Отказ @ {T_FAIL:.0f} с, окно оценки "
                f"{T_FAIL + SETTLE:.0f}–{T_END:.0f} с. Критерии: "
                f"ср ≤ {CRITERIA['pos_mean']} м, макс ≤ {CRITERIA['pos_max']} м, "
                f"курс ≤ {CRITERIA['hdg_max']}°.\n\n"
                f"Методика: {N_RUNS} реализаций шума на испытание, в таблице — "
                f"худшая; максимумы по скользящему среднему 60 с "
                f"(WF-фильтрация).\n\n")
        f.write("| ID | Испытание | Ср. ошибка, м | Макс. ошибка, м | "
                "Макс. ошибка курса, ° | Вердикт |\n")
        f.write("|----|-----------|--------------:|----------------:|"
                "----------------------:|:-------:|\n")
        for trial, m, ok in rows:
            f.write(f"| {trial['id']} | {trial['name']} | "
                    f"{m['pos_mean']:.2f} | {m['pos_max']:.2f} | "
                    f"{m['hdg_max']:.1f} | {'**PASS**' if ok else '**FAIL**'} |\n")
        f.write(f"\n**ИТОГ: {n_pass}/{len(rows)} PASS**\n")
    print("Отчёт сохранён: trials_report.md")


if __name__ == "__main__":
    main()
