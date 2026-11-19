"""Проигрывание лога voyage recorder: графики рейса + отметки событий.

Использование:
    python tools/replay.py logs/voyage_YYYYMMDD_HHMMSS.csv
                           [logs/events_YYYYMMDD_HHMMSS.csv]

Строит: ошибку позиции/курса, ветер, тягу движителей во времени;
события (аварии/команды) — вертикальными отметками.
Зависимости: только matplotlib (csv из stdlib).
"""
import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_rows(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def col(rows, name):
    return [float(r[name]) for r in rows]


def main():
    voyage = sys.argv[1]
    events = (sys.argv[2] if len(sys.argv) > 2
              else voyage.replace("voyage_", "events_"))
    v = read_rows(voyage)
    t = [x / 60.0 for x in col(v, "t")]

    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)

    axes[0].plot(t, col(v, "pos_err"), lw=1, color="#4d8dff",
                 label="ошибка позиции, м")
    axes[0].plot(t, col(v, "hdg_err_deg"), lw=1, color="#ff9f43",
                 label="ошибка курса, °")
    axes[0].axhline(3, color="#ffc542", ls="--", lw=0.8)
    axes[0].axhline(5, color="#ff5252", ls="--", lw=0.8)
    axes[0].set_ylabel("ошибка")
    axes[0].legend(loc="upper right")
    axes[0].grid(alpha=0.3)

    axes[1].plot(t, col(v, "wind_meas_kn"), lw=1, color="#39c2d7",
                 label="ветер изм, kn")
    axes[1].plot(t, col(v, "wind_true_kn"), lw=0.8, color="#5f7396",
                 label="ветер ист, kn")
    axes[1].set_ylabel("ветер, kn")
    axes[1].legend(loc="upper right")
    axes[1].grid(alpha=0.3)

    for i in range(1, 5):
        axes[2].plot(t, col(v, f"u{i}"), lw=1, label=f"ДРУ{i}, %")
    axes[2].set_ylabel("тяга, %")
    axes[2].legend(loc="upper right", ncol=4)
    axes[2].grid(alpha=0.3)

    axes[3].plot(t, col(v, "bias_kn"), lw=1, color="#2fd07c",
                 label="bias (LF), kN")
    axes[3].plot(t, col(v, "sog_kn"), lw=1, color="#ff9f43",
                 label="SOG, kn")
    axes[3].set_ylabel("bias / SOG")
    axes[3].set_xlabel("время рейса, мин")
    axes[3].legend(loc="upper right")
    axes[3].grid(alpha=0.3)

    try:
        e = read_rows(events)
        for r in e:
            colr = {"ALARM2": "#ff5252", "ALARM1": "#ffc542"}.get(
                r["kind"], "#5f7396")
            for ax in axes:
                ax.axvline(float(r["t"]) / 60.0, color=colr, lw=0.6,
                           alpha=0.5)
            axes[0].annotate(r["text"][:40],
                             (float(r["t"]) / 60.0,
                              axes[0].get_ylim()[1]),
                             rotation=90, fontsize=6, color=colr,
                             va="top")
    except OSError:
        print("журнал событий не найден — только телеметрия")

    out = voyage.rsplit(".", 1)[0] + "_replay.png"
    fig.suptitle("Voyage replay: " + voyage.split("/")[-1])
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print("график сохранён:", out)


if __name__ == "__main__":
    main()
