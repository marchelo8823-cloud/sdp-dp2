#!/usr/bin/env python3
"""Отчёт по записи СДП: журнал ядра (sdp_run --log, CSV) или запись NMEA (текстовый файл со строками $... / !...).

Запуск: python3 tools/sdp_report.py <файл> [--порог-м 3] [--порог-град 3] [-o отчёт.md]
Вид файла определяется по содержимому. Только стандартная библиотека Python 3.
"""
import argparse
import csv
import math
import re
import sys
from collections import Counter, defaultdict


def pct(vals, p):
    if not vals:
        return float("nan")
    s = sorted(vals)
    k = min(len(s) - 1, max(0, int(round(p / 100.0 * (len(s) - 1)))))
    return s[k]


def rms(vals):
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else float("nan")


def f(v, nd=2):
    return "—" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.{nd}f}".replace(".", ",")


class ReportError(Exception):
    pass


# ------------------------------------------------------------------ журнал ядра
def core_log(path, lim_m, lim_deg):
    need = ["t_s", "состояние", "режим", "ошибка_м", "ошибка_курса_град", "загрузка", "ветер_мс"]
    rows = []; skipped = 0
    with open(path, encoding="utf-8", errors="replace", newline="") as fh:
        rd = csv.DictReader(fh)
        miss = [k for k in need if k not in (rd.fieldnames or [])]
        if miss:
            raise ReportError("в журнале нет столбцов: " + ", ".join(miss) + ". Есть: " + ", ".join(rd.fieldnames or []) + ". Нужен журнал ядра 1.3 (sdp_run --log).")
        for r in rd:
            try:
                float(r["t_s"])
                rows.append(r)
            except (TypeError, ValueError):
                skipped += 1
    if not rows:
        return "# Отчёт по журналу ядра\n\n⚠ В журнале нет строк.\n"

    def num(r, k):
        try:
            return float(r[k])
        except (TypeError, ValueError, KeyError):
            return float("nan")
    t = [num(r, "t_s") for r in rows]
    dt = [t[i + 1] - t[i] for i in range(len(t) - 1)]
    step = pct(dt, 50) if dt else 0.1
    out = ["# Отчёт по журналу ядра", "", f"Файл: `{path}`", ""]
    out += ["## Общее", "", "| Параметр | Значение |", "|---|---|",
            f"| Длительность | {f(t[-1] - t[0], 0)} с |", f"| Строк | {len(rows)} |", f"| Шаг записи | {f(step, 2)} с |"]
    gaps = [(t[i], d) for i, d in enumerate(dt) if d > 5 * step]
    out.append(f"| Пропуски записи (больше 5 шагов) | {len(gaps)} |")
    out.append(f"| Испорченных строк (пропущены) | {skipped} |")
    out.append("")

    # состояния и режимы
    st = Counter(); md = Counter()
    for r in rows:
        st[r["состояние"]] += step
        md[r["режим"]] += step
    out += ["## Время по состояниям ядра", "", "| Состояние | с |", "|---|---|"]
    out += [f"| {k} | {f(v, 0)} |" for k, v in st.items()]
    out += ["", "## Время по режимам", "", "| Режим | с |", "|---|---|"]
    out += [f"| {k} | {f(v, 0)} |" for k, v in md.items()]
    out.append("")

    # удержание
    hold = [r for r in rows if r["состояние"].startswith("УДЕРЖАНИЕ")]
    out += ["## Удержание", ""]
    if not hold:
        out += ["Режима удержания в журнале нет.", ""]
    else:
        ok = lambda v: [x for x in v if not math.isnan(x)]
        ep = ok([num(r, "ошибка_м") for r in hold])
        eh = ok([abs(num(r, "ошибка_курса_град")) for r in hold])
        ld = ok([num(r, "загрузка") for r in hold])
        wd = ok([num(r, "ветер_мс") for r in hold])
        mx = lambda v: max(v) if v else float("nan")
        out += ["| Величина | Среднеквадр. | 95 % | Наибольшее |", "|---|---|---|---|",
                f"| Ошибка положения, м | {f(rms(ep))} | {f(pct(ep, 95))} | {f(mx(ep))} |",
                f"| Ошибка курса, ° | {f(rms(eh))} | {f(pct(eh, 95))} | {f(mx(eh))} |",
                f"| Загрузка движителей, доля | {f(rms(ld))} | {f(pct(ld, 95))} | {f(mx(ld))} |",
                f"| Ветер, м/с | {f(rms(wd), 1)} | {f(pct(wd, 95), 1)} | {f(mx(wd), 1)} |", ""]
        # эпизоды выхода за пороги
        eps = []
        cur = None
        for r in hold:
            bad = num(r, "ошибка_м") > lim_m or abs(num(r, "ошибка_курса_град")) > lim_deg
            tt = num(r, "t_s")
            if bad and cur is None:
                cur = [tt, tt, num(r, "ошибка_м"), abs(num(r, "ошибка_курса_град"))]
            elif bad:
                cur[1] = tt; cur[2] = max(cur[2], num(r, "ошибка_м")); cur[3] = max(cur[3], abs(num(r, "ошибка_курса_град")))
            elif cur is not None:
                eps.append(cur); cur = None
        if cur is not None:
            eps.append(cur)
        out += [f"### Выход за пороги ({f(lim_m, 1)} м или {f(lim_deg, 1)}°)", ""]
        if not eps:
            out += ["Не было.", ""]
        else:
            out += ["| Начало, с | Длительность, с | Наибольшая ошибка, м | Наибольшая ошибка курса, ° |", "|---|---|---|---|"]
            for e in eps[:50]:
                out.append(f"| {f(e[0], 1)} | {f(e[1] - e[0] + step, 1)} | {f(e[2])} | {f(e[3])} |")
            if len(eps) > 50:
                out.append(f"| … ещё {len(eps) - 50} | | | |")
            out.append("")
    if gaps:
        out += ["## ⚠ Пропуски записи", "", "| Время, с | Пропуск, с |", "|---|---|"]
        out += [f"| {f(g[0], 1)} | {f(g[1], 1)} |" for g in gaps[:30]]
        out.append("")
    return "\n".join(out) + "\n"


# ------------------------------------------------------------------ запись NMEA
NMEA_RE = re.compile(r"[$!]([A-Z]{2})([A-Z]{3}),")


def nmea_checksum_ok(s):
    m = re.search(r"[$!](.*)\*([0-9A-Fa-f]{2})\s*$", s)
    if not m:
        return None
    c = 0
    for ch in m.group(1):
        c ^= ord(ch)
    return c == int(m.group(2), 16)


def nmea_log(path):
    cnt = Counter(); bad = Counter(); nocs = Counter(); other = 0; total = 0
    first = {}
    fields = defaultdict(Counter)
    with open(path, encoding="latin-1", errors="replace") as fh:
        for i, line in enumerate(fh):
            total += 1
            j = max(line.find("$"), line.find("!")) if ("$" in line or "!" in line) else -1
            if j < 0:
                other += 1
                continue
            s = line[j:].strip()
            m = NMEA_RE.match(s)
            if not m:
                other += 1
                continue
            key = m.group(1) + m.group(2)
            cnt[key] += 1
            first.setdefault(key, i + 1)
            ok = nmea_checksum_ok(s)
            if ok is None:
                nocs[key] += 1
            elif not ok:
                bad[key] += 1
            fields[key][len(s.split(","))] += 1
    out = ["# Отчёт по записи NMEA", "", f"Файл: `{path}`", "", f"Строк: {total}, не NMEA: {other}.", "",
           "| Предложение | Источник | Тип | Строк | Ошибка суммы | Без суммы | Полей (чаще всего) | Первая строка |",
           "|---|---|---|---|---|---|---|---|"]
    for k, n in cnt.most_common():
        fc = fields[k].most_common(1)[0][0]
        out.append(f"| {k} | {k[:2]} | {k[2:]} | {n} | {bad[k]} | {nocs[k]} | {fc} | {first[k]} |")
    if sum(bad.values()):
        out += ["", f"⚠ Строк с неверной контрольной суммой: {sum(bad.values())}."]
    out += ["", "⚠ Время в записи NMEA этим отчётом не разбирается: частота посылок не оценивается."]
    return "\n".join(out) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("--порог-м", dest="lim_m", type=float, default=3.0)
    ap.add_argument("--порог-град", dest="lim_deg", type=float, default=3.0)
    ap.add_argument("-o", dest="out")
    a = ap.parse_args()
    try:
        with open(a.file, encoding="utf-8", errors="replace") as fh:
            head = fh.readline()
        h = head.lstrip("\ufeff").strip()
        is_csv = "," in h and not NMEA_RE.search(h) and "$" not in h and "!" not in h
        text = core_log(a.file, a.lim_m, a.lim_deg) if is_csv else nmea_log(a.file)
    except (OSError, ReportError) as e:
        sys.stderr.write(f"Ошибка: {e}\n")
        sys.exit(2)
    if a.out:
        with open(a.out, "w", encoding="utf-8") as fh:
            fh.write(text)
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
