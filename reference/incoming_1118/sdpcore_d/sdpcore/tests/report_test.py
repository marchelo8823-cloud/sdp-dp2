#!/usr/bin/env python3
"""Проверка tools/sdp_report.py. Аргументы: <sdp_run> <корень проекта>."""
import os, subprocess, sys, tempfile
run, root = sys.argv[1], sys.argv[2]
rep = os.path.join(root, "tools", "sdp_report.py")
fails = 0
def check(c, m):
    global fails
    print(("  ok  " if c else "FAIL ") + m)
    fails += 0 if c else 1
with tempfile.TemporaryDirectory() as d:
    log = os.path.join(d, "журнал.csv")
    scen = os.path.join(d, "s.txt")
    open(scen, "w", encoding="utf-8").write("ship load data/Данные_судна_пример.xlsx\nsim start tests/scenarios/имитатор_быстро.xlsx\nwait 20\nthr_take all\nposition\nwait 200\n")
    subprocess.run([run, "--script", scen, "--log", log, "--quiet"], cwd=root, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=300)
    r = subprocess.run([sys.executable, rep, log], capture_output=True, text=True)
    check(r.returncode == 0 and "## Удержание" in r.stdout and "Ошибка положения, м" in r.stdout, "журнал ядра: отчёт с разделом удержания")
    # испорченная строка не роняет отчёт
    with open(log, "a", encoding="utf-8") as fh:
        fh.write("abc,УДЕРЖАНИЕ,удержание позиции,x,y\n")
    r = subprocess.run([sys.executable, rep, log], capture_output=True, text=True)
    check(r.returncode == 0 and "Испорченных строк (пропущены) | 1" in r.stdout, "испорченная строка пропущена и посчитана")
r = subprocess.run([sys.executable, rep, os.path.join(root, "tests/reports/nmea_пример.txt")], capture_output=True, text=True)
check(r.returncode == 0 and "| HEHDT | HE | HDT | 2 | 1 |" in r.stdout and "| WIMWV |" in r.stdout and "не NMEA: 1" in r.stdout, "NMEA: состав, неверная контрольная сумма, чужие строки")
r = subprocess.run([sys.executable, rep, os.path.join(root, "tests/reports/старый_журнал.csv")], capture_output=True, text=True)
check(r.returncode == 2 and "нет столбцов" in r.stderr and "Traceback" not in r.stderr, "журнал другого формата: понятное сообщение, без сбоя")
r = subprocess.run([sys.executable, rep, os.path.join(root, "нет_такого.csv")], capture_output=True, text=True)
check(r.returncode == 2 and "Traceback" not in r.stderr, "нет файла: понятное сообщение")
print("ВСЕ ПРОВЕРКИ ПРОЙДЕНЫ" if not fails else f"ОШИБОК: {fails}")
sys.exit(1 if fails else 0)
