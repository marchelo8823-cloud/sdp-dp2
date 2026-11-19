# Создаёт книги Excel для ввода данных судна: пустой шаблон и заполненный пример.
# Запуск: python3 tools/make_template.py   (нужен пакет openpyxl). Ядру этот скрипт не нужен: он только делает файлы.
import sys
from openpyxl import Workbook
from openpyxl.styles import Font, PatternFill, Alignment, Border, Side
from openpyxl.utils import get_column_letter
from openpyxl.worksheet.datavalidation import DataValidation

F = Font(name="Arial", size=10); FB = Font(name="Arial", size=10, bold=True); FH = Font(name="Arial", size=10, bold=True, color="FFFFFF")
HEAD = PatternFill("solid", fgColor="1F4E79"); REQ = PatternFill("solid", fgColor="FFF2CC"); OPT = PatternFill("solid", fgColor="E2EFDA")
THIN = Side(style="thin", color="999999"); BOX = Border(left=THIN, right=THIN, top=THIN, bottom=THIN)
WRAP = Alignment(wrap_text=True, vertical="top")

HULL = [  # подпись, единица, обязательно, пояснение, пример
 ("Название судна", "", True, "Как называть судно в сообщениях", "Демонстрационное судно"),
 ("Длина по ватерлинии", "м", True, "Расчётная длина корпуса", 60),
 ("Ширина", "м", True, "Наибольшая ширина по ватерлинии", 14),
 ("Осадка", "м", True, "Средняя осадка в рабочем состоянии", 4),
 ("Водоизмещение", "т", True, "Масса судна в рабочем состоянии", 2200),
 ("Лобовая площадь парусности", "м²", True, "Проекция надводной части на плоскость мидель-шпангоута", 120),
 ("Боковая площадь парусности", "м²", True, "Проекция надводной части на диаметральную плоскость", 380),
 ("Широта района работ", "°", True, "Градусы с долями, северная +, южная −. Начало местных координат", 60.0),
 ("Долгота района работ", "°", True, "Градусы с долями, восточная +, западная −", 29.5),
 ("Длина между перпендикулярами", "м", False, "Если пусто — равна длине по ватерлинии", None),
 ("Центр парусности от миделя", "м", False, "+ к носу. Если пусто — на миделе", None),
 ("Поперечная метацентрическая высота", "м", False, "Если пусто — оценка ядра по размерам", None),
 ("Продольная метацентрическая высота", "м", False, "Если пусто — оценка ядра по размерам", None),
 ("Коэффициент полноты ватерлинии", "", False, "Если пусто — 0,8", None),
 ("Коэффициент ветра продольный Cx", "", False, "По продувкам или расчёту; если пусто — значение ядра", 0.7),
 ("Коэффициент ветра поперечный Cy", "", False, "То же", 0.9),
 ("Коэффициент момента ветра Cn", "", False, "То же", 0.12),
 ("Боковая площадь подводной части", "м²", False, "Если пусто — 0,85 × длина × осадка", None),
 ("Уход за 5 мин без положения", "м", False, "Для оценки «вероятный уход» при счислении; пусто — 14 (по имитатору). Уточняется на испытаниях", None),
]
THR_COLS = [  # заголовок, обязательно, ширина, пояснение
 ("Обозначение", True, 14, "До 16 знаков, без пробелов"), ("Тип", True, 15, "туннельный | поворотный | неповоротный"), ("В работе", False, 10, "да | нет; пусто — да"),
 ("X от миделя, м", True, 13, "+ к носу"), ("Y от ДП, м", True, 11, "+ на правый борт"), ("Упор вперёд, кН", True, 13, "Наибольший упор"), ("Упор назад, кН", True, 13, "Для туннельного — на другой борт"),
 ("Мощность, кВт", True, 12, "При наибольшем упоре"), ("Угол установки, °", False, 13, "Только для неповоротного (обязательно): 0 — вдоль судна к носу"),
 ("Темп упора, %/с", False, 12, "Пусто — 25"), ("Скорость поворота, °/с", False, 14, "Для поворотного; пусто — 10"), ("Постоянная времени, с", False, 13, "Пусто — 2"),
 ("Предел нагрузки, %", False, 12, "Пусто — 100"), ("Запретные сектора, °", False, 18, "Для поворотного: «от-до; от-до», направление силы на судно"),
 ("Порог расхождения, %", False, 13, "Уставка и факт; пусто — 25"), ("Порог по углу, °", False, 12, "Пусто — 30"), ("Выдержка расхождения, с", False, 14, "Пусто — 30"),
 ("Предпочтительный угол, °", False, 14, "Для поворотного: встречный упор (bias) — угол постоянного упора; у пары колонок — навстречу друг другу, например 90 и 270; пусто — нет"),
 ("Встречный упор, %", False, 12, "Для поворотного, вместе с углом: 5–50 %; пусто — нет"),
 ("Группа отказа", False, 16, "Анализ последствий: движители одной группы отказывают вместе (ГРЩ, машинное отделение); несколько — через «;»; пусто у всех — одиночные отказы"),
]
THR_EX = [("AZ1", "поворотный", "да", -22, -4, 180, 150, 700, None, None, None, None, None, None, None, None, None, None, None, "ГРЩ-1"),
          ("AZ2", "поворотный", "да", -22, 4, 180, 150, 700, None, None, None, None, None, None, None, None, None, None, None, "ГРЩ-2"),
          ("BOW1", "туннельный", "да", 22, 0, 100, 100, 400, None, None, None, None, None, None, None, None, None, None, None, "ГРЩ-1"),
          ("BOW2", "туннельный", "да", 18, 0, 100, 100, 400, None, None, None, None, None, None, None, None, None, None, None, "ГРЩ-2")]
SEN_COLS = [("Обозначение", True, 14, "Например GNSS1"), ("Вид", True, 14, "спутниковый | гирокомпас | ветер | качка | лазер | трос"),
 ("X от миделя, м", False, 13, "+ к носу; обязательно для систем положения"), ("Y от ДП, м", False, 11, "+ на правый борт"), ("Z вниз от палубы, м", False, 14, "Вниз +, антенна на мачте — отрицательное"),
 ("СКО", False, 9, "Погрешность: м (положение), ° (курс), м/с (ветер). Обязательно для систем положения"),
 ("Подключение", False, 13, "udp | tcp | serial — заполняется при работе от датчиков"), ("Адрес", False, 15, "Для tcp"), ("Порт", False, 9, "Для udp и tcp"),
 ("Устройство", False, 14, "Для serial: COM3, /dev/ttyUSB0"), ("Скорость, бод", False, 12, "Для serial")]
SEN_EX = [("GNSS1", "спутниковый", 20, 3, -15, 0.5), ("GNSS2", "спутниковый", -18, -3, -15, 0.6), ("GYRO1", "гирокомпас"), ("GYRO2", "гирокомпас"), ("GYRO3", "гирокомпас"),
          ("WIND1", "ветер"), ("WIND2", "ветер"), ("MRU1", "качка"), ("MRU2", "качка"), ("LASER", "лазер", 22, 0, -8, 0.3)]
OUT_KV = [("Подключение", "", "tcp | serial. Если лист не заполнен, команды никуда не передаются"), ("Адрес", "", "Для tcp"), ("Порт", "", "Для tcp"), ("Устройство", "", "Для serial"),
          ("Скорость, бод", "", "Для serial"), ("Номер устройства Modbus", "", "Адрес контроллера"), ("Период передачи, мс", "мс", "Пусто — 100"),
          ("Регистр счётчика «жив»", "", "Необязательно"), ("Регистр режима", "", "Необязательно")]
OUT_COLS = [("Движитель", True, 14, "Обозначение с листа «Движители»"), ("Величина", True, 11, "упор | угол"), ("Адрес регистра", True, 13, ""), ("Масштаб", True, 10, "Единиц регистра на 1 % или на 1°"),
            ("Со знаком", False, 10, "да | нет; пусто — да"), ("Наибольшее значение", False, 14, "Предел значения регистра")]
SIM = [("Ветер, м/с", "м/с", True, "Средняя скорость истинного ветра, 0…60", 10), ("Ветер откуда, °", "°", True, "Откуда дует, истинное", 60),
       ("Течение, м/с", "м/с", True, "0…5", 0.5), ("Течение куда, °", "°", True, "Куда направлено, истинное", 200),
       ("Высота волны, м", "м", True, "Значительная высота, 0…15", 1.5), ("Период волны, с", "с", True, "2…20", 7), ("Волна откуда, °", "°", True, "Откуда идёт волна", 60),
       ("Начальный курс, °", "°", True, "Курс судна при запуске имитатора", 10),
       ("Порывистость", "", False, "0 — ровная среда, 1 — обычные порывы и колебания течения, 2 — сильные. Пусто — 1", None),
       ("Ускорение", "", False, "Во сколько раз быстрее реального времени, 0,1…100. Пусто — 1", None),
       ("Начальное число случайных", "", False, "Одинаковое число — одинаковый шум датчиков при повторе. Пусто — 11", None)]

def style(ws):
    for row in ws.iter_rows():
        for c in row:
            if c.font == Font() or c.font.name != "Arial": c.font = F if not c.font.bold else c.font
            c.alignment = WRAP

def keyval(ws, rows, fill_example, start=1, title=None):
    r = start
    for i, h in enumerate(["Параметр", "Значение", "Единица", "Обязательно", "Пояснение"], 1):
        c = ws.cell(row=r, column=i, value=h); c.font = FH; c.fill = HEAD; c.border = BOX
    for lab, unit, req, note, ex in rows:
        r += 1
        ws.cell(row=r, column=1, value=lab).font = FB
        v = ws.cell(row=r, column=2, value=(ex if fill_example else None)); v.fill = REQ if req else OPT
        ws.cell(row=r, column=3, value=unit); ws.cell(row=r, column=4, value="да" if req else "нет"); ws.cell(row=r, column=5, value=note)
        for i in range(1, 6): ws.cell(row=r, column=i).border = BOX
    for i, w in enumerate([40, 26, 10, 13, 70], 1): ws.column_dimensions[get_column_letter(i)].width = w
    return r

def table(ws, cols, rows, start, nrows=12):
    for i, (h, req, w, note) in enumerate(cols, 1):
        c = ws.cell(row=start, column=i, value=h); c.font = FH; c.fill = HEAD; c.border = BOX
        ws.column_dimensions[get_column_letter(i)].width = max(w, ws.column_dimensions[get_column_letter(i)].width or 0)
    for k in range(nrows):
        for i, (h, req, w, note) in enumerate(cols, 1):
            c = ws.cell(row=start + 1 + k, column=i, value=(rows[k][i - 1] if k < len(rows) and i - 1 < len(rows[k]) else None)); c.fill = REQ if req else OPT; c.border = BOX
    r = start + nrows + 2
    ws.cell(row=r, column=1, value="Пояснения к столбцам (эту часть ядро не читает):").font = FB
    for h, req, w, note in cols:
        r += 1; ws.cell(row=r, column=2, value=h).font = FB; ws.cell(row=r, column=3, value="обязательно" if req else "необязательно"); ws.cell(row=r, column=4, value=note)

def listcheck(ws, rng, items):
    dv = DataValidation(type="list", formula1='"' + ",".join(items) + '"', allow_blank=True); dv.error = "Выберите значение из списка"; ws.add_data_validation(dv); dv.add(rng)

def build(path, ex):
    wb = Workbook()
    ws = wb.active; ws.title = "Читать первым"
    lines = ["Данные судна для ядра СДП", "",
             "Заполняются листы «Корпус», «Движители», «Датчики»; лист «Выдача» — когда есть контроллер движителей.",
             "Чтобы датчик давал данные с судна, заполните у него «Подключение». Без подключения датчик используется только имитатором.",
             "Жёлтые ячейки обязательны: пока хоть одна пуста, ядро не запускается и называет лист и строку.",
             "Зелёные ячейки необязательны: если пусто, ядро берёт значение по умолчанию и сообщает об этом в отчёте.",
             "Подписи в столбце A и заголовки таблиц не меняйте: ядро находит данные по ним.",
             "Десятичный знак — запятая или точка. Единицы указаны рядом с каждой величиной.",
             "Оси судна: X — вдоль, + к носу, от миделя; Y — поперёк, + на правый борт, от диаметральной плоскости; Z — вниз.",
             "Углы — в градусах по часовой стрелке, 0 — к носу. Направления ветра и волны — откуда, течения — куда.",
             "", "Проверка без запуска ядра:  sdp_run --check Данные_судна.xlsx", "Загрузка в работающее ядро:  ship load Данные_судна.xlsx",
             "Данные для имитатора — в отдельной книге «Данные_имитатора»: sim start Данные_имитатора.xlsx"]
    for i, t in enumerate(lines, 1):
        c = ws.cell(row=i, column=1, value=t); c.font = Font(name="Arial", size=12 if i == 1 else 10, bold=(i == 1))
    ws.column_dimensions["A"].width = 120
    ws = wb.create_sheet("Корпус"); keyval(ws, HULL, ex)
    ws = wb.create_sheet("Движители"); table(ws, THR_COLS, THR_EX if ex else [], 1, 12)
    listcheck(ws, "B2:B13", ["туннельный", "поворотный", "неповоротный"]); listcheck(ws, "C2:C13", ["да", "нет"])
    ws = wb.create_sheet("Датчики"); table(ws, SEN_COLS, SEN_EX if ex else [], 1, 16)
    listcheck(ws, "B2:B17", ["спутниковый", "гирокомпас", "ветер", "качка", "лазер", "трос"]); listcheck(ws, "G2:G17", ["udp", "tcp", "serial"])
    ws = wb.create_sheet("Выдача")
    r = 1
    for i, h in enumerate(["Параметр", "Значение", "Единица", "Пояснение"], 1):
        c = ws.cell(row=r, column=i, value=h); c.font = FH; c.fill = HEAD; c.border = BOX
    for lab, unit, note in OUT_KV:
        r += 1; ws.cell(row=r, column=1, value=lab).font = FB; ws.cell(row=r, column=2).fill = OPT; ws.cell(row=r, column=3, value=unit); ws.cell(row=r, column=4, value=note)
        for i in range(1, 5): ws.cell(row=r, column=i).border = BOX
    for i, w in enumerate([30, 20, 14, 60, 12, 16], 1): ws.column_dimensions[get_column_letter(i)].width = w
    table(ws, OUT_COLS, [], r + 2, 16)
    for i, w in enumerate([30, 20, 14, 60, 12, 16], 1): ws.column_dimensions[get_column_letter(i)].width = w
    for s in wb.worksheets:
        for row in s.iter_rows():
            for c in row:
                if c.font.name != "Arial": c.font = F
                c.alignment = WRAP
        s.freeze_panes = "B2" if s.title != "Читать первым" else None
    wb.save(path)

def build_sim(path, ex):
    wb = Workbook(); ws = wb.active; ws.title = "Имитатор"; keyval(ws, SIM, ex)
    r = len(SIM) + 3
    for i, s in enumerate(["Книга имитатора: среда и начальный курс. Судно, движители и датчики берутся из загруженной книги судна.",
                           "Запуск в работающем ядре: sim start Данные_имитатора.xlsx (после ship load). Остановка: sim stop.",
                           "Направления — истинные, по часовой стрелке от севера. Ветер и волна — откуда, течение — куда."]):
        ws.cell(row=r + i, column=1, value=s).font = F
    for row in ws.iter_rows():
        for c in row:
            if c.font.name != "Arial": c.font = F
            c.alignment = WRAP
    ws.freeze_panes = "B2"
    wb.save(path)

out = sys.argv[1] if len(sys.argv) > 1 else "data"
build(out + "/Данные_судна_шаблон.xlsx", False)
build(out + "/Данные_судна_пример.xlsx", True)
build_sim(out + "/Данные_имитатора_шаблон.xlsx", False)
build_sim(out + "/Данные_имитатора_пример.xlsx", True)
print("готово:", out)
