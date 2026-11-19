# Книги с намеренными ошибками для теста shipbook_test. Запуск: python3 tools/make_test_books.py <пример.xlsx> <папка>
import sys, os
from openpyxl import load_workbook
src, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
def make(name, fn):
    wb = load_workbook(src); fn(wb); wb.save(os.path.join(out, name))
def row(ws, label):
    for r in ws.iter_rows(min_col=1, max_col=1):
        if r[0].value == label: return r[0].row
    raise KeyError(label)
def col(ws, head, hrow=1):
    for c in ws[hrow]:
        if c.value == head: return c.column
    raise KeyError(head)
def noth(wb):
    wb["Движители"].cell(row=4, column=col(wb["Движители"], "Упор вперёд, кН")).value = None
make("нет_упора.xlsx", noth)
make("не_число.xlsx", lambda wb: wb["Корпус"].cell(row=row(wb["Корпус"], "Осадка"), column=2, value="четыре"))
make("запятая.xlsx", lambda wb: wb["Корпус"].cell(row=row(wb["Корпус"], "Осадка"), column=2, value="4,5"))
make("плохой_тип.xlsx", lambda wb: wb["Движители"].cell(row=2, column=2, value="винт"))
make("вне_корпуса.xlsx", lambda wb: wb["Движители"].cell(row=4, column=col(wb["Движители"], "X от миделя, м"), value=45))
def partial(wb):
    ws = wb["Датчики"]; ws.cell(row=2, column=col(ws, "Подключение"), value="udp"); ws.cell(row=2, column=col(ws, "Порт"), value=10001)
make("датчики_часть.xlsx", partial)
def badport(wb):
    ws = wb["Датчики"]; ws.cell(row=2, column=col(ws, "Подключение"), value="usb")
make("датчики_плохое_подключение.xlsx", badport)
def ports(wb):
    ws = wb["Датчики"]; ct, cp = col(ws, "Подключение"), col(ws, "Порт")
    for i in range(10): ws.cell(row=2 + i, column=ct, value="udp"); ws.cell(row=2 + i, column=cp, value=10001 + i)
make("датчики_порт.xlsx", ports)
def sect(wb):
    ws = wb["Движители"]; ws.cell(row=2, column=col(ws, "Запретные сектора, °"), value="240-300; 60-120"); ws.cell(row=2, column=col(ws, "Темп упора, %/с"), value=15)
make("секторы.xlsx", sect)
def outp(wb):
    ws = wb["Выдача"]
    for k, v in (("Подключение", "tcp"), ("Адрес", "127.0.0.1"), ("Порт", 1502), ("Номер устройства Modbus", 1)): ws.cell(row=row(ws, k), column=2, value=v)
    h = row(ws, "Движитель")
    for i, r in enumerate([("AZ1", "упор", 100, 100, "да"), ("AZ1", "угол", 101, 10, "нет")]):
        for j, v in enumerate(r): ws.cell(row=h + 1 + i, column=1 + j, value=v)
make("выдача.xlsx", outp)
def bias(wb):
    ws = wb["Движители"]; ca, cp = col(ws, "Предпочтительный угол, °"), col(ws, "Встречный упор, %")
    ws.cell(row=2, column=ca, value=90); ws.cell(row=2, column=cp, value=20); ws.cell(row=3, column=ca, value=270); ws.cell(row=3, column=cp, value=20)
    k = wb["Корпус"]; k.cell(row=row(k, "Уход за 5 мин без положения"), column=2, value=9)
make("встречный_упор.xlsx", bias)
def big(wb):
    ws = wb["Движители"]; ws.cell(row=2, column=col(ws, "Предпочтительный угол, °"), value=90); ws.cell(row=2, column=col(ws, "Встречный упор, %"), value=70)
make("встречный_упор_много.xlsx", big)
make("уход_ноль.xlsx", lambda wb: wb["Корпус"].cell(row=row(wb["Корпус"], "Уход за 5 мин без положения"), column=2, value=0))
make("встречный_упор_без_угла.xlsx", lambda wb: wb["Движители"].cell(row=2, column=col(wb["Движители"], "Встречный упор, %"), value=20))
open(os.path.join(out, "не_книга.xlsx"), "w").write("это не книга Excel, а обычный текст\n" * 5)
print("готово")
make("группа_пустая.xlsx", lambda wb: wb["Движители"].cell(row=2, column=col(wb["Движители"], "Группа отказа"), value="ГРЩ-1;;МО-1"))
