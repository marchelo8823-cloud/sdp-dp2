"""Modbus TCP-шлюз телеметрии SDP-DP2 (только чтение, FC03/FC04).

Минимальный сервер на чистом stdlib: судовые SCADA/контроллеры
читают состояние ДП как input registers. Карта регистров —
docs/modbus_map.md; значения масштабированы в int16.

Порт по умолчанию 5020 (502 требует root).
"""
import socket
import struct
import threading

# Карта input registers (адрес: имя) — масштаб в комментариях
REG_NAMES = [
    "pos_err_x100",       # 0  м×100
    "err_n_x100",         # 1  м×100 (знаковое)
    "err_e_x100",         # 2  м×100 (знаковое)
    "hdg_x10",            # 3  °×10
    "hdg_d_x10",          # 4  °×10
    "hdg_err_x100",       # 5  °×100 (знаковое)
    "sog_x100",           # 6  kn×100
    "rot_x10",            # 7  °/мин×10 (знаковое)
    "wind_kn_x10",        # 8  kn×10
    "wind_deg",           # 9  °
    "hs_x10",             # 10 м×10
    "bias_kn_x10",        # 11 kN×10
    "u1_x10",             # 12 %×10 (знаковое)
    "u2_x10",             # 13
    "u3_x10",             # 14
    "u4_x10",             # 15
    "failed_mask",        # 16 бит i — отказ ДРУ i+1
    "mode",               # 17 0=auto 1=man_joy 2=auto_joy
    "alarm_worst",        # 18 0..2
    "conseq_level",       # 19 0..2
    "cap_margin_x10",     # 20 kn×10 (знаковое)
]
N_REG = len(REG_NAMES)
SIGNED = {1, 2, 5, 7, 12, 13, 14, 15, 20}
MODES = {"auto": 0, "man_joy": 1, "auto_joy": 2}

# --- Praxis-совместимое окно (адреса 100+), теги по выгрузке Mega-Guard ---
# Раскладка: docs/praxis_tag_map.md. У судна СДП-2 восемь движителей
# (THR1..THR8); у нас четыре ДРУ → отдаём как THR3..THR6, остальные = 0.
PRAX_BASE = 100
PRAX_NAMES = (
    ["HEADING_x10", "HEADING_SP_x10", "HEADING_ERROR_x100",
     "POS_ERR_X_x100", "POS_ERR_Y_x100",
     "WIND1_SPEED_x10", "WIND1_ANGLE",
     "WIND2_SPEED_x10", "WIND2_ANGLE",
     "WIND3_SPEED_x10", "WIND3_ANGLE",
     "GYRO1_HEADING_x10", "GYRO2_HEADING_x10", "GYRO3_HEADING_x10",
     "MRU1_ROLL_x10", "MRU1_PITCH_x10"]
    + [f"T{n}_{sig}" for n in range(1, 9) for sig in
       ("RPM_SP_x10", "RPM_FB_x10", "ANGLE_SP_x10", "ANGLE_FB_x10")]
    + ["ALARM", "THR_FAIL_MASK", "GPS_SEL",
       "PLANT_LOAD_KW", "PLANT_AVAIL_KW", "PMS_REDUCE_MASK"]
)
PRAX_SIGNED = {2, 3, 4} | {16 + n * 4 + i for n in range(8) for i in (0, 1)}


def _pi16(addr, value):
    v = int(round(value))
    if addr in PRAX_SIGNED:
        return v & 0xFFFF
    return max(0, min(0xFFFF, v))


def _i16(addr, value):
    v = int(round(value))
    if addr in SIGNED:
        return v & 0xFFFF
    return max(0, min(0xFFFF, v))


class ModbusTcpServer:
    """Раздаёт snapshot()-подобный dict как input registers."""

    def __init__(self, get_state, port=5020, host="0.0.0.0"):
        self.get_state = get_state
        self.port = port
        self.host = host
        self.clients = 0
        self._thread = None

    # ------------------------------------------------------- регистры
    def _registers(self):
        s = self.get_state() or {}
        thr = s.get("thrusters", [{}] * 4)
        cap = s.get("capability", {})
        mask = 0
        for i, t in enumerate(thr):
            if t.get("failed"):
                mask |= 1 << i
        vals = [
            s.get("pos_err", 0) * 100,
            s.get("err_n", 0) * 100,
            s.get("err_e", 0) * 100,
            s.get("hdg_deg", 0) * 10,
            s.get("hdg_d_deg", 0) * 10,
            s.get("hdg_err_deg", 0) * 100,
            s.get("sog_kn", 0) * 100,
            s.get("rot_dpm", 0) * 10,
            s.get("wind_meas", {}).get("speed_kn", 0) * 10,
            s.get("wind_meas", {}).get("dir_deg", 0),
            s.get("hs", 0) * 10,
            s.get("bias_kn", 0) * 10,
            thr[0].get("u_pct", 0) * 10,
            thr[1].get("u_pct", 0) * 10,
            thr[2].get("u_pct", 0) * 10,
            thr[3].get("u_pct", 0) * 10,
            mask,
            MODES.get(s.get("mode", "auto"), 0),
            s.get("alarm_worst", 0),
            s.get("conseq", {}).get("level", 0),
            (cap.get("margin_kn") or 0) * 10,
        ]
        return [_i16(a, v) for a, v in enumerate(vals)]

    def _prax_registers(self):
        s = self.get_state() or {}
        thr = s.get("thrusters", [{}] * 4)
        mask = 0
        for i, t in enumerate(thr):
            if t.get("failed"):
                mask |= 1 << i
        wind = s.get("wind_meas", {})
        vals = [
            s.get("hdg_deg", 0) * 10,
            s.get("hdg_d_deg", 0) * 10,
            s.get("hdg_err_deg", 0) * 100,
            s.get("err_e", 0) * 100,          # POS_ERR_X — восток
            s.get("err_n", 0) * 100,          # POS_ERR_Y — север
            wind.get("speed_kn", 0) * 10,     # WIND1
            wind.get("dir_deg", 0),
            0, 0,                             # WIND2 — не моделируется
            0, 0,                             # WIND3 — не моделируется
            s.get("hdg_deg", 0) * 10,         # GYRO1 (вотированный курс)
            0, 0,                             # GYRO2/3 — не моделируются
            0, 0,                             # MRU1 — не моделируется
        ]
        for n in range(8):                    # THR1..THR8
            # профиль sdp2 — один-в-один; 4dru — наши четыре в THR3..6
            t = (thr[n] if len(thr) >= 8
                 else thr[n - 2] if len(thr) == 4 and 2 <= n < 6
                 else None)
            if t:
                u = t.get("u_pct", 0) * 10
                a = (t.get("alpha_deg", 0) % 360) * 10
                vals += [u, u, a, a]          # SP=FB (привод идеальный)
            else:
                vals += [0, 0, 0, 0]
        plant = s.get("plant", {})
        rmask = 0
        for i, lim in enumerate(plant.get("limit_mask", [])):
            if lim:
                rmask |= 1 << i
        vals += [s.get("alarm_worst", 0), mask, 0,  # GPS_SEL — 1..3, 0=авто
                 plant.get("load_kw", 0),
                 plant.get("avail_kw", 0),
                 rmask]
        return [_pi16(a, v) for a, v in enumerate(vals)]

    # ------------------------------------------------------- протокол
    def start(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(8)
        srv.settimeout(1.0)
        self._thread = threading.Thread(target=self._accept,
                                        args=(srv,), daemon=True)
        self._thread.start()

    def _accept(self, srv):
        while True:
            try:
                conn, _ = srv.accept()
            except (socket.timeout, OSError):
                continue
            self.clients += 1
            threading.Thread(target=self._serve, args=(conn,),
                             daemon=True).start()

    def _serve(self, conn):
        conn.settimeout(5.0)
        buf = b""
        try:
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                buf += data
                while len(buf) >= 8:
                    tid, pid, length, unit = struct.unpack(">HHHB",
                                                           buf[:7])
                    if len(buf) < 6 + length:
                        break
                    pdu = buf[7:6 + length]
                    buf = buf[6 + length:]
                    resp = self._handle(pdu)
                    mbap = struct.pack(">HHHB", tid, 0,
                                       len(resp) + 1, unit)
                    conn.sendall(mbap + resp)
        except (socket.timeout, OSError):
            pass
        finally:
            self.clients -= 1
            conn.close()

    def _handle(self, pdu):
        if len(pdu) < 5:
            return b"\x83\x01"      # illegal data value
        fc = pdu[0]
        addr, count = struct.unpack(">HH", pdu[1:5])
        if fc not in (3, 4):
            return bytes([fc | 0x80, 0x01])
        if count == 0:
            return bytes([fc | 0x80, 0x02])
        if addr >= PRAX_BASE:
            off = addr - PRAX_BASE
            if off + count > len(PRAX_NAMES):
                return bytes([fc | 0x80, 0x02])
            regs = self._prax_registers()[off:off + count]
        else:
            if addr + count > N_REG:
                return bytes([fc | 0x80, 0x02])
            regs = self._registers()[addr:addr + count]
        return bytes([fc, 2 * count]) + struct.pack(f">{count}H", *regs)


def build_map_doc():
    lines = ["# Карта регистров Modbus TCP SDP-DP2", "",
             "Порт 5020, unit id любой, FC03/FC04, int16 со знаком"
             " там, где указано.", "",
             "| Адрес | Сигнал | Масштаб |", "|---|---|---|"]
    notes = {
        1: "м×100, знак", 2: "м×100, знак", 5: "°×100, знак",
        7: "°/мин×10, знак", 12: "%×10, знак", 13: "%×10, знак",
        14: "%×10, знак", 15: "%×10, знак", 20: "kn×10, знак",
    }
    for i, name in enumerate(REG_NAMES):
        lines.append(f"| {i} | {name} | {notes.get(i, '—')} |")
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    print(build_map_doc())
