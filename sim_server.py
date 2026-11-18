"""UDP-сервер симуляции SDP-DP2: «железная» сторона пульта.

Отдельный процесс: контур управления (sdp.hmicore.HMIState) шагает
независимо от HMI — ровно как на судне, где контроллер ДП живёт в
своём шкафу, а пульт лишь клиент телеметрии. Тот же протокол потом
пойдёт к реальным контроллерам (заменится транспорт/адресация).

Протокол (JSON-дейтаграммы, UDP):
  клиент → сервер, порт 7200:
      {"action": "subscribe", "port": N}  — подписка на телеметрию
      {"action": "setpoint", "north": .., "east": ..}
      {"action": "wind", "speed_ms": .., "dir_deg": ..}
      {"action": "hs", "hs": ..}
      {"action": "fail", "kind": "thruster|prs|gyro", "idx": i}
      {"action": "heading_bump", "d_deg": ..} / {"action": "heading", "deg": ..}
      {"action": "speed", "value": x} / "pause" / "reset" / "ack"
      {"action": "limits", "warn": .., "alarm": ..}
  сервер → клиентам (на подписанные порты), 10 Гц:
      снимок sdp.hmicore.HMIState.snapshot()

Запуск:  python sim_server.py [--port 7200]
Клиент:  python hmi.py            (сам поднимет сервер, если его нет)
"""
import argparse
import json
import socket
import time

from sdp.hmicore import HMIState

CMD_PORT_DEFAULT = 7200
TX_PERIOD = 0.1            # телеметрия 10 Гц
CLIENT_TTL = 30.0          # молчание клиента → отписка


class SimServer:
    def __init__(self, port=CMD_PORT_DEFAULT):
        self.state = HMIState()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", port))
        self.sock.setblocking(False)
        self.clients = {}    # (ip, tel_port) -> last_heard
        self.port = port

    # ------------------------------------------------------------ команды
    def _drain_commands(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(65535)
            except BlockingIOError:
                return
            try:
                cmd = json.loads(data.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                continue
            if not isinstance(cmd, dict):
                continue
            if cmd.get("action") == "subscribe":
                tel_port = int(cmd.get("port", 0))
                if tel_port:
                    self.clients[(addr[0], tel_port)] = time.monotonic()
                continue
            try:
                self.state.command(cmd)
            except Exception:                        # noqa: BLE001
                pass

    # ---------------------------------------------------------- телеметрия
    def _broadcast(self, msg):
        now = time.monotonic()
        dead = [a for a, t in self.clients.items()
                if now - t > CLIENT_TTL]
        for a in dead:
            del self.clients[a]
        for addr in self.clients:
            try:
                self.sock.sendto(msg, addr)
            except OSError:
                pass

    def serve(self):
        print(f"SDP-DP2 sim server: UDP :{self.port}, телеметрия 10 Гц "
              f"(Ctrl+C — выход)")
        last_tx = 0.0
        try:
            while True:
                self._drain_commands()
                now = time.monotonic()
                if now - last_tx >= TX_PERIOD:
                    last_tx = now
                    if self.clients:
                        msg = json.dumps(self.state.snapshot()).encode()
                        self._broadcast(msg)
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=CMD_PORT_DEFAULT)
    args = p.parse_args()
    SimServer(args.port).serve()


if __name__ == "__main__":
    main()
