"""Web-HMI: операторский пульт SDP-DP2 в браузере (стилистика K-Pos).

Архитектура:
  - контур управления (console.DPConsole, конфигурация C) шагает в
    фоновом потоке с ускорением времени;
  - stdlib HTTP-сервер: GET / — пульт (webhmi/index.html),
    GET /state — снимок телеметрии JSON, POST /cmd — команды оператора
    и инструктора (уставка, ветер, Hs, отказы, пауза, сброс, пределы);
  - аварийный журнал с квитированием (как на реальном DP-пульте).

Без сторонних зависимостей: только numpy/matplotlib проекта.

Запуск:  python webhmi.py [--port 7100]   →  http://localhost:7100/
"""
import json
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from sdp.hmicore import HMIState

PORT_DEFAULT = 7100
INDEX = Path(__file__).parent / "webhmi" / "index.html"


STATE = HMIState()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):        # тишина в консоли
        pass

    def _send(self, code, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            self._send(200, INDEX.read_bytes(), "text/html; charset=utf-8")
        elif path == "/app.js":
            self._send(200, (INDEX.parent / "app.js").read_bytes(),
                       "text/javascript; charset=utf-8")
        elif path == "/state":
            self._send(200, json.dumps(STATE.snapshot()))
        else:
            self._send(404, "{}")

    def do_POST(self):
        if urlparse(self.path).path != "/cmd":
            self._send(404, "{}")
            return
        n = int(self.headers.get("Content-Length", 0))
        try:
            cmd = json.loads(self.rfile.read(n) or b"{}")
            self._send(200, json.dumps(STATE.command(cmd)))
        except Exception as e:                       # noqa: BLE001
            self._send(400, json.dumps({"ok": False, "error": str(e)}))


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=PORT_DEFAULT)
    p.add_argument("--no-browser", action="store_true")
    args = p.parse_args()
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    url = f"http://localhost:{args.port}/"
    print(f"SDP-DP2 Web-HMI: {url}   (Ctrl+C — выход)")
    if not args.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
