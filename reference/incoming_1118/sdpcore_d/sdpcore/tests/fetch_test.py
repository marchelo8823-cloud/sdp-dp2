#!/usr/bin/env python3
"""Проверка tools/fetch_attachments.py: токен не уходит при перенаправлении на другой адрес.
Два локальных сервера: «github» перенаправляет на «чужой»; чужой записывает, пришёл ли заголовок Authorization."""
import http.server, os, subprocess, sys, tempfile, threading, urllib.request
root = sys.argv[1]
seen = {}
class Foreign(http.server.BaseHTTPRequestHandler):
    def do_GET(s):
        seen["auth"] = s.headers.get("Authorization")
        s.send_response(200); s.end_headers(); s.wfile.write(b"data")
    def log_message(s, *a): pass
fs = http.server.HTTPServer(("127.0.0.1", 0), Foreign)
threading.Thread(target=fs.serve_forever, daemon=True).start()
code = r'''
import runpy, sys, urllib.request, urllib.parse
port = int(sys.argv[2])
# подменяем github.com на локальный «github», который перенаправляет на чужой сервер
class Redirect(urllib.request.BaseHandler):
    handler_order = 100
    def https_open(self, req):
        import io, email.message
        assert req.get_header("Authorization"), "к github.com токен должен идти"
        m = email.message.Message(); m["Location"] = f"http://127.0.0.1:{port}/file"
        r = urllib.response.addinfourl(io.BytesIO(b""), m, req.full_url, 302); r.msg = "Found"
        return r
import urllib.response
orig = urllib.request.build_opener
urllib.request.build_opener = lambda *h: orig(Redirect, *h)
runpy.run_path(sys.argv[1], run_name="__main__")
'''
with tempfile.TemporaryDirectory() as d:
    env = dict(os.environ, ISSUE_BODY="https://github.com/user-attachments/files/1/журнал.csv", GITHUB_REPOSITORY="o/r", GH_TOKEN="test-token-123")
    r = subprocess.run([sys.executable, "-c", code, os.path.join(root, "tools/fetch_attachments.py"), str(fs.server_port)], cwd=d, env=env, capture_output=True, text=True, timeout=60)
    print(r.stdout, r.stderr)
    ok_dl = os.path.exists(os.path.join(d, "вход", "журнал.csv"))
fails = 0
for c, m in [(ok_dl, "файл скачан через перенаправление"), ("auth" in seen and seen["auth"] is None, "чужой адрес не получил токен")]:
    print(("  ok  " if c else "FAIL ") + m); fails += 0 if c else 1
sys.exit(1 if fails else 0)
