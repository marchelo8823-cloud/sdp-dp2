#!/usr/bin/env python3
"""Скачивает вложения из текста задачи в папку вход/ (для агента 1 в GitHub Actions).

Текст задачи берётся из переменной окружения ISSUE_BODY. Скачиваются только ссылки вида
https://github.com/user-attachments/... и https://github.com/<владелец>/<репозиторий>/files/..., не больше 10 файлов по 50 МБ.
Токен (переменная GH_TOKEN) передаётся только на github.com.
"""
import os, re, sys, urllib.request, urllib.parse

class NoAuthOffGithub(urllib.request.HTTPRedirectHandler):
    """При перенаправлении на другой адрес (файлы вложений лежат не на github.com) токен не передаётся."""
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        new = super().redirect_request(req, fp, code, msg, headers, newurl)
        if new is not None and (urllib.parse.urlparse(newurl).hostname or "") != "github.com":
            for h in list(new.headers):
                if h.lower() == "authorization":
                    del new.headers[h]
            new.unredirected_hdrs.pop("Authorization", None)
        return new


opener = urllib.request.build_opener(NoAuthOffGithub)
body = os.environ.get("ISSUE_BODY", "")
repo = os.environ.get("GITHUB_REPOSITORY", "")
token = os.environ.get("GH_TOKEN", "")
pat = re.compile(r"https://github\.com/(?:user-attachments/(?:assets|files)/[\w\-./%]+|" + re.escape(repo) + r"/files/[\w\-./%]+)")
urls = list(dict.fromkeys(pat.findall(body)))[:10]
os.makedirs("вход", exist_ok=True)
got = []
for i, u in enumerate(urls, 1):
    name = os.path.basename(urllib.parse.urlparse(u).path) or f"файл_{i}"
    name = re.sub(r"[^\w.\-]", "_", urllib.parse.unquote(name))[:100]
    req = urllib.request.Request(urllib.parse.quote(u, safe=":/%?=&"))   # русские буквы в имени файла — в %-виде
    if token:
        req.add_unredirected_header("Authorization", f"token {token}")   # только для первого запроса, не для перенаправлений
    try:
        with opener.open(req, timeout=60) as r:
            host = urllib.parse.urlparse(r.geturl()).hostname or ""
            data = r.read(50_000_001)
        if len(data) > 50_000_000:
            print(f"⚠ {u}: больше 50 МБ, пропущен"); continue
        with open(os.path.join("вход", name), "wb") as fh:
            fh.write(data)
        got.append(name); print(f"скачан: вход/{name} ({len(data)} байт, с {host})")
    except Exception as e:
        print(f"⚠ {u}: не скачан ({e.__class__.__name__}: {e})")
with open(os.path.join("вход", "СПИСОК.txt"), "w", encoding="utf-8") as fh:
    fh.write("\n".join(got) + ("\n" if got else "Вложений не скачано.\n"))
