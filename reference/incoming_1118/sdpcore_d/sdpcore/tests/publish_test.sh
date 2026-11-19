#!/usr/bin/env bash
# Проверка tools/agent_collect.sh и tools/agent_publish.sh: отправляется только то, что агенту разрешено.
# Аргумент: корень проекта.
set -u
ROOT="$1"; T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
fails=0; ok() { echo "  ok  $1"; }; bad() { echo "FAIL $1"; fails=$((fails+1)); }
export GIT_AUTHOR_NAME=t GIT_AUTHOR_EMAIL=t@t GIT_COMMITTER_NAME=t GIT_COMMITTER_EMAIL=t@t
mkdir -p "$T/bin"; printf '#!/bin/sh\necho "[gh $*]" >> "%s/gh.log"\n' "$T" > "$T/bin/gh"; chmod +x "$T/bin/gh"; export PATH="$T/bin:$PATH"
git init -q -b main "$T/seed"; ( cd "$T/seed" && mkdir -p include отчёты && echo a > include/core.hpp && echo b > README.md && git add . && git commit -qm init )
git clone -q --bare "$T/seed" "$T/origin.git"
# run <ветка> <что сделать в рабочей копии агента> <префикс> <маска>  →  код отправки
run() {
  local br="$1" act="$2" pre="$3" allow="$4"
  rm -rf "$T/agent" "$T/pub"
  git clone -q --depth 1 "file://$T/origin.git" "$T/agent"
  ( cd "$T/agent" && git switch -q -C "$br" && eval "$act" && git add -A && git commit -qm x && echo "# Заголовок" > agent-pr.md && DEFAULT_BRANCH=main "$ROOT/tools/agent_collect.sh" "$br" >/dev/null )
  git clone -q "file://$T/origin.git" "$T/pub"; cp -r "$T/agent/agent-out" "$T/pub/"
  ( cd "$T/pub" && "$ROOT/tools/agent_publish.sh" "$pre" "$allow" 7 >/dev/null 2>&1 )
}
pushed() { git --git-dir="$T/origin.git" rev-parse -q --verify "refs/heads/$1" >/dev/null; }
run agent/data-1 'mkdir -p отчёты/1 && echo r > отчёты/1/отчёт.md' agent/data- '^отчёты/'
[ $? -eq 0 ] && pushed agent/data-1 && ok "разрешённые файлы: ветка отправлена" || bad "разрешённые файлы не отправлены"
grep -q "pr create --head agent/data-1 --title Заголовок" "$T/gh.log" && ok "предложение изменений открыто с заголовком агента" || bad "предложение изменений не открыто"
run agent/data-2 'echo z >> include/core.hpp' agent/data- '^отчёты/'
[ $? -ne 0 ] && ! pushed agent/data-2 && ok "правка вне разрешённых путей: не отправлено" || bad "правка вне путей отправлена"
grep -q "issue comment 7" "$T/gh.log" && ok "в задаче отмечено, что не отправлено" || bad "нет комментария в задаче"
run agent/data-3 'mkdir -p отчёты && git mv include/core.hpp отчёты/core.hpp' agent/data- '^отчёты/'
[ $? -ne 0 ] && ! pushed agent/data-3 && ok "перенос файла из запрещённой папки в разрешённую: не отправлено" || bad "перенос из запрещённой папки отправлен"
run agent/data-4 'git rm -q README.md' agent/data- '^отчёты/'
[ $? -ne 0 ] && ! pushed agent/data-4 && ok "удаление файла вне разрешённых путей: не отправлено" || bad "удаление вне путей отправлено"
run main 'mkdir -p отчёты && echo r > отчёты/x.md' agent/data- '^отчёты/'
[ $? -ne 0 ] && ok "ветка не с префиксом агента (main): не отправлено" || bad "ветка без префикса отправлена"
# GitHub запрещает Actions открывать предложения изменений: ветка отправлена, в задаче — что включить и ссылка
printf '#!/bin/sh\necho "[gh $*]" >> "%s/gh.log"\ncase "$1 $2" in "pr create") echo "pull request create failed: GraphQL: GitHub Actions is not permitted to create or approve pull requests (createPullRequest)" >&2; exit 1;; esac\n' "$T" > "$T/bin/gh"
run agent/data-8 'mkdir -p отчёты/8 && echo r > отчёты/8/о.md' agent/data- '^отчёты/'
rc=$?
[ $rc -ne 0 ] && pushed agent/data-8 && ok "запрет на предложения изменений: ветка отправлена, задание завершилось ошибкой" || bad "запрет на предложения изменений не отмечен"
grep -q "Allow GitHub Actions to create and approve pull requests" "$T/gh.log" && ok "в задаче написано, какую настройку включить, и ссылка" || bad "нет подсказки в задаче"
[ $fails -eq 0 ] && echo "ВСЕ ПРОВЕРКИ ПРОЙДЕНЫ" || echo "ОШИБОК: $fails"
exit $fails
