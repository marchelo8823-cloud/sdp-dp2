#!/usr/bin/env bash
# Шаг после работы агента (в задании без права записи): собирает его локальную ветку в файл для задания отправки.
# tools/agent_collect.sh <ветка>   →   agent-out/изменения.bundle, agent-out/ветка.txt, agent-out/описание.md
set -u
BR="$1"; mkdir -p agent-out
git rev-parse --verify -q "refs/heads/$BR" >/dev/null || { echo "Агент не создал ветку $BR — отправлять нечего."; exit 0; }
BASE=$(git merge-base "$BR" "origin/${DEFAULT_BRANCH:-main}" 2>/dev/null || git rev-list --max-parents=0 "$BR" | tail -1)
[ "$(git rev-list --count "$BASE..$BR")" -gt 0 ] || { echo "В ветке $BR нет новых коммитов."; exit 0; }
git bundle create agent-out/изменения.bundle "$BASE..$BR" && echo "$BR" > agent-out/ветка.txt
if [ -f agent-pr.md ]; then cp agent-pr.md agent-out/описание.md; else echo "Агент не написал описание (agent-pr.md)." > agent-out/описание.md; fi
git -c core.quotepath=off diff --no-renames --name-only "$BASE" "$BR" > agent-out/файлы.txt
echo "Собрано: $BR, коммитов $(git rev-list --count "$BASE..$BR"), файлов $(wc -l < agent-out/файлы.txt)"
