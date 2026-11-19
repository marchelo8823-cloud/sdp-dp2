#!/usr/bin/env bash
# Задание отправки (с правом записи, без Claude): проверяет, что агент менял только разрешённые файлы, отправляет ветку
# и открывает предложение изменений. tools/agent_publish.sh <префикс ветки> <разрешённые пути, регулярное выражение> <№ задачи или пусто>
set -euo pipefail
PREFIX="$1"; ALLOW="$2"; ISSUE="${3:-}"
[ -f agent-out/ветка.txt ] || { echo "Агент ничего не сдал."; exit 0; }
BR=$(head -1 agent-out/ветка.txt)
[[ "$BR" == "$PREFIX"* ]] || { echo "::error::Ветка «$BR» не начинается с «$PREFIX» — не отправляю."; exit 1; }
[[ "$BR" =~ ^[A-Za-z0-9/_.-]+$ ]] || { echo "::error::Недопустимое имя ветки — не отправляю."; exit 1; }
git fetch -q agent-out/изменения.bundle "refs/heads/$BR:refs/heads/$BR"
BASE=$(git merge-base "$BR" HEAD)
BAD=$(git -c core.quotepath=off diff --no-renames --name-only "$BASE" "$BR" | grep -Ev "$ALLOW" || true)
if [ -n "$BAD" ]; then
  echo "::error::Агент изменил файлы вне разрешённых — не отправляю:"; echo "$BAD"
  [ -n "$ISSUE" ] && gh issue comment "$ISSUE" --body "⚠ Изменения агента не отправлены: затронуты файлы вне разрешённых (${PREFIX}*):
\`\`\`
$BAD
\`\`\`" || true
  exit 1
fi
git push -u origin "$BR"
TITLE=$(grep -m1 -E '^# ' agent-out/описание.md | sed 's/^# //' || true); [ -n "$TITLE" ] || TITLE="Изменения агента: $BR"
if ! OUT=$(gh pr create --head "$BR" --title "$TITLE" --body-file agent-out/описание.md 2>&1); then
  echo "$OUT"
  if echo "$OUT" | grep -q "already exists"; then echo "Предложение изменений для $BR уже есть."; exit 0; fi
  URL="https://github.com/${GITHUB_REPOSITORY:-}/compare/${BR}?expand=1"
  MSG="⚠ Ветка агента \`$BR\` отправлена, но предложение изменений не открылось."
  echo "$OUT" | grep -q "not permitted to create" && MSG="$MSG GitHub запрещает Actions открывать предложения изменений: Settings → Actions → General → **Allow GitHub Actions to create and approve pull requests** → Save."
  [ -n "$ISSUE" ] && gh issue comment "$ISSUE" --body "$MSG Открыть вручную: $URL" || true
  echo "::error::Предложение изменений не открыто: $OUT"; exit 1
fi
echo "$OUT"
