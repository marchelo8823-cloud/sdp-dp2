#!/bin/bash
# Сборка установочного пакета SDP-DP2-Installer.pkg для macOS.
#
# Пакет кладёт /Applications/SDP-DP2.app — самодостаточный бандл:
# исходники симулятора внутри Resources/src, лаунчер при первом запуске
# создаёт venv в ~/Library/Application Support/SDP-DP2 и ставит
# зависимости из requirements.txt.
#
# Запуск:  bash installer/build_pkg.sh
# Результат: dist/SDP-DP2-Installer.pkg
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

APP="build/payload/Applications/SDP-DP2.app"
VERSION="1.0.0"

echo "── Очистка build/ и dist/"
rm -rf build dist
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources/src" dist

echo "── Сборка бандла SDP-DP2.app"
cp installer/Info.plist "$APP/Contents/"
cp installer/SDP-DP2 "$APP/Contents/MacOS/SDP-DP2"
chmod +x "$APP/Contents/MacOS/SDP-DP2"

rsync -a --exclude '__pycache__' --exclude '*.pyc' \
      sdp "$APP/Contents/Resources/src/"
cp simulate.py capability.py trials.py hil_trials.py \
   compare_controllers.py compare_dob.py console.py webhmi.py hmi.py \
   hmi_qml.py sim_server.py \
   requirements.txt README.md \
   "$APP/Contents/Resources/src/"
rsync -a --exclude '__pycache__' webhmi docs qml "$APP/Contents/Resources/src/"

# Убрать extended attributes — иначе в пакет попадают AppleDouble-файлы ._*.
xattr -cr build/payload

echo "── pkgbuild → dist/SDP-DP2-Installer.pkg"
COPYFILE_DISABLE=1 pkgbuild \
    --root build/payload \
    --scripts installer/scripts \
    --identifier com.sdp-dp2.simulator \
    --version "$VERSION" \
    --install-location / \
    "dist/SDP-DP2-Installer.pkg"

echo
echo "Готово: $ROOT/dist/SDP-DP2-Installer.pkg"
pkgutil --check-signature "dist/SDP-DP2-Installer.pkg" || true
