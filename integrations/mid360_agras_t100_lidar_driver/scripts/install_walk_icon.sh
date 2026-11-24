#!/bin/bash
# Put the "FAST-LIO 建图录屏" icon on the desktop (runs scripts/fastlio_walk.sh). Run as the desktop user.
set -eu
ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
[ -n "${DBUS_SESSION_BUS_ADDRESS:-}" ] || export DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus"
chmod +x "$ROOT"/scripts/*.sh "$ROOT"/scripts/*.py
DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
mkdir -p "$DESK"
cat > "$DESK/fastlio-walk.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=FAST-LIO 建图录屏
Comment=启动激光雷达驱动、FAST-LIO 和 RViz 并录屏；关掉 RViz 窗口即保存
Exec=$ROOT/scripts/fastlio_walk.sh
Icon=camera-video
Terminal=false
EOF
chmod +x "$DESK/fastlio-walk.desktop"
gio set "$DESK/fastlio-walk.desktop" metadata::trusted true 2>/dev/null || true  # GNOME only launches trusted icons
ls -la "$DESK/fastlio-walk.desktop"
