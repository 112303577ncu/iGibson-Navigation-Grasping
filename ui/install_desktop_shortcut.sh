#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAUNCHER="$ROOT/ui/launch_ui.sh"

if command -v xdg-user-dir >/dev/null 2>&1; then
  DESKTOP="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
fi
if [[ -z "${DESKTOP:-}" ]]; then
  if [[ -d "$HOME/桌面" ]]; then
    DESKTOP="$HOME/桌面"
  else
    DESKTOP="$HOME/Desktop"
  fi
fi
mkdir -p "$DESKTOP"

if [[ -n "${X3PLUS_PYTHON:-}" && -x "${X3PLUS_PYTHON}" ]]; then
  PY="${X3PLUS_PYTHON}"
elif [[ -x "$HOME/grasp_venv/bin/python3" ]]; then
  PY="$HOME/grasp_venv/bin/python3"
elif [[ -x "$ROOT/.venv/bin/python3" ]]; then
  PY="$ROOT/.venv/bin/python3"
else
  PY="$(command -v python3)"
fi

if ! "$PY" -c 'import tkinter' >/dev/null 2>&1; then
  echo "缺少 Tkinter。請先執行：sudo apt install python3-tk" >&2
  exit 1
fi
if ! "$PY" -c 'import qrcode; from PIL import Image' >/dev/null 2>&1; then
  echo "缺少 QR 套件。請先執行：$PY -m pip install qrcode pillow" >&2
  exit 1
fi

chmod +x "$LAUNCHER" "$ROOT/ui/install_desktop_shortcut.sh"
SHORTCUT="$DESKTOP/X3Plus 操作台.desktop"
cat > "$SHORTCUT" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=X3Plus 手機操作台
Comment=顯示手機控制用 QR Code
Exec=/bin/bash "$LAUNCHER"
Icon=applications-system
Terminal=false
Categories=Utility;
EOF
chmod +x "$SHORTCUT"

# GNOME/Nautilus uses this metadata to avoid the first-launch trust warning.
if command -v gio >/dev/null 2>&1; then
  gio set "$SHORTCUT" metadata::trusted true >/dev/null 2>&1 || true
fi

echo "桌面捷徑已建立：$SHORTCUT"
echo "之後雙擊「X3Plus 手機操作台」即可顯示兩步驟 QR 連線畫面。"
