#!/usr/bin/env bash
# Check the operator console on the machine it will actually run on.
#
# Everything in ui/ was developed and tested off the robot, which means the
# parts that can only be wrong on the Jetson have never been exercised: whether
# the port is reachable from a phone, whether shared memory is writable, whether
# the UDP loop survives this kernel, whether the QR code names an address that
# leads anywhere. This runs those, and only those.
#
#     ./ui/jetson_check.sh
#
# Nothing here drives a motor, opens the servo bus, or starts a mission. It is
# safe to run at any time, including with the robot switched on.

set -u

PORT="${PORT:-8080}"
STATUS_PORT="${STATUS_PORT:-8099}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${PY:-python3}"

pass=0; warn=0; fail=0
ok()   { printf '  \033[32m[ ok ]\033[0m %s\n' "$1"; pass=$((pass+1)); }
wrn()  { printf '  \033[33m[warn]\033[0m %s\n' "$1"; warn=$((warn+1)); }
bad()  { printf '  \033[31m[FAIL]\033[0m %s\n' "$1"; fail=$((fail+1)); }
note() { printf '         %s\n' "$1"; }

echo
echo "== X3Plus 操作台：上機檢查 =="
echo "   repo: $ROOT"
echo

# ── 1. Python ───────────────────────────────────────────────────────────────
ver="$($PY -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null)"
if [ -z "$ver" ]; then
  # Stop here rather than run the remaining checks. Every one of them shells out
  # to this interpreter, so carrying on turns one real problem into a page of
  # failures that all say the same thing and bury the line worth reading.
  bad "找不到可用的 $PY"
  note "操作台需要 Python 3.6 以上。"
  note "如果任務程序跑在 venv 裡，先啟用它，或指定："
  note "  PY=~/grasp_venv/bin/python3 ./ui/jetson_check.sh"
  echo
  echo "  其餘檢查全部需要這個直譯器，先解決它再重跑。"
  exit 1
else
  ok "Python $ver"
  # The console is stdlib-only on purpose; this proves it on this machine.
  if $PY - <<'EOF' >/dev/null 2>&1
import http.server, socket, json, queue, threading, mimetypes, signal, subprocess
EOF
  then ok "標準函式庫齊全（操作台不需要 pip install 任何東西）"
  else bad "標準函式庫有缺，這個 Python 可能是精簡版"
  fi
fi

# ── 2. 檔案 ─────────────────────────────────────────────────────────────────
for f in ui/server.py ui/launcher.py ui/launch_ui.sh ui/install_desktop_shortcut.sh \
         ui/static/index.html ui/static/app.js ui/static/style.css \
         integration/mission_status.py; do
  if [ -f "$ROOT/$f" ]; then ok "$f"; else bad "缺少 $f"; fi
done

# ── 3. 離線邏輯 ─────────────────────────────────────────────────────────────
if $PY "$ROOT/ui/test_server.py" >/tmp/x3ui_tests.log 2>&1; then
  # unittest's own count, not a grep for "ok" -- tests with docstrings print
  # their result on a second line and would be miscounted.
  ok "操作台回歸測試全過（$(sed -n 's/^Ran \([0-9]*\) tests.*/\1/p' /tmp/x3ui_tests.log) 項）"
else
  bad "操作台回歸測試失敗"
  note "詳見 /tmp/x3ui_tests.log"
  tail -n 12 /tmp/x3ui_tests.log | sed 's/^/         /'
fi
for m in mission_status; do
  if $PY "$ROOT/integration/$m.py" --selftest >/dev/null 2>&1; then
    ok "$m 自測通過"
  else
    bad "$m 自測失敗"
    note "重跑看原因：$PY integration/$m.py --selftest"
  fi
done

# ── 4. 網路：手機真的連得到嗎 ───────────────────────────────────────────────
ip="$($PY -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    s.connect(('8.8.8.8', 1)); print(s.getsockname()[0])
except OSError:
    print('')
finally:
    s.close()" 2>/dev/null)"
if [ -z "$ip" ]; then
  bad "取不到對外 IP —— 沒有連上任何網路"
  note "手機無法連進來。先接上 Wi-Fi，或改用熱點模式。"
else
  ok "對外 IP $ip"
  note "手機要開的網址：http://$ip:$PORT"
fi

if $PY -c "
import socket, sys
s = socket.socket()
try:
    s.bind(('0.0.0.0', $PORT))
except OSError as e:
    sys.exit(str(e))
finally:
    s.close()" 2>/tmp/x3ui_port.log; then
  ok "連接埠 $PORT 可以綁定"
else
  bad "連接埠 $PORT 被占用：$(cat /tmp/x3ui_port.log)"
  note "換一個：PORT=8090 ./ui/jetson_check.sh，啟動時也要 --port 8090"
fi

# 0.0.0.0 vs 127.0.0.1 is the difference between "works on the robot" and
# "works on a phone", and it is invisible until someone tries from a phone.
if command -v ufw >/dev/null 2>&1 && ufw status 2>/dev/null | grep -qi '^Status: active'; then
  if ufw status | grep -q "$PORT"; then
    ok "防火牆已放行 $PORT"
  else
    wrn "ufw 啟用中，但沒看到 $PORT 的規則"
    note "手機可能連不進來。放行：sudo ufw allow $PORT/tcp"
  fi
else
  ok "沒有啟用中的 ufw 防火牆擋著"
fi

# ── 5. 遙測：UDP 迴路 ───────────────────────────────────────────────────────
if $PY - "$STATUS_PORT" <<'EOF' >/dev/null 2>&1
import json, socket, sys, threading, time
sys.path.insert(0, "integration")
import mission_status

port = int(sys.argv[1])
rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
rx.bind(("127.0.0.1", port))
rx.settimeout(3.0)

class Tr:
    state = "PATROL"; action = "DRIVE_PATROL"; reason = "jetson check"
    changed = True; chassis_allowed = True; arm_allowed = False; terminal = False

mission_status.StatusEmitter("127.0.0.1:%d" % port).publish(Tr())
got = json.loads(rx.recv(65535).decode("utf-8"))
assert got["state"] == "PATROL", got
EOF
then
  ok "遙測 UDP 迴路可用（127.0.0.1:$STATUS_PORT）"
else
  bad "遙測 UDP 迴路不通"
  note "主控台會顯示不出任務狀態。檢查有沒有其他程式占用 UDP $STATUS_PORT。"
fi

# ── 6. 序列埠：任務程序必須是唯一持有者 ─────────────────────────────────────
if [ -e /dev/myserial ]; then
  ok "/dev/myserial 存在"
  if command -v fuser >/dev/null 2>&1; then
    holders="$(fuser /dev/myserial 2>/dev/null | tr -s ' ')"
    if [ -z "$holders" ]; then
      ok "序列埠目前沒有人占用"
    else
      wrn "序列埠已被占用：$holders"
      note "任務程序必須是唯一持有者。啟動前先關掉 ROS 底盤 driver / port 7000 motor server。"
    fi
  else
    wrn "沒有 fuser，無法檢查序列埠占用"
  fi
else
  wrn "/dev/myserial 不存在"
  note "只跑模擬或唯讀監看沒關係；要實際動作就必須有它。"
fi

# ── 7. QR code ──────────────────────────────────────────────────────────────
if $PY -c "import qrcode, PIL" >/dev/null 2>&1; then
  url="$($PY "$ROOT/ui/make_qr.py" --print-url --port "$PORT" 2>/dev/null)"
  if [ -n "$url" ]; then
    ok "QR code 可產生：$url"
  else
    wrn "QR code 產生器取不到 IP"
  fi
else
  wrn "缺少 qrcode / pillow，桌面 QR code 無法產生"
  note "安裝：pip3 install qrcode pillow（不裝也不影響操作台，只是要手動輸入 IP）"
fi

# ── 8. 路線 ─────────────────────────────────────────────────────────────────
if $PY - <<'EOF' >/dev/null 2>&1
import sys; sys.path.insert(0, "integration")
import map_goal_provider as mgp
from pathlib import Path
p = mgp.DEFAULT_ROUTE_HINT
assert Path(p).exists(), p
spec = mgp.load_route(str(p), resample_m=0.75)
assert len(spec.waypoints) > 1
EOF
then
  ok "route.yaml 讀得到，地圖畫得出來"
else
  wrn "讀不到預設 route.yaml"
  note "地圖會是空白的，其餘功能不受影響。在「任務設定」填入路線路徑後按「重新載入路線」。"
fi

# ── 總結 ────────────────────────────────────────────────────────────────────
echo
echo "  通過 $pass，警告 $warn，失敗 $fail"
echo
if [ "$fail" -gt 0 ]; then
  echo "  有項目失敗，先處理再開操作台。"
  exit 1
fi
echo "  接下來："
echo "    1. $PY ui/server.py                 # 唯讀監看，先確認手機連得進來"
echo "    2. 手機開 http://${ip:-<IP>}:$PORT"
echo "    3. 上機前的完整檢查在終端機跑：$PY integration/preflight.py --offline"
echo "    4. 確認沒問題後，改用 --allow-real 啟動，才做實機任務"
echo
[ "$warn" -gt 0 ] && echo "  有 $warn 項警告，看一下上面說明是不是可以接受。"
exit 0
