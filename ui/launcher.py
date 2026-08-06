#!/usr/bin/env python3
"""Two-step, browser-free desktop launcher for the X3Plus phone console.

The Jetson only shows a small Tk window.  The real web page is opened by the
phone after it scans the QR code, so Chromium never consumes RAM on the robot.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Dict, Optional
from urllib.error import URLError
from urllib.request import urlopen

import make_qr


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DEFAULT_PORT = 8080
START_TIMEOUT_S = 12.0


def server_argv(port: int, *, allow_real: bool, simulate: bool) -> list:
    argv = [sys.executable, str(HERE / "server.py"),
            "--port", str(port), "--bind", "0.0.0.0"]
    if allow_real:
        argv.append("--allow-real")
    if simulate:
        argv.append("--simulate")
    return argv


def server_state(port: int, timeout: float = 0.4) -> Optional[Dict]:
    try:
        with urlopen("http://127.0.0.1:%d/api/state" % port,
                     timeout=timeout) as response:
            value = json.loads(response.read().decode("utf-8"))
            return value if isinstance(value, dict) else None
    except (OSError, URLError, ValueError):
        return None


def start_server(port: int, *, allow_real: bool, simulate: bool):
    current = server_state(port)
    if current is not None:
        if allow_real and not current.get("allow_real"):
            raise RuntimeError(
                "8080 連接埠已有唯讀操作台。請先關閉舊的操作台，再重新點擊桌面捷徑。")
        return None

    # The server intentionally stays alive if the QR window is closed.  This
    # lets the operator clear the Jetson desktop after scanning without cutting
    # off the phone.  It ends on shutdown, or can be stopped with Ctrl+C when
    # launched from a terminal.
    return subprocess.Popen(
        server_argv(port, allow_real=allow_real, simulate=simulate),
        cwd=str(ROOT), stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


class Launcher:
    def __init__(self, root, tk, ttk, messagebox, args):
        self.root = root
        self.tk = tk
        self.ttk = ttk
        self.messagebox = messagebox
        self.args = args
        self.started_at = 0.0
        self.qr_image = None
        self.qr_path = Path(tempfile.gettempdir()) / (
            "x3plus_console_%d.png" % args.port)

        root.title("X3Plus 手機操作台")
        root.geometry("520x690")
        root.minsize(460, 600)
        root.configure(bg="#F3F6FA")
        self.frame = ttk.Frame(root, padding=32)
        self.frame.pack(fill="both", expand=True)
        self.show_intro()

    def clear(self):
        for widget in self.frame.winfo_children():
            widget.destroy()

    def title(self, text):
        self.ttk.Label(self.frame, text=text,
                       font=("Sans", 24, "bold"),
                       justify="center").pack(pady=(28, 18))

    def show_intro(self):
        self.clear()
        self.title("連接 X3Plus 操作台")
        self.ttk.Label(
            self.frame,
            text="請將機器人與手機連接到同一個 Wi-Fi 網路",
            font=("Sans", 16, "bold"), justify="center",
            wraplength=420,
        ).pack(pady=(40, 18))
        self.ttk.Label(
            self.frame,
            text=("確認兩台裝置都已連上網路後，按下「下一步」。\n"
                  "系統會自動找出機器人的位址並產生 QR Code，\n"
                  "不需要手動查詢或輸入 IP。"),
            font=("Sans", 12), justify="center", wraplength=410,
        ).pack(pady=16)
        self.ttk.Button(self.frame, text="下一步",
                        command=self.begin).pack(ipadx=46, ipady=13, pady=54)
        self.ttk.Label(
            self.frame,
            text="瀏覽器只會開在手機上，不會占用 Jetson 的瀏覽器記憶體。",
            foreground="#586B82", justify="center", wraplength=410,
        ).pack(side="bottom", pady=8)

    def begin(self):
        self.clear()
        self.title("正在準備操作台")
        self.status = self.ttk.Label(
            self.frame, text="正在確認網路並啟動服務…",
            font=("Sans", 13), justify="center", wraplength=410)
        self.status.pack(expand=True)

        ip = make_qr.lan_address()
        if not ip:
            self.show_error("尚未取得機器人的網路位址。\n請確認 Wi-Fi 已連線後再試一次。")
            return
        self.ip = ip
        try:
            start_server(self.args.port, allow_real=self.args.allow_real,
                         simulate=self.args.simulate)
        except (OSError, RuntimeError) as exc:
            self.show_error(str(exc))
            return
        self.started_at = time.monotonic()
        self.root.after(150, self.wait_for_server)

    def wait_for_server(self):
        if server_state(self.args.port) is not None:
            self.show_qr()
            return
        if time.monotonic() - self.started_at >= START_TIMEOUT_S:
            self.show_error("操作台沒有在預期時間內啟動。\n請執行 ui/jetson_check.sh 查看原因。")
            return
        self.root.after(200, self.wait_for_server)

    def show_qr(self):
        url = make_qr.console_url(self.ip, self.args.port)
        try:
            import qrcode  # noqa: F401
            from PIL import Image, ImageTk
            make_qr.render(url, self.ip, self.qr_path, self.args.port)
            card = Image.open(self.qr_path)
            # Keep the QR comfortably scannable while leaving room for the URL
            # and both action buttons on a 690 px-tall Jetson desktop window.
            max_width = 270
            if card.width > max_width:
                ratio = max_width / float(card.width)
                card = card.resize((max_width, round(card.height * ratio)),
                                   Image.Resampling.LANCZOS)
            image = ImageTk.PhotoImage(card)
        except (ImportError, OSError, RuntimeError) as exc:
            self.show_error(
                "無法產生 QR Code。\n請安裝：pip3 install qrcode pillow\n\n%s" % exc)
            return

        self.clear()
        self.title("用手機掃描 QR Code")
        self.qr_image = image
        self.ttk.Label(self.frame, image=image).pack(pady=(0, 12))
        self.ttk.Label(
            self.frame,
            text="掃描後會直接進入操作台，不需要輸入 IP。",
            font=("Sans", 13, "bold"), justify="center",
        ).pack(pady=6)
        self.ttk.Label(self.frame, text=url, foreground="#1256B8",
                       justify="center", wraplength=430).pack(pady=5)
        self.ttk.Label(
            self.frame,
            text="可以關閉這個 QR 視窗；手機操作台會繼續執行到機器人關機。",
            foreground="#586B82", justify="center", wraplength=420,
        ).pack(pady=12)
        buttons = self.ttk.Frame(self.frame)
        buttons.pack(pady=8)
        self.ttk.Button(buttons, text="重新偵測",
                        command=self.begin).pack(side="left", padx=6)
        self.ttk.Button(buttons, text="完成",
                        command=self.root.destroy).pack(side="left", padx=6)

    def show_error(self, message):
        self.clear()
        self.title("無法顯示 QR Code")
        self.ttk.Label(self.frame, text=message, foreground="#B42318",
                       font=("Sans", 13), justify="center",
                       wraplength=420).pack(expand=True, pady=20)
        self.ttk.Button(self.frame, text="返回上一步",
                        command=self.show_intro).pack(ipadx=24, ipady=8, pady=20)


def selftest() -> int:
    argv = server_argv(8080, allow_real=True, simulate=False)
    assert argv[-1] == "--allow-real"
    assert "--simulate" not in argv
    assert make_qr.console_url("192.168.1.42", 8080) == \
        "http://192.168.1.42:8080/?connect=1"
    print("launcher selftest passed")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--allow-real", action="store_true",
                      help="allow the phone UI to start real missions")
    mode.add_argument("--simulate", action="store_true",
                      help="show simulated telemetry and never drive hardware")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args(argv)
    if args.selftest:
        return selftest()

    try:
        import tkinter as tk
        from tkinter import messagebox, ttk
    except ImportError:
        print("缺少 Tkinter。請在 Jetson 執行：sudo apt install python3-tk",
              file=sys.stderr)
        return 2

    root = tk.Tk()
    Launcher(root, tk, ttk, messagebox, args)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
