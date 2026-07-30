import socket
import json
import time
import msvcrt
import os
import sys

MOTION_ALLOWED = "--real" in sys.argv[1:]

JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "172.31.28.252")
PORT = 7000

# 預設速度：30
speed = 30

SPEED_PRESETS = {
    "1": 10,
    "2": 20,
    "3": 30,
    "4": 40,
    "5": 50,
    "6": 60,
}


def send_cmd(sock, action, speed):
    if not MOTION_ALLOWED and action != "stop":
        raise RuntimeError("keyboard motor test requires the explicit --real flag")
    msg = {
        "action": action,
        "speed": int(speed),
    }
    data = (json.dumps(msg) + "\n").encode("utf-8")
    sock.sendall(data)


def print_help():
    print()
    print("========== Keyboard Motor B Test ==========")
    print("Default speed = 30")
    print("1~6 : speed preset 10~60")
    print("------------------------------------------")
    print("W : forward")
    print("S : backward")
    print("A : turn_left")
    print("D : turn_right")
    print("Q : curve_left")
    print("E : curve_right")
    print("SPACE : stop")
    print("X : exit")
    print("==========================================")
    print()


def main():
    global speed

    if not MOTION_ALLOWED:
        print("[SAFETY] Motor test not started. Re-run with --real in a clear test area.")
        return

    print(f"[INFO] connecting to {JETSON_IP}:{PORT} ...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((JETSON_IP, PORT))

    print("[INFO] connected.")
    print_help()

    current_action = "stop"
    last_send = time.time()

    try:
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode("utf-8", errors="ignore").lower()

                if key in SPEED_PRESETS:
                    speed = SPEED_PRESETS[key]
                    current_action = "stop"
                    print(f"[SPEED] {speed}")

                elif key == "w":
                    current_action = "forward"

                elif key == "s":
                    current_action = "backward"

                elif key == "a":
                    current_action = "turn_left"

                elif key == "d":
                    current_action = "turn_right"

                elif key == "q":
                    current_action = "curve_left"

                elif key == "e":
                    current_action = "curve_right"

                elif key == " ":
                    current_action = "stop"

                elif key == "x":
                    print("[INFO] exit.")
                    break

                print(f"[CMD] action={current_action}, speed={speed}")
                send_cmd(sock, current_action, speed)
                last_send = time.time()

            # 每 0.1 秒持續送目前指令
            now = time.time()
            if now - last_send >= 0.1:
                send_cmd(sock, current_action, speed)
                last_send = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected.")

    finally:
        print("[INFO] sending stop...")
        for _ in range(10):
            send_cmd(sock, "stop", 0)
            time.sleep(0.05)

        sock.close()
        print("[INFO] closed.")


if __name__ == "__main__":
    main()
