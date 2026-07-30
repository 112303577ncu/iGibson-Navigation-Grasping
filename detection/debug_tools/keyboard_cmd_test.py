import time
import msvcrt
import os
import roslibpy
import sys

MOTION_ALLOWED = "--real" in sys.argv[1:]

JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "172.31.28.252")
ROS_PORT = 9090

# ========= 固定速度檔位 =========
# 按 1~5 可以直接切換速度，方便觀察速度是否真的有差
SPEED_PRESETS = {
    "1": (0.005, 0.02),
    "2": (0.010, 0.05),
    "3": (0.020, 0.08),
    "4": (0.030, 0.12),
    "5": (0.050, 0.20),
}

linear_speed = 0.005
angular_speed = 0.03

current_vx = 0.0
current_vy = 0.0
current_wz = 0.0


def publish_cmd(cmd_pub, vx, vy, wz):
    cmd_pub.publish(roslibpy.Message({
        "linear": {
            "x": float(vx),
            "y": float(vy),
            "z": 0.0
        },
        "angular": {
            "x": 0.0,
            "y": 0.0,
            "z": float(wz)
        }
    }))


def print_help():
    print()
    print("========== Keyboard CMD Test ==========")
    print("1 : linear=0.005, angular=0.03")
    print("2 : linear=0.010, angular=0.05")
    print("3 : linear=0.020, angular=0.08")
    print("4 : linear=0.030, angular=0.12")
    print("5 : linear=0.050, angular=0.20")
    print("---------------------------------------")
    print("W : forward")
    print("S : backward")
    print("A : rotate left")
    print("D : rotate right")
    print("Q : strafe left")
    print("E : strafe right")
    print("SPACE : stop")
    print("X : exit")
    print("=======================================")
    print()


def main():
    global linear_speed, angular_speed
    global current_vx, current_vy, current_wz

    if not MOTION_ALLOWED:
        print("[SAFETY] ROS keyboard test not started. Re-run with --real in a clear test area.")
        return

    print(f"[INFO] Connecting to Jetson ROSBridge {JETSON_IP}:{ROS_PORT} ...")

    client = roslibpy.Ros(host=JETSON_IP, port=ROS_PORT)
    client.run()

    cmd_pub = roslibpy.Topic(client, "/cmd_vel", "geometry_msgs/Twist")
    cmd_pub.advertise()

    print("[INFO] Connected.")
    print_help()

    try:
        last_pub = time.time()

        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode("utf-8", errors="ignore").lower()

                if key in SPEED_PRESETS:
                    linear_speed, angular_speed = SPEED_PRESETS[key]
                    current_vx = 0.0
                    current_vy = 0.0
                    current_wz = 0.0
                    print(f"[PRESET {key}] linear={linear_speed:.3f}, angular={angular_speed:.3f}")

                elif key == "w":
                    current_vx = linear_speed
                    current_vy = 0.0
                    current_wz = 0.0

                elif key == "s":
                    current_vx = -linear_speed
                    current_vy = 0.0
                    current_wz = 0.0

                elif key == "a":
                    current_vx = 0.0
                    current_vy = 0.0
                    current_wz = angular_speed

                elif key == "d":
                    current_vx = 0.0
                    current_vy = 0.0
                    current_wz = -angular_speed

                elif key == "q":
                    current_vx = 0.0
                    current_vy = linear_speed
                    current_wz = 0.0

                elif key == "e":
                    current_vx = 0.0
                    current_vy = -linear_speed
                    current_wz = 0.0

                elif key == " ":
                    current_vx = 0.0
                    current_vy = 0.0
                    current_wz = 0.0

                elif key == "x":
                    print("[INFO] Exit.")
                    break

                print(
                    f"[CMD] vx={current_vx:.3f}, vy={current_vy:.3f}, wz={current_wz:.3f} | "
                    f"preset linear={linear_speed:.3f}, angular={angular_speed:.3f}"
                )

            # 10 Hz 持續送目前指令
            now = time.time()
            if now - last_pub >= 0.1:
                publish_cmd(cmd_pub, current_vx, current_vy, current_wz)
                last_pub = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected.")

    finally:
        print("[INFO] Sending stop command...")
        for _ in range(10):
            publish_cmd(cmd_pub, 0.0, 0.0, 0.0)
            time.sleep(0.05)

        try:
            cmd_pub.unadvertise()
        except Exception:
            pass

        try:
            client.terminate()
        except Exception:
            pass

        print("[INFO] Closed.")


if __name__ == "__main__":
    main()
