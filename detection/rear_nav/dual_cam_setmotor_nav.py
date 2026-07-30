import cv2
import math
import time
import json
import os
import socket
import sys
import numpy as np
from stable_baselines3 import PPO
from ultralytics import YOLO

# ========= 1. 檔案絕對路徑 =========
RL_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v3i.yolov11 (1)\best_model.zip"
YOLO_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v3i.yolov11 (1)\runs\detect\train6\weights\best.pt"

# ========= 2. Jetson 網路與相機串流設定 =========
JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "172.31.28.252")

# 影像還是走 web_video_server
TOPIC_REAR = "/back_cam/image_raw"
TOPIC_ARM = "/arm_cam/image_raw"

URL_REAR = f"http://{JETSON_IP}:8080/stream?topic={TOPIC_REAR}"
URL_ARM = f"http://{JETSON_IP}:8080/stream?topic={TOPIC_ARM}"

# 馬達控制改走 TCP set_motor server
MOTOR_PORT = 7000

# ========= 3. 畫面設定 =========
FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2
CENTER_Y = FRAME_H // 2

# ========= 4. 後置相機距離估算參數 =========
# 你之前校正後的值
H_REAR = 0.50
FY_REAR = 600.0
CY_REAR = 240.0
THETA_REAR = 13.0

# ========= 5. 鏡頭切換距離 =========
SWITCH_TO_ARM_DIST_M = 0.80
BACK_TO_REAR_DIST_M = 0.90

# ========= 6. 馬達控制參數 =========
# Legacy experiment safety gate: non-stop commands require an explicit --real.
ENABLE_MOTION = "--real" in sys.argv[1:]

# 先固定速度 30
DRIVE_SPEED = 30
TURN_SPEED = 30

# 目標偏移死區，越大越不容易轉彎
ANGLE_DEADZONE = 0.18

# 如果目標非常偏左/偏右，原地轉；小偏移則曲線修正
HARD_TURN_ANGLE = 0.45

e_stop_active = False

latest_lidar_52 = np.ones(52) * 5.0
current_linear_vel = 0.0


# ========= 7. TCP 馬達控制 =========
def connect_motor_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((JETSON_IP, MOTOR_PORT))
    return sock


def send_motor_action(sock, action, speed):
    if not ENABLE_MOTION and action != "stop":
        action, speed = "stop", 0
    msg = {
        "action": str(action),
        "speed": int(speed)
    }
    data = (json.dumps(msg) + "\n").encode("utf-8")
    sock.sendall(data)


def stop_motor(sock, repeat=8, interval=0.05):
    try:
        for _ in range(repeat):
            send_motor_action(sock, "stop", 0)
            time.sleep(interval)
    except Exception:
        pass


# ========= 8. 距離估算 =========
def estimate_rear_distance_m(y_max, theta_deg=THETA_REAR):
    theta = math.radians(theta_deg)
    alpha = math.atan((y_max - CY_REAR) / FY_REAR)
    total_angle = theta + alpha

    if total_angle <= 0:
        return None

    dist = H_REAR / math.tan(total_angle)

    if dist <= 0 or dist > 10:
        return None

    return dist


def get_distance_state(distance_m):
    if distance_m is None:
        return "UNKNOWN", "distance unavailable"

    if distance_m > 1.50:
        return "FAR", "keep approaching"

    elif distance_m > 1.00:
        return "MID", "target visible"

    elif distance_m > SWITCH_TO_ARM_DIST_M:
        return "NEAR", "prepare switch"

    else:
        return "SWITCH", "switch to arm cam"


# ========= 9. RL observation 保留，但這版先不用 PPO 控馬達 =========
def get_observation(lidar_data, goal_dist, goal_angle, current_vel):
    processed_lidar = np.clip(lidar_data / 5.0, 0, 1.0)

    obs = np.zeros(55)
    obs[0:52] = processed_lidar
    obs[52] = goal_dist
    obs[53] = goal_angle
    obs[54] = current_vel

    return obs.astype(np.float32)


# ========= 10. 畫面輔助 =========
def draw_direction_guide(frame, cx_box, cy_box, goal_angle):
    cv2.line(frame, (CENTER_X, 0), (CENTER_X, FRAME_H), (255, 255, 255), 1)
    cv2.circle(frame, (cx_box, cy_box), 6, (0, 255, 255), -1)
    cv2.line(frame, (CENTER_X, cy_box), (cx_box, cy_box), (0, 255, 255), 2)

    if goal_angle > ANGLE_DEADZONE:
        text = "TARGET LEFT"
    elif goal_angle < -ANGLE_DEADZONE:
        text = "TARGET RIGHT"
    else:
        text = "TARGET CENTERED"

    cv2.putText(
        frame,
        text,
        (20, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2
    )


def draw_target_dashboard(
    dashboard,
    target_found,
    active_camera,
    distance_m,
    goal_angle,
    motor_action,
    motor_speed,
    distance_state,
    distance_hint
):
    panel_x = FRAME_W + 20
    panel_y = 20
    panel_w = 470
    panel_h = 210

    overlay = dashboard.copy()
    cv2.rectangle(
        overlay,
        (panel_x, panel_y),
        (panel_x + panel_w, panel_y + panel_h),
        (0, 0, 0),
        -1
    )
    dashboard[:] = cv2.addWeighted(overlay, 0.45, dashboard, 0.55, 0)

    cv2.putText(
        dashboard,
        f"CAMERA MODE: {active_camera}",
        (panel_x + 15, panel_y + 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2
    )

    if not target_found:
        cv2.putText(
            dashboard,
            "TARGET: LOST",
            (panel_x + 15, panel_y + 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )
        cv2.putText(
            dashboard,
            "Motor: STOP",
            (panel_x + 15, panel_y + 110),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )
        return

    dist_text = "N/A" if distance_m is None else f"{distance_m:.2f} m"

    cv2.putText(
        dashboard,
        f"DISTANCE: {dist_text}",
        (panel_x + 15, panel_y + 65),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 0),
        2
    )

    cv2.putText(
        dashboard,
        f"STATE: {distance_state}",
        (panel_x + 15, panel_y + 95),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        1
    )

    cv2.putText(
        dashboard,
        f"HINT: {distance_hint}",
        (panel_x + 15, panel_y + 125),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (255, 255, 255),
        1
    )

    cv2.putText(
        dashboard,
        f"ANGLE: {goal_angle:.3f}",
        (panel_x + 15, panel_y + 155),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (255, 255, 255),
        1
    )

    cv2.putText(
        dashboard,
        f"MOTOR: {motor_action} | SPEED: {motor_speed}",
        (panel_x + 15, panel_y + 190),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2
    )


# ========= 11. 根據 YOLO 目標決定 motor action =========
def decide_motor_action(target_found, goal_angle, active_camera, distance_m):
    if not target_found:
        return "stop", 0

    # 到達近距離時先停，避免撞上垃圾
    if distance_m is not None and active_camera == "ARM" and distance_m < 0.45:
        return "stop", 0

    # 目標極度偏左/右：原地轉比較快對準
    if goal_angle > HARD_TURN_ANGLE:
        return "turn_left", TURN_SPEED

    if goal_angle < -HARD_TURN_ANGLE:
        return "turn_right", TURN_SPEED

    # 目標中度偏左/右：曲線修正
    if goal_angle > ANGLE_DEADZONE:
        return "curve_left", TURN_SPEED

    if goal_angle < -ANGLE_DEADZONE:
        return "curve_right", TURN_SPEED

    # 置中：前進
    return "forward", DRIVE_SPEED


# ========= 12. 主程式 =========
def main():
    global current_linear_vel
    global e_stop_active

    if not ENABLE_MOTION:
        print("[SAFETY] Vision-only mode; add --real to permit motor motion.")

    print("[INFO] 載入 YOLO 視覺神經...")
    yolo_model = YOLO(YOLO_MODEL_PATH)

    print("[INFO] 載入 RL 決策大腦，這版先保留載入但不直接控制馬達...")
    rl_model = PPO.load(RL_MODEL_PATH)

    print(f"[INFO] 連線至 Jetson motor server ({JETSON_IP}:{MOTOR_PORT})...")
    motor_sock = connect_motor_server()
    print("[INFO] motor server connected.")

    print("[INFO] 開啟雙鏡頭串流...")
    cap_rear = cv2.VideoCapture(URL_REAR)
    cap_arm = cv2.VideoCapture(URL_ARM)

    active_camera = "REAR"
    target_dt = 0.1

    last_sent_action = None
    last_sent_speed = None

    try:
        while True:
            loop_start = time.time()

            ret_r, frame_rear = cap_rear.read()
            ret_a, frame_arm = cap_arm.read()

            if not ret_r or not ret_a:
                print("[WARN] 相機串流讀取失敗，停車...")
                stop_motor(motor_sock, repeat=1, interval=0.01)
                time.sleep(0.1)
                continue

            frame_rear = cv2.resize(frame_rear, (FRAME_W, FRAME_H))
            frame_arm = cv2.resize(frame_arm, (FRAME_W, FRAME_H))

            target_found = False
            goal_angle = 0.0
            goal_dist = 5.0

            distance_m = None
            distance_state = "NONE"
            distance_hint = "no target"

            cx_box = None
            cy_box = None

            motor_action = "stop"
            motor_speed = 0

            frame_to_process = frame_rear if active_camera == "REAR" else frame_arm

            results = yolo_model.predict(
                source=frame_to_process,
                conf=0.3,
                imgsz=640,
                verbose=False
            )[0]

            annotated_frame = results.plot()

            if results.boxes is not None and len(results.boxes) > 0:
                target_found = True

                # 取第一個目標
                box = results.boxes[0]

                x1 = int(box.xyxy[0][0])
                y1 = int(box.xyxy[0][1])
                x2 = int(box.xyxy[0][2])
                y2 = int(box.xyxy[0][3])

                box_w = max(1, x2 - x1)
                box_h = max(1, y2 - y1)

                cx_box = int((x1 + x2) / 2)
                cy_box = int((y1 + y2) / 2)

                # 目標偏左為正，偏右為負
                goal_angle = (CENTER_X - cx_box) / CENTER_X

                # 後置鏡頭才使用校正過的距離公式
                if active_camera == "REAR":
                    distance_m = estimate_rear_distance_m(y2)
                else:
                    # ARM 目前先用框高比例粗估，之後可再校正
                    bbox_height_ratio = box_h / FRAME_H
                    distance_m = max(0.2, 1.0 - bbox_height_ratio)

                distance_state, distance_hint = get_distance_state(distance_m)

                if distance_m is not None:
                    goal_dist = np.clip(distance_m / 2.0, 0.0, 1.0)

                draw_direction_guide(annotated_frame, cx_box, cy_box, goal_angle)

                # 鏡頭切換
                if active_camera == "REAR" and distance_m is not None and distance_m <= SWITCH_TO_ARM_DIST_M:
                    active_camera = "ARM"
                    print("[INFO] 距離 <= 0.80 m，切換到 ARM CAMERA")

                elif active_camera == "ARM" and distance_m is not None and distance_m >= BACK_TO_REAR_DIST_M:
                    active_camera = "REAR"
                    print("[INFO] 距離 >= 0.90 m，切回 REAR CAMERA")

                # RL observation 保留，之後要接 PPO 時可用
                obs = get_observation(
                    latest_lidar_52,
                    goal_dist,
                    goal_angle,
                    current_linear_vel
                )

                _rl_action, _ = rl_model.predict(obs, deterministic=True)

            else:
                target_found = False
                distance_state = "NONE"
                distance_hint = "target lost"

            # 顯示左右畫面
            if active_camera == "REAR":
                display_rear = annotated_frame
                display_arm = frame_arm

                cv2.putText(
                    display_rear,
                    "[ ACTIVE: SEARCH / APPROACH ]",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 255, 0),
                    2
                )

            else:
                display_rear = frame_rear
                display_arm = annotated_frame

                cv2.putText(
                    display_arm,
                    "[ ACTIVE: CLOSE POSITIONING ]",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 255, 0),
                    2
                )

            dashboard = cv2.hconcat([display_rear, display_arm])

            cv2.putText(
                dashboard,
                "REAR CAMERA",
                (20, 455),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

            cv2.putText(
                dashboard,
                "ARM CAMERA",
                (660, 455),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

            # 決策 motor action
            motor_action, motor_speed = decide_motor_action(
                target_found=target_found,
                goal_angle=goal_angle,
                active_camera=active_camera,
                distance_m=distance_m
            )

            if e_stop_active or not ENABLE_MOTION:
                motor_action = "stop"
                motor_speed = 0

            # 發送 motor action
            send_motor_action(motor_sock, motor_action, motor_speed)

            last_sent_action = motor_action
            last_sent_speed = motor_speed

            if motor_action == "forward":
                current_linear_vel = motor_speed / 100.0
            else:
                current_linear_vel = 0.0

            # 儀表板
            draw_target_dashboard(
                dashboard=dashboard,
                target_found=target_found,
                active_camera=active_camera,
                distance_m=distance_m,
                goal_angle=goal_angle,
                motor_action=motor_action,
                motor_speed=motor_speed,
                distance_state=distance_state,
                distance_hint=distance_hint
            )

            cv2.putText(
                dashboard,
                "Press S: E-STOP | Press Q: Quit",
                (20, 235),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2
            )

            if e_stop_active:
                cv2.putText(
                    dashboard,
                    "!!! E-STOP ACTIVE !!!",
                    (420, 240),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.5,
                    (0, 0, 255),
                    4
                )

            print(
                f"Target: {target_found} | "
                f"DistState: {distance_state} | "
                f"Dist: {distance_m} | "
                f"Angle: {goal_angle:.3f} | "
                f"Motor: {motor_action} | Speed: {motor_speed} | "
                f"Mode: {active_camera}"
            )

            cv2.imshow("Twin-Cam SetMotor AI Dashboard", dashboard)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("[INFO] 按下 Q，準備結束程式...")
                break

            elif key == ord("s"):
                e_stop_active = not e_stop_active

                if e_stop_active:
                    print("[E-STOP] 急停啟動！")
                    stop_motor(motor_sock)
                else:
                    print("[E-STOP] 急停解除。")

            loop_time = time.time() - loop_start

            if loop_time < target_dt:
                time.sleep(target_dt - loop_time)

    except KeyboardInterrupt:
        print("\n🚨 偵測到 Ctrl+C！")

    finally:
        print("[INFO] 正在緊急煞車並關閉資源...")

        try:
            stop_motor(motor_sock)
            motor_sock.close()
        except Exception as e:
            print(f"[WARN] motor socket close failed: {e}")

        cap_rear.release()
        cap_arm.release()
        cv2.destroyAllWindows()

        print("[INFO] 程式已安全結束。")


if __name__ == "__main__":
    main()
