import cv2
import math
import os
import time
import sys
import numpy as np
import roslibpy
from stable_baselines3 import PPO
from ultralytics import YOLO

# ========= 1. 檔案絕對路徑 =========
RL_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v3i.yolov11 (1)\best_model.zip"
YOLO_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v3i.yolov11 (1)\runs\detect\train6\weights\best.pt"

# ========= 2. 網路與相機串流設定 =========
JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "172.31.28.252")
ROS_PORT = 9090

TOPIC_REAR = "/back_cam/image_raw"
TOPIC_ARM = "/arm_cam/image_raw"

URL_REAR = f"http://{JETSON_IP}:8080/stream?topic={TOPIC_REAR}"
URL_ARM = f"http://{JETSON_IP}:8080/stream?topic={TOPIC_ARM}"

# ========= 3. 物理與安全參數設定 =========
MAX_LINEAR_VEL = 0.2
MAX_ANGULAR_VEL = 0.8

# Windows 端比例。之後你主要應該去 Jetson 底層改馬達比例。
SAFE_SPEED_RATIO = 0.5
SAFE_TURN_RATIO = 0.5

# Windows 端硬限制，避免 PPO 輸出異常。
# 如果你去底層改完速度後，這裡可以再慢慢放大。
LINEAR_HARD_LIMIT = 0.03
ANGULAR_HARD_LIMIT = 0.12

e_stop_active = False

latest_lidar_52 = np.ones(52) * 5.0
current_linear_vel = 0.0

# ========= 4. 脈衝速度控制 =========
# 你現在要去改底層，所以這裡先關掉，避免測試被干擾。
USE_PULSE_CONTROL = False

PULSE_ON_TIME = 0.08
PULSE_OFF_TIME = 0.45

last_pulse_time = time.time()
pulse_on = False

KEEP_TURN_WHEN_PULSE_OFF = False
TURN_WHEN_OFF_RATIO = 0.3

# ========= 5. 視覺距離與鏡頭切換設定 =========
FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2
CENTER_Y = FRAME_H // 2

# 用 YOLO 框高度比例估計距離
# bbox_height_ratio = 框高度 / 畫面高度
#
# 你可以現場調這兩個值：
# 垃圾很小、很遠時，可能是 0.05 ~ 0.15
# 垃圾快靠近車時，可能是 0.30 ~ 0.60
SWITCH_TO_ARM_HEIGHT_RATIO = 0.35
BACK_TO_REAR_HEIGHT_RATIO = 0.25

# 顯示上用的距離分段
FAR_RATIO = 0.12
MID_RATIO = 0.22
NEAR_RATIO = 0.32

# 如果你只想看畫面、不想車動，改成 False
# Legacy ROS experiment safety gate: cmd_vel motion requires an explicit --real.
ENABLE_MOTION = "--real" in sys.argv[1:]


# ========= 6. 核心運算函數 =========
def get_observation(lidar_data, goal_dist, goal_angle, current_vel):
    processed_lidar = np.clip(lidar_data / 5.0, 0, 1.0)

    obs = np.zeros(55)
    obs[0:52] = processed_lidar
    obs[52] = goal_dist
    obs[53] = goal_angle
    obs[54] = current_vel

    return obs.astype(np.float32)


def publish_stop(cmd_pub, repeat=5, interval=0.05):
    stop_msg = roslibpy.Message({
        'linear': {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0
        },
        'angular': {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0
        }
    })

    for _ in range(repeat):
        cmd_pub.publish(stop_msg)
        time.sleep(interval)


def get_distance_state(bbox_height_ratio):
    """
    用 YOLO 框高度比例，估計垃圾距離狀態。
    這不是真實公分，是視覺相對距離。
    """
    if bbox_height_ratio < FAR_RATIO:
        return "FAR", "keep approaching"
    elif bbox_height_ratio < MID_RATIO:
        return "MID", "target visible"
    elif bbox_height_ratio < SWITCH_TO_ARM_HEIGHT_RATIO:
        return "NEAR", "prepare switch"
    else:
        return "SWITCH", "switch to arm cam"


def draw_target_dashboard(
    dashboard,
    target_found,
    active_camera,
    bbox_height_ratio,
    bbox_area_ratio,
    goal_angle,
    send_linear_x,
    send_angular_z,
    distance_state,
    distance_hint
):
    """
    在合併後的 dashboard 上畫更直覺的狀態面板。
    """
    panel_x = FRAME_W + 20
    panel_y = 20
    panel_w = 470
    panel_h = 190

    # 背景面板
    overlay = dashboard.copy()
    cv2.rectangle(
        overlay,
        (panel_x, panel_y),
        (panel_x + panel_w, panel_y + panel_h),
        (0, 0, 0),
        -1
    )
    dashboard[:] = cv2.addWeighted(overlay, 0.45, dashboard, 0.55, 0)

    # 基本資訊
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
            "Robot stopped. Waiting for YOLO detection.",
            (panel_x + 15, panel_y + 110),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1
        )
        return

    # 距離狀態
    cv2.putText(
        dashboard,
        f"DISTANCE STATE: {distance_state}",
        (panel_x + 15, panel_y + 65),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 0),
        2
    )

    cv2.putText(
        dashboard,
        f"HINT: {distance_hint}",
        (panel_x + 15, panel_y + 95),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        1
    )

    # 距離進度條：越滿代表越近
    bar_x = panel_x + 15
    bar_y = panel_y + 120
    bar_w = 350
    bar_h = 22

    closeness = np.clip(bbox_height_ratio / SWITCH_TO_ARM_HEIGHT_RATIO, 0.0, 1.0)
    fill_w = int(bar_w * closeness)

    cv2.rectangle(
        dashboard,
        (bar_x, bar_y),
        (bar_x + bar_w, bar_y + bar_h),
        (255, 255, 255),
        2
    )

    cv2.rectangle(
        dashboard,
        (bar_x, bar_y),
        (bar_x + fill_w, bar_y + bar_h),
        (0, 255, 0),
        -1
    )

    # 切換門檻線
    switch_x = bar_x + bar_w
    cv2.line(
        dashboard,
        (switch_x, bar_y - 8),
        (switch_x, bar_y + bar_h + 8),
        (0, 0, 255),
        2
    )

    cv2.putText(
        dashboard,
        "far",
        (bar_x, bar_y + 45),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1
    )

    cv2.putText(
        dashboard,
        "switch",
        (bar_x + bar_w - 55, bar_y + 45),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1
    )

    # 數值資訊
    cv2.putText(
        dashboard,
        f"box_h: {bbox_height_ratio:.3f} | box_area: {bbox_area_ratio:.3f}",
        (panel_x + 15, panel_y + 175),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1
    )

    # 右側速度資訊
    cv2.putText(
        dashboard,
        f"V: {send_linear_x:.3f}",
        (panel_x + 370, panel_y + 130),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        1
    )

    cv2.putText(
        dashboard,
        f"W: {send_angular_z:.3f}",
        (panel_x + 370, panel_y + 155),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        1
    )


def draw_direction_guide(frame, cx_box, cy_box, goal_angle):
    """
    畫目標位置與左右偏移提示。
    不畫圓圈，只畫中心線、目標線、方向提示。
    """
    # 畫畫面中心垂直線
    cv2.line(frame, (CENTER_X, 0), (CENTER_X, FRAME_H), (255, 255, 255), 1)

    # 畫目標中心
    cv2.circle(frame, (cx_box, cy_box), 6, (0, 255, 255), -1)

    # 畫目標到畫面中心的水平偏移
    cv2.line(frame, (CENTER_X, cy_box), (cx_box, cy_box), (0, 255, 255), 2)

    if goal_angle > 0.15:
        text = "TARGET LEFT"
    elif goal_angle < -0.15:
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


# ========= 7. 主程式 =========
def main():
    global current_linear_vel
    global e_stop_active
    global last_pulse_time
    global pulse_on

    if not ENABLE_MOTION:
        print("[SAFETY] Vision-only mode; add --real to permit /cmd_vel motion.")

    print("[INFO] 載入 YOLO 視覺神經...")
    yolo_model = YOLO(YOLO_MODEL_PATH)

    print("[INFO] 載入 RL 決策大腦...")
    rl_model = PPO.load(RL_MODEL_PATH)

    print(f"[INFO] 連線至 Jetson Nano ROS ({JETSON_IP}:{ROS_PORT})...")
    client = roslibpy.Ros(host=JETSON_IP, port=ROS_PORT)
    client.run()

    cmd_pub = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
    cmd_pub.advertise()

    print("[INFO] 開啟雙鏡頭串流...")
    cap_rear = cv2.VideoCapture(URL_REAR)
    cap_arm = cv2.VideoCapture(URL_ARM)

    active_camera = 'REAR'
    target_dt = 0.1

    try:
        while True:
            loop_start = time.time()

            # ========= 讀取雙鏡頭 =========
            ret_r, frame_rear = cap_rear.read()
            ret_a, frame_arm = cap_arm.read()

            if not ret_r or not ret_a:
                print("[WARN] 相機串流讀取失敗，暫停一下...")
                publish_stop(cmd_pub, repeat=1, interval=0.01)
                time.sleep(0.1)
                continue

            frame_rear = cv2.resize(frame_rear, (FRAME_W, FRAME_H))
            frame_arm = cv2.resize(frame_arm, (FRAME_W, FRAME_H))

            goal_dist = 5.0
            goal_angle = 0.0
            target_found = False

            bbox_height_ratio = 0.0
            bbox_area_ratio = 0.0
            distance_state = "NONE"
            distance_hint = "no target"

            cx_box = None
            cy_box = None

            # ========= 選擇目前推論鏡頭 =========
            frame_to_process = frame_rear if active_camera == 'REAR' else frame_arm

            results = yolo_model.predict(
                source=frame_to_process,
                conf=0.3,
                imgsz=640,
                verbose=False
            )[0]

            annotated_frame = results.plot()

            # ========= YOLO 目標處理 =========
            if results.boxes is not None and len(results.boxes) > 0:
                target_found = True

                # 目前先取第一個 box
                box = results.boxes[0]

                x1 = int(box.xyxy[0][0])
                y1 = int(box.xyxy[0][1])
                x2 = int(box.xyxy[0][2])
                y2 = int(box.xyxy[0][3])

                box_w = max(1, x2 - x1)
                box_h = max(1, y2 - y1)

                cx_box = int((x1 + x2) / 2)
                cy_box = int((y1 + y2) / 2)

                bbox_height_ratio = box_h / FRAME_H
                bbox_area_ratio = (box_w * box_h) / (FRAME_W * FRAME_H)

                distance_state, distance_hint = get_distance_state(bbox_height_ratio)

                # 目標角度：中心偏左為正，偏右為負
                goal_angle = (CENTER_X - cx_box) / CENTER_X

                # 給 RL 的 goal_dist：
                # 因為框越大代表越近，所以這裡轉成「越近越小」的距離感。
                #
                # bbox_height_ratio = 0.05 -> goal_dist 接近 1，代表遠
                # bbox_height_ratio = 0.35 -> goal_dist 接近 0，代表近
                goal_dist = 1.0 - np.clip(
                    bbox_height_ratio / SWITCH_TO_ARM_HEIGHT_RATIO,
                    0.0,
                    1.0
                )

                # 畫更直覺的目標方向提示
                draw_direction_guide(annotated_frame, cx_box, cy_box, goal_angle)

                # ========= 新的鏡頭切換邏輯 =========
                # 不再用「目標進圓圈」
                # 改成「YOLO 框變大」，代表垃圾更靠近車子。
                if active_camera == 'REAR' and bbox_height_ratio >= SWITCH_TO_ARM_HEIGHT_RATIO:
                    active_camera = 'ARM'
                    print("[INFO] 目標已足夠接近，切換到 ARM CAMERA")

                elif active_camera == 'ARM' and bbox_height_ratio <= BACK_TO_REAR_HEIGHT_RATIO:
                    active_camera = 'REAR'
                    print("[INFO] 目標變遠或手臂鏡頭不適合，切回 REAR CAMERA")

            # ========= 畫面顯示 =========
            if active_camera == 'REAR':
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

            # ========= 運動決策 =========
            if target_found:
                obs = get_observation(
                    latest_lidar_52,
                    goal_dist,
                    goal_angle,
                    current_linear_vel
                )

                action, _ = rl_model.predict(obs, deterministic=True)

                raw_linear = float(action[0]) * MAX_LINEAR_VEL * SAFE_SPEED_RATIO
                raw_angular = float(action[1]) * MAX_ANGULAR_VEL * SAFE_TURN_RATIO

                linear_x = float(np.clip(raw_linear, -LINEAR_HARD_LIMIT, LINEAR_HARD_LIMIT))
                angular_z = float(np.clip(raw_angular, -ANGULAR_HARD_LIMIT, ANGULAR_HARD_LIMIT))

            else:
                action = [0.0, 0.0]
                raw_linear = 0.0
                raw_angular = 0.0
                linear_x = 0.0
                angular_z = 0.0

                cv2.putText(
                    dashboard,
                    "TARGET LOST - STOP",
                    (500, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 0, 255),
                    3
                )

            # ========= 急停 =========
            if e_stop_active:
                linear_x = 0.0
                angular_z = 0.0

                cv2.putText(
                    dashboard,
                    "!!! E-STOP ACTIVE !!!",
                    (420, 240),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.5,
                    (0, 0, 255),
                    4
                )

            # ========= 脈衝控制，預設已關閉 =========
            send_linear_x = linear_x
            send_angular_z = angular_z

            if USE_PULSE_CONTROL and not e_stop_active:
                now = time.time()

                if pulse_on:
                    if now - last_pulse_time >= PULSE_ON_TIME:
                        pulse_on = False
                        last_pulse_time = now
                else:
                    if now - last_pulse_time >= PULSE_OFF_TIME:
                        pulse_on = True
                        last_pulse_time = now

                if not pulse_on:
                    send_linear_x = 0.0

                    if KEEP_TURN_WHEN_PULSE_OFF:
                        send_angular_z = angular_z * TURN_WHEN_OFF_RATIO
                    else:
                        send_angular_z = 0.0

            # 沒看到目標，一律停
            if not target_found:
                send_linear_x = 0.0
                send_angular_z = 0.0

            # 急停最高優先權
            if e_stop_active:
                send_linear_x = 0.0
                send_angular_z = 0.0

            # 如果只想測畫面，不想車動
            if not ENABLE_MOTION:
                send_linear_x = 0.0
                send_angular_z = 0.0

            current_linear_vel = send_linear_x

            # ========= 發送 cmd_vel =========
            cmd_pub.publish(roslibpy.Message({
                'linear': {
                    'x': send_linear_x,
                    'y': 0.0,
                    'z': 0.0
                },
                'angular': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': send_angular_z
                }
            }))

            # ========= 畫面儀表板 =========
            draw_target_dashboard(
                dashboard=dashboard,
                target_found=target_found,
                active_camera=active_camera,
                bbox_height_ratio=bbox_height_ratio,
                bbox_area_ratio=bbox_area_ratio,
                goal_angle=goal_angle,
                send_linear_x=send_linear_x,
                send_angular_z=send_angular_z,
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

            # ========= 終端機監視 =========
            print(
                f"Target: {target_found} | "
                f"DistState: {distance_state} | "
                f"box_h_ratio: {bbox_height_ratio:.3f} | "
                f"goal_dist: {goal_dist:.3f} | "
                f"angle: {goal_angle:.3f} | "
                f"Send V: {send_linear_x:.3f} | W: {send_angular_z:.3f} | "
                f"Mode: {active_camera}"
            )

            cv2.imshow("Twin-Cam AI Dashboard", dashboard)

            # ========= 鍵盤控制 =========
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("[INFO] 按下 Q，準備結束程式...")
                break

            elif key == ord("s"):
                e_stop_active = not e_stop_active

                if e_stop_active:
                    print("[E-STOP] 急停啟動！")
                    publish_stop(cmd_pub, repeat=3, interval=0.03)
                else:
                    print("[E-STOP] 急停解除。")

            # ========= 控制迴圈頻率 =========
            loop_time = time.time() - loop_start

            if loop_time < target_dt:
                time.sleep(target_dt - loop_time)

    except KeyboardInterrupt:
        print("\n🚨 偵測到 Ctrl+C！")

    finally:
        print("[INFO] 正在緊急煞車並關閉資源...")

        try:
            publish_stop(cmd_pub, repeat=10, interval=0.05)
        except Exception as e:
            print(f"[WARN] 煞車訊息發送失敗：{e}")

        try:
            cmd_pub.unadvertise()
        except Exception:
            pass

        try:
            client.terminate()
        except Exception:
            pass

        cap_rear.release()
        cap_arm.release()
        cv2.destroyAllWindows()

        print("[INFO] 程式已安全結束。")


if __name__ == "__main__":
    main()
