import cv2
import time
import json
import os
import socket
import sys
import threading
import numpy as np
from ultralytics import YOLO


# ============================================================
# 1. 模型路徑
# ============================================================

YOLO_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v3i.yolov11 (1)\runs\detect\train6\weights\best.pt"


# ============================================================
# 2. Jetson 網路設定
# ============================================================

JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "172.31.28.252")

# 後鏡頭串流
TOPIC_REAR = "/back_cam/image_raw"
URL_REAR = f"http://{JETSON_IP}:8080/stream?topic={TOPIC_REAR}"

# motor server
MOTOR_PORT = 7000


# ============================================================
# 3. 畫面設定
# ============================================================

FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2


# ============================================================
# 4. 校正結果
# ============================================================

# 你量到的 vx：
# vx = 0.006995 * speed
KX = 0.006995

# 你量到的 wz 資料估出來的近似值：
# wz ≈ 0.02384 * speed
KZ = 0.02384


# ============================================================
# 5. 控制參數
# ============================================================

# Legacy experiment safety gate: non-stop commands require an explicit --real.
ENABLE_MOTION = "--real" in sys.argv[1:]

# 持續重送 action，避免底層 timeout 自己停
COMMAND_INTERVAL = 0.05

# 沒有新畫面超過這個時間就停車
MAX_FRAME_AGE = 0.8

# YOLO
YOLO_CONF = 0.3

# 目標偏移死區
ANGLE_DEADZONE = 0.06

# 希望前進速度 m/s
# 用 KX 反推 speed
FAR_VX_MPS = 0.24
MID_VX_MPS = 0.20
NEAR_VX_MPS = 0.15

# speed 限制
MIN_SPEED = 28
MAX_SPEED = 48

# 轉向模式門檻
# 目標偏移很小：forward
# 偏移中等：curve_left / curve_right
# 偏移太大：turn_left / turn_right 原地修正一下
CURVE_ANGLE_THRESHOLD = 0.06
TURN_ANGLE_THRESHOLD = 0.42

# curve 的速度會用 base_speed
# turn 的速度會稍微低一點，避免原地轉太猛
TURN_SPEED_RATIO = 0.85

# 距離判斷：用 bbox 高度比例
# 這些你可以現場調
FAR_RATIO = 0.12
MID_RATIO = 0.22
NEAR_RATIO = 0.30

# 到這個比例就停車，交給 ARM camera
SWITCH_TO_ARM_HEIGHT_RATIO = 0.34

# 如果目標消失，停車
STOP_WHEN_TARGET_LOST = True


# ============================================================
# 6. 狀態
# ============================================================

e_stop_active = False
ready_for_arm = False

last_action = "stop"
last_speed = 0


# ============================================================
# 7. 只保留最新畫面的讀取器
# ============================================================

class LatestFrameReader:
    """
    背景執行緒一直讀 camera stream。
    主程式每次只拿最新 frame，避免 OpenCV buffer 堆積造成延遲。
    """

    def __init__(self, url, width=640, height=480):
        self.url = url
        self.width = width
        self.height = height

        self.cap = cv2.VideoCapture(url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.latest_frame = None
        self.latest_time = 0.0
        self.running = True
        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self._reader_loop)
        self.thread.daemon = True
        self.thread.start()

    def _reader_loop(self):
        while self.running:
            ret, frame = self.cap.read()

            if ret:
                frame = cv2.resize(frame, (self.width, self.height))

                with self.lock:
                    self.latest_frame = frame
                    self.latest_time = time.time()
            else:
                time.sleep(0.01)

    def read(self):
        with self.lock:
            if self.latest_frame is None:
                return False, None, 0.0

            return True, self.latest_frame.copy(), self.latest_time

    def release(self):
        self.running = False
        time.sleep(0.1)
        self.cap.release()


# ============================================================
# 8. TCP motor 控制
# ============================================================

def connect_motor_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((JETSON_IP, MOTOR_PORT))
    sock.settimeout(1.0)
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


def stop_motor(sock, repeat=8, interval=0.03):
    for _ in range(repeat):
        try:
            send_motor_action(sock, "stop", 0)
        except Exception:
            pass
        time.sleep(interval)


# ============================================================
# 9. YOLO 目標選擇
# ============================================================

def select_largest_box(results):
    """
    如果畫面裡有多個偵測框，選面積最大的那個。
    """
    if results.boxes is None or len(results.boxes) == 0:
        return None

    best_box = None
    best_area = -1

    for box in results.boxes:
        x1 = int(box.xyxy[0][0])
        y1 = int(box.xyxy[0][1])
        x2 = int(box.xyxy[0][2])
        y2 = int(box.xyxy[0][3])

        area = max(1, x2 - x1) * max(1, y2 - y1)

        if area > best_area:
            best_area = area
            best_box = box

    return best_box


# ============================================================
# 10. 距離與控制決策
# ============================================================

def speed_from_vx(vx_mps):
    """
    用你量到的 KX 反推 motor speed。
    vx = KX * speed
    speed = vx / KX
    """
    if KX <= 1e-9:
        return MIN_SPEED

    speed = vx_mps / KX
    speed = int(round(np.clip(speed, MIN_SPEED, MAX_SPEED)))
    return speed


def get_distance_state(bbox_height_ratio):
    if bbox_height_ratio <= 0:
        return "NO_TARGET"

    if bbox_height_ratio < FAR_RATIO:
        return "FAR"

    elif bbox_height_ratio < MID_RATIO:
        return "MID"

    elif bbox_height_ratio < SWITCH_TO_ARM_HEIGHT_RATIO:
        return "NEAR"

    else:
        return "READY_ARM"


def choose_base_speed(distance_state):
    """
    根據距離決定 forward/curve 的 speed。
    """
    if distance_state == "FAR":
        return speed_from_vx(FAR_VX_MPS)

    elif distance_state == "MID":
        return speed_from_vx(MID_VX_MPS)

    elif distance_state == "NEAR":
        return speed_from_vx(NEAR_VX_MPS)

    else:
        return 0


def decide_rear_action(target_found, goal_angle, bbox_height_ratio):
    """
    連續追蹤決策：
    - 不用 pulse
    - 每一輪都輸出 action + speed
    - 外層會 keep-alive 連續送
    """

    if not target_found:
        return "stop", 0, "TARGET LOST", "NO_TARGET"

    distance_state = get_distance_state(bbox_height_ratio)

    if distance_state == "READY_ARM":
        return "stop", 0, "READY FOR ARM", distance_state

    base_speed = choose_base_speed(distance_state)

    # 中心附近：直走
    if abs(goal_angle) < CURVE_ANGLE_THRESHOLD:
        return "forward", base_speed, "CENTER: FORWARD", distance_state

    # 偏太多：先原地轉正一點
    if abs(goal_angle) >= TURN_ANGLE_THRESHOLD:
        turn_speed = int(round(base_speed * TURN_SPEED_RATIO))
        turn_speed = int(np.clip(turn_speed, MIN_SPEED, MAX_SPEED))

        if goal_angle > 0:
            return "turn_left", turn_speed, "LARGE OFFSET: TURN LEFT", distance_state
        else:
            return "turn_right", turn_speed, "LARGE OFFSET: TURN RIGHT", distance_state

    # 一般偏移：前進中彎曲
    if goal_angle > 0:
        return "curve_left", base_speed, "CURVE LEFT", distance_state
    else:
        return "curve_right", base_speed, "CURVE RIGHT", distance_state


# ============================================================
# 11. 畫面儀表板
# ============================================================

def draw_dashboard(
    frame,
    target_found,
    goal_angle,
    bbox_height_ratio,
    bbox_area_ratio,
    action,
    speed,
    status_text,
    distance_state,
    frame_age,
    inference_time,
    ready_for_arm_flag
):
    # 中心線
    cv2.line(frame, (CENTER_X, 0), (CENTER_X, FRAME_H), (255, 255, 255), 1)

    # deadzone / curve threshold
    deadzone_px = int(CENTER_X * CURVE_ANGLE_THRESHOLD)
    cv2.line(frame, (CENTER_X - deadzone_px, 0), (CENTER_X - deadzone_px, FRAME_H), (0, 255, 255), 1)
    cv2.line(frame, (CENTER_X + deadzone_px, 0), (CENTER_X + deadzone_px, FRAME_H), (0, 255, 255), 1)

    # 面板
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (635, 235), (0, 0, 0), -1)
    frame[:] = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)

    cv2.putText(
        frame,
        "REAR CAMERA CONTINUOUS CURVE FOLLOW",
        (22, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2
    )

    target_text = "FOUND" if target_found else "LOST"
    target_color = (0, 255, 0) if target_found else (0, 0, 255)

    cv2.putText(
        frame,
        f"TARGET: {target_text} | DIST: {distance_state}",
        (22, 72),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        target_color,
        2
    )

    cv2.putText(
        frame,
        f"ANGLE: {goal_angle:.3f} | box_h: {bbox_height_ratio:.3f} | area: {bbox_area_ratio:.3f}",
        (22, 102),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        1
    )

    cv2.putText(
        frame,
        f"ACTION: {action} | SPEED: {speed}",
        (22, 132),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (0, 255, 255),
        2
    )

    cv2.putText(
        frame,
        f"STATUS: {status_text}",
        (22, 162),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1
    )

    cv2.putText(
        frame,
        f"frame_age: {frame_age:.3f}s | yolo: {inference_time:.3f}s",
        (22, 190),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        1
    )

    cv2.putText(
        frame,
        f"KX: {KX:.6f} | KZ: {KZ:.5f} | READY_ARM: {ready_for_arm_flag}",
        (22, 218),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 255),
        1
    )

    cv2.putText(
        frame,
        "Q: Quit | S: E-STOP | R: Reset READY_ARM",
        (22, 462),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (255, 255, 255),
        2
    )


# ============================================================
# 12. 主程式
# ============================================================

def main():
    global e_stop_active
    global ready_for_arm
    global last_action
    global last_speed

    if not ENABLE_MOTION:
        print("[SAFETY] Vision-only mode; add --real to permit motor motion.")

    print("[INFO] Loading YOLO...")
    yolo_model = YOLO(YOLO_MODEL_PATH)

    print(f"[INFO] Connecting Jetson motor server {JETSON_IP}:{MOTOR_PORT} ...")
    motor_sock = connect_motor_server()
    print("[INFO] motor server connected.")

    print("[INFO] Opening REAR camera stream...")
    cap_rear = LatestFrameReader(URL_REAR, FRAME_W, FRAME_H)

    last_processed_frame_time = 0.0
    last_command_time = 0.0

    try:
        while True:
            loop_start = time.time()

            ret, frame, frame_time = cap_rear.read()

            if not ret:
                print("[WARN] Rear camera no frame. Stop.")
                stop_motor(motor_sock, repeat=1, interval=0.01)
                time.sleep(0.1)
                continue

            # 只處理新畫面
            if frame_time == last_processed_frame_time:
                # 但仍然要 keep-alive 發送上一個 action
                now = time.time()
                if ENABLE_MOTION and not e_stop_active and not ready_for_arm:
                    if now - last_command_time >= COMMAND_INTERVAL:
                        send_motor_action(motor_sock, last_action, last_speed)
                        last_command_time = now
                time.sleep(0.005)
                continue

            last_processed_frame_time = frame_time
            frame_age = time.time() - frame_time

            # 畫面太舊，停車
            if frame_age > MAX_FRAME_AGE:
                print(f"[WARN] frame too old: {frame_age:.2f}s, stop")
                last_action = "stop"
                last_speed = 0
                stop_motor(motor_sock, repeat=1, interval=0.01)

                cv2.putText(
                    frame,
                    f"FRAME TOO OLD: {frame_age:.2f}s",
                    (35, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 0, 255),
                    2
                )

                cv2.imshow("Rear Continuous Curve Follow", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("s"):
                    e_stop_active = not e_stop_active
                    stop_motor(motor_sock)
                elif key == ord("r"):
                    ready_for_arm = False
                    print("[RESET] READY_ARM = False")

                continue

            # READY_ARM 狀態：停車等你接手臂
            if ready_for_arm:
                last_action = "stop"
                last_speed = 0
                stop_motor(motor_sock, repeat=1, interval=0.01)

                display = frame.copy()
                cv2.putText(
                    display,
                    "READY FOR ARM CAMERA",
                    (90, 230),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 255, 0),
                    3
                )
                cv2.putText(
                    display,
                    "Press R to reset | Q to quit",
                    (125, 280),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (255, 255, 255),
                    2
                )

                cv2.imshow("Rear Continuous Curve Follow", display)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("r"):
                    ready_for_arm = False
                    print("[RESET] READY_ARM = False")
                elif key == ord("s"):
                    e_stop_active = not e_stop_active
                    stop_motor(motor_sock)

                continue

            # YOLO 推論
            yolo_start = time.time()
            results = yolo_model.predict(
                source=frame,
                conf=YOLO_CONF,
                imgsz=640,
                verbose=False
            )[0]
            inference_time = time.time() - yolo_start

            annotated = results.plot()

            target_found = False
            goal_angle = 0.0
            bbox_height_ratio = 0.0
            bbox_area_ratio = 0.0
            distance_state = "NO_TARGET"

            action = "stop"
            speed = 0
            status_text = "NO TARGET"

            box = select_largest_box(results)

            if box is not None:
                target_found = True

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

                # 目標偏左為正，偏右為負
                goal_angle = (CENTER_X - cx_box) / CENTER_X

                # 畫目標中心與偏移線
                cv2.circle(annotated, (cx_box, cy_box), 6, (0, 255, 255), -1)
                cv2.line(
                    annotated,
                    (CENTER_X, cy_box),
                    (cx_box, cy_box),
                    (0, 255, 255),
                    2
                )

                action, speed, status_text, distance_state = decide_rear_action(
                    target_found=True,
                    goal_angle=goal_angle,
                    bbox_height_ratio=bbox_height_ratio
                )

                if distance_state == "READY_ARM":
                    ready_for_arm = True
                    action = "stop"
                    speed = 0
                    status_text = "READY FOR ARM CAMERA"
                    stop_motor(motor_sock, repeat=5, interval=0.03)
                    print("[INFO] Target close enough. READY FOR ARM CAMERA.")

            else:
                target_found = False
                distance_state = "NO_TARGET"

                if STOP_WHEN_TARGET_LOST:
                    action = "stop"
                    speed = 0
                    status_text = "TARGET LOST - STOP"
                else:
                    action = "stop"
                    speed = 0
                    status_text = "TARGET LOST"

            # 急停 / 關閉運動
            if e_stop_active:
                action = "stop"
                speed = 0
                status_text = "E-STOP"

            elif not ENABLE_MOTION:
                action = "stop"
                speed = 0
                status_text = "MOTION DISABLED"

            # 更新上一個 action
            last_action = action
            last_speed = speed

            # keep-alive 連續送 action
            now = time.time()
            if now - last_command_time >= COMMAND_INTERVAL:
                send_motor_action(motor_sock, last_action, last_speed)
                last_command_time = now

            # dashboard
            draw_dashboard(
                frame=annotated,
                target_found=target_found,
                goal_angle=goal_angle,
                bbox_height_ratio=bbox_height_ratio,
                bbox_area_ratio=bbox_area_ratio,
                action=action,
                speed=speed,
                status_text=status_text,
                distance_state=distance_state,
                frame_age=frame_age,
                inference_time=inference_time,
                ready_for_arm_flag=ready_for_arm
            )

            print(
                f"Target:{target_found} | "
                f"Dist:{distance_state} | "
                f"angle:{goal_angle:.3f} | "
                f"box_h:{bbox_height_ratio:.3f} | "
                f"action:{action} | speed:{speed} | "
                f"status:{status_text}"
            )

            cv2.imshow("Rear Continuous Curve Follow", annotated)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("[INFO] Q pressed. Quit.")
                break

            elif key == ord("s"):
                e_stop_active = not e_stop_active
                if e_stop_active:
                    print("[E-STOP] ON")
                    stop_motor(motor_sock)
                else:
                    print("[E-STOP] OFF")

            elif key == ord("r"):
                ready_for_arm = False
                print("[RESET] READY_ARM = False")

            loop_time = time.time() - loop_start
            if loop_time < 0.01:
                time.sleep(0.01 - loop_time)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected.")

    finally:
        print("[INFO] Stopping and closing resources...")

        try:
            last_action = "stop"
            last_speed = 0
            stop_motor(motor_sock, repeat=10, interval=0.03)
        except Exception as e:
            print(f"[WARN] stop failed: {e}")

        try:
            motor_sock.close()
        except Exception:
            pass

        try:
            cap_rear.release()
        except Exception:
            pass

        cv2.destroyAllWindows()

        print("[INFO] Program ended safely.")


if __name__ == "__main__":
    main()
