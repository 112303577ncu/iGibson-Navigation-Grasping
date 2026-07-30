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
# 1. 模型與 Jetson 設定
# ============================================================

YOLO_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v3i.yolov11 (1)\runs\detect\train6\weights\best.pt"

JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "172.31.28.252")

# 後鏡頭串流
TOPIC_REAR = "/back_cam/image_raw"
URL_REAR = f"http://{JETSON_IP}:8080/stream?topic={TOPIC_REAR}"

# 你舊程式用的 motor server
MOTOR_PORT = 7000


# ============================================================
# 2. 畫面設定
# ============================================================

FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2
CENTER_Y = FRAME_H // 2


# ============================================================
# 3. 控制參數
# ============================================================

# Legacy experiment safety gate: wheel motion requires an explicit --real.
ENABLE_MOTION = "--real" in sys.argv[1:]

# 馬達速度範圍，依照 Yahboom/Rosmaster 常見 set_motor 範圍
# 通常是 -100 ~ 100
MAX_MOTOR_SPEED = 45
MIN_MOTOR_SPEED = 18

# 前進基礎速度
BASE_VX = 32

# 左右斜移比例
# 目標越偏，vy 越大
KP_VY = 38

# 微轉向比例
# 不要太大，主要靠斜移，不靠原地轉
KP_WZ = 10

# 如果左右斜移方向反了，把這個改成 -1
Y_SIGN = 1

# 如果微轉向方向反了，把這個改成 -1
Z_SIGN = 1

# 中心死區，避免左右抖動
ANGLE_DEADZONE = 0.05

# 距離門檻：bbox 高度比例到這個值，代表夠近，可以切手臂
SWITCH_TO_ARM_HEIGHT_RATIO = 0.34

# 接近時減速
NEAR_SLOWDOWN_START_RATIO = 0.18

# YOLO 信心門檻
YOLO_CONF = 0.3

# 控制迴圈週期
TARGET_DT = 0.05

# 如果 frame 太舊，就停車
MAX_FRAME_AGE = 0.8


# ============================================================
# 4. 狀態
# ============================================================

e_stop_active = False
ready_for_arm = False


# ============================================================
# 5. 只保留最新畫面的讀取器
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
# 6. TCP 馬達控制
# ============================================================

def connect_motor_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((JETSON_IP, MOTOR_PORT))
    return sock


def send_json(sock, msg):
    data = (json.dumps(msg) + "\n").encode("utf-8")
    sock.sendall(data)


def send_motor_wheels(sock, fl, fr, rl, rr):
    """
    傳四顆馬達速度給 Jetson motor server。

    這裡假設你的 Jetson server 支援：
    {
        "action": "set_motor",
        "fl": int,
        "fr": int,
        "rl": int,
        "rr": int
    }

    如果你的 server 用 m1/m2/m3/m4，
    就把 key 改掉即可。
    """

    if not ENABLE_MOTION:
        fl = fr = rl = rr = 0
    msg = {
        "action": "set_motor",
        "fl": int(fl),
        "fr": int(fr),
        "rl": int(rl),
        "rr": int(rr),

        # 下面這四個一起送，是為了相容你 server 如果用 m1/m2/m3/m4
        "m1": int(fl),
        "m2": int(fr),
        "m3": int(rl),
        "m4": int(rr)
    }

    send_json(sock, msg)


def stop_motor(sock, repeat=5, interval=0.03):
    try:
        for _ in range(repeat):
            send_motor_wheels(sock, 0, 0, 0, 0)
            time.sleep(interval)
    except Exception:
        pass


# ============================================================
# 7. YOLO 目標選擇
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
# 8. 麥克納姆輪速度換算
# ============================================================

def apply_deadzone(value, deadzone):
    if abs(value) < deadzone:
        return 0.0
    return value


def normalize_motor_speeds(fl, fr, rl, rr, max_motor_speed):
    """
    如果四輪中有某顆超過最大速度，就等比例縮小。
    保留方向比例，不讓某顆爆掉。
    """
    max_abs = max(abs(fl), abs(fr), abs(rl), abs(rr), 1)

    if max_abs > max_motor_speed:
        scale = max_motor_speed / max_abs
        fl *= scale
        fr *= scale
        rl *= scale
        rr *= scale

    return fl, fr, rl, rr


def apply_min_motor_speed(value):
    """
    馬達速度太小會被靜摩擦吃掉。
    只要不是 0，就至少給 MIN_MOTOR_SPEED。
    """
    if abs(value) < 1e-6:
        return 0

    sign = 1 if value > 0 else -1
    value = sign * max(abs(value), MIN_MOTOR_SPEED)
    value = int(np.clip(value, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED))

    return value


def mecanum_mix(vx, vy, wz):
    """
    vx:
        前進速度，正值代表前進

    vy:
        左右速度，正值代表左移或右移，依照實車方向可能要用 Y_SIGN 調整

    wz:
        旋轉速度，正值代表左轉或右轉，依照實車方向可能要用 Z_SIGN 調整

    四輪順序假設：
        fl = front left
        fr = front right
        rl = rear left
        rr = rear right
    """

    fl = vx - vy - wz
    fr = vx + vy + wz
    rl = vx + vy - wz
    rr = vx - vy + wz

    fl, fr, rl, rr = normalize_motor_speeds(
        fl, fr, rl, rr,
        MAX_MOTOR_SPEED
    )

    fl = apply_min_motor_speed(fl)
    fr = apply_min_motor_speed(fr)
    rl = apply_min_motor_speed(rl)
    rr = apply_min_motor_speed(rr)

    return fl, fr, rl, rr


def compute_diagonal_follow(goal_angle, bbox_height_ratio):
    """
    goal_angle:
        > 0 代表垃圾在畫面左邊
        < 0 代表垃圾在畫面右邊

    這裡輸出 vx, vy, wz，不直接輸出四輪。
    """

    # 距離接近程度
    closeness = np.clip(
        bbox_height_ratio / SWITCH_TO_ARM_HEIGHT_RATIO,
        0.0,
        1.0
    )

    # 基礎前進速度，越靠近越慢
    vx = BASE_VX * (1.0 - 0.55 * closeness)

    # 如果目標很偏，前進速度再降一點，避免斜衝過頭
    vx *= (1.0 - 0.30 * min(abs(goal_angle), 1.0))

    # 中心附近不要左右修正
    if abs(goal_angle) < ANGLE_DEADZONE:
        vy = 0.0
        wz = 0.0
    else:
        # 主要靠左右斜移
        vy = Y_SIGN * KP_VY * goal_angle

        # 微轉向，不要太大
        wz = Z_SIGN * KP_WZ * goal_angle

    # 限制範圍
    vx = float(np.clip(vx, 0, MAX_MOTOR_SPEED))
    vy = float(np.clip(vy, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED))
    wz = float(np.clip(wz, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED))

    return vx, vy, wz


def get_distance_state(bbox_height_ratio):
    if bbox_height_ratio <= 0:
        return "NO_TARGET"
    elif bbox_height_ratio < NEAR_SLOWDOWN_START_RATIO:
        return "FAR"
    elif bbox_height_ratio < SWITCH_TO_ARM_HEIGHT_RATIO:
        return "NEAR"
    else:
        return "READY_ARM"


# ============================================================
# 9. 畫面顯示
# ============================================================

def draw_dashboard(
    frame,
    target_found,
    goal_angle,
    box_h_ratio,
    box_bottom_ratio,
    vx,
    vy,
    wz,
    fl,
    fr,
    rl,
    rr,
    distance_state,
    frame_age,
    inference_time
):
    # 中心線
    cv2.line(frame, (CENTER_X, 0), (CENTER_X, FRAME_H), (255, 255, 255), 1)

    # deadzone
    deadzone_px = int(CENTER_X * ANGLE_DEADZONE)
    cv2.line(frame, (CENTER_X - deadzone_px, 0), (CENTER_X - deadzone_px, FRAME_H), (0, 255, 255), 1)
    cv2.line(frame, (CENTER_X + deadzone_px, 0), (CENTER_X + deadzone_px, FRAME_H), (0, 255, 255), 1)

    # 面板
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (635, 240), (0, 0, 0), -1)
    frame[:] = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)

    cv2.putText(
        frame,
        "REAR DIAGONAL MECANUM FOLLOW",
        (25, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (0, 255, 255),
        2
    )

    target_text = "FOUND" if target_found else "LOST"
    color = (0, 255, 0) if target_found else (0, 0, 255)

    cv2.putText(
        frame,
        f"TARGET: {target_text} | STATE: {distance_state}",
        (25, 75),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        color,
        2
    )

    cv2.putText(
        frame,
        f"angle: {goal_angle:.3f} | box_h: {box_h_ratio:.3f} | bottom: {box_bottom_ratio:.3f}",
        (25, 105),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1
    )

    cv2.putText(
        frame,
        f"vx: {vx:.1f} | vy: {vy:.1f} | wz: {wz:.1f}",
        (25, 135),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (0, 255, 255),
        1
    )

    cv2.putText(
        frame,
        f"FL:{fl:4d}  FR:{fr:4d}  RL:{rl:4d}  RR:{rr:4d}",
        (25, 165),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (0, 255, 255),
        1
    )

    cv2.putText(
        frame,
        f"frame_age: {frame_age:.3f}s | YOLO: {inference_time:.3f}s",
        (25, 195),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        1
    )

    cv2.putText(
        frame,
        "Q: Quit | S: E-STOP | R: Reset READY_ARM",
        (25, 225),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        1
    )


# ============================================================
# 10. 主程式
# ============================================================

def main():
    global e_stop_active
    global ready_for_arm

    if not ENABLE_MOTION:
        print("[SAFETY] Vision-only mode; add --real to permit wheel motion.")

    print("[INFO] 載入 YOLO...")
    yolo_model = YOLO(YOLO_MODEL_PATH)

    print(f"[INFO] 連線至 Jetson motor server {JETSON_IP}:{MOTOR_PORT} ...")
    motor_sock = connect_motor_server()
    print("[INFO] motor server connected.")

    print("[INFO] 開啟 REAR camera stream...")
    cap_rear = LatestFrameReader(URL_REAR, FRAME_W, FRAME_H)

    last_processed_frame_time = 0.0

    try:
        while True:
            loop_start = time.time()

            ret, frame, frame_time = cap_rear.read()

            if not ret:
                print("[WARN] REAR camera 還沒有畫面，停車...")
                stop_motor(motor_sock, repeat=1, interval=0.01)
                time.sleep(0.1)
                continue

            # 避免同一張 frame 重複處理
            if frame_time == last_processed_frame_time:
                time.sleep(0.01)
                continue

            last_processed_frame_time = frame_time
            frame_age = time.time() - frame_time

            # frame 太舊，不根據舊畫面動作
            if frame_age > MAX_FRAME_AGE:
                print(f"[WARN] frame too old: {frame_age:.2f}s, stop")
                stop_motor(motor_sock, repeat=1, interval=0.01)

                cv2.putText(
                    frame,
                    f"FRAME TOO OLD: {frame_age:.2f}s",
                    (30, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 0, 255),
                    2
                )

                cv2.imshow("Rear Diagonal Mecanum Follow", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("s"):
                    e_stop_active = not e_stop_active
                    stop_motor(motor_sock)
                elif key == ord("r"):
                    ready_for_arm = False
                    print("[RESET] READY_ARM -> False")

                continue

            # 已經到 ARM 可接手距離，停車等待
            if ready_for_arm:
                stop_motor(motor_sock, repeat=1, interval=0.01)

                display = frame.copy()
                cv2.putText(
                    display,
                    "READY FOR ARM CAMERA",
                    (85, 230),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.25,
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

                cv2.imshow("Rear Diagonal Mecanum Follow", display)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("r"):
                    ready_for_arm = False
                    print("[RESET] READY_ARM -> False")
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
            box_h_ratio = 0.0
            box_bottom_ratio = 0.0
            distance_state = "NO_TARGET"

            vx = 0.0
            vy = 0.0
            wz = 0.0

            fl = 0
            fr = 0
            rl = 0
            rr = 0

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

                box_h_ratio = box_h / FRAME_H
                box_bottom_ratio = y2 / FRAME_H

                # 目標偏左為正，偏右為負
                goal_angle = (CENTER_X - cx_box) / CENTER_X

                distance_state = get_distance_state(box_h_ratio)

                # 畫目標中心與偏移線
                cv2.circle(annotated, (cx_box, cy_box), 6, (0, 255, 255), -1)
                cv2.line(
                    annotated,
                    (CENTER_X, cy_box),
                    (cx_box, cy_box),
                    (0, 255, 255),
                    2
                )

                # 夠近，交給手臂鏡頭
                if box_h_ratio >= SWITCH_TO_ARM_HEIGHT_RATIO:
                    ready_for_arm = True
                    stop_motor(motor_sock, repeat=3, interval=0.03)
                    print("[INFO] Target close enough. READY FOR ARM CAMERA.")

                else:
                    vx, vy, wz = compute_diagonal_follow(
                        goal_angle=goal_angle,
                        bbox_height_ratio=box_h_ratio
                    )

                    fl, fr, rl, rr = mecanum_mix(vx, vy, wz)

            else:
                target_found = False
                distance_state = "NO_TARGET"
                vx = 0.0
                vy = 0.0
                wz = 0.0
                fl = 0
                fr = 0
                rl = 0
                rr = 0

            # 急停或停用運動
            if e_stop_active or not ENABLE_MOTION:
                vx = 0.0
                vy = 0.0
                wz = 0.0
                fl = 0
                fr = 0
                rl = 0
                rr = 0

            # 沒看到目標，一律停
            if not target_found:
                fl = 0
                fr = 0
                rl = 0
                rr = 0

            # 發送四輪速度
            if ENABLE_MOTION and not e_stop_active:
                send_motor_wheels(motor_sock, fl, fr, rl, rr)
            else:
                stop_motor(motor_sock, repeat=1, interval=0.01)

            # 儀表板
            draw_dashboard(
                frame=annotated,
                target_found=target_found,
                goal_angle=goal_angle,
                box_h_ratio=box_h_ratio,
                box_bottom_ratio=box_bottom_ratio,
                vx=vx,
                vy=vy,
                wz=wz,
                fl=fl,
                fr=fr,
                rl=rl,
                rr=rr,
                distance_state=distance_state,
                frame_age=frame_age,
                inference_time=inference_time
            )

            print(
                f"Target:{target_found} | "
                f"State:{distance_state} | "
                f"angle:{goal_angle:.3f} | "
                f"box_h:{box_h_ratio:.3f} | "
                f"vx:{vx:.1f} vy:{vy:.1f} wz:{wz:.1f} | "
                f"FL:{fl} FR:{fr} RL:{rl} RR:{rr} | "
                f"E_STOP:{e_stop_active}"
            )

            cv2.imshow("Rear Diagonal Mecanum Follow", annotated)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("[INFO] 按下 Q，結束程式...")
                break

            elif key == ord("s"):
                e_stop_active = not e_stop_active

                if e_stop_active:
                    print("[E-STOP] 急停啟動")
                    stop_motor(motor_sock)
                else:
                    print("[E-STOP] 急停解除")

            elif key == ord("r"):
                ready_for_arm = False
                print("[RESET] READY_ARM -> False")

            loop_time = time.time() - loop_start

            if loop_time < TARGET_DT:
                time.sleep(TARGET_DT - loop_time)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected.")

    finally:
        print("[INFO] 停車並關閉資源...")

        try:
            stop_motor(motor_sock)
        except Exception:
            pass

        try:
            motor_sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass

        try:
            motor_sock.close()
        except Exception as e:
            print(f"[WARN] motor socket close failed: {e}")

        try:
            cap_rear.release()
        except Exception:
            pass

        cv2.destroyAllWindows()

        print("[INFO] 程式已結束")


if __name__ == "__main__":
    main()
