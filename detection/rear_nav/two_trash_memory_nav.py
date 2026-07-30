import cv2
import time
import math
import json
import os
import socket
import sys
import threading
import numpy as np
from ultralytics import YOLO


# ============================================================
# 1. 基本設定
# ============================================================

YOLO_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v3i.yolov11 (1)\runs\detect\train7\weights\best.pt"

JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "172.31.28.252")
MOTOR_PORT = 7000

TOPIC_REAR = "/back_cam/image_raw"
TOPIC_ARM = "/arm_cam/image_raw"

URL_REAR = f"http://{JETSON_IP}:8080/stream?topic={TOPIC_REAR}"
URL_ARM = f"http://{JETSON_IP}:8080/stream?topic={TOPIC_ARM}"

FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2

YOLO_CONF = 0.25
IMG_SIZE = 640

COMMAND_INTERVAL = 0.05
REAR_DECISION_INTERVAL = 0.5
ARM_DECISION_INTERVAL = 0.5

MAX_FRAME_AGE = 0.8

# Legacy experiment safety gate: non-stop commands require an explicit --real.
ENABLE_MOTION = "--real" in sys.argv[1:]
AUTO_CONTINUE_AFTER_READY = False
READY_AUTO_WAIT_SEC = 2.0


# ============================================================
# 2. 後鏡頭距離校正
# ============================================================

THETA_REAR = 13.92
H_REAR = 0.50

FY_REAR = 956.0294
CY_REAR = 221.3649
FX_REAR = 957.6253
CX_REAR = 320.0

REAR_CAM_TO_FRONT_M = 0.20


# ============================================================
# 3. ARM 鏡頭距離校正
# ============================================================

THETA_ARM = 27.3
H_ARM = 0.35

FY_ARM = 956.0294
CY_ARM = 221.3649
FX_ARM = 957.6253
CX_ARM = 320.0


# ============================================================
# 4. 車速校正
# ============================================================

KX = 0.006995
KZ = 0.02384

MOTION_SCALE = 0.70
TURN_SCALE = 0.70
USE_MOTION_UPDATE = True


# ============================================================
# 5. 最多追兩顆垃圾
# ============================================================

MAX_TRACKED_TARGETS = 2
MAX_YOLO_CANDIDATES = 10

MIN_BOX_AREA = 300
MIN_BOX_ASPECT = 0.45
MAX_BOX_ASPECT = 2.20

BASE_SAME_TARGET_RADIUS = 0.45
MAX_SAME_TARGET_RADIUS = 1.00

ENABLE_DUPLICATE_MERGE = True
DUPLICATE_MERGE_RADIUS = 0.35

INITIAL_SIGMA = 0.15
MIN_SIGMA = 0.08
MAX_SIGMA = 0.90

SIGMA_GROW_PER_M = 0.18
SIGMA_GROW_PER_RAD = 0.28
SIGMA_GROW_PER_SEC = 0.002

VISIBLE_MISSING_DELETE_SEC = 5.0

EXPECTED_VISIBLE_MAX_X = 1.60
EXPECTED_VISIBLE_MIN_X = 0.25
EXPECTED_VISIBLE_MAX_ABS_Y = 0.45
CAMERA_HALF_FOV_DEG = 18
MAX_SIGMA_FOR_VISIBLE_DELETE = 0.35

MIN_CONFIDENCE_TO_SELECT = 1


# ============================================================
# 6. REAR 導航參數
# ============================================================

FAR_VX_MPS = 0.22
MID_VX_MPS = 0.18
NEAR_VX_MPS = 0.13

MIN_SPEED = 28
MAX_SPEED = 48

FAR_DIST_M = 1.60
MID_DIST_M = 1.10
NEAR_DIST_M = 0.90

REAR_BLIND_START_DIST_M = 0.90
ARM_EXPECTED_CAPTURE_DIST_M = 0.70

MIN_REAR_BLIND_DIST_M = 0.12
MAX_REAR_BLIND_DIST_M = 0.38
REAR_BLIND_VX_MPS = 0.13

OFFSET_FORWARD_DEADZONE_M = 0.06
OFFSET_TURN_THRESHOLD_M = 0.28
REAR_BLIND_MAX_OFFSET_M = 0.13

TURN_SPEED_RATIO = 0.85


# ============================================================
# 7. ARM 導航參數
# ============================================================

ARM_FAR_VX_MPS = 0.10
ARM_MID_VX_MPS = 0.07
ARM_NEAR_VX_MPS = 0.045

ARM_MIN_SPEED = 24
ARM_MAX_SPEED = 38

ARM_FAR_DIST_M = 0.70
ARM_MID_DIST_M = 0.45
ARM_NEAR_DIST_M = 0.30

ARM_BLIND_START_DIST_M = 0.24

ARM_OFFSET_FORWARD_DEADZONE_M = 0.035
ARM_OFFSET_TURN_THRESHOLD_M = 0.12

ARM_KP_WZ = 0.30
ARM_MAX_WZ = 0.25
ARM_MIN_TURN_SPEED = 18

ARM_BLIND_FORWARD_SPEED = 30
ARM_BLIND_FORWARD_TIME = 3.0


# ============================================================
# 8. 轉向下一顆垃圾參數
# ============================================================

TURN_TO_NEXT_SPEED = 30
TURN_TO_NEXT_DEADZONE_DEG = 8.0
TURN_TO_NEXT_TIMEOUT_SEC = 8.0


# ============================================================
# 9. Radar 顯示
# ============================================================

RADAR_W = 500
RADAR_H = 480
RADAR_SCALE = 100
RADAR_MAX_X = 8.0
RADAR_MAX_Y = 4.0


# ============================================================
# 10. 全域狀態
# ============================================================

e_stop_active = False

# 狀態：
# REAR_NAV
# REAR_BLIND
# ARM_ALIGN
# ARM_BLIND_PUSH
# READY_GRAB
# TURN_TO_NEXT
state = "REAR_NAV"

last_action = "stop"
last_speed = 0
last_command_time = 0.0

last_rear_decision_time = 0.0
last_arm_decision_time = 0.0

collecting_target_id = None
ready_since_time = None
turn_to_next_start_time = None

last_rear_info = {
    "target_found": False,
    "target_id": None,
    "dist_front_m": -1.0,
    "dist_rear_m": -1.0,
    "robot_y": 0.0,
    "offset_x_m": 0.0,
    "goal_angle": 0.0,
    "action": "stop",
    "speed": 0,
    "status_text": "INIT",
    "distance_state": "INIT",
    "inference_time": 0.0,
}

last_arm_info = {
    "target_found": False,
    "dist_arm_m": -1.0,
    "offset_x_m": 0.0,
    "goal_angle": 0.0,
    "desired_wz": 0.0,
    "action": "stop",
    "speed": 0,
    "status_text": "INIT",
    "distance_state": "INIT",
    "inference_time": 0.0,
}

last_rear_target = {
    "target_id": None,
    "dist_front_m": None,
    "robot_y": 0.0,
    "offset_x_m": 0.0,
    "blind_dist_m": 0.0,
    "blind_time_s": 0.0,
}


# ============================================================
# 11. Camera Reader
# ============================================================

class LatestFrameReader:
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
# 12. Motor Socket，不用 cmd_vel
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
    global last_action
    global last_speed

    last_action = "stop"
    last_speed = 0

    for _ in range(repeat):
        try:
            send_motor_action(sock, "stop", 0)
        except Exception:
            pass
        time.sleep(interval)


def keepalive_send(sock, action, speed):
    global last_command_time

    now = time.time()

    if now - last_command_time >= COMMAND_INTERVAL:
        send_motor_action(sock, action, speed)
        last_command_time = now


# ============================================================
# 13. 距離、速度換算
# ============================================================

def estimate_ground_distance(y_pixel, theta_deg, h_m, fy, cy):
    theta = math.radians(theta_deg)
    alpha = math.atan((y_pixel - cy) / fy)
    total_angle = theta + alpha

    if total_angle <= 0.01:
        return -1.0

    dist_m = h_m / math.tan(total_angle)

    if dist_m <= 0 or dist_m > 10:
        return -1.0

    return dist_m


def rear_dist_to_front_dist(dist_rear_m):
    if dist_rear_m <= 0:
        return -1.0

    return max(0.0, dist_rear_m - REAR_CAM_TO_FRONT_M)


def estimate_offset_x(cx_pixel, dist_m, fx, cx):
    """
    offset_x_m > 0：目標在畫面右邊
    offset_x_m < 0：目標在畫面左邊
    """
    if dist_m <= 0:
        return 0.0

    return (cx_pixel - cx) * dist_m / fx


def speed_from_vx(vx_mps, min_speed=MIN_SPEED, max_speed=MAX_SPEED):
    if KX <= 1e-9:
        return min_speed

    speed = vx_mps / KX
    return int(round(np.clip(speed, min_speed, max_speed)))


def speed_from_wz(wz_rad_s, min_speed=MIN_SPEED, max_speed=MAX_SPEED):
    if KZ <= 1e-9:
        return min_speed

    speed = abs(wz_rad_s) / KZ
    return int(round(np.clip(speed, min_speed, max_speed)))


def detection_to_robot_xy(cx_box, y2_box):
    """
    robot 座標：
    x = 車頭前方距離
    y = 左正右負
    """
    dist_rear_m = estimate_ground_distance(
        y2_box,
        THETA_REAR,
        H_REAR,
        FY_REAR,
        CY_REAR
    )

    if dist_rear_m <= 0:
        return None

    dist_front_m = rear_dist_to_front_dist(dist_rear_m)

    offset_x_m = estimate_offset_x(
        cx_box,
        dist_rear_m,
        FX_REAR,
        CX_REAR
    )

    robot_x = dist_front_m
    robot_y = -offset_x_m

    return robot_x, robot_y, dist_rear_m, dist_front_m, offset_x_m


# ============================================================
# 14. YOLO 選前兩顆
# ============================================================

def select_top_two_detections_from_yolo(boxes):
    candidates = []

    for box in boxes:
        xyxy = box.xyxy[0].cpu().numpy()
        conf = float(box.conf[0].cpu().numpy())

        x1, y1, x2, y2 = xyxy
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)

        w = max(1, x2 - x1)
        h = max(1, y2 - y1)
        area = w * h
        aspect = w / h

        if area < MIN_BOX_AREA:
            continue

        if aspect < MIN_BOX_ASPECT or aspect > MAX_BOX_ASPECT:
            continue

        cx_box = int((x1 + x2) / 2)
        y2_box = int(y2)

        converted = detection_to_robot_xy(cx_box, y2_box)

        if converted is None:
            continue

        robot_x, robot_y, dist_rear_m, dist_front_m, offset_x_m = converted

        if dist_front_m > 3.0:
            continue

        if abs(robot_y) > 1.8:
            continue

        area_score = min(area / 5000.0, 1.5)
        score = conf * 0.75 + area_score * 0.25

        det = {
            "x": robot_x,
            "y": robot_y,
            "dist_rear_m": dist_rear_m,
            "dist_front_m": dist_front_m,
            "offset_x_m": offset_x_m,
            "bbox": (x1, y1, x2, y2),
            "conf": conf,
            "area": area,
            "aspect": aspect,
            "score": score
        }

        candidates.append(det)

    candidates.sort(key=lambda d: d["score"], reverse=True)
    return candidates[:MAX_TRACKED_TARGETS]


def select_largest_box(results):
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
# 15. Target Memory
# ============================================================

class TargetMemory:
    def __init__(self):
        self.targets = []
        self.next_id = 1

    def update_with_detections(self, detections_xy):
        now = time.time()
        matched_ids = set()
        used_target_ids = set()

        for det in detections_xy:
            x = det["x"]
            y = det["y"]

            matched_target = None
            min_dist = 999.0

            for target in self.targets:
                if target["status"] != "active":
                    continue

                if target["id"] in used_target_ids:
                    continue

                d = math.sqrt((x - target["x"]) ** 2 + (y - target["y"]) ** 2)

                gate = BASE_SAME_TARGET_RADIUS + target["sigma"]
                gate = min(gate, MAX_SAME_TARGET_RADIUS)

                if d < gate and d < min_dist:
                    matched_target = target
                    min_dist = d

            if matched_target is not None:
                self.update_existing_target(matched_target, det, now)
                matched_ids.add(matched_target["id"])
                used_target_ids.add(matched_target["id"])
            else:
                self.add_new_target(det, now)

        if ENABLE_DUPLICATE_MERGE:
            self.merge_duplicate_targets()

        self.limit_to_two_targets()
        self.handle_not_seen_targets(matched_ids, now)

    def update_existing_target(self, target, det, now):
        a = 0.35 + min(target["sigma"], 0.5) * 0.45
        a = max(0.35, min(a, 0.60))

        target["x"] = target["x"] * (1 - a) + det["x"] * a
        target["y"] = target["y"] * (1 - a) + det["y"] * a

        target["last_seen"] = now
        target["confidence"] += 1
        target["bbox"] = det["bbox"]
        target["det_conf"] = det["conf"]

        target["visible_missing_since"] = None
        target["miss_total_count"] = 0

        target["sigma"] = max(MIN_SIGMA, target["sigma"] * 0.50)

    def add_new_target(self, det, now):
        new_target = {
            "id": self.next_id,
            "x": det["x"],
            "y": det["y"],
            "confidence": 1,
            "last_seen": now,
            "first_seen": now,
            "status": "active",
            "bbox": det["bbox"],
            "det_conf": det["conf"],
            "sigma": INITIAL_SIGMA,
            "visible_missing_since": None,
            "miss_total_count": 0,
        }

        self.targets.append(new_target)
        self.next_id += 1

    def merge_duplicate_targets(self):
        active = [t for t in self.targets if t["status"] == "active"]
        removed_ids = set()

        for i in range(len(active)):
            a = active[i]

            if a["id"] in removed_ids:
                continue

            for j in range(i + 1, len(active)):
                b = active[j]

                if b["id"] in removed_ids:
                    continue

                d = math.sqrt((a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2)

                merge_gate = DUPLICATE_MERGE_RADIUS + 0.30 * max(a["sigma"], b["sigma"])
                merge_gate = min(merge_gate, 0.55)

                if d < merge_gate:
                    if (b["confidence"] > a["confidence"]) or (
                        b["confidence"] == a["confidence"] and b["id"] < a["id"]
                    ):
                        keep = b
                        drop = a
                    else:
                        keep = a
                        drop = b

                    total_conf = max(1, keep["confidence"] + drop["confidence"])
                    wk = keep["confidence"] / total_conf
                    wd = drop["confidence"] / total_conf

                    keep["x"] = keep["x"] * wk + drop["x"] * wd
                    keep["y"] = keep["y"] * wk + drop["y"] * wd
                    keep["confidence"] += drop["confidence"]
                    keep["sigma"] = min(MAX_SIGMA, max(keep["sigma"], drop["sigma"]) * 0.85)
                    keep["last_seen"] = max(keep["last_seen"], drop["last_seen"])

                    removed_ids.add(drop["id"])

                    print(f"[MERGE] ID{drop['id']} -> ID{keep['id']} | d={d:.2f}")

        if removed_ids:
            self.targets = [t for t in self.targets if t["id"] not in removed_ids]

    def limit_to_two_targets(self):
        active = [t for t in self.targets if t["status"] == "active"]

        if len(active) <= MAX_TRACKED_TARGETS:
            return

        def keep_score(t):
            return (
                t["confidence"] * 2.0
                - t["sigma"] * 3.0
                - abs(t["y"]) * 0.8
                - t["x"] * 0.2
            )

        active_sorted = sorted(active, key=keep_score, reverse=True)
        keep_ids = set(t["id"] for t in active_sorted[:MAX_TRACKED_TARGETS])

        removed = []
        new_targets = []

        for t in self.targets:
            if t["id"] in keep_ids:
                new_targets.append(t)
            else:
                removed.append(t["id"])

        if removed:
            print(f"[LIMIT] remove extra targets: {removed}")

        self.targets = new_targets

    def handle_not_seen_targets(self, matched_ids, now):
        kept_targets = []

        for target in self.targets:
            if target["status"] != "active":
                continue

            if target["id"] in matched_ids:
                kept_targets.append(target)
                continue

            should_visible = self.should_be_visible_in_camera(target)

            if should_visible:
                target["miss_total_count"] += 1

                if target["visible_missing_since"] is None:
                    target["visible_missing_since"] = now
                    print(f"[VISIBLE-MISS-START] ID{target['id']}")

                missing_time = now - target["visible_missing_since"]

                if missing_time >= VISIBLE_MISSING_DELETE_SEC:
                    print(f"[DELETE] ID{target['id']} visible but YOLO missing > {VISIBLE_MISSING_DELETE_SEC}s")
                    continue
            else:
                target["visible_missing_since"] = None

            kept_targets.append(target)

        self.targets = kept_targets

    def should_be_visible_in_camera(self, target):
        x = target["x"]
        y = target["y"]
        sigma = target["sigma"]

        if sigma > MAX_SIGMA_FOR_VISIBLE_DELETE:
            return False

        if x <= EXPECTED_VISIBLE_MIN_X:
            return False

        if x > EXPECTED_VISIBLE_MAX_X:
            return False

        if abs(y) > EXPECTED_VISIBLE_MAX_ABS_Y:
            return False

        bearing_deg = math.degrees(math.atan2(y, x))
        return abs(bearing_deg) <= CAMERA_HALF_FOV_DEG

    def update_by_motion(self, action, speed, dt):
        if not USE_MOTION_UPDATE:
            return

        vx = 0.0
        wz = 0.0

        if action == "forward":
            vx = KX * speed * MOTION_SCALE
        elif action == "backward":
            vx = -KX * speed * MOTION_SCALE
        elif action == "turn_left":
            wz = KZ * speed * TURN_SCALE
        elif action == "turn_right":
            wz = -KZ * speed * TURN_SCALE
        elif action == "curve_left":
            vx = KX * speed * 0.8 * MOTION_SCALE
            wz = KZ * speed * 0.35 * TURN_SCALE
        elif action == "curve_right":
            vx = KX * speed * 0.8 * MOTION_SCALE
            wz = -KZ * speed * 0.35 * TURN_SCALE

        dx = vx * dt
        dtheta = wz * dt

        self.apply_robot_motion(dx, dtheta)

    def apply_robot_motion(self, dx, dtheta):
        c = math.cos(dtheta)
        s = math.sin(dtheta)

        for target in self.targets:
            if target["status"] != "active":
                continue

            x = target["x"] - dx
            y = target["y"]

            x_new = c * x + s * y
            y_new = -s * x + c * y

            target["x"] = x_new
            target["y"] = y_new

            target["sigma"] += SIGMA_GROW_PER_M * abs(dx)
            target["sigma"] += SIGMA_GROW_PER_RAD * abs(dtheta)
            target["sigma"] = min(target["sigma"], MAX_SIGMA)

    def grow_uncertainty_by_time(self, dt):
        for target in self.targets:
            if target["status"] != "active":
                continue

            target["sigma"] += SIGMA_GROW_PER_SEC * dt
            target["sigma"] = min(target["sigma"], MAX_SIGMA)

    def mark_collected(self, target_id):
        if target_id is None:
            return

        new_targets = []

        for target in self.targets:
            if target["id"] == target_id:
                print(f"[COLLECTED] ID{target_id} removed from memory")
                continue

            new_targets.append(target)

        self.targets = new_targets

    def get_target_by_id(self, target_id):
        for target in self.targets:
            if target["id"] == target_id and target["status"] == "active":
                return target
        return None

    def select_nearest_target(self):
        candidates = []

        for target in self.targets:
            if target["status"] != "active":
                continue

            if target["confidence"] < MIN_CONFIDENCE_TO_SELECT:
                continue

            if target["x"] <= 0:
                continue

            score = target["x"] + 1.2 * abs(target["y"]) + 0.8 * target["sigma"]
            candidates.append((score, target))

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        return candidates[0][1]

    def has_any_target(self):
        return self.select_nearest_target() is not None

    def debug_print(self):
        items = []

        for t in self.targets:
            missing = None

            if t["visible_missing_since"] is not None:
                missing = round(time.time() - t["visible_missing_since"], 1)

            items.append({
                "id": t["id"],
                "x": round(t["x"], 2),
                "y": round(t["y"], 2),
                "sig": round(t["sigma"], 2),
                "conf": t["confidence"],
                "vis": self.should_be_visible_in_camera(t),
                "miss": missing
            })

        print("[MEMORY]", items)


# ============================================================
# 16. 導航決策
# ============================================================

def get_rear_distance_state(dist_front_m):
    if dist_front_m <= 0:
        return "INVALID"

    if dist_front_m <= REAR_BLIND_START_DIST_M:
        return "BLIND_READY"

    if dist_front_m <= NEAR_DIST_M:
        return "NEAR"

    if dist_front_m <= MID_DIST_M:
        return "MID"

    if dist_front_m <= FAR_DIST_M:
        return "FAR"

    return "VERY_FAR"


def choose_rear_base_speed(distance_state):
    if distance_state in ["VERY_FAR", "FAR"]:
        return speed_from_vx(FAR_VX_MPS)

    if distance_state == "MID":
        return speed_from_vx(MID_VX_MPS)

    if distance_state in ["NEAR", "BLIND_READY"]:
        return speed_from_vx(NEAR_VX_MPS)

    return 0


def compute_rear_blind_plan(dist_front_m):
    raw_blind_dist = dist_front_m - ARM_EXPECTED_CAPTURE_DIST_M

    blind_dist_m = float(np.clip(
        raw_blind_dist,
        MIN_REAR_BLIND_DIST_M,
        MAX_REAR_BLIND_DIST_M
    ))

    blind_time_s = blind_dist_m / REAR_BLIND_VX_MPS
    blind_time_s = float(np.clip(blind_time_s, 0.6, 3.2))

    return blind_dist_m, blind_time_s


def decide_rear_action_from_memory_target(target):
    """
    target:
    x = 前方距離
    y = 左正右負

    原本 offset_x_m：
    右正左負

    所以：
    offset_x_m = -y
    """
    if target is None:
        return "stop", 0, "NO MEMORY TARGET", "NO_TARGET"

    dist_front_m = target["x"]
    robot_y = target["y"]
    offset_x_m = -robot_y

    distance_state = get_rear_distance_state(dist_front_m)
    base_speed = choose_rear_base_speed(distance_state)

    if distance_state == "INVALID":
        return "stop", 0, "INVALID MEMORY DIST", distance_state

    if distance_state == "BLIND_READY":
        if abs(offset_x_m) <= REAR_BLIND_MAX_OFFSET_M:
            return "arrived", 0, "ARRIVED TARGET - ENTER REAR BLIND", distance_state

        turn_speed = int(round(base_speed * TURN_SPEED_RATIO))
        turn_speed = int(np.clip(turn_speed, MIN_SPEED, MAX_SPEED))

        if offset_x_m < 0:
            return "turn_left", turn_speed, "NEAR BUT LEFT OFFSET", distance_state
        else:
            return "turn_right", turn_speed, "NEAR BUT RIGHT OFFSET", distance_state

    if abs(offset_x_m) <= OFFSET_FORWARD_DEADZONE_M:
        return "forward", base_speed, "MEMORY CENTER: FORWARD", distance_state

    if abs(offset_x_m) >= OFFSET_TURN_THRESHOLD_M:
        turn_speed = int(round(base_speed * TURN_SPEED_RATIO))
        turn_speed = int(np.clip(turn_speed, MIN_SPEED, MAX_SPEED))

        if offset_x_m < 0:
            return "turn_left", turn_speed, "MEMORY LARGE LEFT: TURN LEFT", distance_state
        else:
            return "turn_right", turn_speed, "MEMORY LARGE RIGHT: TURN RIGHT", distance_state

    if offset_x_m < 0:
        return "curve_left", base_speed, "MEMORY LEFT: CURVE LEFT", distance_state
    else:
        return "curve_right", base_speed, "MEMORY RIGHT: CURVE RIGHT", distance_state


def get_arm_distance_state(dist_arm_m):
    if dist_arm_m <= 0:
        return "INVALID"

    if dist_arm_m <= ARM_BLIND_START_DIST_M:
        return "ARM_BLIND_READY"

    if dist_arm_m <= ARM_NEAR_DIST_M:
        return "ARM_NEAR"

    if dist_arm_m <= ARM_MID_DIST_M:
        return "ARM_MID"

    if dist_arm_m <= ARM_FAR_DIST_M:
        return "ARM_FAR"

    return "ARM_VERY_FAR"


def choose_arm_forward_speed(distance_state):
    if distance_state in ["ARM_VERY_FAR", "ARM_FAR"]:
        return speed_from_vx(ARM_FAR_VX_MPS, ARM_MIN_SPEED, ARM_MAX_SPEED)

    if distance_state == "ARM_MID":
        return speed_from_vx(ARM_MID_VX_MPS, ARM_MIN_SPEED, ARM_MAX_SPEED)

    if distance_state in ["ARM_NEAR", "ARM_BLIND_READY"]:
        return speed_from_vx(ARM_NEAR_VX_MPS, ARM_MIN_SPEED, ARM_MAX_SPEED)

    return 0


def decide_arm_action_by_distance(target_found, offset_x_m, dist_arm_m):
    if not target_found:
        return "stop", 0, "ARM TARGET LOST", "NO_TARGET", 0.0

    distance_state = get_arm_distance_state(dist_arm_m)

    if distance_state == "INVALID":
        return "stop", 0, "ARM INVALID DIST", distance_state, 0.0

    if distance_state == "ARM_BLIND_READY":
        if abs(offset_x_m) <= ARM_OFFSET_TURN_THRESHOLD_M:
            return "arm_blind_push", 0, "ENTER ARM BLIND PUSH", distance_state, 0.0

    angle_error = math.atan2(offset_x_m, dist_arm_m) if dist_arm_m > 0 else 0.0

    desired_wz = ARM_KP_WZ * angle_error
    desired_wz = float(np.clip(desired_wz, -ARM_MAX_WZ, ARM_MAX_WZ))

    if abs(offset_x_m) <= ARM_OFFSET_FORWARD_DEADZONE_M:
        speed = choose_arm_forward_speed(distance_state)
        return "forward", speed, "ARM CENTER: FORWARD", distance_state, desired_wz

    turn_speed = speed_from_wz(
        desired_wz,
        min_speed=ARM_MIN_TURN_SPEED,
        max_speed=ARM_MAX_SPEED
    )

    if offset_x_m < 0:
        return "turn_left", turn_speed, "ARM OFFSET LEFT: TURN LEFT", distance_state, desired_wz
    else:
        return "turn_right", turn_speed, "ARM OFFSET RIGHT: TURN RIGHT", distance_state, desired_wz


# ============================================================
# 17. blind 動作
# ============================================================

def execute_rear_blind(sock, memory, blind_time_s):
    global last_action
    global last_speed
    global last_command_time

    speed = speed_from_vx(REAR_BLIND_VX_MPS)

    print("\n[REAR_BLIND] Start")
    print(f"[REAR_BLIND] speed={speed}, time={blind_time_s:.2f}s")

    stop_motor(sock, repeat=3, interval=0.03)
    time.sleep(0.1)

    t0 = time.time()
    last_t = time.time()
    last_command_time = 0.0

    while True:
        if e_stop_active:
            break

        now = time.time()
        dt = now - last_t
        last_t = now

        memory.update_by_motion("forward", speed, dt)
        memory.grow_uncertainty_by_time(dt)

        if now - t0 >= blind_time_s:
            break

        keepalive_send(sock, "forward", speed)
        time.sleep(0.005)

    stop_motor(sock, repeat=8, interval=0.03)

    last_action = "stop"
    last_speed = 0

    print("[REAR_BLIND] Done -> ARM_ALIGN")


def execute_arm_blind_push(sock, memory):
    print("\n[ARM_BLIND_PUSH] Start")

    stop_motor(sock, repeat=3, interval=0.03)
    time.sleep(0.1)

    t0 = time.time()
    last_t = time.time()

    while True:
        if e_stop_active:
            break

        now = time.time()
        dt = now - last_t
        last_t = now

        memory.update_by_motion("forward", ARM_BLIND_FORWARD_SPEED, dt)
        memory.grow_uncertainty_by_time(dt)

        if now - t0 >= ARM_BLIND_FORWARD_TIME:
            break

        send_motor_action(sock, "forward", ARM_BLIND_FORWARD_SPEED)
        time.sleep(COMMAND_INTERVAL)

    stop_motor(sock, repeat=8, interval=0.03)
    print("[ARM_BLIND_PUSH] Done -> READY_GRAB")


# ============================================================
# 18. 視覺化
# ============================================================

def draw_object_distance_label(frame, x1, y1, primary_dist_m, secondary_dist_m, offset_x_m, label_prefix):
    if primary_dist_m > 0:
        cv2.putText(
            frame,
            f"{label_prefix}: {primary_dist_m * 100:.1f} cm",
            (x1, max(25, y1 - 45)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2
        )

        if secondary_dist_m is not None and secondary_dist_m > 0:
            cv2.putText(
                frame,
                f"Raw: {secondary_dist_m * 100:.1f} cm",
                (x1, max(50, y1 - 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1
            )

        side_text = "Right" if offset_x_m > 0 else "Left" if offset_x_m < 0 else "Center"

        cv2.putText(
            frame,
            f"{side_text}: {abs(offset_x_m) * 100:.1f} cm",
            (x1, min(FRAME_H - 10, y1 + 25)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 100, 255),
            2
        )


def robot_xy_to_radar_px(x, y):
    origin_x = RADAR_W // 2
    origin_y = RADAR_H - 45

    px = int(origin_x - y * RADAR_SCALE)
    py = int(origin_y - x * RADAR_SCALE)

    return px, py


def draw_radar(memory, selected_target_id, collecting_id):
    radar = np.zeros((RADAR_H, RADAR_W, 3), dtype=np.uint8)

    origin_x = RADAR_W // 2
    origin_y = RADAR_H - 45

    cv2.circle(radar, (origin_x, origin_y), 10, (255, 255, 255), -1)
    cv2.putText(radar, "ROBOT", (origin_x - 35, origin_y + 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    cv2.line(radar, (origin_x, origin_y), (origin_x, 20), (80, 80, 80), 1)

    for d in [0.5, 1.0, 1.5, 2.0, 3.0, 4.0]:
        r = int(d * RADAR_SCALE)
        cv2.circle(radar, (origin_x, origin_y), r, (45, 45, 45), 1)

    for target in memory.targets:
        x = target["x"]
        y = target["y"]

        if x < -3.0 or x > RADAR_MAX_X:
            continue

        if abs(y) > RADAR_MAX_Y:
            continue

        px, py = robot_xy_to_radar_px(x, y)

        if target["id"] == collecting_id:
            color = (255, 0, 255)
            radius = 10
        elif target["id"] == selected_target_id:
            color = (0, 0, 255)
            radius = 9
        else:
            color = (0, 255, 255)
            radius = 7

        cv2.circle(radar, (px, py), radius, color, -1)

        sigma_px = int(target["sigma"] * RADAR_SCALE)
        cv2.circle(radar, (px, py), sigma_px, (80, 80, 80), 1)

        cv2.putText(
            radar,
            f"ID{target['id']} x={x:.2f} y={y:.2f}",
            (px + 8, py - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            color,
            1
        )

    cv2.putText(radar, f"STATE: {state}", (15, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)

    cv2.putText(radar, "RED=selected | PURPLE=collecting | YELLOW=next",
                (15, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)

    cv2.putText(radar, "Q quit | S estop | R reset | G grabbed/next | C clear memory",
                (15, RADAR_H - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (180, 180, 180), 1)

    return radar


def draw_status_overlay(frame, title, selected_target, action, speed, status_text, extra_text=""):
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (635, 165), (0, 0, 0), -1)
    frame[:] = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)

    cv2.line(frame, (CENTER_X, 0), (CENTER_X, FRAME_H), (255, 255, 255), 1)

    cv2.putText(frame, title, (22, 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)

    if selected_target is not None:
        cv2.putText(
            frame,
            f"TARGET ID{selected_target['id']} x={selected_target['x']:.2f} y={selected_target['y']:.2f} sigma={selected_target['sigma']:.2f}",
            (22, 72),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            1
        )
    else:
        cv2.putText(frame, "TARGET: NONE", (22, 72),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

    cv2.putText(frame, f"action: {action} | speed: {speed}",
                (22, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2)

    cv2.putText(frame, f"status: {status_text}",
                (22, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

    if extra_text:
        cv2.putText(frame, extra_text, (22, 462),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)


# ============================================================
# 19. 主程式
# ============================================================

def main():
    global state
    global e_stop_active
    global last_action
    global last_speed
    global last_command_time
    global last_rear_decision_time
    global last_arm_decision_time
    global collecting_target_id
    global ready_since_time
    global turn_to_next_start_time
    global last_rear_target
    global last_rear_info
    global last_arm_info

    if not ENABLE_MOTION:
        print("[SAFETY] Vision-only mode; add --real to permit motor motion.")

    print("[INFO] Loading YOLO...")
    yolo_model = YOLO(YOLO_MODEL_PATH)
    print("[INFO] model classes:", yolo_model.names)

    print(f"[INFO] Connecting motor server {JETSON_IP}:{MOTOR_PORT}...")
    motor_sock = connect_motor_server()
    print("[INFO] motor server connected.")

    print("[INFO] Opening cameras...")
    cap_rear = LatestFrameReader(URL_REAR, FRAME_W, FRAME_H)
    cap_arm = LatestFrameReader(URL_ARM, FRAME_W, FRAME_H)

    memory = TargetMemory()

    last_time = time.time()
    last_debug_time = 0.0

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            memory.update_by_motion(last_action, last_speed, dt)
            memory.grow_uncertainty_by_time(dt)

            if state in ["REAR_NAV", "REAR_BLIND", "TURN_TO_NEXT"]:
                active_cap = cap_rear
                active_name = "REAR"
            else:
                active_cap = cap_arm
                active_name = "ARM"

            ret, frame, frame_time = active_cap.read()

            if not ret:
                print(f"[WARN] {active_name} no frame")
                stop_motor(motor_sock, repeat=1, interval=0.01)
                time.sleep(0.05)
                continue

            frame_age = time.time() - frame_time

            if frame_age > MAX_FRAME_AGE:
                print(f"[WARN] {active_name} frame too old: {frame_age:.2f}s")
                last_action = "stop"
                last_speed = 0
                stop_motor(motor_sock, repeat=1, interval=0.01)
                time.sleep(0.03)
                continue

            selected_target = memory.select_nearest_target()
            selected_id = selected_target["id"] if selected_target is not None else None

            # ==================================================
            # REAR_NAV：用 rear cam 更新 memory，導航最近一顆
            # ==================================================
            if state == "REAR_NAV":
                need_decision = (now - last_rear_decision_time) >= REAR_DECISION_INTERVAL

                if need_decision:
                    yolo_start = time.time()
                    results = yolo_model.predict(
                        source=frame,
                        conf=YOLO_CONF,
                        imgsz=IMG_SIZE,
                        iou=0.8,
                        max_det=MAX_YOLO_CANDIDATES,
                        verbose=False
                    )[0]
                    inference_time = time.time() - yolo_start

                    annotated = results.plot()

                    detections_xy = []

                    if results.boxes is not None:
                        print(f"[YOLO] raw boxes = {len(results.boxes)}")
                        detections_xy = select_top_two_detections_from_yolo(results.boxes)
                        print(f"[YOLO] used boxes = {len(detections_xy)} / max {MAX_TRACKED_TARGETS}")

                    memory.update_with_detections(detections_xy)

                    selected_target = memory.select_nearest_target()
                    selected_id = selected_target["id"] if selected_target is not None else None

                    action, speed, status_text, distance_state = decide_rear_action_from_memory_target(selected_target)

                    if action == "arrived" and selected_target is not None:
                        collecting_target_id = selected_target["id"]

                        blind_dist_m, blind_time_s = compute_rear_blind_plan(selected_target["x"])

                        last_rear_target = {
                            "target_id": collecting_target_id,
                            "dist_front_m": selected_target["x"],
                            "robot_y": selected_target["y"],
                            "offset_x_m": -selected_target["y"],
                            "blind_dist_m": blind_dist_m,
                            "blind_time_s": blind_time_s,
                        }

                        print("\n[ARRIVED] selected target reached by rear memory nav")
                        print(f"[ARRIVED] target_id={collecting_target_id}")
                        print(f"[ARRIVED] blind_time={blind_time_s:.2f}s")

                        stop_motor(motor_sock, repeat=5, interval=0.03)
                        last_action = "stop"
                        last_speed = 0
                        state = "REAR_BLIND"
                        last_rear_decision_time = now
                        continue

                    if e_stop_active or not ENABLE_MOTION:
                        action = "stop"
                        speed = 0
                        status_text = "E-STOP" if e_stop_active else "MOTION DISABLED"

                    last_action = action
                    last_speed = speed
                    last_rear_decision_time = time.time()

                    last_rear_info = {
                        "target_found": selected_target is not None,
                        "target_id": selected_id,
                        "dist_front_m": selected_target["x"] if selected_target is not None else -1.0,
                        "robot_y": selected_target["y"] if selected_target is not None else 0.0,
                        "offset_x_m": -selected_target["y"] if selected_target is not None else 0.0,
                        "action": action,
                        "speed": speed,
                        "status_text": status_text,
                        "distance_state": distance_state,
                        "inference_time": inference_time,
                    }

                else:
                    annotated = frame.copy()

                if state == "REAR_NAV":
                    keepalive_send(motor_sock, last_action, last_speed)

                draw_status_overlay(
                    annotated,
                    "REAR MEMORY NAV - nearest trash first",
                    selected_target,
                    last_action,
                    last_speed,
                    last_rear_info["status_text"],
                    extra_text="G after grab | R reset | S estop | Q quit"
                )

                radar = draw_radar(memory, selected_id, collecting_target_id)

                cv2.imshow("Rear / Arm Nav", annotated)
                cv2.imshow("Target Memory Radar", radar)

                print(
                    f"State:{state} | target:{selected_id} | "
                    f"x:{last_rear_info['dist_front_m']:.2f} "
                    f"y:{last_rear_info['robot_y']:.2f} | "
                    f"action:{last_action} speed:{last_speed} | "
                    f"{last_rear_info['status_text']}"
                )

            # ==================================================
            # REAR_BLIND
            # ==================================================
            elif state == "REAR_BLIND":
                execute_rear_blind(motor_sock, memory, last_rear_target["blind_time_s"])
                state = "ARM_ALIGN"
                last_arm_decision_time = 0.0
                continue

            # ==================================================
            # ARM_ALIGN
            # ==================================================
            elif state == "ARM_ALIGN":
                need_decision = (now - last_arm_decision_time) >= ARM_DECISION_INTERVAL

                if need_decision:
                    yolo_start = time.time()
                    results = yolo_model.predict(
                        source=frame,
                        conf=YOLO_CONF,
                        imgsz=IMG_SIZE,
                        verbose=False
                    )[0]
                    inference_time = time.time() - yolo_start

                    annotated = results.plot()
                    box = select_largest_box(results)

                    target_found = False
                    dist_arm_m = -1.0
                    offset_x_m = 0.0
                    desired_wz = 0.0
                    action = "stop"
                    speed = 0
                    status_text = "ARM TARGET LOST"
                    distance_state = "NO_TARGET"

                    if box is not None:
                        target_found = True

                        x1 = int(box.xyxy[0][0])
                        y1 = int(box.xyxy[0][1])
                        x2 = int(box.xyxy[0][2])
                        y2 = int(box.xyxy[0][3])

                        cx_box = int((x1 + x2) / 2)
                        cy_box = int((y1 + y2) / 2)

                        dist_arm_m = estimate_ground_distance(y2, THETA_ARM, H_ARM, FY_ARM, CY_ARM)
                        offset_x_m = estimate_offset_x(cx_box, dist_arm_m, FX_ARM, CX_ARM)

                        cv2.circle(annotated, (cx_box, cy_box), 6, (0, 255, 255), -1)
                        cv2.line(annotated, (CENTER_X, cy_box), (cx_box, cy_box), (0, 255, 255), 2)

                        draw_object_distance_label(
                            frame=annotated,
                            x1=x1,
                            y1=y1,
                            primary_dist_m=dist_arm_m,
                            secondary_dist_m=None,
                            offset_x_m=offset_x_m,
                            label_prefix="ARM"
                        )

                        action, speed, status_text, distance_state, desired_wz = decide_arm_action_by_distance(
                            target_found=True,
                            offset_x_m=offset_x_m,
                            dist_arm_m=dist_arm_m
                        )

                        if action == "arm_blind_push":
                            stop_motor(motor_sock, repeat=5, interval=0.03)
                            state = "ARM_BLIND_PUSH"
                            last_action = "stop"
                            last_speed = 0
                            continue

                    if e_stop_active or not ENABLE_MOTION:
                        action = "stop"
                        speed = 0
                        status_text = "E-STOP" if e_stop_active else "MOTION DISABLED"

                    last_action = action
                    last_speed = speed
                    last_arm_decision_time = time.time()

                    last_arm_info = {
                        "target_found": target_found,
                        "dist_arm_m": dist_arm_m,
                        "offset_x_m": offset_x_m,
                        "desired_wz": desired_wz,
                        "action": action,
                        "speed": speed,
                        "status_text": status_text,
                        "distance_state": distance_state,
                        "inference_time": inference_time,
                    }

                else:
                    annotated = frame.copy()

                if state == "ARM_ALIGN":
                    keepalive_send(motor_sock, last_action, last_speed)

                draw_status_overlay(
                    annotated,
                    "ARM ALIGN - collecting current target",
                    memory.get_target_by_id(collecting_target_id),
                    last_action,
                    last_speed,
                    last_arm_info["status_text"],
                    extra_text=f"Collecting ID{collecting_target_id} | G after grab"
                )

                radar = draw_radar(memory, None, collecting_target_id)

                cv2.imshow("Rear / Arm Nav", annotated)
                cv2.imshow("Target Memory Radar", radar)

                print(
                    f"State:{state} | collecting:{collecting_target_id} | "
                    f"arm_dist:{last_arm_info['dist_arm_m']:.2f} "
                    f"offset:{last_arm_info['offset_x_m']:.2f} | "
                    f"action:{last_action} speed:{last_speed} | "
                    f"{last_arm_info['status_text']}"
                )

            # ==================================================
            # ARM_BLIND_PUSH
            # ==================================================
            elif state == "ARM_BLIND_PUSH":
                execute_arm_blind_push(motor_sock, memory)
                state = "READY_GRAB"
                ready_since_time = time.time()
                last_action = "stop"
                last_speed = 0
                continue

            # ==================================================
            # READY_GRAB
            # ==================================================
            elif state == "READY_GRAB":
                stop_motor(motor_sock, repeat=1, interval=0.01)

                annotated = frame.copy()

                cv2.putText(annotated, "READY TO GRAB", (150, 210),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.25, (0, 255, 0), 4)

                cv2.putText(annotated, f"Current ID{collecting_target_id}", (210, 260),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)

                cv2.putText(annotated, "Press G after grabbed -> go next trash", (80, 315),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255, 255, 255), 2)

                if AUTO_CONTINUE_AFTER_READY and ready_since_time is not None:
                    remain = READY_AUTO_WAIT_SEC - (time.time() - ready_since_time)
                    cv2.putText(annotated, f"Auto continue in {remain:.1f}s", (160, 360),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.70, (0, 255, 255), 2)

                    if time.time() - ready_since_time >= READY_AUTO_WAIT_SEC:
                        memory.mark_collected(collecting_target_id)
                        collecting_target_id = None
                        ready_since_time = None

                        if memory.has_any_target():
                            state = "TURN_TO_NEXT"
                            turn_to_next_start_time = time.time()
                        else:
                            state = "REAR_NAV"

                        continue

                radar = draw_radar(memory, None, collecting_target_id)

                cv2.imshow("Rear / Arm Nav", annotated)
                cv2.imshow("Target Memory Radar", radar)

            # ==================================================
            # TURN_TO_NEXT
            # ==================================================
            elif state == "TURN_TO_NEXT":
                selected_target = memory.select_nearest_target()
                selected_id = selected_target["id"] if selected_target is not None else None

                if selected_target is None:
                    print("[TURN_TO_NEXT] no target, back to REAR_NAV")
                    state = "REAR_NAV"
                    last_rear_decision_time = 0.0
                    continue

                bearing_deg = math.degrees(math.atan2(selected_target["y"], selected_target["x"]))

                if abs(bearing_deg) <= TURN_TO_NEXT_DEADZONE_DEG:
                    print(f"[TURN_TO_NEXT] target ID{selected_id} centered enough -> REAR_NAV")
                    stop_motor(motor_sock, repeat=5, interval=0.03)
                    state = "REAR_NAV"
                    last_rear_decision_time = 0.0
                    continue

                if turn_to_next_start_time is not None:
                    if time.time() - turn_to_next_start_time > TURN_TO_NEXT_TIMEOUT_SEC:
                        print("[TURN_TO_NEXT] timeout -> REAR_NAV")
                        stop_motor(motor_sock, repeat=5, interval=0.03)
                        state = "REAR_NAV"
                        last_rear_decision_time = 0.0
                        continue

                if selected_target["y"] > 0:
                    action = "turn_left"
                    status_text = "TURN LEFT TO NEXT TARGET"
                else:
                    action = "turn_right"
                    status_text = "TURN RIGHT TO NEXT TARGET"

                speed = TURN_TO_NEXT_SPEED

                last_action = action
                last_speed = speed

                keepalive_send(motor_sock, last_action, last_speed)

                annotated = frame.copy()
                draw_status_overlay(
                    annotated,
                    "TURN TO NEXT TRASH",
                    selected_target,
                    last_action,
                    last_speed,
                    f"{status_text} | bearing={bearing_deg:.1f} deg",
                    extra_text="Turning by memory target position"
                )

                radar = draw_radar(memory, selected_id, collecting_target_id)

                cv2.imshow("Rear / Arm Nav", annotated)
                cv2.imshow("Target Memory Radar", radar)

            # ==================================================
            # debug
            # ==================================================
            if time.time() - last_debug_time >= 1.0:
                memory.debug_print()
                last_debug_time = time.time()

            # ==================================================
            # keyboard
            # ==================================================
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("[INFO] Q pressed")
                break

            elif key == ord("s"):
                e_stop_active = not e_stop_active

                if e_stop_active:
                    print("[E-STOP] ON")
                    stop_motor(motor_sock)
                else:
                    print("[E-STOP] OFF")

            elif key == ord("r"):
                state = "REAR_NAV"
                e_stop_active = False
                collecting_target_id = None
                ready_since_time = None
                turn_to_next_start_time = None
                last_rear_decision_time = 0.0
                last_arm_decision_time = 0.0
                stop_motor(motor_sock)
                print("[RESET] state -> REAR_NAV")

            elif key == ord("c"):
                memory = TargetMemory()
                collecting_target_id = None
                print("[KEY] memory cleared")

            elif key == ord("g"):
                if state == "READY_GRAB":
                    print(f"[G] grabbed ID{collecting_target_id}, go next")

                    memory.mark_collected(collecting_target_id)
                    collecting_target_id = None
                    ready_since_time = None

                    if memory.has_any_target():
                        state = "TURN_TO_NEXT"
                        turn_to_next_start_time = time.time()
                    else:
                        state = "REAR_NAV"
                        last_rear_decision_time = 0.0
                else:
                    print("[G] only works in READY_GRAB")

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected")

    finally:
        print("[INFO] stopping motor and closing resources...")

        try:
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

        try:
            cap_arm.release()
        except Exception:
            pass

        cv2.destroyAllWindows()
        print("[INFO] program ended safely")


if __name__ == "__main__":
    main()
