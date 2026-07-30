#!/usr/bin/env python3
"""Unified self-driving grasp pipeline for X3Plus.

One process owns the Rosmaster serial link and does the whole loop:

    1. detect object (YOLOv11, dual camera)
    2. drive the mecanum base toward it       (set_car_motion)
    3. stop once it is within arm reach        (hand off, no blind push)
    4. grasp with the PPO arm policy           (vision gives x/y/z + width)
    5. if the grasp failed (object still seen at the same spot), retry from 2
       — up to --max-retries times.

Why one process: the same Rosmaster object drives BOTH the wheels
(`set_car_motion`) and the arm servos (`set_uart_servo_angle_array`). Running
the chassis and the arm from separate processes would fight over the Rosmaster
serial port,
so the orchestrator builds the grasp controller (which opens Rosmaster) and
reuses `controller.servo.device` for the chassis.

Navigation logic (distance/offset geometry, decision thresholds, speed
calibration) is ported from detection/rear_nav/rear_to_arm_blind_handoff.py,
but the actuator is changed from the port-7000 motor server to direct
`set_car_motion`, and the final ARM blind-push is replaced by a hand-off to the
arm policy (which already reaches objects at x≈0.25 m).

HARDWARE PREREQUISITES
    * The port-7000 motor server / ROS base driver must NOT be running
      (they would own the serial port).
    * Camera streams: arm + rear MJPEG on :8080 (see URL_ARM / URL_REAR).

Run:
    python vision_grasp_pipeline.py --selftest          # logic check, no hw/cam
    python vision_grasp_pipeline.py                      # dry-run (no motion), needs cams
    python vision_grasp_pipeline.py --real --show       # full self-driving grasp
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Heavy / platform deps (cv2, ultralytics, the grasp module with torch+pybullet)
# are imported lazily inside the classes/functions that need them, so this file
# imports — and --selftest runs — on a plain dev machine.

# ════════════════════════════════════════════════════════════════════════════
# Calibration constants — measured on the robot 2026-07-08 (Phase 1/2,
# chessboard 6x9 inner corners @20mm; see progress.md / CALIBRATION_PLAN.md).
# ════════════════════════════════════════════════════════════════════════════

JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "127.0.0.1")
URL_REAR = f"http://{JETSON_IP}:8080/stream?topic=/back_cam/image_raw"
URL_ARM = f"http://{JETSON_IP}:8080/stream?topic=/arm_cam/image_raw"

FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2

# Rear camera distance model (rear_cam_intrinsics.json, RMS 0.374px).
# Distortion at the bbox bottom-center is 1.79px -> ignored (Phase 1 decision).
# Ground model validated 0.70–1.50m, max error 0.98cm.
THETA_REAR = 16.35
H_REAR = 0.503
REAR_CAM_TO_FRONT_M = 0.20
FY_REAR = 544.82
CY_REAR = 244.79
FX_REAR = 544.16
CX_REAR = 316.98

# Arm camera distance model (arm_cam_intrinsics.json, RMS 0.495px), valid at
# nav home only (camera rides on arm_link4). Ground model validated
# 0.25–0.50m, max error 0.43cm — measured on UNDISTORTED pixels, and the
# distortion moves the bbox bottom-center by ~9px, so (cx_box, y2) MUST go
# through undistort_pixel() before this model (theta was solved that way).
THETA_ARM = 36.40
H_ARM = 0.332
FY_ARM = 919.41
CY_ARM = 168.42
FX_ARM = 919.08
CX_ARM = 212.23
DIST_ARM = (-0.3764, -0.0748, -0.0015, 0.0035, 0.4793)  # k1 k2 p1 p2 k3

# Chassis speed calibration: vx ≈ KX*speed (m/s), wz ≈ KZ*speed (rad/s)
KX = 0.006995
KZ = 0.02384

YOLO_CONF = 0.3
IMG_SIZE = 640
MAX_FRAME_AGE = 0.8

REAR_DECISION_INTERVAL = 0.5
ARM_DECISION_INTERVAL = 0.5

# REAR control
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

# ARM control
ARM_FAR_VX_MPS = 0.10
ARM_MID_VX_MPS = 0.07
ARM_NEAR_VX_MPS = 0.045
ARM_MIN_SPEED = 24
ARM_MAX_SPEED = 38
ARM_FAR_DIST_M = 0.70
ARM_MID_DIST_M = 0.45
ARM_NEAR_DIST_M = 0.30
ARM_BLIND_START_DIST_M = 0.24   # object this close & centered -> hand off to arm policy
ARM_OFFSET_FORWARD_DEADZONE_M = 0.035
ARM_OFFSET_TURN_THRESHOLD_M = 0.12
ARM_KP_WZ = 0.5
ARM_MAX_WZ = 0.35
ARM_MIN_TURN_SPEED = 24

# ── pipeline-specific (new) ──
# Camera -> arm-base frame mapping for the latched grasp target (Phase 3,
# measured 2026-07-16; same convention as vision_grasp_bridge.py).
CAM_TO_BASE_X = 0.1639  # forward offset added to arm-cam distance -> grasp X (m)
CAM_TO_BASE_Y = 0.0331  # lateral offset -> grasp Y (m)
SIGN_Y = -1.0           # arm-cam right-positive offset -> grasp/base +Y
OBJ_Z_FIXED = 0.02      # training default; tune per class from Phase-4 real grasps
# Width gate: an object wider than the gripper can open is physically
# ungraspable — abort immediately instead of wasting retries. Should match the
# gripper's real max opening (grasp DeployConfig.grip_max_object_width_m=0.06).
MAX_GRASP_WIDTH_M = 0.06
VERIFY_TOL_M = 0.06     # re-detected object within this of latched pos ⇒ grasp failed
TARGET_LOST_TIMEOUT_S = 4.0   # give up approach if nothing seen this long
APPROACH_MAX_DURATION_S = 120.0  # hard stop even if detections keep arriving
RETREAT_TIME_S = 0.6          # short reverse before a retry
RETREAT_VX_MPS = 0.12


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def parse_class_z(values: List[str], fallback_z: float) -> Dict[str, float]:
    """Parse repeated --class-z entries like bottle-cap=0.012."""
    out: Dict[str, float] = {}
    for raw in values:
        if "=" not in raw:
            raise ValueError(f"--class-z must be NAME=HEIGHT_M, got: {raw}")
        name, value = raw.split("=", 1)
        name = name.strip()
        if not name:
            raise ValueError(f"--class-z has an empty class name: {raw}")
        out[name] = float(value)
    out.setdefault("_fallback", float(fallback_z))
    return out


def class_name_for_box(model, box) -> str:
    cls_id = int(box.cls[0].item())
    names = model.names
    if isinstance(names, dict):
        return str(names.get(cls_id, cls_id))
    if 0 <= cls_id < len(names):
        return str(names[cls_id])
    return str(cls_id)


# ════════════════════════════════════════════════════════════════════════════
# Geometry + decision functions (ported, motor-server-free)
# ════════════════════════════════════════════════════════════════════════════

def distort_pixel(u, v, fx, fy, cx, cy, dist):
    """Forward plumb-bob distortion of an ideal pixel (selftest round-trip)."""
    k1, k2, p1, p2, k3 = dist
    x = (u - cx) / fx
    y = (v - cy) / fy
    r2 = x * x + y * y
    radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
    xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    return cx + xd * fx, cy + yd * fy


def undistort_pixel(u, v, fx, fy, cx, cy, dist, iters=8):
    """Undistort one pixel coordinate (plumb-bob k1,k2,p1,p2,k3).

    Pure-math equivalent of cv2.undistortPoints(..., P=K) (same fixed-point
    iteration) so this module stays importable without OpenCV for --selftest.
    """
    k1, k2, p1, p2, k3 = dist
    xd = (u - cx) / fx
    yd = (v - cy) / fy
    x, y = xd, yd
    for _ in range(iters):
        r2 = x * x + y * y
        radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x = (xd - dx) / radial
        y = (yd - dy) / radial
    return cx + x * fx, cy + y * fy


def estimate_ground_distance(y2, theta_deg, h_m, fy, cy):
    """Forward ground distance to the box bottom (m); -1 if above horizon."""
    theta = math.radians(theta_deg)
    alpha = math.atan((y2 - cy) / fy)
    total_angle = theta + alpha
    if total_angle <= 0:
        return -1.0
    return h_m / math.tan(total_angle)


def optical_depth_from_ground_distance(dist_m, theta_deg, h_m):
    """Convert horizontal ground distance to pinhole optical-axis depth.

    A pitched camera observes a ground point at
    ``Z_cam = D*cos(theta) + H*sin(theta)``. Horizontal pixel offsets and
    widths must use this optical depth; using D directly under-scales the arm
    camera's lateral geometry by roughly 40% at the Phase-3 working distance.
    """
    if dist_m <= 0:
        return 0.0
    theta = math.radians(theta_deg)
    return dist_m * math.cos(theta) + h_m * math.sin(theta)


def estimate_pitched_offset_x(cx_box, dist_m, fx, cx_cam, theta_deg, h_m):
    """Right-positive lateral offset for a pitched camera."""
    depth_m = optical_depth_from_ground_distance(dist_m, theta_deg, h_m)
    return depth_m * (cx_box - cx_cam) / fx


def estimate_pitched_width(width_px, dist_m, fx, theta_deg, h_m):
    """Metric image-plane width for a pitched camera."""
    if width_px <= 0:
        return 0.0
    depth_m = optical_depth_from_ground_distance(dist_m, theta_deg, h_m)
    return width_px * depth_m / fx


def rear_dist_to_front_dist(dist_rear_m):
    if dist_rear_m <= 0:
        return -1.0
    return max(0.0, dist_rear_m - REAR_CAM_TO_FRONT_M)


def estimate_offset_x(cx_box, dist_m, fx, cx_cam):
    """Lateral offset (m): >0 object to the right, <0 to the left."""
    if dist_m <= 0:
        return 0.0
    return dist_m * (cx_box - cx_cam) / fx


def speed_from_vx(vx_mps, min_speed=MIN_SPEED, max_speed=MAX_SPEED):
    if KX <= 1e-9:
        return min_speed
    return int(round(clamp(vx_mps / KX, min_speed, max_speed)))


def speed_from_wz(wz_rad_s, min_speed=MIN_SPEED, max_speed=MAX_SPEED):
    if KZ <= 1e-9:
        return min_speed
    return int(round(clamp(abs(wz_rad_s) / KZ, min_speed, max_speed)))


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
    if distance_state in ("VERY_FAR", "FAR"):
        return speed_from_vx(FAR_VX_MPS)
    if distance_state == "MID":
        return speed_from_vx(MID_VX_MPS)
    if distance_state in ("NEAR", "BLIND_READY"):
        return speed_from_vx(NEAR_VX_MPS)
    return 0


def compute_rear_blind_plan(dist_front_m):
    blind_dist_m = clamp(dist_front_m - ARM_EXPECTED_CAPTURE_DIST_M,
                         MIN_REAR_BLIND_DIST_M, MAX_REAR_BLIND_DIST_M)
    if REAR_BLIND_VX_MPS <= 1e-6:
        blind_time_s = 1.0
    else:
        blind_time_s = blind_dist_m / REAR_BLIND_VX_MPS
    blind_time_s = clamp(blind_time_s, 0.6, 3.2)
    return blind_dist_m, blind_time_s


def decide_rear_action_by_offset(target_found, offset_x_m, dist_front_m):
    if not target_found:
        return "stop", 0, "TARGET LOST", "NO_TARGET"

    distance_state = get_rear_distance_state(dist_front_m)
    base_speed = choose_rear_base_speed(distance_state)

    if distance_state == "INVALID":
        return "stop", 0, "INVALID DIST", distance_state

    if distance_state == "BLIND_READY":
        if abs(offset_x_m) <= REAR_BLIND_MAX_OFFSET_M:
            return "stop", 0, "ENTER REAR BLIND", distance_state
        turn_speed = int(clamp(round(base_speed * TURN_SPEED_RATIO), MIN_SPEED, MAX_SPEED))
        if offset_x_m < 0:
            return "turn_left", turn_speed, "BLIND NEAR BUT LEFT OFFSET", distance_state
        return "turn_right", turn_speed, "BLIND NEAR BUT RIGHT OFFSET", distance_state

    if abs(offset_x_m) <= OFFSET_FORWARD_DEADZONE_M:
        return "forward", base_speed, "CENTER: FORWARD", distance_state

    if abs(offset_x_m) >= OFFSET_TURN_THRESHOLD_M:
        turn_speed = int(clamp(round(base_speed * TURN_SPEED_RATIO), MIN_SPEED, MAX_SPEED))
        if offset_x_m < 0:
            return "turn_left", turn_speed, "LARGE LEFT OFFSET: TURN LEFT", distance_state
        return "turn_right", turn_speed, "LARGE RIGHT OFFSET: TURN RIGHT", distance_state

    if offset_x_m < 0:
        return "curve_left", base_speed, "LEFT OFFSET: CURVE LEFT", distance_state
    return "curve_right", base_speed, "RIGHT OFFSET: CURVE RIGHT", distance_state


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
    if distance_state in ("ARM_VERY_FAR", "ARM_FAR"):
        return speed_from_vx(ARM_FAR_VX_MPS, ARM_MIN_SPEED, ARM_MAX_SPEED)
    if distance_state == "ARM_MID":
        return speed_from_vx(ARM_MID_VX_MPS, ARM_MIN_SPEED, ARM_MAX_SPEED)
    if distance_state in ("ARM_NEAR", "ARM_BLIND_READY"):
        return speed_from_vx(ARM_NEAR_VX_MPS, ARM_MIN_SPEED, ARM_MAX_SPEED)
    return 0


def decide_arm_action_by_distance(target_found, offset_x_m, dist_arm_m):
    """Returns (action, speed, status, distance_state, desired_wz).

    action == "handoff" means: centered and within arm reach -> stop & grasp
    (this replaces the old "arm_blind_push").
    """
    if not target_found:
        return "stop", 0, "TARGET LOST", "NO_TARGET", 0.0

    distance_state = get_arm_distance_state(dist_arm_m)

    if distance_state == "INVALID":
        return "stop", 0, "INVALID DIST", distance_state, 0.0

    if distance_state == "ARM_BLIND_READY":
        if abs(offset_x_m) <= ARM_OFFSET_TURN_THRESHOLD_M:
            return "handoff", 0, "IN REACH -> HANDOFF TO ARM", distance_state, 0.0

    angle_error = math.atan2(offset_x_m, dist_arm_m) if dist_arm_m > 0 else 0.0
    desired_wz = clamp(ARM_KP_WZ * angle_error, -ARM_MAX_WZ, ARM_MAX_WZ)

    if abs(offset_x_m) <= ARM_OFFSET_FORWARD_DEADZONE_M:
        return "forward", choose_arm_forward_speed(distance_state), "ARM CENTER: FORWARD", distance_state, desired_wz

    turn_speed = speed_from_wz(desired_wz, ARM_MIN_TURN_SPEED, ARM_MAX_SPEED)
    if offset_x_m < 0:
        return "turn_left", turn_speed, "ARM OFFSET LEFT: TURN LEFT", distance_state, desired_wz
    return "turn_right", turn_speed, "ARM OFFSET RIGHT: TURN RIGHT", distance_state, desired_wz


def action_to_vxyz(action, speed) -> Tuple[float, float, float]:
    """Map a string nav action + speed number to set_car_motion (vx, vy, vz).

    Uses the same KX/KZ calibration the original speed numbers were derived from.
    vy (mecanum strafe) is left at 0 for now — turning is done with yaw (vz),
    matching the original motor-server behaviour.
    """
    spd = float(speed)
    vx = spd * KX
    vz = spd * KZ
    if action in ("forward", "arm_forward"):
        return (vx, 0.0, 0.0)
    if action == "turn_left":
        return (0.0, 0.0, +vz)
    if action == "turn_right":
        return (0.0, 0.0, -vz)
    if action == "curve_left":
        return (vx, 0.0, +0.5 * vz)
    if action == "curve_right":
        return (vx, 0.0, -0.5 * vz)
    # stop / handoff / unknown
    return (0.0, 0.0, 0.0)


def select_largest_box(results):
    boxes = results.boxes
    if boxes is None or len(boxes) == 0:
        return None
    best_box, best_area = None, -1
    for box in boxes:
        x1, y1, x2, y2 = (int(box.xyxy[0][i]) for i in range(4))
        area = max(1, x2 - x1) * max(1, y2 - y1)
        if area > best_area:
            best_area, best_box = area, box
    return best_box


# ════════════════════════════════════════════════════════════════════════════
# Camera reader (lazy cv2)
# ════════════════════════════════════════════════════════════════════════════

class LatestFrameReader:
    def __init__(self, url, width=FRAME_W, height=FRAME_H):
        import cv2
        import threading
        self._cv2 = cv2
        self.url, self.width, self.height = url, width, height
        # A bare integer string ("0") means a local webcam index, not a URL —
        # lets the pipeline dry-run against a dev-machine camera.
        self.cap = cv2.VideoCapture(int(url) if isinstance(url, str) and url.isdigit() else url)
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError(f"Unable to open camera source: {url}")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.latest_frame = None
        self.latest_time = 0.0
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def _reader_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                frame = self._cv2.resize(frame, (self.width, self.height))
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
        self.cap.release()
        self.thread.join(timeout=2.0)


# ════════════════════════════════════════════════════════════════════════════
# Navigator: dual-camera approach + grasp-target latch + verify, via set_car_motion
# ════════════════════════════════════════════════════════════════════════════

class Navigator:
    def __init__(self, device, model, *, show=False, dry_run=False,
                 rear_url=URL_REAR, arm_url=URL_ARM,
                 class_z_m: Optional[Dict[str, float]] = None,
                 cam_to_base_x: float = CAM_TO_BASE_X,
                 cam_to_base_y: float = CAM_TO_BASE_Y,
                 sign_y: float = SIGN_Y):
        mapping = (float(cam_to_base_x), float(cam_to_base_y), float(sign_y))
        if not all(math.isfinite(v) for v in mapping):
            raise ValueError("camera-to-base mapping values must be finite")
        if sign_y not in (-1.0, 1.0):
            raise ValueError("camera lateral sign must be +1 or -1")
        self.device = device          # Rosmaster handle (None in dry-run)
        self.model = model            # ultralytics YOLO
        self.show = show
        self.dry_run = dry_run
        self.rear_url, self.arm_url = rear_url, arm_url
        self.class_z_m = class_z_m or {"_fallback": OBJ_Z_FIXED}
        self.cam_to_base_x = float(cam_to_base_x)
        self.cam_to_base_y = float(cam_to_base_y)
        self.sign_y = float(sign_y)
        self._rear = None
        self._arm = None
        # last ARM detection captured at hand-off (for latch)
        self._last_arm = None         # dict: dist, offset, box_w_px, class_name

    # ── camera lifecycle ──
    def open_cameras(self):
        if self._rear is None:
            self._rear = LatestFrameReader(self.rear_url)
        if self._arm is None:
            self._arm = LatestFrameReader(self.arm_url)

    def release(self):
        for r in (self._rear, self._arm):
            if r is not None:
                try:
                    r.release()
                except Exception as e:
                    print(f"[nav][WARN] camera cleanup failed: {e}")
        self._rear = self._arm = None

    # ── chassis ──
    def _drive(self, action, speed):
        vx, vy, vz = action_to_vxyz(action, speed)
        if self.dry_run or self.device is None:
            print(f"[nav][dry] {action:>11} spd={speed:>3} -> set_car_motion({vx:+.3f},{vy:+.3f},{vz:+.3f})")
            return
        self.device.set_car_motion(vx, vy, vz)

    def stop(self):
        if self.dry_run or self.device is None:
            print("[nav][dry] STOP -> set_car_motion(0,0,0)")
            return
        self.device.set_car_motion(0.0, 0.0, 0.0)

    def back_off(self, t=RETREAT_TIME_S):
        """Short reverse to give room before a retry."""
        speed = speed_from_vx(RETREAT_VX_MPS)
        t0 = time.time()
        while time.time() - t0 < t:
            if self.dry_run or self.device is None:
                print(f"[nav][dry] back_off -> set_car_motion({-RETREAT_VX_MPS:+.3f},0,0)")
                break
            self.device.set_car_motion(-RETREAT_VX_MPS, 0.0, 0.0)
            time.sleep(0.05)
        self.stop()

    # ── detection helper ──
    def _detect_arm(self):
        """Return (found, dist_arm_m, offset_x_m, box_w_px, class_name)."""
        ok, frame, ts = self._arm.read()
        if not ok or (time.time() - ts) > MAX_FRAME_AGE:
            return False, -1.0, 0.0, 0.0, None
        res = self.model.predict(source=frame, conf=YOLO_CONF, imgsz=IMG_SIZE, verbose=False)[0]
        box = select_largest_box(res)
        if box is None:
            return False, -1.0, 0.0, 0.0, None
        class_name = class_name_for_box(self.model, box)
        x1, y1, x2, y2 = (int(box.xyxy[0][i]) for i in range(4))
        cx_box = (x1 + x2) / 2.0
        box_w_px = max(1, x2 - x1)
        # Arm-cam distortion is ~9px at the bbox bottom-center; theta was
        # solved on undistorted pixels, so undistort before the ground model.
        cx_u, y2_u = undistort_pixel(cx_box, y2, FX_ARM, FY_ARM, CX_ARM, CY_ARM, DIST_ARM)
        dist = estimate_ground_distance(y2_u, THETA_ARM, H_ARM, FY_ARM, CY_ARM)
        offset = estimate_pitched_offset_x(
            cx_u, dist, FX_ARM, CX_ARM, THETA_ARM, H_ARM
        )
        return True, dist, offset, box_w_px, class_name

    def _detect_rear(self):
        ok, frame, ts = self._rear.read()
        if not ok or (time.time() - ts) > MAX_FRAME_AGE:
            return False, -1.0, 0.0
        res = self.model.predict(source=frame, conf=YOLO_CONF, imgsz=IMG_SIZE, verbose=False)[0]
        box = select_largest_box(res)
        if box is None:
            return False, -1.0, 0.0
        x1, y1, x2, y2 = (int(box.xyxy[0][i]) for i in range(4))
        cx_box = (x1 + x2) / 2.0
        dist_rear = estimate_ground_distance(y2, THETA_REAR, H_REAR, FY_REAR, CY_REAR)
        dist_front = rear_dist_to_front_dist(dist_rear)
        offset = estimate_offset_x(cx_box, dist_rear, FX_REAR, CX_REAR)
        return True, dist_front, offset

    # ── main approach state machine ──
    def approach(self) -> bool:
        """Drive to the hand-off point. Returns True when ready to grasp."""
        self.open_cameras()
        state = "REAR_FOLLOW"
        last_seen = time.time()
        started_at = last_seen
        print(f"[nav] approach start (state={state})")

        while True:
            now = time.time()
            if now - started_at > APPROACH_MAX_DURATION_S:
                self.stop()
                print("[nav] approach timeout -> stopping")
                return False

            if state == "REAR_FOLLOW":
                found, dist_front, offset = self._detect_rear()
                if found:
                    last_seen = now
                    action, speed, status, dstate = decide_rear_action_by_offset(True, offset, dist_front)
                    print(f"[nav] REAR dist_front={dist_front:.2f} off={offset:+.2f} {action} ({status})")
                    if action == "stop" and dstate == "BLIND_READY":
                        self.stop()
                        _, blind_t = compute_rear_blind_plan(dist_front)
                        state = "REAR_BLIND"
                        print(f"[nav] -> REAR_BLIND for {blind_t:.2f}s")
                        self._rear_blind(blind_t)
                        state = "ARM_ALIGN"
                        # ARM target acquisition gets its own loss window; do not
                        # count the intentional blind-transfer time as target loss.
                        last_seen = time.time()
                        print("[nav] -> ARM_ALIGN")
                    else:
                        self._drive(action, speed)
                else:
                    self.stop()
                    if now - last_seen > TARGET_LOST_TIMEOUT_S:
                        print("[nav] target lost (REAR) — give up approach")
                        return False
                time.sleep(REAR_DECISION_INTERVAL)

            elif state == "ARM_ALIGN":
                found, dist_arm, offset, box_w, class_name = self._detect_arm()
                if found:
                    last_seen = now
                    action, speed, status, dstate, _wz = decide_arm_action_by_distance(True, offset, dist_arm)
                    print(f"[nav] ARM dist={dist_arm:.2f} off={offset:+.2f} {action} ({status})")
                    if action == "handoff":
                        self.stop()
                        self._last_arm = {
                            "dist": dist_arm,
                            "offset": offset,
                            "box_w_px": box_w,
                            "class_name": class_name,
                        }
                        print(f"[nav] HANDOFF: class={class_name} dist={dist_arm:.3f} "
                              f"off={offset:+.3f} box_w={box_w}px")
                        return True
                    self._drive(action, speed)
                else:
                    self.stop()
                    if now - last_seen > TARGET_LOST_TIMEOUT_S:
                        print("[nav] target lost (ARM) — give up approach")
                        return False
                time.sleep(ARM_DECISION_INTERVAL)

            if self.show:
                self._maybe_show()

    def _rear_blind(self, blind_time_s):
        t0 = time.time()
        while time.time() - t0 < blind_time_s:
            self._drive("forward", speed_from_vx(REAR_BLIND_VX_MPS))
            time.sleep(0.05)
            if self.dry_run or self.device is None:
                break
        self.stop()

    def _maybe_show(self):
        try:
            import cv2
            ok, frame, _ = self._arm.read()
            if ok:
                cv2.imshow("pipeline arm cam", frame)
                cv2.waitKey(1)
        except Exception:
            pass

    # ── grasp target latch + verify ──
    def latch_arm_object(self) -> Tuple[list, Optional[float]]:
        """Map the hand-off arm detection to a grasp-frame target + width."""
        if self._last_arm is None:
            # fall back to a sane straight-ahead target
            return [0.25 + self.cam_to_base_x, self.cam_to_base_y, OBJ_Z_FIXED], None
        d = self._last_arm["dist"]
        off = self._last_arm["offset"]
        class_name = self._last_arm.get("class_name")
        obj_x = d + self.cam_to_base_x
        obj_y = self.sign_y * off + self.cam_to_base_y
        obj_z = self.class_z_m.get(class_name, self.class_z_m.get("_fallback", OBJ_Z_FIXED))
        width_m = estimate_pitched_width(
            self._last_arm["box_w_px"], d, FX_ARM, THETA_ARM, H_ARM
        )
        pos = [round(obj_x, 4), round(obj_y, 4), round(obj_z, 4)]
        print(f"[nav] latched grasp target class={class_name} pos={pos} width={width_m:.3f}m")
        return pos, round(width_m, 4)

    def verify_grasp(self, latched_pos) -> bool:
        """Re-detect from the arm cam. Object still at the same spot ⇒ failed.

        Returns True if the grasp looks successful (object gone / moved away).
        """
        self.open_cameras()
        time.sleep(0.3)
        hits = 0
        for _ in range(5):
            found, dist, offset, _w, _class_name = self._detect_arm()
            if found and dist > 0:
                obj_x = dist + self.cam_to_base_x
                obj_y = self.sign_y * offset + self.cam_to_base_y
                if (abs(obj_x - latched_pos[0]) <= VERIFY_TOL_M
                        and abs(obj_y - latched_pos[1]) <= VERIFY_TOL_M):
                    hits += 1
            time.sleep(0.1)
        if hits >= 2:
            print(f"[verify] object still at spot ({hits}/5) -> grasp FAILED")
            return False
        print(f"[verify] object gone ({hits}/5 near spot) -> grasp OK")
        return True


# ════════════════════════════════════════════════════════════════════════════
# Orchestrator
# ════════════════════════════════════════════════════════════════════════════

def _load_grasp_module():
    """Import GraspController/DeployConfig from ../grasp (added to sys.path)."""
    grasp_dir = Path(__file__).resolve().parent.parent / "grasp"
    sys.path.insert(0, str(grasp_dir))
    import x3plus_real_grasp as g
    return g


def run_pipeline(args):
    if args.real:
        raise SystemExit(
            "real integrated grasp is disabled until the final alignment/latch "
            "uses the calibrated grasp-home homography; the existing nav-home "
            "camera geometry can see farther than the arm can reach"
        )
    if args.real and not args.i_confirm_camera_frame:
        raise SystemExit("real grasp requires a confirmed camera->base mapping")
    g = _load_grasp_module()
    from ultralytics import YOLO

    model_path = (Path(__file__).resolve().parent.parent
                  / "detection" / "models" / "best.pt")
    print(f"[pipeline] loading YOLO: {model_path}")
    model = YOLO(str(model_path))

    cfg = g.DeployConfig(serial_port=args.port)
    cfg.grip_width_control = True   # use the latched width to size the gripper
    if args.nav_home_deg is not None:
        cfg.home_deg = g.parse_deg6_csv(args.nav_home_deg)
    if args.grasp_home_deg is not None:
        cfg.grasp_home_deg = g.parse_deg6_csv(args.grasp_home_deg)
    class_z_m = parse_class_z(args.class_z, OBJ_Z_FIXED)
    if args.class_z:
        print(f"[pipeline] class z overrides: {class_z_m}")
    if args.handoff_dist is not None:
        globals()["ARM_BLIND_START_DIST_M"] = args.handoff_dist
    print(f"[pipeline] camera->base mapping: x={args.cam_x:+.4f}m "
          f"y={args.cam_y:+.4f}m sign_y={args.sign_y:+.0f}")

    controller = g.GraspController(cfg, real_servo=args.real, use_socket=False)
    try:
        nav = Navigator(
            controller.servo.device,
            model,
            show=args.show,
            dry_run=not args.real,
            class_z_m=class_z_m,
            cam_to_base_x=args.cam_x,
            cam_to_base_y=args.cam_y,
            sign_y=args.sign_y,
        )
    except Exception:
        controller.close()
        raise

    try:
        success = False
        for attempt in range(1, args.max_retries + 1):
            print("\n" + "=" * 60)
            print(f"[pipeline] ATTEMPT {attempt}/{args.max_retries}")
            print("=" * 60)

            print(f"[pipeline] moving arm to navigation home {list(cfg.home_deg)}")
            controller.servo.move_to_home()
            time.sleep(cfg.nav_home_wait_sec)

            if not nav.approach():
                print("[pipeline] no object reachable — stopping.")
                break

            obj_pos, width = nav.latch_arm_object()

            # Width gate: if the object is wider than the gripper can open, no
            # arm pose will ever grasp it — abort the whole run (retrying the
            # same too-wide object is pointless).
            if width is not None and width > MAX_GRASP_WIDTH_M:
                print(f"[pipeline] OBJECT TOO LARGE: width={width*100:.1f}cm > "
                      f"max {MAX_GRASP_WIDTH_M*100:.1f}cm — aborting (cannot grasp).")
                break

            controller.obj_provider = (lambda p=obj_pos, w=width: (p, w))

            grasp_seq_ok = controller.run(max_steps=args.max_steps)

            if not grasp_seq_ok:
                print("[pipeline] grasp sequence did not complete (stage machine "
                      "never reached done) — counting as failure.")
            elif nav.verify_grasp(obj_pos):
                print(f"[pipeline] OK grasp succeeded on attempt {attempt}.")
                success = True
                break
            print("[pipeline] FAILED grasp — retreating and retrying.")
            controller.servo.move_to_home()   # ensure gripper open for retry
            time.sleep(1.0)
            nav.back_off()

        if not success:
            print(f"\n[pipeline] gave up after {args.max_retries} attempt(s).")
    except KeyboardInterrupt:
        print("\n[pipeline] interrupted — emergency stop.")
        try:
            controller.servo.emergency_stop()
        except Exception:
            pass
    finally:
        try:
            nav.stop()
        except Exception as e:
            print(f"[pipeline][WARN] chassis stop failed: {e}")
        try:
            nav.release()
        finally:
            controller.close()


# ════════════════════════════════════════════════════════════════════════════
# Self-test (pure functions, no hardware / cameras / torch)
# ════════════════════════════════════════════════════════════════════════════

def run_selftest():
    print("== action_to_vxyz ==")
    for act, spd in [("forward", 30), ("turn_left", 30), ("turn_right", 40),
                     ("curve_left", 35), ("curve_right", 35), ("handoff", 0), ("stop", 0)]:
        print(f"  {act:>11} spd={spd:>3} -> {tuple(round(v,3) for v in action_to_vxyz(act, spd))}")

    print("\n== rear decision (offset=0.0) ==")
    for d in [2.0, 1.3, 0.95, 0.85]:
        print(f"  dist_front={d:.2f} -> {decide_rear_action_by_offset(True, 0.0, d)}")
    print("== rear decision (offset=±) at MID ==")
    for off in [-0.30, -0.15, 0.0, 0.15, 0.30]:
        print(f"  off={off:+.2f} -> {decide_rear_action_by_offset(True, off, 1.0)}")

    print("\n== arm decision ==")
    for d, off in [(0.60, 0.0), (0.40, 0.10), (0.30, -0.05), (0.22, 0.02), (0.22, 0.20)]:
        print(f"  dist={d:.2f} off={off:+.2f} -> {decide_arm_action_by_distance(True, off, d)}")

    print("\n== latch mapping (dist=0.22, off=+0.03, box_w=90px) ==")
    d, off, bw = 0.22, 0.03, 90
    obj_x = d + CAM_TO_BASE_X
    obj_y = SIGN_Y * off + CAM_TO_BASE_Y
    width = estimate_pitched_width(bw, d, FX_ARM, THETA_ARM, H_ARM)
    print(f"  pos=({obj_x:.3f},{obj_y:.3f},{OBJ_Z_FIXED}) width={width*100:.2f}cm")

    print("\n== rear blind plan ==")
    for d in [0.90, 1.20]:
        print(f"  dist_front={d:.2f} -> (blind_dist, blind_time)={tuple(round(x,3) for x in compute_rear_blind_plan(d))}")

    print("\n== arm-cam undistort round-trip + ground model ==")
    # Phase 2 measured points (raw clicks): D=0.25m@(286,428) ... D=0.50m@(238,126)
    for d_true, u, v in [(0.25, 286, 428), (0.30, 280, 352), (0.40, 249, 220),
                         (0.50, 238, 126)]:
        uu, vv = undistort_pixel(u, v, FX_ARM, FY_ARM, CX_ARM, CY_ARM, DIST_ARM)
        ur, vr = distort_pixel(uu, vv, FX_ARM, FY_ARM, CX_ARM, CY_ARM, DIST_ARM)
        rt_err = max(abs(ur - u), abs(vr - v))
        assert rt_err < 0.05, f"undistort round-trip {rt_err:.3f}px at ({u},{v})"
        d_est = estimate_ground_distance(vv, THETA_ARM, H_ARM, FY_ARM, CY_ARM)
        d_err = abs(d_est - d_true)
        assert d_err < 0.01, f"ground model err {d_err*100:.2f}cm at D={d_true}"
        print(f"  D={d_true:.2f} raw({u},{v}) -> undist({uu:6.1f},{vv:6.1f}) "
              f"est={d_est:.4f}m (err {d_err*100:.2f}cm, rt {rt_err:.4f}px)")

    print("\n== Phase-3 camera-to-base regression (2026-07-16 real measurements) ==")
    # Post-fix bridge outputs at four measured base-frame positions.  This
    # locks the deployed camera-to-base defaults and guards both lateral sign
    # and forward-distance drift.
    phase3_rows = [
        ("front", 0.4219, +0.0182, 0.2593, +0.0127),
        ("left",  0.4219, +0.0682, 0.2610, -0.0330),
        ("right", 0.4219, -0.0318, 0.2592, +0.0636),
        ("far",   0.5219, +0.0182, 0.3525, +0.0163),
    ]
    max_error_x = 0.0
    max_error_y = 0.0
    for label, base_x, base_y, raw_x, raw_right in phase3_rows:
        predicted_x = raw_x + CAM_TO_BASE_X
        predicted_y = SIGN_Y * raw_right + CAM_TO_BASE_Y
        error_x = abs(base_x - predicted_x)
        error_y = abs(base_y - predicted_y)
        max_error_x = max(max_error_x, error_x)
        max_error_y = max(max_error_y, error_y)
        print(f"  {label:>5}: pred=({predicted_x:.4f},{predicted_y:+.4f})m "
              f"base=({base_x:.4f},{base_y:+.4f})m "
              f"err=({error_x*100:.2f},{error_y*100:.2f})cm")

    assert max_error_x < 0.02, f"Phase-3 X residual {max_error_x*100:.2f}cm"
    assert max_error_y < 0.02, f"Phase-3 Y residual {max_error_y*100:.2f}cm"
    print("\n[selftest] OK")


def parse_args():
    p = argparse.ArgumentParser(description="X3Plus unified self-driving grasp pipeline")
    p.add_argument("--real", action="store_true", help="drive real wheels + servos (default dry-run)")
    p.add_argument("--show", action="store_true", help="show arm camera window")
    p.add_argument("--max-retries", type=int, default=3, help="grasp attempts before giving up")
    p.add_argument("--max-steps", type=int, default=300, help="grasp policy steps per attempt")
    p.add_argument("--port", type=str, default="/dev/myserial", help="Rosmaster serial port")
    p.add_argument("--handoff-dist", type=float, default=None,
                   help="override hand-off distance (m); default ARM_BLIND_START_DIST_M")
    p.add_argument("--nav-home-deg", type=str, default=None,
                   help="Navigation/cruise home servo degrees as S1,...,S6")
    p.add_argument("--grasp-home-deg", type=str, default=None,
                   help="PPO grasp initial servo degrees as S1,...,S6")
    p.add_argument("--class-z", action="append", default=[],
                   help="YOLO class height override, NAME=HEIGHT_M. Repeatable; "
                        "unlisted classes use OBJ_Z_FIXED.")
    p.add_argument("--cam-x", type=float, default=CAM_TO_BASE_X,
                   help="calibrated camera-origin to PPO base_link X offset (m)")
    p.add_argument("--cam-y", type=float, default=CAM_TO_BASE_Y,
                   help="calibrated camera-origin to PPO base_link Y offset (m)")
    p.add_argument("--sign-y", type=float, choices=(-1.0, 1.0), default=SIGN_Y,
                   help="camera-right to PPO base_link Y sign")
    p.add_argument("--i-confirm-camera-frame", action="store_true",
                   help="confirm Phase-3 camera->base mapping was measured on the real robot")
    p.add_argument("--selftest", action="store_true", help="run pure-logic self-test and exit")
    return p.parse_args()


def main():
    args = parse_args()
    if args.selftest:
        run_selftest()
        return
    if args.max_retries <= 0:
        raise SystemExit("--max-retries must be > 0")
    if args.max_steps <= 0:
        raise SystemExit("--max-steps must be > 0")
    if args.handoff_dist is not None and (
            not math.isfinite(args.handoff_dist) or args.handoff_dist <= 0.0):
        raise SystemExit("--handoff-dist must be finite and > 0")
    if not all(math.isfinite(v) for v in (args.cam_x, args.cam_y, args.sign_y)):
        raise SystemExit("--cam-x/--cam-y/--sign-y must be finite")
    if args.real and not args.i_confirm_camera_frame:
        raise SystemExit(
            "real grasp refused: Phase-3 camera->PPO base_link mapping is not "
            "proven; pass calibrated --cam-x/--cam-y/--sign-y and "
            "--i-confirm-camera-frame"
        )
    run_pipeline(args)


if __name__ == "__main__":
    main()
