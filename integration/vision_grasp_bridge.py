#!/usr/bin/env python3
"""Vision → Grasp bridge for X3Plus.

Runs YOLOv11 (Ultralytics) on the arm-camera stream, turns the detected
bounding box into an object position + width in the arm/grasp coordinate
frame, and pushes it to the grasp controller over TCP.

Pipeline:
    arm camera ──YOLO──▶ bbox ──geometry──▶ {x, y, z, w}(m) ──TCP 5555──▶
        x3plus_real_grasp.py  (run with --real --socket [--width-grip])

The grasp side (grasp/x3plus_real_grasp.py, class DetectionReceiver) accepts
one JSON object per TCP connection and reads until EOF, so this bridge opens a
fresh connection for every message and closes it right after sending.

Navigation home uses the measured fixed-pitch ground model.  Grasp home uses
an independently calibrated planar homography from undistorted bbox bottom
pixels directly to base_link XY; the navigation-home extrinsics are never
reused after the arm moves.

Calibration status:
    * All built-in H/theta/cam-offset constants are NAV-HOME ONLY.
    * GRASP-HOME has no guessed defaults and requires --homography at runtime.
    * Runtime rejects extrapolation: the calibration pixel/base hull must cover
      the target bottom-center AND both bbox bottom corners (its full width).
    * FIXED_THETA / camera intrinsics — DONE 2026-07-08 (Phase 1/2), constants
      below; bbox bottom-center is undistorted before the ground model.
    * --cam-x / --cam-y / --sign-y — DONE 2026-07-16 (Phase 3)
    * --obj-z — Phase 4 per-class grasp tuning; bottle-cap starts at 0.02m

Examples:
    # collect one median grasp-home calibration pixel (never opens TCP)
    python vision_grasp_bridge.py --stream 0 --calibration-only --once --show

    # send one verified grasp-home detection
    python vision_grasp_bridge.py --stream 0 --homography grasp_home.json --once
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import statistics
import time
from pathlib import Path
from typing import Dict, List, Optional

import cv2
from ultralytics import YOLO

# ── Default model (relative to this script: integration/ → detection/models/) ──
DEFAULT_MODEL = str(Path(__file__).resolve().parent.parent / "detection" / "models" / "best.pt")

# ── Arm-camera intrinsics / mounting (measured 2026-07-08, Phase 1/2;
#    arm_cam_intrinsics.json RMS 0.495px; valid at nav home — the camera rides
#    on arm_link4). Distortion is ~9px at the bbox bottom-center and THETA was
#    solved on undistorted pixels, so (cx_box, y2) go through undistort_pixel()
#    before the ground model. ──
CX_CAM = 212.23  # principal point x (px)
CY = 168.42      # principal point y (px)
FX = 919.08      # focal length x (px)
FY = 919.41      # focal length y (px)
DIST = (-0.3764, -0.0748, -0.0015, 0.0035, 0.4793)  # k1 k2 p1 p2 k3
H = 0.332        # camera height above ground (m)
FIXED_THETA = 36.40  # camera pitch angle (deg), Phase 2 median (std 0.18)

DEFAULT_STREAM = "http://127.0.0.1:8080/stream?topic=/arm_cam/image_raw"
IMG_SIZE = 640
CAMERA_REOPEN_FAILURES = 10
CAMERA_REOPEN_DELAY_SEC = 0.5
CAMERA_OPEN_ATTEMPTS = 3
CAMERA_PROBE_READS = 3
CAMERA_PROBE_DELAY_SEC = 0.05

# Phase-3 camera ground coordinates -> PPO/URDF base_link mapping.
CAM_TO_BASE_X = 0.1639
CAM_TO_BASE_Y = 0.0331
SIGN_X = 1.0
SIGN_Y = -1.0


def resolve_camera_geometry(args) -> None:
    """Resolve the selected pose mapping and fail closed at grasp home.

    Navigation home retains the measured pinhole-ground model.  Grasp home is
    mapped directly by a calibrated pixel-to-base homography, avoiding unsafe
    assumptions about its very different height, pitch and forward direction.
    Calibration-only mode never opens the TCP connection and therefore may
    collect pixels before a homography exists.
    """
    args.homography_matrix = None
    args.homography_document = None
    if args.calibration_only and args.camera_pose != "grasp-home":
        raise SystemExit("--calibration-only is only supported at --camera-pose grasp-home")
    if args.camera_pose == "grasp-home":
        if args.calibration_only:
            return
        if not args.homography:
            raise SystemExit(
                "grasp-home detection requires --homography; nav-home H/theta/"
                "cam offsets are invalid after the arm moves"
            )
        try:
            try:
                from .grasp_home_homography import load_calibration
            except ImportError:
                from grasp_home_homography import load_calibration
            document = load_calibration(
                args.homography, min_points=6, max_error_m=0.02
            )
        except Exception as exc:
            raise SystemExit(f"invalid grasp-home homography: {exc}") from exc
        args.homography_document = document
        args.homography_matrix = document["homography"]
        args.homography_pixel_hull = convex_hull(
            [(row["u"], row["v"]) for row in document["points"]]
        )
        args.homography_base_hull = convex_hull(
            [(row["x"], row["y"]) for row in document["points"]]
        )
        return

    defaults = (H, FIXED_THETA, CAM_TO_BASE_X, CAM_TO_BASE_Y, SIGN_X, SIGN_Y)
    values = (
        args.camera_height, args.camera_theta, args.cam_x, args.cam_y,
        args.sign_x, args.sign_y,
    )
    resolved = [default if value is None else value
                for value, default in zip(values, defaults)]
    (args.camera_height, args.camera_theta, args.cam_x, args.cam_y,
     args.sign_x, args.sign_y) = (float(v) for v in resolved)
    numeric = (args.camera_height, args.camera_theta, args.cam_x, args.cam_y)
    if not all(math.isfinite(v) for v in numeric):
        raise SystemExit("camera geometry values must be finite")
    if args.camera_height <= 0.0:
        raise SystemExit("--camera-height must be > 0")
    if not 0.0 < args.camera_theta < 90.0:
        raise SystemExit("--camera-theta must be between 0 and 90 degrees")
    if args.sign_x not in (-1.0, 1.0) or args.sign_y not in (-1.0, 1.0):
        raise SystemExit("--sign-x and --sign-y must be +1 or -1")


def convex_hull(points):
    """Return a counter-clockwise convex hull (monotonic chain)."""
    unique = sorted({(float(x), float(y)) for x, y in points})
    if len(unique) <= 1:
        return unique

    def cross(o, a, b):
        return ((a[0] - o[0]) * (b[1] - o[1])
                - (a[1] - o[1]) * (b[0] - o[0]))

    lower = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def point_in_convex_hull(point, hull, tolerance: float = 1e-7) -> bool:
    """True when a point is inside/on a counter-clockwise convex hull."""
    if len(hull) < 3:
        return False
    px, py = float(point[0]), float(point[1])
    for index, start in enumerate(hull):
        end = hull[(index + 1) % len(hull)]
        cross = ((end[0] - start[0]) * (py - start[1])
                 - (end[1] - start[1]) * (px - start[0]))
        if cross < -tolerance:
            return False
    return True


def apply_grasp_home_mapping(args, u: float, v: float):
    """Map one undistorted pixel and reject homography extrapolation."""
    pixel = (float(u), float(v))
    if not point_in_convex_hull(pixel, args.homography_pixel_hull):
        raise ValueError(
            f"undistorted pixel ({u:.1f},{v:.1f}) is outside calibration hull"
        )
    try:
        try:
            from .grasp_home_homography import apply_homography
        except ImportError:
            from grasp_home_homography import apply_homography
        mapped = apply_homography(args.homography_matrix, pixel)
    except Exception as exc:
        raise ValueError(f"homography mapping failed: {exc}") from exc
    base_xy = (float(mapped[0]), float(mapped[1]))
    if not point_in_convex_hull(base_xy, args.homography_base_hull):
        raise ValueError(
            f"mapped base point ({base_xy[0]:.4f},{base_xy[1]:.4f}) is "
            "outside calibrated base hull"
        )
    return base_xy


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
        height = float(value)
        if not math.isfinite(height) or height < 0.0:
            raise ValueError(f"--class-z height must be finite and non-negative: {raw}")
        out[name] = height
    fallback_z = float(fallback_z)
    if not math.isfinite(fallback_z) or fallback_z < 0.0:
        raise ValueError(f"--obj-z must be finite and non-negative, got {fallback_z}")
    out.setdefault("_fallback", fallback_z)
    return out


def class_name_for_box(model: YOLO, box) -> str:
    cls_id = int(box.cls[0].item())
    names = model.names
    if isinstance(names, dict):
        return str(names.get(cls_id, cls_id))
    if 0 <= cls_id < len(names):
        return str(names[cls_id])
    return str(cls_id)


def undistort_pixel(u: float, v: float, iters: int = 8):
    """Undistort one arm-cam pixel (plumb-bob), same iteration as
    cv2.undistortPoints(..., P=K). Mirror of vision_grasp_pipeline.py."""
    k1, k2, p1, p2, k3 = DIST
    xd = (u - CX_CAM) / FX
    yd = (v - CY) / FY
    x, y = xd, yd
    for _ in range(iters):
        r2 = x * x + y * y
        radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x = (xd - dx) / radial
        y = (yd - dy) / radial
    return CX_CAM + x * FX, CY + y * FY


def estimate_distance(y_max: float, camera_height: float = H,
                      camera_theta_deg: float = FIXED_THETA) -> float:
    """Ground distance from the arm camera to the object's lowest pixel (m).

    Same pin-hole + fixed-pitch model as detection/arm_cam.py. y_max must be
    an UNDISTORTED pixel row (see undistort_pixel). Returns -1.0 when the
    geometry is degenerate (object above the horizon).
    """
    theta = math.radians(camera_theta_deg)
    alpha = math.atan((y_max - CY) / FY)
    total_angle = theta + alpha
    if total_angle <= 0 or total_angle >= math.pi / 2:
        return -1.0
    return camera_height / math.tan(total_angle)


def optical_depth_from_ground_distance(
        dist_m: float, camera_height: float = H,
        camera_theta_deg: float = FIXED_THETA) -> float:
    """Return pinhole Z-depth for a ground point seen by the pitched camera.

    ``dist_m`` is horizontal ground distance. Horizontal pixel spans project
    against ``Z_cam = D*cos(theta) + H*sin(theta)``, not against D itself.
    """
    if dist_m <= 0.0:
        return 0.0
    theta = math.radians(camera_theta_deg)
    return dist_m * math.cos(theta) + camera_height * math.sin(theta)


def estimate_lateral_offset(cx_pixel: float, dist_m: float,
                            camera_height: float = H,
                            camera_theta_deg: float = FIXED_THETA) -> float:
    """Right-positive ground-plane lateral offset in metres."""
    depth_m = optical_depth_from_ground_distance(
        dist_m, camera_height, camera_theta_deg
    )
    return depth_m * (cx_pixel - CX_CAM) / FX


def estimate_metric_width(width_px: float, dist_m: float,
                          camera_height: float = H,
                          camera_theta_deg: float = FIXED_THETA) -> float:
    """Convert an image-plane pixel width using the same optical depth."""
    if width_px <= 0.0:
        return 0.0
    return width_px * optical_depth_from_ground_distance(
        dist_m, camera_height, camera_theta_deg
    ) / FX


def open_capture(stream: str) -> cv2.VideoCapture:
    """Open a cv2 capture. A bare integer string is treated as a webcam index."""
    if stream.isdigit():
        return cv2.VideoCapture(int(stream))
    return cv2.VideoCapture(stream)


def open_capture_checked(stream: str, attempts: int = CAMERA_OPEN_ATTEMPTS,
                         probe_reads: int = CAMERA_PROBE_READS,
                         retry_delay: float = CAMERA_REOPEN_DELAY_SEC,
                         probe_delay: float = CAMERA_PROBE_DELAY_SEC):
    """Open a capture and prove it by reading a real frame.

    ``VideoCapture.isOpened()`` alone is not sufficient for hot-unplugged or
    busy V4L2 devices: some backends briefly report open and then every read
    fails.  A capture is returned only after one non-empty frame.  Failed
    handles are always released and retry count is finite.

    Returns ``(capture, first_frame, error_text)``.  On failure the first two
    values are ``None`` and ``error_text`` explains the last failed attempt.
    """
    attempts = max(1, int(attempts))
    probe_reads = max(1, int(probe_reads))
    last_error = "unknown camera error"

    for attempt in range(1, attempts + 1):
        cap = None
        success = False
        try:
            if stream.isdigit() and Path("/dev").is_dir():
                device_path = Path("/dev") / f"video{int(stream)}"
                if not device_path.exists():
                    last_error = f"{device_path} does not exist"
                    print(f"[bridge] camera open attempt {attempt}/{attempts} "
                          f"failed: {last_error}")
                    if attempt < attempts and retry_delay > 0.0:
                        time.sleep(retry_delay)
                    continue

            cap = open_capture(stream)
            if cap is None or not cap.isOpened():
                last_error = "backend could not open the source"
            else:
                for _ in range(probe_reads):
                    ok, frame = cap.read()
                    if ok and frame is not None and getattr(frame, "size", 0) > 0:
                        success = True
                        return cap, frame, ""
                    if probe_delay > 0.0:
                        time.sleep(probe_delay)
                last_error = (
                    f"source opened but produced no frame in {probe_reads} probe reads"
                )
        except Exception as exc:
            last_error = f"{type(exc).__name__}: {exc}"
        finally:
            # Python executes finally even on return; keep only a capture that
            # has already produced a non-empty frame.
            if cap is not None and not success:
                try:
                    cap.release()
                except Exception:
                    pass

        print(f"[bridge] camera open attempt {attempt}/{attempts} "
              f"failed: {last_error}")
        if attempt < attempts and retry_delay > 0.0:
            time.sleep(retry_delay)

    return None, None, last_error


def send_detection(host: str, port: int, payload: dict, timeout: float = 2.0) -> bool:
    """Open a fresh TCP connection, send one JSON payload, close. True on success."""
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.sendall(json.dumps(payload).encode("utf-8"))
        return True
    except OSError as e:
        print(f"[bridge] send failed → {host}:{port}: {e}")
        return False


def pick_best_box(result):
    """Return the highest-confidence box, or None."""
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return None
    confs = boxes.conf.tolist()
    best_i = max(range(len(confs)), key=lambda i: confs[i])
    return boxes[best_i]


def median_calibration_record(records: List[dict]) -> dict:
    """Collapse one stable detection batch to median bbox ground pixels."""
    if not records:
        raise ValueError("calibration record batch is empty")

    def med(key: str) -> float:
        return round(float(statistics.median(row[key] for row in records)), 3)

    return {
        "camera_pose": "grasp-home",
        "samples": len(records),
        "class": records[0]["class"],
        "u": med("u"),
        "v": med("v"),
        "left_u": med("left_u"),
        "left_v": med("left_v"),
        "right_u": med("right_u"),
        "right_v": med("right_v"),
        "raw_u": med("raw_u"),
        "raw_v": med("raw_v"),
    }


def parse_args():
    p = argparse.ArgumentParser(description="X3Plus vision → grasp TCP bridge")
    p.add_argument("--host", default="127.0.0.1", help="grasp controller host (default 127.0.0.1)")
    p.add_argument("--port", type=int, default=5555, help="grasp controller TCP port (default 5555)")
    p.add_argument("--model", default=DEFAULT_MODEL, help="YOLO .pt path (default detection/models/best.pt)")
    p.add_argument("--stream", default=DEFAULT_STREAM, help="camera URL or webcam index (e.g. 0)")
    p.add_argument("--conf", type=float, default=0.3, help="YOLO confidence threshold (default 0.3)")
    p.add_argument("--imgsz", type=int, default=IMG_SIZE, help="YOLO inference size (default 640)")
    p.add_argument("--rate", type=float, default=0.3, help="min seconds between sends (default 0.3)")
    p.add_argument("--once", action="store_true",
                   help="send/output the first valid detection batch then exit")
    p.add_argument("--show", action="store_true", help="show annotated camera window")
    p.add_argument("--camera-pose", choices=("grasp-home", "nav-home"),
                   default="grasp-home",
                   help="arm pose used for detection (default grasp-home)")
    p.add_argument("--homography", default=None,
                   help="verified grasp-home pixel-to-base calibration JSON")
    p.add_argument("--calibration-only", action="store_true",
                   help="print undistorted bbox pixels only; never connect to TCP")
    p.add_argument("--calibration-samples", type=int, default=10,
                   help="valid frames per median calibration output (default 10)")
    p.add_argument("--camera-height", type=float, default=None,
                   help="nav-home lens height above ground override (m)")
    p.add_argument("--camera-theta", type=float, default=None,
                   help="nav-home downward camera pitch override (deg)")
    # ── coordinate-frame calibration ──
    p.add_argument("--cam-x", type=float, default=None,
                   help="forward offset added to camera distance → grasp X (m)")
    p.add_argument("--cam-y", type=float, default=None,
                   help="lateral offset added to grasp Y (m)")
    p.add_argument("--obj-z", type=float, default=0.02,
                   help="object height in the grasp frame (m, default 0.02)")
    p.add_argument("--class-z", action="append", default=[],
                   help="YOLO class height override, NAME=HEIGHT_M. Repeatable; "
                        "unlisted classes use --obj-z.")
    p.add_argument("--sign-x", type=float, default=None, choices=(-1.0, 1.0),
                   help="nav-home camera-forward to base +X sign")
    p.add_argument("--sign-y", type=float, default=None, choices=(-1.0, 1.0),
                   help="sign mapping camera lateral offset → grasp +Y (+1 or -1)")
    return p.parse_args()


def main():
    args = parse_args()
    resolve_camera_geometry(args)
    if not 0.0 <= args.conf <= 1.0:
        raise SystemExit("--conf must be between 0 and 1")
    if args.imgsz <= 0:
        raise SystemExit("--imgsz must be > 0")
    if not math.isfinite(args.rate) or args.rate < 0.0:
        raise SystemExit("--rate must be finite and >= 0")
    if args.calibration_samples <= 0:
        raise SystemExit("--calibration-samples must be > 0")

    print(f"[bridge] loading YOLO model: {args.model}")
    model = YOLO(args.model)
    print(f"[bridge] classes: {model.names}")
    class_z = parse_class_z(args.class_z, args.obj_z)
    if args.class_z:
        print(f"[bridge] class z overrides: {class_z}")

    print(f"[bridge] opening camera: {args.stream}")
    cap, pending_frame, camera_error = open_capture_checked(args.stream)
    if cap is None:
        raise SystemExit(
            f"[bridge] ERROR: camera {args.stream!r} failed after "
            f"{CAMERA_OPEN_ATTEMPTS} attempts: {camera_error}. "
            "Check that the device exists and no other process owns it "
            "(for USB index 0: fuser -v /dev/video0)."
        )
    print(f"[bridge] camera ready: {args.stream} "
          f"frame_shape={tuple(pending_frame.shape)}")

    if args.calibration_only:
        print(f"[bridge] calibration-only: pose={args.camera_pose}, median of "
              f"{args.calibration_samples} valid frames; TCP disabled")
    elif args.camera_pose == "grasp-home":
        fit = args.homography_document["fit"]
        print(f"[bridge] grasp-home homography: points={fit['point_count']} "
              f"max_error={fit['max_error_m'] * 100:.2f}cm")
    else:
        print(f"[bridge] nav-home model: H={args.camera_height:.4f}m "
              f"theta={args.camera_theta:.3f}deg signs="
              f"({args.sign_x:+.0f},{args.sign_y:+.0f})")
    if not args.calibration_only:
        print(f"[bridge] streaming detections → {args.host}:{args.port} "
              f"(rate={args.rate}s, conf={args.conf})"
              f"{' [--once]' if args.once else ''}")

    last_send = 0.0
    consecutive_read_failures = 0
    calibration_records: List[dict] = []
    try:
        while True:
            if pending_frame is not None:
                ok, frame = True, pending_frame
                pending_frame = None
            else:
                ok, frame = cap.read()
            if not ok or frame is None:
                consecutive_read_failures += 1
                if consecutive_read_failures in (1, CAMERA_REOPEN_FAILURES):
                    print(f"[bridge] WARN: failed to read frame "
                          f"({consecutive_read_failures}/"
                          f"{CAMERA_REOPEN_FAILURES})")
                if consecutive_read_failures >= CAMERA_REOPEN_FAILURES:
                    print(f"[bridge] reopening camera after "
                          f"{consecutive_read_failures} consecutive read failures")
                    cap.release()
                    cap, pending_frame, camera_error = open_capture_checked(args.stream)
                    if cap is None:
                        raise SystemExit(
                            f"[bridge] ERROR: camera {args.stream!r} could not "
                            f"recover after {CAMERA_OPEN_ATTEMPTS} attempts: "
                            f"{camera_error}. Stopping instead of sending stale data."
                        )
                    consecutive_read_failures = 0
                    print(f"[bridge] camera reopened and frame verified: "
                          f"{args.stream} frame_shape={tuple(pending_frame.shape)}")
                else:
                    time.sleep(0.05)
                continue
            consecutive_read_failures = 0

            result = model.predict(source=frame, conf=args.conf, imgsz=args.imgsz, verbose=False)[0]
            annotated = result.plot() if args.show else None

            box = pick_best_box(result)
            payload: Optional[dict] = None
            if box is not None:
                class_name = class_name_for_box(model, box)
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx_box = (x1 + x2) / 2.0
                box_w_px = max(1.0, x2 - x1)

                cx_u, y2_u = undistort_pixel(cx_box, y2)
                left_u, left_v = undistort_pixel(x1, y2)
                right_u, right_v = undistort_pixel(x2, y2)

                if args.calibration_only:
                    if (calibration_records
                            and calibration_records[0]["class"] != class_name):
                        print("[bridge][calibration] class changed; resetting batch")
                        calibration_records.clear()
                    calibration_records.append({
                        "class": class_name,
                        "u": cx_u,
                        "v": y2_u,
                        "left_u": left_u,
                        "left_v": left_v,
                        "right_u": right_u,
                        "right_v": right_v,
                        "raw_u": cx_box,
                        "raw_v": y2,
                    })
                    if len(calibration_records) >= args.calibration_samples:
                        record = median_calibration_record(calibration_records)
                        print("[bridge][calibration] "
                              + json.dumps(record, sort_keys=True))
                        calibration_records.clear()
                        if args.once:
                            break
                else:
                    try:
                        if args.camera_pose == "grasp-home":
                            obj_x, obj_y = apply_grasp_home_mapping(args, cx_u, y2_u)
                            left_xy = apply_grasp_home_mapping(args, left_u, left_v)
                            right_xy = apply_grasp_home_mapping(args, right_u, right_v)
                            width_m = math.hypot(
                                right_xy[0] - left_xy[0],
                                right_xy[1] - left_xy[1],
                            )
                        else:
                            arm_dist = estimate_distance(
                                y2_u, args.camera_height, args.camera_theta
                            )
                            if arm_dist <= 0.0:
                                raise ValueError("ground ray does not intersect in front of camera")
                            offset_x = estimate_lateral_offset(
                                cx_u, arm_dist, args.camera_height, args.camera_theta
                            )
                            width_m = estimate_metric_width(
                                box_w_px, arm_dist,
                                args.camera_height, args.camera_theta,
                            )
                            obj_x = args.cam_x + args.sign_x * arm_dist
                            obj_y = args.cam_y + args.sign_y * offset_x
                    except ValueError as exc:
                        print(f"[bridge] WARN: rejected detection: {exc}")
                    else:
                        obj_z = class_z.get(class_name, class_z["_fallback"])
                        values = (obj_x, obj_y, obj_z, width_m)
                        if not all(math.isfinite(v) for v in values) or width_m < 0.0:
                            print(f"[bridge] WARN: rejected invalid geometry {values}")
                        else:
                            payload = {
                                "x": round(obj_x, 4),
                                "y": round(obj_y, 4),
                                "z": round(obj_z, 4),
                                "w": round(width_m, 4),
                                "class": class_name,
                                "camera_pose": args.camera_pose,
                            }

                            if args.show and annotated is not None:
                                cv2.circle(annotated, (int(cx_box), int(y2)),
                                           6, (0, 0, 255), -1)
                                cv2.putText(
                                    annotated,
                                    f"{class_name} x={obj_x:.2f} y={obj_y:.2f} "
                                    f"z={obj_z:.2f} w={width_m:.3f}m",
                                    (int(x1), max(20, int(y1) - 8)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (0, 255, 255), 2,
                                )

            now = time.monotonic()
            if payload is not None and (now - last_send) >= args.rate:
                if send_detection(args.host, args.port, payload):
                    last_send = now
                    print(f"[bridge] sent {payload}")
                    if args.once:
                        break

            if args.show and annotated is not None:
                center = int(CX_CAM)
                cv2.line(annotated, (center, 0), (center, annotated.shape[0]), (0, 255, 255), 1)
                cv2.imshow("X3Plus vision → grasp bridge", annotated)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
    except KeyboardInterrupt:
        print("\n[bridge] interrupted.")
    finally:
        if cap is not None:
            cap.release()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
