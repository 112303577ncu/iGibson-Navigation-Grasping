#!/usr/bin/env python3
"""Camera intrinsic calibration (chessboard) for the X3Plus cameras.

Estimates fx, fy, cx, cy + distortion for one camera and reports the
reprojection error. Works against the MJPEG stream (stream_cam.py :8080) or a
local /dev/videoN device, so it can run on the dev PC or on the Jetson.

Board: print a standard chessboard (default 9x6 INNER corners, 25 mm squares)
on A4, glue it to something flat. --cols/--rows count INNER corners, not
squares. Measure a printed square with a ruler and pass the true --square-mm.

Capture: frames are auto-captured when corners are found AND the board has
moved enough since the last shot (so you just wave the board around slowly).
Cover: center, 4 corners of the image, near+far, tilted ~30 deg each way.

Usage:
    # arm camera via stream (arm must be at NAV HOME so the view is the real one)
    python3 calibrate_intrinsics.py \
        --source "http://127.0.0.1:8080/stream?topic=/arm_cam/image_raw" \
        --out arm_cam_intrinsics.json

    # mast (body) camera, local device on the Jetson
    python3 calibrate_intrinsics.py --source 1 --out rear_cam_intrinsics.json

    # verify an existing result: live undistort preview (needs a display)
    python3 calibrate_intrinsics.py --source 1 --check rear_cam_intrinsics.json

Pass criteria (see CALIBRATION_PLAN.md Phase 1):
    reprojection error < 0.5 px (ideally < 0.3), fx ~= fy within ~2%,
    (cx, cy) within ~(320, 240) +- 40 for the 640x480 streams.
"""
from __future__ import annotations

import argparse
import json
import os
import time

import cv2
import numpy as np


def open_source(src: str) -> cv2.VideoCapture:
    if src.isdigit():
        cap = cv2.VideoCapture(int(src))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    else:
        cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        raise SystemExit(f"cannot open source: {src}")
    return cap


def corner_motion(prev: np.ndarray, cur: np.ndarray) -> float:
    """Mean corner displacement (px) between two detections."""
    return float(np.mean(np.linalg.norm(prev.reshape(-1, 2) - cur.reshape(-1, 2), axis=1)))


def calibrate(args):
    pattern = (args.cols, args.rows)
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= args.square_mm / 1000.0

    cap = open_source(args.source)
    obj_points, img_points = [], []
    last_corners = None
    last_shot = 0.0
    img_size = None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    print(f"[calib] need {args.shots} shots — move the board: center / 4 corners "
          f"/ near / far / tilted. Ctrl+C to stop early.")
    try:
        while len(obj_points) < args.shots:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img_size = gray.shape[::-1]
            found, corners = cv2.findChessboardCorners(
                gray, pattern,
                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            if found:
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
                now = time.time()
                moved = (last_corners is None
                         or corner_motion(last_corners, corners) > args.min_move_px)
                if moved and now - last_shot > args.min_interval_s:
                    obj_points.append(objp)
                    img_points.append(corners)
                    last_corners, last_shot = corners, now
                    print(f"[calib] shot {len(obj_points)}/{args.shots}")
            if args.show:
                vis = frame.copy()
                if found:
                    cv2.drawChessboardCorners(vis, pattern, corners, found)
                cv2.putText(vis, f"{len(obj_points)}/{args.shots}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.imshow("calib", vis)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()

    if len(obj_points) < 8:
        raise SystemExit(f"only {len(obj_points)} shots — need >= 8 for a usable fit")

    print(f"[calib] solving with {len(obj_points)} views...")
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None)

    # per-view reprojection errors (catches one bad view poisoning the fit)
    errs = []
    for op, ip, rv, tv in zip(obj_points, img_points, rvecs, tvecs):
        proj, _ = cv2.projectPoints(op, rv, tv, K, dist)
        errs.append(float(cv2.norm(ip, proj, cv2.NORM_L2) / np.sqrt(len(proj))))

    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])

    # Distortion displacement: how far raw pixels move after undistortion.
    # The ground-distance model reads RAW y2/cx_box, so this decides whether
    # distortion can be ignored (see CALIBRATION_PLAN.md Phase 1).
    w, h = img_size
    probes = np.array([
        [w * 0.50, h * 0.85],   # bbox-bottom region (drives estimate_ground_distance)
        [w * 0.50, h * 0.50],
        [w * 0.10, h * 0.85], [w * 0.90, h * 0.85],
        [w * 0.10, h * 0.10], [w * 0.90, h * 0.10],
    ], dtype=np.float32).reshape(-1, 1, 2)
    und = cv2.undistortPoints(probes, K, dist, P=K).reshape(-1, 2)
    disp = np.linalg.norm(und - probes.reshape(-1, 2), axis=1)
    disp_bottom_center = float(disp[0])
    disp_max = float(np.max(disp))

    result = {
        "image_size": list(img_size),
        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
        "dist": [float(d) for d in dist.ravel()],
        "rms_reprojection_px": float(rms),
        "per_view_err_px": [round(e, 3) for e in errs],
        "distortion_disp_px": {"bbox_bottom_center": round(disp_bottom_center, 2),
                               "max_probe": round(disp_max, 2)},
        "views": len(obj_points),
        "board": {"cols": args.cols, "rows": args.rows, "square_mm": args.square_mm},
        "source": args.source,
        "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(result, f, indent=2)

    print("\n══════════ RESULT ══════════")
    print(f"  fx={fx:.2f}  fy={fy:.2f}   (fx/fy ratio {fx / fy:.4f})")
    print(f"  cx={cx:.2f}  cy={cy:.2f}   (image {img_size[0]}x{img_size[1]})")
    print(f"  dist={np.round(dist.ravel(), 4).tolist()}")
    print(f"  RMS reprojection = {rms:.3f} px  (worst view {max(errs):.3f})")
    print(f"  distortion displacement: bbox-bottom-center {disp_bottom_center:.2f} px, "
          f"max probe {disp_max:.2f} px")
    if disp_bottom_center < 2.0:
        print("  -> distortion NEGLIGIBLE for the ground-distance model "
              "(record 'ignored' in CALIBRATION_PLAN)")
    else:
        print("  -> distortion NOT negligible: undistort frames before YOLO, or "
              "run cv2.undistortPoints on bbox bottom/center before the "
              "distance/offset math")
    print(f"  saved -> {args.out}")
    if abs(cx - img_size[0] / 2) > 40 or abs(cy - img_size[1] / 2) > 40:
        print("  WARNING: principal point far from image center — check the "
              "stream is not cropped/scaled and the board covered the corners")
    ok = rms < 0.5 and abs(fx / fy - 1.0) < 0.02
    verdict = ("YES" if ok else
               "NO — re-shoot (more tilt/corner coverage, flatter board, "
               "check --square-mm)")
    print(f"  PASS: {verdict}")


def check(args):
    with open(args.check) as f:
        c = json.load(f)
    K = np.array([[c["fx"], 0, c["cx"]], [0, c["fy"], c["cy"]], [0, 0, 1]])
    dist = np.array(c["dist"])
    cap = open_source(args.source)
    print("[check] undistort preview — straight edges (door frames, tiles) must "
          "look straight. q/ESC to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.1)
            continue
        und = cv2.undistort(frame, K, dist)
        cv2.imshow("undistorted", np.hstack([frame, und]))
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            break
    cap.release()
    cv2.destroyAllWindows()


def main():
    p = argparse.ArgumentParser(description="chessboard intrinsic calibration")
    p.add_argument("--source", required=True,
                   help="stream URL or local device index (e.g. 0, 1)")
    p.add_argument("--out", default="intrinsics.json")
    p.add_argument("--check", default=None,
                   help="load this json and show a live undistort preview instead")
    p.add_argument("--cols", type=int, default=9, help="INNER corners per row")
    p.add_argument("--rows", type=int, default=6, help="INNER corners per column")
    p.add_argument("--square-mm", type=float, default=25.0)
    p.add_argument("--shots", type=int, default=18)
    p.add_argument("--min-move-px", type=float, default=25.0)
    p.add_argument("--min-interval-s", type=float, default=0.8)
    p.add_argument("--show", action="store_true", help="preview window (needs display)")
    args = p.parse_args()
    if args.check:
        check(args)
    else:
        calibrate(args)


if __name__ == "__main__":
    main()
