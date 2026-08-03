#!/usr/bin/env python3
"""Run the validated v21 arm-camera grasp entirely on the Jetson.

This launcher owns the process ordering that was previously done by hand on
Windows and the Jetson:

1. start the real-arm controller and wait until the C3 home pose is confirmed;
2. run one local YOLO inference from the arm camera and send it to localhost;
3. release the YOLO process after the detection is latched, then let PPO grasp;
4. forward Ctrl+C and clean up both children on every exit path.

YOLO deliberately uses ``--once``.  The arm-camera geometry is valid only at
the C3 home pose, and freeing YOLO before policy motion also matters on a Jetson
Nano with limited RAM.
"""
from __future__ import annotations

import argparse
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from typing import IO, Optional


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
BRIDGE = REPO_ROOT / "integration" / "vision_grasp_bridge.py"
CONTROLLER = HERE / "x3plus_real_grasp.py"
PPO_MODEL = HERE / "models" / "candidate_v21_seed816_ckpt550000.zip"
VECNORM = HERE / "models" / "candidate_v21_seed816_ckpt550000_vec.pkl"
YOLO_MODEL = REPO_ROOT / "detection" / "models" / "best.pt"
ARM_CAMERA = ("/dev/v4l/by-id/"
              "usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera-video-index0")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="One-command, Jetson-local v21 vision grasp")
    p.add_argument("--camera", default=ARM_CAMERA,
                   help="stable local arm-camera path (defaults to the non-SN0001 "
                        "Sonix camera, currently /dev/video1)")
    p.add_argument("--port", default="/dev/myserial",
                   help="Rosmaster serial port (default /dev/myserial)")
    p.add_argument("--cam-x", type=float, default=0.2970,
                   help="validated C3 camera X plus grasp compensation")
    p.add_argument("--cam-y", type=float, default=0.0034)
    p.add_argument("--sign-y", type=float, choices=(-1.0, 1.0), default=-1.0)
    p.add_argument("--class-height", type=float, default=0.065,
                   help="sugarbox full height in metres (default 0.065)")
    p.add_argument("--entry-xy-mm", type=float, default=10.0)
    p.add_argument("--s6-stall-steps", type=int, default=2)
    p.add_argument("--pose-tol-deg", type=float, default=3.0,
                   help="maximum C3 arm-pose residual accepted by both startup and "
                        "the detection stamp (default 3.0 deg; controller caps "
                        "startup recovery at 3.0 deg)")
    p.add_argument("--latch-wait", type=float, default=120.0)
    p.add_argument("--stale-timeout", type=float, default=1.0)
    p.add_argument("--max-steps", type=int, default=300)
    p.add_argument("--startup-timeout", type=float, default=180.0,
                   help="seconds allowed for model load and confirmed C3 home")
    p.add_argument("--check", action="store_true",
                   help="check files/imports only; do not open hardware or move")
    return p.parse_args()


def preflight(args: argparse.Namespace) -> bool:
    ok = True
    for label, path in (
            ("controller", CONTROLLER), ("vision bridge", BRIDGE),
            ("PPO model", PPO_MODEL), ("VecNormalize", VECNORM),
            ("YOLO model", YOLO_MODEL)):
        if path.is_file():
            print(f"[check] {label}: {path}")
        else:
            print(f"[FATAL] missing {label}: {path}")
            ok = False

    for module in ("cv2", "ultralytics", "stable_baselines3", "pybullet"):
        if importlib.util.find_spec(module) is None:
            print(f"[FATAL] Python module {module!r} is not installed in {sys.executable}")
            ok = False
        else:
            print(f"[check] import {module}: OK")

    camera = str(args.camera)
    if camera.startswith("/dev/") and not Path(camera).exists():
        print(f"[FATAL] camera device does not exist: {camera}")
        ok = False
    elif camera.startswith("/dev/"):
        print(f"[check] camera: {camera}")

    if not Path(args.port).exists():
        print(f"[FATAL] Rosmaster port does not exist: {args.port}")
        ok = False
    else:
        print(f"[check] Rosmaster port: {args.port}")
    return ok


def pump(proc: subprocess.Popen, log: IO[str], *,
         home_ready: Optional[threading.Event] = None,
         detection_sent: Optional[threading.Event] = None):
    assert proc.stdout is not None
    for line in proc.stdout:
        print(line, end="", flush=True)
        log.write(line)
        log.flush()
        if home_ready is not None and "[Start] home: reached=True" in line:
            home_ready.set()
        if detection_sent is not None and line.startswith("[bridge] sent "):
            detection_sent.set()


def stop_process(proc: Optional[subprocess.Popen], label: str) -> None:
    if proc is None or proc.poll() is not None:
        return
    print(f"[launcher] stopping {label} (SIGINT)")
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3.0)


def main() -> int:
    args = parse_args()
    if not preflight(args):
        return 2
    if args.check:
        print("[check] PASS: files and Python dependencies are present; no hardware opened.")
        return 0

    stamp = time.strftime("%Y%m%d_%H%M%S")
    ctrl_log_path = Path.home() / f"mode_b_integrated_{stamp}_controller.log"
    vision_log_path = Path.home() / f"mode_b_integrated_{stamp}_vision.log"
    print("[launcher] REAL supervised grasp: stay beside the robot, hand on power.")
    print(f"[launcher] controller log: {ctrl_log_path}")
    print(f"[launcher] vision log    : {vision_log_path}")
    print("[launcher] camera is read locally; stop stream_cam.py first if it owns the device.")

    ctrl_cmd = [
        sys.executable, str(CONTROLLER),
        "--real", "--socket", "--latch-obj",
        "--i-confirm-external-frame", "--unlock-candidate-real",
        "--port", str(args.port),
        "--model", str(PPO_MODEL),
        "--vecnorm", str(VECNORM),
        "--contract", "obs_28_incremental",
        "--pose-tol-deg", str(args.pose_tol_deg),
        "--entry-xy-mm", str(args.entry_xy_mm),
        "--s6-stall-grasp-steps", str(args.s6_stall_steps),
        "--latch-wait", str(args.latch_wait),
        "--stale-timeout", str(args.stale_timeout),
        "--max-steps", str(args.max_steps),
    ]
    bridge_cmd = [
        sys.executable, str(BRIDGE),
        "--host", "127.0.0.1", "--port", "5555",
        "--model", str(YOLO_MODEL),
        "--stream", str(args.camera),
        "--cam-x", str(args.cam_x), "--cam-y", str(args.cam_y),
        "--sign-y", str(args.sign_y),
        "--i-accept-predicted-extrinsics",
        "--class-height", f"sugarbox={args.class_height}",
        "--once",
    ]

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    ctrl = None
    bridge = None
    ctrl_thread = None
    bridge_thread = None
    home_ready = threading.Event()
    detection_sent = threading.Event()
    ctrl_log = open(ctrl_log_path, "w", encoding="utf-8")
    vision_log = open(vision_log_path, "w", encoding="utf-8")
    try:
        ctrl = subprocess.Popen(
            ctrl_cmd, cwd=str(HERE), env=env, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, encoding="utf-8",
            errors="replace", bufsize=1)
        ctrl_thread = threading.Thread(
            target=pump, args=(ctrl, ctrl_log),
            kwargs={"home_ready": home_ready}, daemon=True)
        ctrl_thread.start()

        deadline = time.monotonic() + max(1.0, args.startup_timeout)
        while not home_ready.wait(0.1):
            if ctrl.poll() is not None:
                print(f"[FATAL] controller exited before confirming C3 home (exit {ctrl.returncode}).")
                return ctrl.returncode or 2
            if time.monotonic() >= deadline:
                print("[FATAL] timed out waiting for the controller to confirm C3 home.")
                return 2

        print("[launcher] C3 home confirmed; starting one local YOLO detection.")
        bridge = subprocess.Popen(
            bridge_cmd, cwd=str(REPO_ROOT), env=env, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, encoding="utf-8",
            errors="replace", bufsize=1)
        bridge_thread = threading.Thread(
            target=pump, args=(bridge, vision_log),
            kwargs={"detection_sent": detection_sent}, daemon=True)
        bridge_thread.start()

        while ctrl.poll() is None:
            if bridge.poll() is not None:
                # Let the stdout pump consume the final "sent" line before deciding
                # that a clean --once exit produced no detection.
                if bridge_thread is not None:
                    bridge_thread.join(timeout=2.0)
                if bridge.returncode not in (None, 0):
                    print(f"[FATAL] vision bridge exited with {bridge.returncode}; "
                          "stopping arm controller.")
                    stop_process(ctrl, "controller")
                    return bridge.returncode or 2
                if not detection_sent.is_set():
                    print("[FATAL] vision bridge exited without sending a valid "
                          "detection; stopping arm controller.")
                    stop_process(ctrl, "controller")
                    return 2
            time.sleep(0.1)
        return int(ctrl.returncode or 0)
    except KeyboardInterrupt:
        print("\n[launcher] Ctrl+C received; stopping vision and controller.")
        return 130
    finally:
        stop_process(bridge, "vision bridge")
        stop_process(ctrl, "controller")
        if bridge_thread is not None:
            bridge_thread.join(timeout=2.0)
        if ctrl_thread is not None:
            ctrl_thread.join(timeout=2.0)
        vision_log.close()
        ctrl_log.close()


if __name__ == "__main__":
    raise SystemExit(main())
