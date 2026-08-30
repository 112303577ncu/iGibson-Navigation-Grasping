#!/usr/bin/env python3
"""Run the v23 arm-camera grasp entirely on the Jetson.

Copied from grasp/v21/jetson_one_command_grasp.py and repointed at the v23
stack. The v23 grasp home is E1 (API 90, 74.2, 8.6, 8.6, 90, 30), NOT C3, so
the C3 homography does not apply here: the camera sits 1.6 cm higher and its
optical axis has crossed vertical. A fresh --calibrate run is mandatory, and
the default calibration path is deliberately a different filename so a C3 file
cannot be picked up by accident.

This launcher owns the process ordering that was previously done by hand on
Windows and the Jetson:

1. start the real-arm controller and wait until the E1 home pose is confirmed;
2. run one local YOLO inference from the arm camera and send it to localhost;
3. release the YOLO process after the detection is latched, then let PPO grasp;
4. forward Ctrl+C and clean up both children on every exit path.

YOLO deliberately uses ``--once``.  The arm-camera geometry is valid only at
the E1 home pose, and freeing YOLO before policy motion also matters on a Jetson
Nano with limited RAM.

At E1 the bridge maps pixels to base coordinates through a MEASURED homography
and rejects everything else -- the old nav-home H/theta/cam offsets included, and
``--i-accept-predicted-extrinsics`` cannot waive it.  So this launcher passes
``--homography`` and validates that file with the bridge's own gate BEFORE the
serial port is opened: the alternative is what used to happen, where the arm
powered up, moved to E1 home, and only then did the bridge exit and take the run
down with it.

``--calibrate`` is the other half of the same problem.  The calibration has to be
measured at E1 home, which means something has to hold the arm there while the
pixels are collected; that something is this launcher.  See
docs/calibration/CALIBRATION_PLAN.md "grasp-home homography".

This file is deliberately kept parseable by the Jetson's SYSTEM python3 (3.6.9):
no ``from __future__ import annotations``, no walrus, no PEP 585 generics.  It is
the command people type by hand, so running it under the wrong interpreter has to
report itself rather than die with a SyntaxError in the import block.  Everything
it launches runs under the 3.8 venv and may use all of them.
"""

import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from typing import IO, Optional


if sys.version_info < (3, 8):
    sys.exit(
        "[FATAL] 需要 Python >= 3.8，目前是 {}（Jetson 的系統 python3 是 3.6.9）。\n"
        "        先啟用虛擬環境再重跑：source ~/grasp_venv/bin/activate".format(
            sys.version.split()[0]))
if "VIRTUAL_ENV" not in os.environ:
    # Warning, not an error: what actually matters is whether the deps import,
    # and preflight() checks exactly that a few lines below.
    print("[WARN] 沒有偵測到 VIRTUAL_ENV。若下面的 import 檢查失敗，"
          "先執行 source ~/grasp_venv/bin/activate")


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
BRIDGE = REPO_ROOT / "integration" / "vision_grasp_bridge.py"
SCANNER = HERE / "three_pose_scan.py"
SCAN_CONFIG = HERE / "three_pose_scan.json"
CONTROLLER = HERE / "x3plus_real_grasp.py"
PPO_MODEL = HERE / "models" / "candidate_v23_seed23401_ckpt250000.zip"
VECNORM = HERE / "models" / "candidate_v23_seed23401_ckpt250000_vec.pkl"
YOLO_MODEL = REPO_ROOT / "detection" / "models" / "best.pt"
INTEGRATION = REPO_ROOT / "integration"
# A DIFFERENT filename from v21's grasp_home_homography.json, on purpose. Both
# files pass the same >=6-point / <2 cm gate, both are valid JSON, and neither
# records which arm pose it was measured at -- so the only thing standing between
# a C3 calibration and a v23 run is that they are not the same path.
HOMOGRAPHY = INTEGRATION / "grasp_home_homography_e1.json"

# The registry row the bridge stamps detections with. The grasp side compares
# this stamp against its encoders (--pose-tol-deg) and refuses a mismatch, so
# leaving it at the C3 default would make every single detection bounce with a
# 7.1 deg residual on S2.
CAMERA_POSE_NAME = "v23_e1_grasp_home"

# The band the v23 policy was trained on -- narrower than v21's while the E1 FOV
# is 44% larger, so the camera genuinely sees ground the policy has never been
# evaluated on. Passed to the bridge so a detection out there is dropped in front
# of the operator, not after the arm has moved.
POLICY_X_RANGE = ("0.205", "0.280")
POLICY_Y_RANGE = ("-0.070", "0.065")
ARM_CAMERA = ("/dev/v4l/by-id/"
              "usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera-video-index0")

# The runtime gate the bridge itself applies (vision_grasp_bridge.resolve_pose).
# Duplicated here on purpose: checking it earlier is the whole point, and a
# looser check here would pass a file the bridge then rejects at E1.
HOMOGRAPHY_MIN_POINTS = 6
HOMOGRAPHY_MAX_ERROR_M = 0.02
# Keep the scanner out of the launcher's foreground process group. Otherwise one
# terminal Ctrl+C reaches both processes and the launcher immediately sends the
# scanner a second SIGINT while its finally block is trying to return the arm to
# E1. The scanner gets one deliberate SIGINT below and a long guarded-return
# window before termination is considered.
SCAN_START_NEW_SESSION = os.name == "posix"

# How long the scanner may take to finish its guarded, encoder-confirmed return
# to E1 after one SIGINT. This used to be a flat 45 s, which is a number with no
# derivation behind it: exceed it and stop_process escalates to terminate and
# then kill, aborting the return mid-move and leaving the arm at an arbitrary
# pose. The next run re-homes first so it recovers, but the interrupt stops
# being clean at exactly the moment cleanliness matters.
#
# So it is computed from what the return actually costs.
# move_guarded_and_verified iterates up to max_iters, and each iteration is one
# servo write, a settle sleep, and one guarded read -- the read being the slow
# and variable part on a half-duplex bus that retries.
SCAN_MOVE_MAX_ITERS = 40            # move_guarded_and_verified default
SCAN_MOVE_SETTLE_S = 0.35           # pose_explorer.Mover.move_to
SCAN_MOVE_IO_BUDGET_S = 0.35        # one write + one read, worst case, per iter
SCAN_TEARDOWN_BUDGET_S = 6.0        # camera release, PyBullet close, flush
SCAN_RETURN_BUDGET_SEC = SCAN_MOVE_MAX_ITERS * (
    SCAN_MOVE_SETTLE_S + SCAN_MOVE_IO_BUDGET_S)


def scan_interrupt_grace_sec(config_path=None) -> float:
    """Seconds to allow the scanner after its single SIGINT.

    The confirming reference detection runs inside that same finally block, and
    its cost is per_pose_timeout_sec -- READ from the scan config rather than
    assumed, so raising that timeout cannot silently outgrow the grace and start
    killing returns again. A missing or unreadable config falls back to the
    return budget alone, which is the conservative direction: it only shortens
    the window, and the caller is about to fail on that config anyway.
    """
    confirm = 0.0
    if config_path:
        try:
            with open(str(config_path), "r", encoding="utf-8") as handle:
                raw = json.load(handle)
            confirm = float(raw.get("per_pose_timeout_sec", 0.0) or 0.0)
            if not (0.0 <= confirm <= 120.0):
                confirm = 0.0
        except Exception:
            confirm = 0.0
    return SCAN_RETURN_BUDGET_SEC + confirm + SCAN_TEARDOWN_BUDGET_S


# Fallback for callers with no config in hand (and the value the tests pin).
SCAN_INTERRUPT_GRACE_SEC = scan_interrupt_grace_sec()


def parse_args_from(argv) -> argparse.Namespace:
    """Parse an explicit argv. Separate from parse_args() so the command builders
    below can be exercised with the real defaults, without a hardware run."""
    p = argparse.ArgumentParser(
        description="One-command, Jetson-local v23 (E1) vision grasp")
    p.add_argument("--camera", default=ARM_CAMERA,
                   help="stable local arm-camera path (defaults to the non-SN0001 "
                        "Sonix camera, currently /dev/video1)")
    p.add_argument("--port", default="/dev/myserial",
                   help="Rosmaster serial port (default /dev/myserial)")
    # --cam-x/--cam-y/--sign-y are gone, not defaulted: at E1 the bridge maps
    # through the homography alone, so forwarding them would print numbers that
    # look like they steer the arm while changing nothing.
    p.add_argument("--homography", default=str(HOMOGRAPHY),
                   help="measured grasp-home pixel->base calibration JSON "
                        "(default integration/grasp_home_homography_e1.json). "
                        "Produce it with --calibrate; the bridge refuses "
                        "grasp-home detections without one. A C3 file is NOT "
                        "interchangeable -- E1 moved the camera.")
    p.add_argument("--calibrate", action="store_true",
                   help="hold the arm at E1 home and print median undistorted "
                        "pixels instead of grasping. Nothing is sent to the arm "
                        "and no grasp is attempted.")
    p.add_argument("--calibration-samples", type=int, default=20,
                   help="valid frames per printed median in --calibrate (default 20)")
    p.add_argument("--show", action="store_true",
                   help="show the annotated camera window (needs a display; not "
                        "over a plain SSH session)")
    p.add_argument("--allow-top-clipped", action="store_true",
                   help="accept a sugarbox bbox that touches the TOP image edge only. "
                        "The measured E1 homography still requires its bottom-centre "
                        "inside the calibration hull; side edges are width-only and "
                        "jaw-bounded. This never allows left/right/bottom clipping.")
    p.add_argument("--accept-single-rotated-view", action="store_true",
                   help="let the scan release a target that only ONE rotated "
                        "(LEFT/RIGHT) view saw and that E1 could not confirm "
                        "afterwards. Off by default: with one view nothing "
                        "tests the S1 rotation at runtime, so a wrong pivot or "
                        "yaw sign would hand the policy a confident wrong "
                        "coordinate.")
    p.add_argument("--three-pose-scan", action="store_true",
                   help="before loading PPO, search from exactly three calibrated "
                        "arm poses, return to E1, release camera/serial ownership, "
                        "then grasp the fused fixed base-frame target")
    p.add_argument("--scan-config", default=str(SCAN_CONFIG),
                   help="three-pose scan JSON (default grasp/v23/three_pose_scan.json)")
    p.add_argument("--scan-calibrate-pose", default=None, metavar="NAME",
                   help="move to one configured scan pose and print calibration "
                        "pixels; Ctrl+C returns to E1 and no grasp is attempted")
    p.add_argument("--class-height", type=float, default=0.065,
                   help="sugarbox full height in metres (default 0.065)")
    p.add_argument("--entry-xy-mm", type=float, default=10.0)
    p.add_argument("--floor-finger-error-mm", type=float, default=0.0,
                   help="measured URDF finger error, forwarded to the controller. "
                        "It corrects BOTH the floor guard and the stage-1 "
                        "pads_ready gate. At 0 (default) pads_ready fires while "
                        "the real pads are still above the object and the jaw "
                        "shuts on the approach; E1 measured 11.9mm open / 15.4mm "
                        "closed on 2026-08-29, so 15 is the value that pass "
                        "measured. Confirm with pose_check.py --real before "
                        "raising it further.")
    p.add_argument("--s6-stall-steps", type=int, default=2)
    p.add_argument("--pose-tol-deg", type=float, default=3.0,
                   help="maximum E1 arm-pose residual accepted by both startup and "
                        "the detection stamp (default 3.0 deg; controller caps "
                        "startup recovery at 3.0 deg)")
    p.add_argument("--latch-wait", type=float, default=120.0)
    p.add_argument("--stale-timeout", type=float, default=1.0)
    p.add_argument("--max-steps", type=int, default=300)
    p.add_argument("--startup-timeout", type=float, default=180.0,
                   help="seconds allowed for model load and confirmed E1 home")
    p.add_argument("--check", action="store_true",
                   help="check files/imports only; do not open hardware or move")
    return p.parse_args(argv)


def parse_args() -> argparse.Namespace:
    return parse_args_from(None)


def check_homography(path_str: str) -> bool:
    """Validate the calibration with the SAME gate the bridge applies at E1.

    Runs before any hardware is opened. A missing or weak calibration is the one
    failure that used to cost a full startup: model load, serial port, arm to E1
    home, and only then the bridge exiting mid-run.
    """
    path = Path(path_str)
    if not path.is_file():
        print(f"[FATAL] missing grasp-home homography: {path}")
        print("        E1 偵測一定要有實測校正檔，nav-home 的 H/theta/offset 不能替代。")
        print("        產生方式：本檔加 --calibrate（詳見下方說明與")
        print("        docs/calibration/CALIBRATION_PLAN.md「grasp-home homography」）")
        return False

    sys.path.insert(0, str(INTEGRATION))
    try:
        from grasp_home_homography import (HomographyCalibrationError,
                                           load_calibration)
    except ImportError as exc:
        print(f"[FATAL] cannot import grasp_home_homography: {exc}")
        return False

    try:
        document = load_calibration(str(path), min_points=HOMOGRAPHY_MIN_POINTS,
                                    max_error_m=HOMOGRAPHY_MAX_ERROR_M)
    except HomographyCalibrationError as exc:
        print(f"[FATAL] grasp-home homography rejected: {exc}")
        print(f"        校正檔要過 runtime 的同一道閘：>= {HOMOGRAPHY_MIN_POINTS} 個不共線的點、"
              f"最大誤差 < {HOMOGRAPHY_MAX_ERROR_M*100:.0f} cm、內容與擬合結果一致。")
        print("        用 --calibrate 重新量測，再用 "
              "integration/grasp_home_homography.py 重解。")
        return False

    fit = document["fit"]
    print(f"[check] homography: {path} ({fit['point_count']} points, "
          f"RMSE {fit['rmse_m']*100:.2f} cm, max {fit['max_error_m']*100:.2f} cm)")
    return True


def check_scan_config(args: argparse.Namespace) -> bool:
    """Validate all scan poses before either camera or serial port is opened."""
    try:
        sys.path.insert(0, str(HERE))
        from three_pose_scan import ScanConfigError, load_scan_config
        config = load_scan_config(
            args.scan_config,
            require_homographies=args.scan_calibrate_pose is None,
            calibration_pose=args.scan_calibrate_pose)
    except (ImportError, ScanConfigError) as exc:
        print(f"[FATAL] three-pose scan config rejected: {exc}")
        return False
    print(f"[check] scan config: {config['path']} "
          f"({', '.join(pose['name'] for pose in config['poses'])} → "
          f"{config['return_pose']['name']})")
    if args.scan_calibrate_pose is not None:
        pose = next(p for p in config["poses"]
                    if p["name"] == args.scan_calibrate_pose)
        print(f"[check] scan calibration pose {pose['name']}: {list(pose['arm_deg'])}")
        return True
    return check_homography(str(config["mapping"]["homography"]))


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

    if args.three_pose_scan or args.scan_calibrate_pose is not None:
        for label, path in (("three-pose scanner", SCANNER),
                            ("scan config", Path(args.scan_config))):
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
        # This launcher opens the camera itself; there is no separate streaming
        # terminal to start. What there can be is a leftover one still holding
        # the device, and V4L2 hands out no second reader.
        holders = camera_holders(camera)
        if holders:
            print(f"[FATAL] camera {camera} is already open in another process:")
            for pid, cmdline in holders:
                print(f"          pid {pid}: {cmdline}")
            print("        關掉那個終端機／程序後重跑（本檔會自己讀相機，不需要另外開串流）。")
            ok = False
        else:
            print(f"[check] camera: {camera} (free; this launcher reads it directly)")

    if not Path(args.port).exists():
        print(f"[FATAL] Rosmaster port does not exist: {args.port}")
        ok = False
    else:
        print(f"[check] Rosmaster port: {args.port}")

    # Skipped under --calibrate for the obvious reason: that mode exists to
    # create this file, so requiring it first would be a closed loop.
    if args.three_pose_scan or args.scan_calibrate_pose is not None:
        if not check_scan_config(args):
            ok = False
    elif not args.calibrate and not check_homography(args.homography):
        ok = False
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


def parse_scan_result_line(line: str):
    """Parse the scanner's sole handoff record; ordinary log lines return None."""
    prefix = "[scan][result] "
    if not line.startswith(prefix):
        return None
    try:
        payload = json.loads(line[len(prefix):])
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid scan result JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("scan result must be a JSON object")
    for key in ("x", "y", "z", "height"):
        value = payload.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"scan result {key!r} must be numeric")
        if not math.isfinite(float(value)):
            raise ValueError(f"scan result {key!r} must be finite")
    if float(payload["height"]) <= 0.0:
        raise ValueError("scan result height must be positive")
    count = payload.get("pose_count")
    poses = payload.get("poses")
    if not isinstance(count, int) or not 1 <= count <= 3:
        raise ValueError("scan result pose_count must be in [1, 3]")
    if not isinstance(poses, list) or len(poses) != count:
        raise ValueError("scan result poses must match pose_count")
    return payload


# Processes that have already been sent their one SIGINT. Keyed by pid, because
# the guarantee has to survive being called twice with the same handle.
_SIGNALLED_PIDS = set()


def stop_process(proc: Optional[subprocess.Popen], label: str,
                 grace_seconds: float = 8.0) -> None:
    """Send ONE SIGINT, then escalate only if the grace expires.

    The once-only part is load-bearing for the scanner. Its SIGINT handler runs
    a guarded, encoder-confirmed return to E1 inside a finally block; a second
    SIGINT arriving mid-return raises KeyboardInterrupt inside that handler and
    abandons the arm wherever it happens to be. Relying on poll() to make the
    second call a no-op works only when the first call has already reaped the
    child, which is a timing property, not a guarantee -- so the pid is recorded
    instead.
    """
    if proc is None or proc.poll() is not None:
        return
    if proc.pid in _SIGNALLED_PIDS:
        # Already asked once, and it has not exited yet. Waiting is the correct
        # action here; signalling again is the thing being prevented.
        try:
            proc.wait(timeout=grace_seconds)
        except subprocess.TimeoutExpired:
            print(f"[launcher] {label} did not exit within its grace window; "
                  f"terminating.")
            proc.terminate()
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=3.0)
        return
    _SIGNALLED_PIDS.add(proc.pid)
    print(f"[launcher] stopping {label} (SIGINT)")
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=grace_seconds)
    except subprocess.TimeoutExpired:
        print(f"[launcher] {label} did not finish its guarded return within "
              f"{grace_seconds:.0f}s; terminating. The arm may be off E1 -- the "
              f"next run re-homes before it does anything else.")
        proc.terminate()
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3.0)


def build_ctrl_cmd(args: argparse.Namespace, fixed_target=None):
    """Arm-controller command line."""
    # The controller holds the arm at E1 by waiting for a detection that
    # --calibrate never sends, so that wait has to outlast a measuring session
    # rather than a one-shot detection.
    latch_wait = max(args.latch_wait, 1800.0) if args.calibrate else args.latch_wait
    cmd = [
        sys.executable, str(CONTROLLER),
        "--real", "--unlock-candidate-real",
        "--port", str(args.port),
        "--model", str(PPO_MODEL),
        "--vecnorm", str(VECNORM),
        "--contract", "obs_28_incremental",
        "--pose-tol-deg", str(args.pose_tol_deg),
        "--entry-xy-mm", str(args.entry_xy_mm),
        "--floor-finger-error-mm", str(args.floor_finger_error_mm),
        "--s6-stall-grasp-steps", str(args.s6_stall_steps),
        "--latch-wait", str(latch_wait),
        "--stale-timeout", str(args.stale_timeout),
        "--max-steps", str(args.max_steps),
    ]
    if fixed_target is None:
        cmd[3:3] = ["--socket", "--latch-obj",
                    "--i-confirm-external-frame"]
    else:
        cmd += [
            "--obj-x", str(fixed_target["x"]),
            "--obj-y", str(fixed_target["y"]),
            "--obj-z", str(fixed_target["z"]),
            "--object-height", str(fixed_target["height"]),
        ]
    return cmd


def build_scan_cmd(args: argparse.Namespace):
    """Three-pose scanner command; it exits before the PPO controller starts."""
    cmd = [
        sys.executable, str(SCANNER),
        "--config", str(args.scan_config),
        "--camera", str(args.camera),
        "--port", str(args.port),
        "--model", str(YOLO_MODEL),
        "--class-height", str(args.class_height),
        "--pose-tol-deg", str(min(3.0, args.pose_tol_deg)),
        "--policy-x-range", POLICY_X_RANGE[0], POLICY_X_RANGE[1],
        "--policy-y-range", POLICY_Y_RANGE[0], POLICY_Y_RANGE[1],
        "--calibration-samples", str(args.calibration_samples),
        "--i-am-beside-the-robot",
    ]
    if args.allow_top_clipped:
        cmd.append("--allow-top-clipped")
    if args.scan_calibrate_pose is not None:
        cmd += ["--calibrate-pose", str(args.scan_calibrate_pose)]
    if getattr(args, "accept_single_rotated_view", False):
        cmd.append("--accept-single-rotated-view")
    return cmd


def build_bridge_cmd(args: argparse.Namespace):
    """Vision-bridge command line.

    Kept separate from main() so the flag wiring is testable without hardware.
    The bug this guards against already happened once: the launcher went on
    passing nav-home extrinsics after the bridge started requiring a measured
    homography at E1, and the mismatch only surfaced on the robot, after the arm
    had powered up and moved.
    """
    cmd = [
        sys.executable, str(BRIDGE),
        "--host", "127.0.0.1", "--port", "5555",
        "--model", str(YOLO_MODEL),
        "--stream", str(args.camera),
        "--class-height", f"sugarbox={args.class_height}",
        # Not optional. The bridge's grasp-home default is still C3, so without
        # this every detection is stamped with the wrong pose and the controller
        # refuses it -- at E1 the S2 residual alone is 7.1 deg.
        "--pose", CAMERA_POSE_NAME,
        "--policy-x-range", POLICY_X_RANGE[0], POLICY_X_RANGE[1],
        "--policy-y-range", POLICY_Y_RANGE[0], POLICY_Y_RANGE[1],
    ]
    if args.show:
        cmd.append("--show")
    if args.calibrate:
        # --calibration-only never opens the TCP socket, so the controller sits
        # at E1 untouched for the whole session.
        cmd += ["--dry-run", "--calibration-only",
                "--calibration-samples", str(args.calibration_samples)]
    else:
        cmd += ["--homography", str(args.homography), "--once"]
        if args.allow_top_clipped:
            cmd.append("--allow-top-clipped-grasp-home")
    return cmd


def print_calibration_next_steps(args: argparse.Namespace) -> None:
    print("")
    print("[launcher] 接下來：把每組 (u, v) 與實測 base (x, y) 寫成 JSON，例如")
    print('             [{"u": 318.4, "v": 402.1, "x": 0.247, "y": 0.018}, ...]')
    print("           留至少 2 組不要拿去擬合，之後當驗證點。然後：")
    print(f"             python3 {INTEGRATION / 'grasp_home_homography.py'} \\")
    print("               --points-json grasp_home_points.json \\")
    print(f"               --output {args.homography} --max-rmse-cm 1")
    print("           產生後直接跑 `python3 jetson_one_command_grasp.py --check` 驗收。")


def camera_holders(device: str):
    """(pid, cmdline) of other processes holding the camera, best effort.

    Without this, a leftover stream_cam.py in another terminal surfaces as a
    generic OpenCV open failure, which reads like a broken camera rather than
    "something else already has it".
    """
    proc_root = Path("/proc")
    if not proc_root.is_dir():
        return []
    target = os.path.realpath(device)
    mine = os.getpid()
    holders = []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit() or int(entry.name) == mine:
            continue
        try:
            fds = list((entry / "fd").iterdir())
        except OSError:
            continue          # not ours, or the process just exited
        for fd in fds:
            try:
                if os.path.realpath(str(fd)) != target:
                    continue
                raw = (entry / "cmdline").read_bytes()
            except OSError:
                continue
            cmdline = raw.decode("utf-8", "replace").replace("\0", " ").strip()
            holders.append((entry.name, cmdline or "?"))
            break
    return holders


def run_scanner(args: argparse.Namespace, env, log_path: Path):
    """Run the exclusive camera/serial search phase and return its fixed target."""
    cmd = build_scan_cmd(args)
    proc = None
    result = None
    with open(log_path, "w", encoding="utf-8") as log:
        try:
            proc = subprocess.Popen(
                cmd, cwd=str(HERE), env=env, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                errors="replace", bufsize=1,
                start_new_session=SCAN_START_NEW_SESSION)
            assert proc.stdout is not None
            for line in proc.stdout:
                print(line, end="", flush=True)
                log.write(line)
                log.flush()
                parsed = parse_scan_result_line(line)
                if parsed is not None:
                    if result is not None:
                        raise ValueError("scanner emitted more than one result")
                    result = parsed
            code = proc.wait()
        except KeyboardInterrupt:
            # The child is in its own POSIX session, so it did not receive the
            # terminal's Ctrl+C. It gets exactly one SIGINT -- from the finally
            # below, which runs on this path too. Signalling here as well was
            # the bug: two calls, and the second one only stayed harmless while
            # the first happened to have reaped the child already.
            raise
        finally:
            stop_process(proc, "three-pose scanner",
                         grace_seconds=scan_interrupt_grace_sec(args.scan_config))
    if code != 0:
        raise RuntimeError(f"three-pose scanner exited with {code}")
    if args.scan_calibrate_pose is not None:
        return None
    if result is None:
        raise RuntimeError("three-pose scanner exited without a verified E1 result")
    return result


def main() -> int:
    args = parse_args()
    if args.calibrate and (args.three_pose_scan or args.scan_calibrate_pose is not None):
        print("[FATAL] --calibrate is the legacy E1-only calibration path; use "
              "--scan-calibrate-pose NAME by itself for a scan pose.")
        return 2
    if args.three_pose_scan and args.scan_calibrate_pose is not None:
        print("[FATAL] --three-pose-scan and --scan-calibrate-pose are mutually exclusive.")
        return 2
    if args.scan_calibrate_pose is not None and args.allow_top_clipped:
        print("[FATAL] calibration never accepts clipped bboxes; remove "
              "--allow-top-clipped.")
        return 2
    if not preflight(args):
        return 2
    if args.check:
        print("[check] PASS: files and Python dependencies are present; no hardware opened.")
        return 0
    if args.calibrate and args.allow_top_clipped:
        print("[FATAL] --allow-top-clipped is for runtime grasp only; calibration "
              "must keep rejecting every clipped bbox.")
        return 2

    stamp = time.strftime("%Y%m%d_%H%M%S")
    ctrl_log_path = Path.home() / f"mode_b_integrated_{stamp}_controller.log"
    vision_log_path = Path.home() / f"mode_b_integrated_{stamp}_vision.log"
    scan_log_path = Path.home() / f"mode_b_integrated_{stamp}_three_pose_scan.log"
    if args.calibrate:
        print("[launcher] CALIBRATE: 手臂只會移到 E1 home 並停在那裡，不會夾任何東西。")
    elif args.scan_calibrate_pose is not None:
        print(f"[launcher] SCAN CALIBRATE {args.scan_calibrate_pose}: "
              "只量這個姿態；Ctrl+C 後先回 E1，不會啟動 PPO。")
    else:
        print("[launcher] REAL supervised grasp: stay beside the robot, hand on power.")
        if args.three_pose_scan:
            print("[launcher] THREE-POSE: scanner exclusively owns camera+serial, "
                  "returns to E1, exits, then PPO starts with a fixed target.")
        if args.allow_top_clipped:
            print("[launcher] TOP-CLIP OPT-IN: only the top edge may be clipped; "
                  "bottom/left/right and target-centre homography-hull gates remain active.")
    print(f"[launcher] controller log: {ctrl_log_path}")
    print(f"[launcher] vision log    : {vision_log_path}")
    if args.three_pose_scan or args.scan_calibrate_pose is not None:
        print(f"[launcher] scan log      : {scan_log_path}")
    print("[launcher] 相機由本程序自己開，不需要另外開串流終端機。")

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    fixed_target = None
    if args.three_pose_scan or args.scan_calibrate_pose is not None:
        try:
            fixed_target = run_scanner(args, env, scan_log_path)
        except KeyboardInterrupt:
            print("\n[launcher] Ctrl+C received; scanner was asked to return E1.")
            return 130
        except (RuntimeError, ValueError, OSError) as exc:
            print(f"[FATAL] scan phase failed: {exc}")
            return 2
        if args.scan_calibrate_pose is not None:
            return 0
        print(f"[launcher] scan target fixed at "
              f"({fixed_target['x']:+.4f}, {fixed_target['y']:+.4f}, "
              f"{fixed_target['z']:+.4f}) from "
              f"{fixed_target['pose_count']} pose(s). Starting PPO only now.")

    ctrl_cmd = build_ctrl_cmd(args, fixed_target=fixed_target)
    bridge_cmd = None if fixed_target is not None else build_bridge_cmd(args)

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

        if fixed_target is not None:
            # The scanner already returned to and verified E1, then exited. The
            # controller independently re-confirms E1 before using the fixed
            # base-frame target. No camera/socket process exists in this phase.
            while ctrl.poll() is None:
                time.sleep(0.1)
            return int(ctrl.returncode or 0)

        deadline = time.monotonic() + max(1.0, args.startup_timeout)
        while not home_ready.wait(0.1):
            if ctrl.poll() is not None:
                print(f"[FATAL] controller exited before confirming E1 home (exit {ctrl.returncode}).")
                return ctrl.returncode or 2
            if time.monotonic() >= deadline:
                print("[FATAL] timed out waiting for the controller to confirm E1 home.")
                return 2

        if args.calibrate:
            print("[launcher] E1 home confirmed; starting the calibration camera pass.")
            print("[launcher] 每擺一個位置，等下面印出一行 [bridge][calibration] {...}，")
            print("[launcher] 把裡面的 u,v 和你量到的 base 座標 x,y 記成一組。")
            print(f"[launcher] 至少 {HOMOGRAPHY_MIN_POINTS} 組不共線的點，另外多留 2 組"
                  "當驗證點。量完按 Ctrl+C。")
        else:
            print("[launcher] E1 home confirmed; starting one local YOLO detection.")
        assert bridge_cmd is not None
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
                if args.calibrate:
                    # Nothing is ever "sent" in this mode, so the detection check
                    # below would read a finished calibration as a failure.
                    print("[launcher] calibration pass finished; stopping the arm.")
                    print_calibration_next_steps(args)
                    stop_process(ctrl, "controller")
                    return 0
                if not detection_sent.is_set():
                    print("[FATAL] vision bridge exited without sending a valid "
                          "detection; stopping arm controller.")
                    stop_process(ctrl, "controller")
                    return 2
            time.sleep(0.1)
        return int(ctrl.returncode or 0)
    except KeyboardInterrupt:
        print("\n[launcher] Ctrl+C received; stopping vision and controller.")
        if args.calibrate:
            # The expected way to end a measuring session, not an error.
            print_calibration_next_steps(args)
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
