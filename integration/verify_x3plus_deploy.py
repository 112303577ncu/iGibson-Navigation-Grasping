#!/usr/bin/env python3
"""Staged deployment verifier for the X3Plus real-robot grasp pipeline.

This helper implements the real-robot verification plan without changing the
robot control logic. By default it only runs safe preflight checks plus the
pure-logic self-test. Camera dry-run and real hardware execution require
explicit flags.

Typical Jetson usage:

    python3 integration/verify_x3plus_deploy.py
    python3 integration/verify_x3plus_deploy.py --run-dry
    python3 integration/verify_x3plus_deploy.py --run-real \
      --i-understand-real-motion --i-confirm-camera-frame \
      --cam-x 0.1639 --cam-y 0.0331 --sign-y -1
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import inspect
import json
import math
import os
import platform
import socket
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence


ROOT = Path(__file__).resolve().parent.parent
GRASP_DIR = ROOT / "grasp"
# The pipelines import the grasp module from grasp/v21 (see
# vision_grasp_pipeline._load_grasp_module), so this verifier checks the v21 pair.
# Verifying the v17 pair here while modes A and C run v21 would report a healthy
# deployment for weights nothing loads.
V21_DIR = GRASP_DIR / "v21"
PIPELINE = ROOT / "integration" / "vision_grasp_pipeline.py"
MODEL = ROOT / "detection" / "models" / "best.pt"
PPO_MODEL = V21_DIR / "models" / "candidate_v21_seed816_ckpt550000.zip"
VECNORM = V21_DIR / "models" / "candidate_v21_seed816_ckpt550000_vec.pkl"
GRASP_SCRIPT = V21_DIR / "x3plus_real_grasp.py"
MANIFEST = V21_DIR / "manifest.json"
URDF = GRASP_DIR / "x3plus" / "yahboomcar.urdf"


@dataclass
class Check:
    name: str
    ok: bool
    detail: str
    fatal_for_real: bool = False

    @property
    def status(self) -> str:
        return "OK" if self.ok else "WARN"


def _path_exists(path: Path, fatal_for_real: bool = True) -> Check:
    return Check(
        name=str(path.relative_to(ROOT)),
        ok=path.exists(),
        detail="found" if path.exists() else "missing",
        fatal_for_real=fatal_for_real,
    )


def _module_available(name: str, extra_path: Optional[Path] = None) -> bool:
    old_path = list(sys.path)
    try:
        if extra_path is not None:
            sys.path.insert(0, str(extra_path))
        return importlib.util.find_spec(name) is not None
    finally:
        sys.path[:] = old_path


def _pipeline_targets_v21() -> Check:
    """The pipelines' grasp module must resolve to grasp/v21, not grasp/.

    Both stacks expose DeployConfig/GraspController with identical shapes, so a
    stale path does not raise anywhere — it just runs v17's absolute-action policy
    with v21 expectations. Resolve the path the way the pipeline does and look.
    """
    try:
        sys.path.insert(0, str(ROOT / "integration"))
        import vision_grasp_pipeline as vgp  # noqa: E402
        src = inspect.getsource(vgp._load_grasp_module)
    except Exception as e:
        return Check("pipeline → grasp/v21", False,
                     f"could not inspect _load_grasp_module: {e}", fatal_for_real=True)
    finally:
        sys.path[:] = [p for p in sys.path if p != str(ROOT / "integration")]
    ok = '"v21"' in src or "'v21'" in src
    return Check(
        "pipeline → grasp/v21", ok,
        "modes A and C import the v21 grasp module" if ok
        else "_load_grasp_module does NOT point at grasp/v21 — modes A/C would silently "
             "run the v17 absolute-action policy",
        fatal_for_real=True,
    )


def _manifest_matches_weights() -> Check:
    """The .zip on disk must be the one the manifest signed.

    scp truncates silently often enough that this is worth a hash, not an exists().
    """
    if not (MANIFEST.exists() and PPO_MODEL.exists()):
        return Check("manifest sha256", False, "manifest or weights missing",
                     fatal_for_real=True)
    try:
        man = json.loads(MANIFEST.read_text(encoding="utf-8"))
        expected = json.dumps(man)  # search the whole document; layout has moved before
        actual = hashlib.sha256(PPO_MODEL.read_bytes()).hexdigest()
    except Exception as e:
        return Check("manifest sha256", False, f"unreadable: {e}", fatal_for_real=True)
    ok = actual in expected
    return Check(
        "manifest sha256", ok,
        f"{actual[:16]}… matches manifest" if ok
        else f"{actual[:16]}… NOT in manifest — the weights on disk are not the ones "
             f"that were validated; re-copy models/",
        fatal_for_real=True,
    )


def _port_open(host: str, port: int, timeout: float = 0.2) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _serial_exists(port: str) -> bool:
    if platform.system().lower() == "windows":
        return True
    return Path(port).exists()


def run_command(cmd: Sequence[str], *, timeout: Optional[int] = None) -> int:
    print("\n$ " + " ".join(cmd))
    sys.stdout.flush()
    try:
        completed = subprocess.run(cmd, cwd=str(ROOT), timeout=timeout)
        return int(completed.returncode)
    except subprocess.TimeoutExpired:
        print(f"[WARN] command timed out after {timeout}s")
        return 124


def preflight(port: str) -> List[Check]:
    checks: List[Check] = []
    checks.append(Check("python", True, sys.version.split()[0]))
    checks.append(Check("platform", True, platform.platform()))

    venv = os.environ.get("VIRTUAL_ENV")
    checks.append(Check(
        "virtualenv",
        bool(venv),
        venv or "not active; activate ~/grasp_venv on Jetson",
        fatal_for_real=False,
    ))

    checks.extend([
        _path_exists(PIPELINE),
        _path_exists(MODEL),
        _path_exists(GRASP_SCRIPT),
        _path_exists(MANIFEST),
        _path_exists(PPO_MODEL),
        _path_exists(VECNORM),
        _path_exists(URDF),
    ])
    checks.append(_pipeline_targets_v21())
    checks.append(_manifest_matches_weights())

    checks.append(Check(
        "Rosmaster_Lib import",
        _module_available("Rosmaster_Lib", V21_DIR) or _module_available("Rosmaster_Lib", GRASP_DIR),
        "available from grasp/v21/, grasp/ or environment"
        if _module_available("Rosmaster_Lib", V21_DIR) or _module_available("Rosmaster_Lib", GRASP_DIR)
        else "not importable; copy Rosmaster_Lib beside grasp/v21/x3plus_real_grasp.py on Jetson",
        fatal_for_real=True,
    ))

    checks.append(Check(
        "ultralytics import",
        _module_available("ultralytics"),
        "available" if _module_available("ultralytics") else "missing; install detection requirements",
        fatal_for_real=True,
    ))
    checks.append(Check(
        "cv2 import",
        _module_available("cv2"),
        "available" if _module_available("cv2") else "missing; install opencv-python",
        fatal_for_real=True,
    ))

    checks.append(Check(
        f"serial port {port}",
        _serial_exists(port),
        "exists or unchecked on Windows" if _serial_exists(port) else "missing on this machine",
        fatal_for_real=True,
    ))

    motor_server_open = _port_open("127.0.0.1", 7000)
    checks.append(Check(
        "port 7000 motor server",
        not motor_server_open,
        "not listening" if not motor_server_open else "listening; stop it before --real",
        fatal_for_real=True,
    ))

    return checks


def print_checks(checks: Iterable[Check]) -> None:
    print("\n== Preflight checks ==")
    for item in checks:
        print(f"[{item.status:4}] {item.name}: {item.detail}")
    sys.stdout.flush()


def print_acceptance_checklist() -> None:
    print(
        """
== Real-run acceptance checklist ==
1. REAR stage: centered object drives forward; left/right offsets turn the correct way.
2. ARM stage: robot stops at handoff distance, no final blind push.
3. Handoff log: latched grasp target pos=[x,y,z] and width match the real object.
4. PPO grasp: arm moves toward the object; Stage 1 closes S6 from 30 deg toward 180 deg.
5. Stage 2: arm returns home while keeping the gripper closed.
6. Verify/retry: object still at the same spot => retreat and retry, max 3 attempts.

Only tune if symptoms appear:
- left/right reversed: SIGN_Y
- fixed left/right offset: CAM_TO_BASE_Y
- fixed forward/back offset: CAM_TO_BASE_X or ARM_BLIND_START_DIST_M
- fixed height error: OBJ_Z_FIXED
- distance scale error: THETA_ARM/H_ARM/FX_ARM/FY_ARM
- turn direction reversed: action_to_vxyz turn_left/turn_right vz sign
"""
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Verify X3Plus real deployment readiness.")
    parser.add_argument("--port", default="/dev/myserial", help="Rosmaster serial port.")
    parser.add_argument("--skip-selftest", action="store_true", help="Skip pipeline --selftest.")
    parser.add_argument("--run-dry", action="store_true", help="Run camera dry-run: pipeline --show.")
    parser.add_argument("--run-real", action="store_true", help="Run real hardware: pipeline --real --show.")
    parser.add_argument(
        "--i-understand-real-motion",
        action="store_true",
        help="Required with --run-real; confirms the robot is clear and supervised.",
    )
    parser.add_argument("--max-retries", type=int, default=3, help="Forwarded to the pipeline.")
    parser.add_argument("--max-steps", type=int, default=300, help="Forwarded to the pipeline.")
    parser.add_argument("--handoff-dist", type=float, default=None, help="Forwarded to the pipeline.")
    parser.add_argument("--cam-x", type=float, default=0.1639, help="Phase-3 camera->base X offset.")
    parser.add_argument("--cam-y", type=float, default=0.0331, help="Phase-3 camera->base Y offset.")
    parser.add_argument("--sign-y", type=float, choices=(-1.0, 1.0), default=-1.0)
    parser.add_argument(
        "--i-confirm-camera-frame",
        action="store_true",
        help="Required with --run-real; confirms Phase-3 camera mapping was measured.",
    )
    return parser.parse_args()


def pipeline_args(args: argparse.Namespace, *, real: bool) -> List[str]:
    cmd = [
        sys.executable,
        str(PIPELINE),
        "--show",
        "--port",
        args.port,
        "--max-retries",
        str(args.max_retries),
        "--max-steps",
        str(args.max_steps),
    ]
    if real:
        cmd.extend(["--real", "--i-confirm-camera-frame"])
    cmd.extend([
        "--cam-x", str(args.cam_x),
        "--cam-y", str(args.cam_y),
        "--sign-y", str(args.sign_y),
    ])
    if args.handoff_dist is not None:
        cmd.extend(["--handoff-dist", str(args.handoff_dist)])
    return cmd


def main() -> int:
    args = parse_args()
    if not all(math.isfinite(v) for v in (args.cam_x, args.cam_y, args.sign_y)):
        print("[FAIL] --cam-x/--cam-y/--sign-y must be finite.")
        return 2
    checks = preflight(args.port)
    print_checks(checks)

    fatal_real = [check for check in checks if check.fatal_for_real and not check.ok]

    if not args.skip_selftest:
        rc = run_command([sys.executable, str(PIPELINE), "--selftest"], timeout=30)
        if rc != 0:
            print("[FAIL] selftest failed; do not continue to robot tests.")
            return rc

    print_acceptance_checklist()

    if args.run_dry:
        print("\n[RUN] Starting camera dry-run. Press Ctrl+C to stop.")
        rc = run_command(pipeline_args(args, real=False))
        if rc != 0:
            return rc

    if args.run_real:
        if not args.i_understand_real_motion:
            print("[FAIL] --run-real requires --i-understand-real-motion.")
            return 2
        if not args.i_confirm_camera_frame:
            print("[FAIL] --run-real requires --i-confirm-camera-frame after Phase 3.")
            return 2
        if fatal_real:
            print("[FAIL] real run blocked by preflight warnings:")
            for item in fatal_real:
                print(f"  - {item.name}: {item.detail}")
            return 2
        print("\n[RUN] Starting REAL robot motion. Keep emergency stop ready.")
        return run_command(pipeline_args(args, real=True))

    print("\nNext commands:")
    print(f"  {sys.executable} integration/verify_x3plus_deploy.py --run-dry")
    print(
        "  "
        f"{sys.executable} integration/verify_x3plus_deploy.py "
        "--run-real --i-understand-real-motion --i-confirm-camera-frame "
        "--cam-x 0.1639 --cam-y 0.0331 --sign-y -1"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
