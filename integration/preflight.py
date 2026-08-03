#!/usr/bin/env python3
"""One command that says whether this machine is ready, and what is left.

Two halves:

  --offline   everything checkable without the robot. Run it on any machine,
              including the one you are reading this on. If it is not green,
              nothing on the robot will work either.

  --onboard   run this ON the Jetson with ROS up. It checks the things that
              only exist there -- serial ownership, /scan, AMCL, the lidar's
              forward coverage -- and prints what is still unverified.

Neither half touches a motor or a servo. Both are safe to run at any time.

    python3 integration/preflight.py --offline
    python3 integration/preflight.py --onboard --ros-host 127.0.0.1
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

OK, WARN, BAD = "PASS", "WARN", "FAIL"
_MARK = {OK: "[ ok ]", WARN: "[warn]", BAD: "[FAIL]"}


class Report:
    def __init__(self):
        self.rows = []

    def add(self, status, name, detail=""):
        self.rows.append((status, name, detail))
        print(f"  {_MARK[status]} {name}" + (f"\n         {detail}" if detail else ""))
        return status

    def counts(self):
        c = {OK: 0, WARN: 0, BAD: 0}
        for s, _, _ in self.rows:
            c[s] += 1
        return c

    def finish(self, title):
        c = self.counts()
        print(f"\n  {title}: {c[OK]} pass, {c[WARN]} warn, {c[BAD]} fail")
        return 0 if c[BAD] == 0 else 1


# ════════════════════════════════════════════════════════════════════════════
# Offline
# ════════════════════════════════════════════════════════════════════════════

SUITES = [
    ("integration/map_goal_provider.py", ["--selftest"]),
    ("integration/feedback_odom.py", ["--selftest"]),
    ("integration/ros_io.py", ["--selftest"]),
    ("integration/mission_fsm.py", ["--selftest"]),
    ("integration/mission_pipeline.py", ["--selftest"]),
    ("integration/nav_rl.py", ["--selftest"]),
    ("integration/vision_grasp_pipeline.py", ["--selftest"]),
    ("integration/nav_rl_grasp_pipeline.py", ["--selftest"]),
    ("tests/test_safety_guards.py", []),
    ("tests/test_mission_end_to_end.py", []),
    ("grasp/v21/test_deploy_controller.py", []),
    ("grasp/v21/test_deploy_floor_guard.py", []),
]


def _run(script, args, timeout=300):
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    try:
        # Decode as UTF-8 explicitly: the suites print em-dashes and box
        # characters, and a cp950 console would otherwise raise mid-read and
        # report a passing suite as a crash.
        p = subprocess.run([sys.executable, str(ROOT / script), *args],
                           capture_output=True, text=True, timeout=timeout,
                           encoding="utf-8", errors="replace",
                           env=env, cwd=str(ROOT),
                           stdin=subprocess.DEVNULL)
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except subprocess.TimeoutExpired:
        return 124, "timed out"
    except Exception as exc:                                   # pragma: no cover
        return 1, str(exc)


def check_offline(args) -> int:
    r = Report()
    print("\n== OFFLINE: everything checkable without the robot ==\n")

    for script, extra in SUITES:
        if not (ROOT / script).exists():
            r.add(BAD, script, "missing")
            continue
        code, out = _run(script, extra)
        r.add(OK if code == 0 else BAD, script,
              "" if code == 0 else out.strip().splitlines()[-1][:160])

    # ── the grasp model pair must match its manifest ──
    try:
        sys.path.insert(0, str(HERE))
        import mission_pipeline as mp
        model, vec = mp.resolve_grasp_model(verify_hashes=True)
        r.add(OK, "grasp model pair matches manifest sha256",
              f"{Path(model).name} + {Path(vec).name}")
    except SystemExit as exc:
        r.add(BAD, "grasp model pair", str(exc)[:200])
    except Exception as exc:
        r.add(BAD, "grasp model pair", f"{type(exc).__name__}: {exc}"[:200])

    # ── the nav weights must be present and paired ──
    # The nav model and its VecNormalize are one unit; a missing half loads
    # nothing and a mismatched half normalises the observation wrongly.
    nav = HERE / "nav_best_model"
    z = nav / "ppo_nav_281440_steps.zip"
    p = nav / "ppo_nav_vecnormalize_281440_steps.pkl"
    paired = z.exists() and p.exists()
    r.add(OK if paired else BAD, "nav weights present and paired",
          "" if paired else f"missing {'zip' if not z.exists() else 'pkl'} in {nav}")

    # ── the route has to load and be drivable ──
    route = args.route
    if route and Path(route).exists():
        code, out = _run("integration/map_goal_provider.py",
                         ["--validate", "--route", route,
                          "--resample-m", str(args.resample_m)])
        line = next((l for l in out.splitlines() if "re-sampled" in l), "")
        r.add(OK if code == 0 else BAD, "route.yaml loads and validates",
              line.strip()[:160])
    else:
        r.add(WARN, "route.yaml", f"not found at {route!r}; pass --route")

    # ── arm homes must not have collapsed back into one ──
    try:
        import mission_pipeline as mp
        distinct = mp.NAV_HOME_DEG != mp.GRASP_HOME_DEG
        r.add(OK if distinct else BAD, "arm nav/grasp homes are distinct",
              f"nav={list(mp.NAV_HOME_DEG)} grasp={list(mp.GRASP_HOME_DEG)}")
    except Exception as exc:
        r.add(BAD, "arm homes", str(exc)[:120])

    return r.finish("OFFLINE")


# ════════════════════════════════════════════════════════════════════════════
# Onboard
# ════════════════════════════════════════════════════════════════════════════

def check_onboard(args) -> int:
    r = Report()
    print("\n== ONBOARD: run this on the Jetson, with ROS up ==\n")

    # ── serial ownership ──
    try:
        p = subprocess.run(["fuser", "-v", "/dev/myserial"], capture_output=True,
                           text=True, timeout=10)
        holders = [l for l in (p.stdout + p.stderr).splitlines() if "/dev" not in l]
        n = len([h for h in holders if h.strip() and "USER" not in h])
        r.add(OK if n <= 1 else BAD, "serial /dev/myserial has at most one owner",
              (p.stdout + p.stderr).strip()[:200])
    except FileNotFoundError:
        r.add(WARN, "serial owner check", "fuser not available on this machine")
    except Exception as exc:
        r.add(WARN, "serial owner check", str(exc)[:120])

    # ── competing drivers ──
    try:
        p = subprocess.run(["ps", "-ef"], capture_output=True, text=True, timeout=10)
        bad = [l for l in p.stdout.splitlines()
               if any(k in l for k in ("rosmaster_main.py", "Mcnamu_driver.py",
                                       "ai_motor_server_B.py", "route_a_runtime"))
               and "grep" not in l]
        r.add(OK if not bad else BAD, "no competing chassis driver running",
              "\n         ".join(b[:150] for b in bad))
    except Exception as exc:
        r.add(WARN, "competing driver check", str(exc)[:120])

    # ── /scan + forward coverage + AMCL, all through the real adapters ──
    try:
        sys.path.insert(0, str(HERE))
        import nav_rl as nr
        import ros_io

        cfg = nr.NavRLConfig()
        if args.lidar_yaw_offset_deg:
            cfg.lidar_yaw_offset_deg = args.lidar_yaw_offset_deg
        lidar = nr.make_lidar(cfg, "ros", ros_host=args.ros_host,
                              ros_port=args.ros_port)
        import time
        deadline = time.time() + 5.0
        while time.time() < deadline and not lidar.get_points():
            time.sleep(0.2)
        pts = lidar.get_points()
        r.add(OK if pts else BAD, "/scan is publishing",
              f"{len(pts)} points, age {lidar.age():.2f}s")

        info = lidar.scan_info(cfg)
        if info is None:
            r.add(BAD, "lidar forward coverage", "no scan to inspect")
        else:
            detail = (f"frame={info['frame_id']!r} "
                      f"window [{info.get('angle_min_deg', float('nan')):.0f}, "
                      f"{info.get('angle_max_deg', float('nan')):.0f}] deg, "
                      f"forward covered {info.get('forward_fraction', 0)*100:.0f}%")
            r.add(BAD if info["warnings"] else OK,
                  "lidar covers the policy's forward arc",
                  detail + ("\n         " + "\n         ".join(info["warnings"])
                            if info["warnings"] else ""))
        lidar.close()
    except Exception as exc:
        r.add(BAD, "/scan via rosbridge", f"{type(exc).__name__}: {exc}"[:200])

    try:
        rio = ros_io.RosBridgeIO(args.ros_host, args.ros_port)
        import time
        deadline = time.time() + 5.0
        while time.time() < deadline and rio.latest_pose() is None:
            time.sleep(0.2)
        pose = rio.latest_pose()
        good, why = rio.pose_quality_ok()
        if pose is None:
            r.add(BAD, "/amcl_pose is publishing",
                  "set a TIGHT 2D Pose Estimate in RViz (std 0.15 m / 7 deg)")
        else:
            r.add(OK if good else BAD, "AMCL pose is usable",
                  f"({pose.x:.2f}, {pose.y:.2f}) " + (why or "covariance ok"))
        rio.close()
    except Exception as exc:
        r.add(BAD, "/amcl_pose via rosbridge", f"{type(exc).__name__}: {exc}"[:200])

    print("\n  Still needing a human, in this order — see TEST_PLAN.md:")
    for line in (
        "T1  push the robot 50 cm by hand, confirm /odom_setmotor grows to match",
        "T2  --probe: object in FRONT/LEFT/RIGHT lights the matching sector",
        "T2  --probe at both arm poses: a fixed 5-19 cm return dead ahead is the ARM",
        "T3  one patrol lap with --detection-streak 999 (never leaves the route)",
        "T4  place an object beside the route; confirm it does NOT bounce back to PATROL",
        "T5  grasp: base must be completely still while the arm moves",
        "T6  bin: log must say the bin approach point, not a patrol waypoint",
    ):
        print(f"    [ ] {line}")

    return r.finish("ONBOARD")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--offline", action="store_true")
    ap.add_argument("--onboard", action="store_true")
    ap.add_argument("--route", default=str(
        ROOT.parent / "Navigation" / "02_route_c_external_map_handoff"
        / "config" / "routes" / "route.yaml"))
    ap.add_argument("--resample-m", type=float, default=0.75)
    ap.add_argument("--ros-host", default="127.0.0.1")
    ap.add_argument("--ros-port", type=int, default=9090)
    ap.add_argument("--lidar-yaw-offset-deg", type=float, default=0.0)
    args = ap.parse_args()

    if not (args.offline or args.onboard):
        args.offline = True
    rc = 0
    if args.offline:
        rc |= check_offline(args)
    if args.onboard:
        rc |= check_onboard(args)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
