#!/usr/bin/env python3
"""Move the arm to a named pose and hold it — for calibration / measurement.

Standalone: talks to Rosmaster_Lib directly, so it does NOT import pybullet /
torch and starts instantly (unlike the full grasp module). After the command is
sent the servos hold position (powered position servos), so the arm stays put
after this script exits — ideal for waving a chessboard in front of the arm
camera during intrinsic calibration.

Run inside grasp_venv (Python 3.8). The Jetson's SYSTEM python3 is 3.6 and will
reject modern syntax / lack the deps:
    source ~/grasp_venv/bin/activate

Poses (API degrees; kept in sync with x3plus_real_grasp.DeployConfig):
    nav    -> (90,140,0,0,90,30)              cruise / observation home
    grasp  -> (90,32.704,9.786,32.704,90,30)  PPO grasp start pose
    custom -> --deg S1,S2,S3,S4,S5,S6

Usage (on the Jetson, in grasp/):
    python3 move_arm.py                 # dry-run, print nav home only
    python3 move_arm.py --real          # -> nav home (real motion)
    python3 move_arm.py --real --pose grasp
    python3 move_arm.py --real --deg 90,140,0,0,90,30
"""
import argparse
import math
import time

# Source of truth: x3plus_real_grasp.DeployConfig home_deg / grasp_home_deg.
# Duplicated here so this tool stays lightweight (no pybullet import); if those
# poses change, update both.
NAV_HOME = (90.0, 140.0, 0.0, 0.0, 90.0, 30.0)
GRASP_HOME = (90.0, 32.704, 9.786, 32.704, 90.0, 30.0)
# API-angle hardware limits: S1-S4 [0,180], S5 [0,270], S6 [0,180].
LIMITS = [(0, 180), (0, 180), (0, 180), (0, 180), (0, 270), (0, 180)]


def parse_deg6(s):
    vals = [float(x) for x in s.split(",")]
    if len(vals) != 6:
        raise argparse.ArgumentTypeError("need 6 comma-separated degrees S1..S6")
    if not all(math.isfinite(v) for v in vals):
        raise argparse.ArgumentTypeError("all servo degrees must be finite")
    return vals


def main():
    p = argparse.ArgumentParser(description="move X3Plus arm to a pose and hold")
    p.add_argument("--pose", choices=("nav", "grasp"), default="nav",
                   help="named pose (default nav = observation/measurement home)")
    p.add_argument("--deg", type=parse_deg6, default=None,
                   help="custom API degrees S1,...,S6 (overrides --pose)")
    p.add_argument("--port", type=str, default="/dev/myserial",
                   help="Rosmaster serial port symlink (check with: ls -l /dev/myserial)")
    p.add_argument("--run-ms", type=int, default=2000,
                   help="servo travel time in ms (Rosmaster limit: 2000)")
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--real", action="store_true", help="actually move the arm")
    mode.add_argument("--dry-run", action="store_true",
                      help="print angles only (default; kept for compatibility)")
    args = p.parse_args()

    if args.run_ms < 0:
        p.error("--run-ms must be >= 0")

    if args.deg is not None:
        target, label = list(args.deg), "custom"
    elif args.pose == "grasp":
        target, label = list(GRASP_HOME), "grasp home"
    else:
        target, label = list(NAV_HOME), "nav home"

    # clip to hardware limits and flag anything out of range
    safe, oob = [], []
    for i, (v, (lo, hi)) in enumerate(zip(target, LIMITS)):
        c = min(max(v, lo), hi)
        if c != v:
            oob.append(i + 1)
        safe.append(c)
    flag = f"   *** clipped S{oob} ***" if oob else ""
    actual_run_ms = min(args.run_ms, 2000)
    if actual_run_ms != args.run_ms:
        print(f"[move_arm] requested {args.run_ms}ms; Rosmaster limit is 2000ms")
    print(f"[move_arm] {label}: {safe} over {actual_run_ms}ms{flag}")

    if not args.real:
        print("[move_arm] dry-run — no hardware.")
        return

    from Rosmaster_Lib import Rosmaster
    bot = Rosmaster(com=args.port)
    try:
        bot.create_receive_threading()
        time.sleep(1.0)
        # Rosmaster_Lib clamps run_time to 2000ms; keep the wait/log truthful.
        bot.set_uart_servo_angle_array(angle_s=safe, run_time=actual_run_ms)
        time.sleep(actual_run_ms / 1000.0 + 0.5)
    finally:
        try:
            bot.cancel_receive_threading()
        except Exception:
            pass
        serial_obj = getattr(bot, "ser", None)
        if serial_obj is not None:
            try:
                serial_obj.close()
            except Exception:
                pass
    print("[move_arm] done — servos holding this pose. Safe to Ctrl+C / exit.")


if __name__ == "__main__":
    main()
