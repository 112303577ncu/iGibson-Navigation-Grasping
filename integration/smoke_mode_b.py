#!/usr/bin/env python3
"""Mode-B loopback smoke test: run the whole detection → grasp path on a desk.

Mode B is two processes talking over TCP:

    vision_grasp_bridge.py  ──{x,y,z,height,cam_pose}──▶  x3plus_real_grasp.py --socket

Everything about that path except the camera and the servos can be exercised
without hardware, and it is worth doing before every real run: the failures this
catches (a stale detection, a pose stamp that does not match the arm, a height
the grasp side rejects) all look identical on the robot — the arm simply sits
there, or moves to the wrong place — and cost a trip to the bench to diagnose.

What it does:
    1. starts x3plus_real_grasp.py in DRY RUN with --socket --latch-obj
    2. feeds it detections from a fake bridge, built through the real
       vision_grasp_bridge.build_payload() so the wire format is the real one
    3. checks the grasp side latched what was sent, at the pose it claimed

Scenarios (--case):
    ok          a well-formed stamped detection is latched            → expect PASS
    stale       the bridge sends once, then stops; latch must time out
    no-detection nothing is ever sent; latch must time out, not use the default
    wrong-pose  the detection is stamped with the v17 nav home        → refused
    out-of-reach a plausible detection beyond the policy's evaluated x range
    no-height   no --class-height; the grasp side falls back and says so
    all         every scenario above (default)

Run:
    python integration/smoke_mode_b.py
    python integration/smoke_mode_b.py --case wrong-pose --verbose
"""
from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_cam_geometry as acg  # noqa: E402
import vision_grasp_bridge as vgb  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
GRASP = ROOT / "grasp" / "v21" / "x3plus_real_grasp.py"
MODEL = "models/candidate_v21_seed816_ckpt550000.zip"
VECNORM = "models/candidate_v21_seed816_ckpt550000_vec.pkl"

# Loopback only, and a port away from the real 5555 so a smoke test can never be
# mistaken for -- or collide with -- a live bridge.
HOST = "127.0.0.1"
PORT = 5566


def send_once(payload: dict, port: int = PORT, timeout: float = 2.0) -> bool:
    try:
        with socket.create_connection((HOST, port), timeout=timeout) as sock:
            sock.sendall(json.dumps(payload).encode("utf-8"))
        return True
    except OSError:
        return False


# While the grasp process is still starting there is nothing listening, and every
# knock costs a full connect timeout. At 2 s that made the retry cadence hundreds
# of milliseconds rather than the 20 ms intended, so the first message to land
# could arrive most of the way through the move to home -- and the stale scenario,
# which needs its detection to be comfortably OLD by latch time, would sometimes
# find it fresh instead. That is a flaky test, which is worse than a failing one.
# A short timeout while knocking puts the first delivery within ~100 ms of the
# socket opening, leaving the whole 350 ms settle as margin.
KNOCK_TIMEOUT_S = 0.25


def make_payload(pose: acg.ArmCamPose, *, with_height: bool,
                 stamp_as: acg.ArmCamPose | None = None,
                 bbox=(225.0, 300.0, 315.0, 360.0)) -> dict:
    """One detection, built by the real bridge code path.

    The default bbox is a plausible sugarbox: ~90 px wide, sitting a little right
    of centre and below the principal point — which at the C3 pose is a NEGATIVE
    ground distance, i.e. exactly the case the old `if dist > 0` filters dropped.

    ``stamp_as`` re-stamps a payload with a DIFFERENT pose than the one its
    geometry came from. That is how the wrong-pose scenario stays a test of the
    pose stamp: computing the coordinates with the nav-home extrinsics puts them
    at x=0.47, which the bridge's own reach gate now refuses before the stamp is
    ever examined. The dangerous case is not an absurd coordinate — it is a
    perfectly plausible one carrying a stamp that does not match the arm.
    """
    heights = {"sugarbox": 0.065} if with_height else {}
    class_z = vgb.reconcile_z_and_height({"_fallback": 0.02}, heights)
    payload, note = vgb.build_payload(bbox, "sugarbox", pose, class_z, heights)
    if payload is None:
        raise SystemExit(f"the smoke test's own bbox is unusable: {note}")
    if stamp_as is not None:
        acg.stamp_payload(payload, stamp_as)
    return payload


def feed(payload: dict, stop: threading.Event, period: float = 0.25,
         burst: int | None = None) -> None:
    """Keep sending until told to stop, or send exactly ``burst`` messages.

    Retries hard until the first message lands, then settles to ``period``. The
    grasp side binds its socket at the end of construction and latches a fraction
    of a second later, so a lazy retry loop can miss that window entirely and
    deliver its "first" message inside the latch wait — which makes the stale
    scenario a race rather than a test.
    """
    sent = 0
    while not stop.is_set():
        if burst is not None and sent >= burst:
            return
        connected = send_once(payload, timeout=(2.0 if sent else KNOCK_TIMEOUT_S))
        if connected:
            sent += 1
            stop.wait(period)
        else:
            stop.wait(0.02)      # not up yet — keep knocking


def run_case(case: str, *, verbose: bool, timeout: float = 240.0) -> tuple[bool, str]:
    """Run one scenario. Returns (passed, detail)."""
    c3 = acg.V21_C3_GRASP_HOME
    nav = acg.V17_NAV_HOME

    if case == "wrong-pose":
        # Plausible coordinates, wrong pose stamp — see make_payload.
        payload = make_payload(c3, with_height=True, stamp_as=nav)
    elif case == "out-of-reach":
        # A detection the camera can genuinely produce but the policy was never
        # evaluated on. Built with a deliberately mis-calibrated cam_x so the
        # bridge's own gate does not eat it first; the grasp side must refuse.
        payload = make_payload(c3, with_height=True)
        payload["x"] = 0.38
    else:
        payload = make_payload(c3, with_height=(case != "no-height"))

    # The stale case sends ONE detection and then goes quiet. On hardware the move
    # to the home pose takes seconds, so a --once bridge is reliably stale by latch
    # time; in dry run the arm "arrives" in a single step, so the timeout is shrunk
    # to make the same condition deterministic instead of a race.
    stale_timeout = "0.05" if case == "stale" else "1.0"
    cmd = [
        sys.executable, str(GRASP),
        "--model", MODEL, "--vecnorm", VECNORM,
        "--contract", "obs_28_incremental",
        "--socket", "--socket-port", str(PORT), "--latch-obj",
        "--latch-wait", "3", "--stale-timeout", stale_timeout,
        "--max-steps", "3",
    ]
    proc = subprocess.Popen(cmd, cwd=str(GRASP.parent), stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True,
                            encoding="utf-8", errors="replace")

    stop = threading.Event()
    # The grasp side binds its socket during construction but only latches after
    # it has driven to the home pose, so start feeding immediately and let the
    # stale timeout decide what is current at latch time.
    burst = 0 if case == "no-detection" else (1 if case == "stale" else None)
    feeder = threading.Thread(target=feed, args=(payload, stop),
                              kwargs={"burst": burst}, daemon=True)
    feeder.start()

    try:
        out, _ = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        out, _ = proc.communicate()
        stop.set()
        return False, "grasp process did not exit within the timeout"
    finally:
        stop.set()

    if verbose:
        print(out)

    lines = out.splitlines()

    def has(needle: str) -> bool:
        return any(needle in ln for ln in lines)

    if case == "ok":
        if not has("cam_pose stamp v21_c3_grasp_home agrees with the encoders"):
            return False, "the pose stamp was not verified against the encoders"
        if not has("[Latch] Object latched at home pose"):
            return False, "no latch happened"
        want_x = f"{payload['x']:.4f}".rstrip("0")
        if not any("Object latched" in ln and want_x[:5] in ln for ln in lines):
            return False, f"latched position does not carry the sent x={payload['x']}"
        if not has("episode wrist_z_offset"):
            return False, "wrist_z_offset was never resolved"
        return True, f"latched {payload['x']:+.4f},{payload['y']:+.4f} h={payload.get('height')}"

    if case in ("stale", "no-detection"):
        # stale:        one message, then silence, with a stale window shorter
        #               than the move to home -- the hardware behaviour of a
        #               bridge run with --once.
        # no-detection: nothing is ever sent.
        # Either way the latch must NOT freeze anything, and must say why.
        if has("[Latch] Object latched at home pose"):
            return False, "a detection that is not current was latched anyway"
        if not has("no detection within"):
            return False, "the latch did not report the timeout"
        if case == "stale" and not has("New detection from"):
            return False, ("the detection never reached the grasp side, so this run "
                           "proved nothing about staleness")
        return True, ("stale detection withdrawn, fell back with a warning"
                      if case == "stale" else
                      "silence produced a timeout, not a default-position grasp")

    if case == "wrong-pose":
        if not has("cam_pose does not match the arm"):
            return False, "a detection stamped with the wrong pose was accepted"
        if not has("v17_nav_home"):
            return False, "the refusal did not name the offending pose"
        return True, "detection computed at the nav home was rejected at C3"

    if case == "out-of-reach":
        if not has("outside the region this policy was evaluated in"):
            return False, ("a target beyond the evaluated reach envelope was "
                           "accepted; the camera sees further than the arm was "
                           "trained to go, so this must refuse")
        return True, "target outside the evaluated x range was refused"

    if case == "no-height":
        if not has("[Latch] Object latched at home pose"):
            return False, "no latch happened"
        if not has("resting symmetric-object geometry"):
            return False, ("the grasp side did not announce its height fallback; a "
                           "silently invented height is the failure to avoid")
        return True, "missing height fell back to documented geometry, out loud"

    return False, f"unknown case {case!r}"


CASES = ("ok", "stale", "no-detection", "wrong-pose", "out-of-reach", "no-height")


def main() -> int:
    p = argparse.ArgumentParser(description="Mode-B loopback smoke test")
    p.add_argument("--case", default="all", choices=("all",) + CASES)
    p.add_argument("--verbose", action="store_true",
                   help="print the grasp process output")
    p.add_argument("--timeout", type=float, default=240.0)
    args = p.parse_args()

    if not GRASP.exists():
        print(f"[smoke] cannot find {GRASP}")
        return 2

    cases = CASES if args.case == "all" else (args.case,)
    print(f"[smoke] mode-B loopback on {HOST}:{PORT} (dry run, no hardware)")
    print(f"[smoke] active arm-cam pose: {acg.DEFAULT_POSE.describe()}")

    failures = []
    for case in cases:
        print(f"\n[smoke] --- {case} ---")
        t0 = time.monotonic()
        ok, detail = run_case(case, verbose=args.verbose, timeout=args.timeout)
        dt = time.monotonic() - t0
        print(f"[smoke] {'PASS' if ok else 'FAIL'} {case} ({dt:.1f}s): {detail}")
        if not ok:
            failures.append(case)

    print("\n" + "=" * 66)
    if failures:
        print(f"[smoke] FAILED: {', '.join(failures)}")
        return 1
    print(f"[smoke] all {len(cases)} mode-B scenarios behaved as designed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
