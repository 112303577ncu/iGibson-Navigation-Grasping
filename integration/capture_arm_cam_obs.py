#!/usr/bin/env python3
"""Capture arm-camera calibration observations, one object placement at a time.

docs/calibration/CALIBRATION_PLAN.md Phase 2 needs, for each placement of the object, a base-frame
coordinate you measured with a ruler and the bbox bottom-centre pixel the camera
sees it at. Doing that by hand means reading a number off a video window,
transcribing it, and typing it into a solver command — for eight placements,
twice over if you keep a held-out set. Every one of those steps is a chance to
put the right pixel next to the wrong coordinate, and a swapped pair does not
look like an error afterwards: it just makes the residuals worse and sends you
back to re-measure something that was fine.

So this does the reading. You place the object, type where you put it, and press
Enter; it samples the detector for a moment, takes the MEDIAN pixel, and stores
the pair. At the end it prints the solver command with every observation already
filled in, and can run it for you.

Why the median of many frames: the fit is sensitive to pixel noise, and it is the
cheapest thing to improve. In simulation, going from 2 px to 1 px of noise on the
same placements moved held-out prediction error from 1.4 mm to 0.9 mm. Twenty
frames cut the noise by roughly a factor of four, and cost a second.

Sessions are written after every placement, so an interrupted session resumes
instead of starting over.

Rehearse it first
-----------------
``--simulate`` replaces the camera with the registered pose's own geometry plus
noise, so you can practise the whole flow — the prompts, the held-out split, the
solve — at a desk, with no robot and no camera. What it prints at the end is what
a real session prints.

    python integration/capture_arm_cam_obs.py --simulate --out /tmp/rehearsal.json

Real session
------------
    python integration/capture_arm_cam_obs.py \
        --stream http://<JETSON_IP>:8080/stream?topic=/arm_cam/image_raw \
        --out c3_calib.json --h 0.2151

Coordinates are BASE-FRAME ABSOLUTE, not distances from the ground mark. If your
reference mark is at (0.167, 0.018) and you put the object 8 cm ahead of it, that
is 0.247, 0.018 — see docs/calibration/CALIBRATION_PLAN.md Phase 3 step 1.

Where the object has to be: at the C3 pose the camera sees the ground over
roughly x = 0.195-0.32 m, and laterally from about -5.9 cm to +10.7 cm. That
range is ASYMMETRIC — the principal point sits at x=212 in a 640-wide frame, not
at 320 — so do not plan a symmetric left/right pair. Around -4 cm and +6 cm
works. Outside it the object is simply not in frame and will not record.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import statistics
import subprocess
import sys
from pathlib import Path
from typing import List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_cam_geometry as acg  # noqa: E402

DEFAULT_MODEL = str(Path(__file__).resolve().parent.parent / "detection" / "models" / "best.pt")
DEFAULT_STREAM = "http://127.0.0.1:8080/stream?topic=/arm_cam/image_raw"

# Spread across the sampled frames beyond which the placement is not trustworthy:
# the detector is flickering between edges, or something is moving in frame.
PIXEL_SPREAD_WARN = 4.0

# What the rehearsal pretends the hardware is, so the numbers it prints look like
# a real session's rather than landing exactly on the registered prediction.
SIM_TRUTH = dict(theta_deg=89.30, h_m=0.2170, cam_x_m=0.2600, cam_y_m=-0.0050)
SIM_PIXEL_NOISE_PX = 1.5


class Sampler:
    """Yields the bbox bottom-centre pixel of the best detection, or None."""

    def __init__(self, stream: str, model_path: str, conf: float, imgsz: int):
        import cv2
        from ultralytics import YOLO

        self._cv2 = cv2
        print(f"[capture] loading YOLO: {model_path}")
        self.model = YOLO(model_path)
        print(f"[capture] classes: {self.model.names}")
        print(f"[capture] opening camera: {stream}")
        self.cap = cv2.VideoCapture(int(stream) if stream.isdigit() else stream)
        if not self.cap.isOpened():
            raise SystemExit(f"[capture] cannot open camera stream: {stream}")
        self.conf = conf
        self.imgsz = imgsz
        self._size_checked = False

    def one(self) -> Optional[Tuple[float, float, str]]:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        if not self._size_checked:
            self._size_checked = True
            problem = acg.frame_size_mismatch(frame.shape[1], frame.shape[0])
            if problem:
                raise SystemExit(f"[capture] {problem}")
        res = self.model.predict(source=frame, conf=self.conf, imgsz=self.imgsz,
                                 verbose=False)[0]
        boxes = res.boxes
        if boxes is None or len(boxes) == 0:
            return None
        confs = boxes.conf.tolist()
        box = boxes[max(range(len(confs)), key=lambda i: confs[i])]
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        if acg.bbox_touches_border(x1, y1, x2, y2) is not None:
            # A clipped box would put a wrong pixel next to a correct ruler
            # coordinate, which is the single worst thing that can enter this fit.
            return None
        cls_id = int(box.cls[0].item())
        names = self.model.names
        name = str(names.get(cls_id, cls_id) if isinstance(names, dict) else names[cls_id])
        # The SILHOUETTE CENTRE, matching what the bridge locates objects by at
        # runtime. Recording the bottom edge here and the centre there would fit
        # the extrinsics to one convention and use them under another.
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0, name

    def close(self):
        try:
            self.cap.release()
        except Exception:
            pass


class SimSampler:
    """Rehearsal stand-in: projects the typed coordinate through a known pose."""

    def __init__(self, pose: acg.ArmCamPose, seed: int = 0,
                 height_m: float = 0.065, width_m: float = 0.023):
        self.truth = pose.replace(**SIM_TRUTH)
        self.height_m, self.width_m = height_m, width_m
        self.rng = random.Random(seed)
        self.target: Optional[Tuple[float, float]] = None
        print("[capture] SIMULATED camera — no hardware, no images. Pixels are "
              "projected from a pretend-measured pose plus noise.")

    def aim(self, base_x: float, base_y: float) -> bool:
        """Project a BOX of the configured size and take its silhouette centre.

        Projecting a point would rehearse a problem the operator does not have.
        A real object has height, its top face projects further from the nadir
        than its base, and that is what decides both the pixel recorded and
        whether the placement fits in frame at all.
        """
        import itertools
        th = math.radians(self.truth.theta_deg)
        half = self.width_m / 2.0
        us, vs = [], []
        for sx, sy, sz in itertools.product((-half, half), (-half, half),
                                            (0.0, self.height_m)):
            dx = base_x + sx - self.truth.cam_x_m
            dy = base_y + sy - self.truth.cam_y_m
            dz = sz - self.truth.h_m
            xc = dy * self.truth.sign_y
            yc = -dx * math.sin(th) - dz * math.cos(th)
            zc = dx * math.cos(th) - dz * math.sin(th)
            if zc <= 1e-9:
                self.target = None
                return False
            us.append(acg.CX + acg.FX * xc / zc)
            vs.append(acg.CY + acg.FY * yc / zc)
        if acg.bbox_touches_border(min(us), min(vs), max(us), max(vs)) is not None:
            self.target = None
            return False
        self.target = ((min(us) + max(us)) / 2.0, (min(vs) + max(vs)) / 2.0)
        return True

    def one(self):
        if self.target is None:
            return None
        u, v = self.target
        return (u + self.rng.gauss(0, SIM_PIXEL_NOISE_PX),
                v + self.rng.gauss(0, SIM_PIXEL_NOISE_PX), "sugarbox")

    def close(self):
        pass


def parse_xy(raw: str) -> Optional[Tuple[float, float]]:
    parts = [p for p in raw.replace(",", " ").split() if p]
    if len(parts) != 2:
        return None
    try:
        x, y = float(parts[0]), float(parts[1])
    except ValueError:
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return x, y


def sample_placement(sampler, samples: int, base_xy: Tuple[float, float]):
    """Sample the detector repeatedly and return the median pixel + diagnostics."""
    if isinstance(sampler, SimSampler) and not sampler.aim(*base_xy):
        return None, "that coordinate is outside the simulated camera's view"

    us: List[float] = []
    vs: List[float] = []
    names = set()
    misses = 0
    for _ in range(samples * 3):          # allow for dropped frames / missed detections
        if len(us) >= samples:
            break
        got = sampler.one()
        if got is None:
            misses += 1
            continue
        u, v, name = got
        us.append(u)
        vs.append(v)
        names.add(name)

    if len(us) < max(3, samples // 4):
        return None, (f"only {len(us)} detections in {samples * 3} attempts "
                      f"({misses} misses) — is the object in frame and lit?")

    u_med, v_med = statistics.median(us), statistics.median(vs)
    spread = max(statistics.pstdev(us) if len(us) > 1 else 0.0,
                 statistics.pstdev(vs) if len(vs) > 1 else 0.0)
    note = (f"{len(us)} samples, median pixel ({u_med:.1f}, {v_med:.1f}), "
            f"spread {spread:.1f} px, class {'/'.join(sorted(names))}")
    if spread > PIXEL_SPREAD_WARN:
        note += ("  ⚠ the detector is not settling; re-seat the object or check "
                 "for movement in frame")
    return {"base_x": base_xy[0], "base_y": base_xy[1],
            "u": round(u_med, 2), "v": round(v_med, 2),
            "samples": len(us), "spread_px": round(spread, 2),
            "class": sorted(names)[0] if names else None}, note


def load_session(path: Path) -> List[dict]:
    if not path.exists():
        return []
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        raise SystemExit(f"[capture] cannot read {path}: {e}")
    obs = data.get("observations", [])
    print(f"[capture] resuming {path} with {len(obs)} placement(s) already recorded")
    return obs


def save_session(path: Path, obs: List[dict], h_m: Optional[float],
                 pose_name: str) -> None:
    path.write_text(json.dumps(
        {"pose": pose_name, "h_m": h_m, "observations": obs},
        indent=2), encoding="utf-8")


def coverage_report(obs: List[dict], window=None) -> List[str]:
    """What the fit still needs. Missing coverage is why a solve comes out loose.

    ``window`` is the (x_lo, x_hi, y_lo, y_hi) region this object actually fits
    in, from arm_cam_geometry.usable_placement_window. Judging against it rather
    than against fixed prose matters: the window shrinks as the object gets
    taller, and advice written for one object is wrong for another.
    """
    problems = []
    if len(obs) < 5:
        problems.append(
            f"only {len(obs)} placements; below five the fit is occasionally very "
            f"wrong (p95 33 mm) even though its residuals look fine. Six to eight "
            f"is comfortable.")
    if not obs:
        return problems

    xs = [o["base_x"] for o in obs]
    ys = [o["base_y"] for o in obs]
    vs = [o["v"] for o in obs]
    us = [o["u"] for o in obs]

    if window is not None:
        x_lo, x_hi, y_lo, y_hi = window
        span_x = x_hi - x_lo
        span_y = y_hi - y_lo
        if max(xs) - min(xs) < 0.6 * span_x:
            problems.append(
                f"forward span is {(max(xs)-min(xs))*100:.1f} cm out of the "
                f"{span_x*100:.1f} cm available ({x_lo:.3f}-{x_hi:.3f}); use more of "
                f"it or theta and cam_x cannot be told apart")
        if max(ys) - min(ys) < 0.5 * span_y:
            problems.append(
                f"lateral span is {(max(ys)-min(ys))*100:.1f} cm out of the "
                f"{span_y*100:.1f} cm available ({y_lo:+.3f} to {y_hi:+.3f}); sign_y "
                f"and cam_y will be barely constrained")
        outside = [(x, y) for x, y in zip(xs, ys)
                   if not (x_lo <= x <= x_hi and y_lo <= y <= y_hi)]
        if outside:
            problems.append(
                f"{len(outside)} placement(s) sit outside the window this object "
                f"fits in; they may have been recorded near the frame edge")
    else:
        if max(xs) - min(xs) < 0.03:
            problems.append(
                f"forward span is only {(max(xs)-min(xs))*100:.1f} cm; spread the "
                f"placements out or theta and cam_x cannot be told apart")
        if len([u for u in us if abs(u - acg.CX) > 90]) < 2:
            problems.append(
                "fewer than two placements are well off-centre laterally; sign_y "
                "and cam_y will be barely constrained")

    if max(vs) - min(vs) < 100:
        problems.append(
            f"the placements only cover {max(vs)-min(vs):.0f} image rows; use the "
            f"top and bottom of the usable band too")
    return problems


def emit_solver_command(obs: List[dict], h_m: Optional[float], pose_name: str,
                        holdout: int, object_height: float = 0.0
                        ) -> Tuple[List[str], List[dict], List[dict]]:
    fit = obs[:len(obs) - holdout] if holdout else list(obs)
    held = obs[len(obs) - holdout:] if holdout else []
    cmd = [sys.executable, str(Path(__file__).resolve().parent / "solve_arm_cam_extrinsics.py"),
           "--pose-name", pose_name]
    if h_m is not None:
        cmd += ["--h", f"{h_m:.4f}"]
    if object_height:
        cmd += ["--object-height", f"{object_height:.4f}"]
    for o in fit:
        cmd += ["--obs", f"{o['base_x']:.4f},{o['base_y']:.4f},{o['u']:.2f},{o['v']:.2f}"]
    return cmd, fit, held


def main() -> int:
    p = argparse.ArgumentParser(description="Capture arm-camera calibration observations")
    p.add_argument("--out", type=Path, default=Path("arm_cam_calib.json"),
                   help="session file; resumed if it already exists")
    p.add_argument("--stream", default=DEFAULT_STREAM,
                   help="camera URL or webcam index")
    p.add_argument("--model", default=DEFAULT_MODEL)
    p.add_argument("--conf", type=float, default=0.3)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--samples", type=int, default=20,
                   help="frames per placement; the median is stored (default 20)")
    p.add_argument("--h", type=float, default=None,
                   help="ruler-measured camera height above the floor (m)")
    p.add_argument("--object-height", type=float, default=0.065,
                   help="height of the object you are placing (m, default 0.065 = "
                        "sugarbox). Decides the usable placement window and is "
                        "passed to the solver, which fits against the plane at "
                        "half that height.")
    p.add_argument("--object-width", type=float, default=0.023,
                   help="width of the object you are placing (m, default 0.023)")
    p.add_argument("--pose-name", default=acg.DEFAULT_POSE.name)
    p.add_argument("--holdout", type=int, default=2,
                   help="placements kept OUT of the fit, to check it afterwards "
                        "(default 2). Judge a calibration by these, not by theta.")
    p.add_argument("--simulate", action="store_true",
                   help="rehearse with no camera and no robot")
    p.add_argument("--solve", action="store_true",
                   help="run the solver when the session ends")
    args = p.parse_args()

    if args.samples < 1:
        raise SystemExit("--samples must be >= 1")
    if args.holdout < 0:
        raise SystemExit("--holdout must be >= 0")

    pose = acg.get_pose(args.pose_name)
    print(f"[capture] pose: {pose.describe()}")
    try:
        win = acg.usable_placement_window(pose, args.object_height, args.object_width)
        print(f"[capture] an object {args.object_height*100:.1f} cm tall fits entirely "
              f"in frame over:")
        print(f"[capture]   base x {win[0]:.3f} .. {win[1]:.3f} m")
        print(f"[capture]   base y {win[2]:+.3f} .. {win[3]:+.3f} m  (measured at mid-x)")
        print(f"[capture] Placements outside that are clipped by the frame edge and "
              f"will not record. Note this is much smaller than the camera's ground "
              f"footprint -- a tall object seen from near vertical has a larger "
              f"silhouette than its footprint.")
    except acg.GroundGeometryError as e:
        print(f"[capture] WARN: {e}")
        win = None
    if args.h is None and not args.simulate:
        print("[capture] NOTE: no --h given. Measure the lens centre to the floor "
              "with a ruler and pass it; at a near-vertical pose it is the most "
              "accurate of the four extrinsics and the solver needs it.")

    obs = load_session(args.out)
    sampler = (SimSampler(pose, height_m=args.object_height,
                          width_m=args.object_width) if args.simulate
               else Sampler(args.stream, args.model, args.conf, args.imgsz))

    print("\nType the object's BASE-FRAME coordinate as 'x y' (metres) and press")
    print("Enter. That is the absolute base coordinate, not a distance from your")
    print("ground mark — docs/calibration/CALIBRATION_PLAN.md Phase 3 step 1.")
    print("Enter 'u' to undo the last placement, 'd' to done, 'q' to quit.\n")

    try:
        while True:
            n = len(obs)
            try:
                raw = input(f"[{n}] base x y (or u/d/q) > ").strip()
            except EOFError:
                print()
                break
            if not raw:
                continue
            low = raw.lower()
            if low in ("q", "quit"):
                print("[capture] quitting; session file keeps what was recorded.")
                break
            if low in ("d", "done"):
                break
            if low in ("u", "undo"):
                if obs:
                    gone = obs.pop()
                    save_session(args.out, obs, args.h, args.pose_name)
                    print(f"[capture] removed ({gone['base_x']}, {gone['base_y']})")
                else:
                    print("[capture] nothing to undo")
                continue

            xy = parse_xy(raw)
            if xy is None:
                print("[capture] give two finite numbers, e.g. '0.247 0.018'")
                continue

            rec, note = sample_placement(sampler, args.samples, xy)
            if rec is None:
                print(f"[capture] not recorded — {note}")
                continue
            obs.append(rec)
            save_session(args.out, obs, args.h, args.pose_name)
            print(f"[capture] recorded #{len(obs)}: base=({xy[0]:.3f}, {xy[1]:.3f})  {note}")
    finally:
        sampler.close()

    print(f"\n[capture] {len(obs)} placement(s) saved to {args.out}")
    if not obs:
        return 0

    holdout = min(args.holdout, max(0, len(obs) - 3))
    cmd, fit, held = emit_solver_command(obs, args.h, args.pose_name, holdout,
                                        args.object_height)
    # Judge the FIT set: coverage the held-out placements provide does not
    # constrain the solve, and counting them hides a fit that is barely determined.
    for problem in coverage_report(fit, win):
        print(f"[capture] COVERAGE (of the {len(fit)} fitted placements): {problem}")
    print(f"\n[capture] {len(fit)} placement(s) go into the fit, "
          f"{len(held)} held out for the check.")
    print("\n== solve with ==\n")
    print("  " + " ".join(f'"{c}"' if " " in c else c for c in cmd))
    if held:
        print("\n== then verify against the held-out placements ==")
        for o in held:
            print(f"  base=({o['base_x']:.4f}, {o['base_y']:.4f}) "
                  f"pixel=({o['u']:.1f}, {o['v']:.1f})")
        print("  Put the solved extrinsics into vision_grasp_bridge.py --dry-run and")
        print("  check it reports these coordinates to within 1 cm. That, not how")
        print("  close theta lands to its prediction, is what says the fit is good.")

    if args.solve:
        if args.h is None:
            print("\n[capture] cannot solve without --h")
            return 1
        print("\n[capture] running the solver...\n")
        return subprocess.call(cmd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
