#!/usr/bin/env python3
"""Solve the arm camera's extrinsics at whatever pose you measured them from.

The arm camera rides on arm_link4, so theta / H / cam_x / cam_y are mounting
geometry belonging to ONE arm pose. Moving the policy from the v17 nav home to
v21's C3 grasp home invalidated all four, and the old Phase 2 recipe cannot
re-derive them at C3 for two reasons:

  * It solves theta from ``atan(H / D)``. At C3 the camera is within a degree of
    vertical, so D passes through zero and goes negative across the frame;
    ``atan(H/D)`` is undefined at D=0 and returns the wrong branch for D<0.
    This script uses ``atan2(H, D)``, which is correct either side of vertical.
  * It asks you to put marks at 0.3–0.8 m. At C3 the camera can only see ground
    over roughly x = 0.20–0.32 m in the base frame, so those marks are all out
    of frame.

So instead of measuring "distance from the camera's ground point", you place the
object at positions you know in the BASE frame — the same frame the policy
grasps in, established exactly as CALIBRATION_PLAN.md Phase 3 describes — and
this solves the extrinsics that map pixels onto them.

    obj_x = cam_x + H / tan(theta + alpha)          alpha = atan((v - cy) / fy)
    obj_y = cam_y + sign_y * (u - cx) * Z / fx      Z     = H * cos(alpha) / sin(theta + alpha)

H is measured with a ruler (lens centre to floor) — at a near-vertical pose that
is the easiest and most accurate of the four. theta and cam_x come from the
forward observations, cam_y and sign_y from the lateral ones.

Usage
-----
Each --obs is BASE_X,BASE_Y,PIXEL_U,PIXEL_V, with raw (distorted) pixels taken
from the bbox bottom-centre, exactly what the bridge feeds its ground model.

    python integration/solve_arm_cam_extrinsics.py --h 0.2151 \
        --obs 0.205,0.000,233,413 \
        --obs 0.230,-0.040,64,306 \
        --obs 0.255,0.000,234,200 \
        --obs 0.280,0.040,403,95  \
        --obs 0.300,0.000,233,11  \
        --obs 0.240,0.035,381,264

Give at least three observations spanning the visible band for theta/cam_x, and
at least two at different lateral offsets for cam_y/sign_y. At the C3 pose that
band is about x = 0.20-0.30 m and only +-5 cm laterally, so put the off-centre
placements at +-3.5 to 4.5 cm — beyond that the object is out of frame. Add --solve-h to fit
H as well when a ruler measurement is not available — expect a looser fit, since
H and theta trade off against each other over a small band.

    python integration/solve_arm_cam_extrinsics.py --selftest
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_cam_geometry as acg  # noqa: E402

# Residual gates. The grasp entry tolerances are 22 mm in xy, so extrinsics that
# leave more than a centimetre on the table have eaten half the budget before the
# policy starts.
PASS_TOL_M = 0.010
WARN_TOL_M = 0.005

Obs = Tuple[float, float, float, float]   # base_x, base_y, u, v (raw pixels)


def parse_obs(raw: str) -> Obs:
    try:
        vals = tuple(float(v) for v in raw.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid --obs: {raw}") from exc
    if len(vals) != 4:
        raise argparse.ArgumentTypeError("--obs needs BASE_X,BASE_Y,PIXEL_U,PIXEL_V")
    if not all(math.isfinite(v) for v in vals):
        raise argparse.ArgumentTypeError(f"--obs values must be finite: {raw}")
    return vals  # type: ignore[return-value]


def alpha_of(v_undistorted: float) -> float:
    return math.atan((v_undistorted - acg.CY) / acg.FY)


def ground_dist(theta_deg: float, h_m: float, alpha: float) -> Optional[float]:
    """Signed forward ground distance, or None when the ray misses the floor."""
    total = math.radians(theta_deg) + alpha
    if not (0.0 < total < math.pi):
        return None
    t = math.tan(total)
    if not math.isfinite(t) or abs(t) < 1e-9:
        return None
    return h_m / t


def depth_of(theta_deg: float, h_m: float, alpha: float) -> Optional[float]:
    total = math.radians(theta_deg) + alpha
    s = math.sin(total)
    if s <= 1e-9:
        return None
    return h_m * math.cos(alpha) / s


def fit_forward(obs: Sequence[Obs], h_m: float,
                theta_lo: float = 20.0, theta_hi: float = 130.0):
    """Solve (theta, cam_x) by least squares over a 1-D search in theta.

    cam_x enters linearly, so for any theta the best cam_x is just the mean
    residual; that reduces the fit to a scan over theta, which is well behaved
    over the whole range including straight down. No derivatives, no starting
    guess, no local minima to fall into.
    """
    und = [(bx, *acg.undistort_pixel(u, v)) for bx, _by, u, v in obs]

    def sse_for(theta: float):
        deltas = []
        for base_x, _uu, vv in und:
            d = ground_dist(theta, h_m, alpha_of(vv))
            if d is None:
                return None, None
            deltas.append(base_x - d)
        cam_x = sum(deltas) / len(deltas)
        sse = sum((base_x - (cam_x + ground_dist(theta, h_m, alpha_of(vv)) or 0.0)) ** 2
                  for base_x, _uu, vv in und)
        return sse, cam_x

    best = (float("inf"), None, None)
    step = 0.5
    lo, hi = theta_lo, theta_hi
    for _refine in range(5):          # 0.5 deg -> 0.00005 deg
        theta = lo
        while theta <= hi + 1e-9:
            sse, cam_x = sse_for(theta)
            if sse is not None and sse < best[0]:
                best = (sse, theta, cam_x)
            theta += step
        if best[1] is None:
            raise SystemExit(
                "no theta in the search range puts every observation on the floor; "
                "check H and the pixel rows")
        lo, hi = best[1] - step, best[1] + step
        step /= 20.0
    _sse, theta, cam_x = best
    return float(theta), float(cam_x)


def fit_lateral(obs: Sequence[Obs], theta_deg: float, h_m: float):
    """Solve (cam_y, sign_y). sign_y is chosen by whichever fits better."""
    rows = []
    for _bx, base_y, u, v in obs:
        uu, vv = acg.undistort_pixel(u, v)
        z = depth_of(theta_deg, h_m, alpha_of(vv))
        if z is None:
            continue
        rows.append((base_y, (uu - acg.CX) * z / acg.FX))
    if not rows:
        raise SystemExit("no usable observations for the lateral fit")

    best = None
    for sign in (1.0, -1.0):
        cam_y = sum(by - sign * lat for by, lat in rows) / len(rows)
        sse = sum((by - (cam_y + sign * lat)) ** 2 for by, lat in rows)
        if best is None or sse < best[0]:
            best = (sse, sign, cam_y)
    _sse, sign_y, cam_y = best
    spread = max(abs(lat) for _by, lat in rows)
    return float(sign_y), float(cam_y), spread


def residuals(obs: Sequence[Obs], pose: acg.ArmCamPose):
    out = []
    for base_x, base_y, u, v in obs:
        try:
            hit = acg.ground_hit_from_raw(u, v, pose)
        except acg.GroundGeometryError as e:
            out.append((base_x, base_y, None, None, str(e)))
            continue
        out.append((base_x, base_y, hit.obj_x - base_x, hit.obj_y - base_y, ""))
    return out


# Below this many observations the fit is occasionally catastrophic even though
# its median looks fine. Simulated at 1.5 px / 2 mm of noise, held-out error:
#     3 placements: median 1.4 mm, p95 33 mm, worst 36 mm
#     5 placements: median 1.0 mm, p95  2.5 mm, worst 4.4 mm
#     8 placements: median 0.8 mm, p95  1.8 mm, worst 2.8 mm
# Three points can be arranged so that noise mimics a different (theta, cam_x)
# pair almost exactly, and nothing in the residuals shows it -- which is the
# whole problem with judging this fit by how well it fits.
MIN_SAFE_OBS = 5


def solve(obs: Sequence[Obs], h_m: float, *, solve_h: bool = False,
          pose_name: str = "measured", arm_deg: Sequence[float] = ()):
    if len(obs) < 3:
        raise SystemExit("need at least three observations")
    if len(obs) < MIN_SAFE_OBS:
        print(f"[solve] WARNING: only {len(obs)} observations. At this count the fit "
              f"is usually fine and occasionally very wrong -- simulated held-out "
              f"error is 1.4 mm median but 33 mm at p95, and the residuals do not "
              f"distinguish the two cases. Use at least {MIN_SAFE_OBS}.")

    if solve_h:
        # H and theta trade off, so this is a coarse 2-D scan; a ruler beats it.
        best = None
        h = max(0.05, h_m - 0.05)
        while h <= h_m + 0.05 + 1e-9:
            theta, cam_x = fit_forward(obs, h)
            trial = acg.ArmCamPose(
                name=pose_name, arm_deg=tuple(arm_deg) or (0.0,) * 6,
                theta_deg=theta, h_m=h, cam_x_m=cam_x, cam_y_m=0.0, sign_y=1.0,
                distance_model_measured=True, base_offset_measured=True,
                source="solver")
            sse = sum(dx * dx for _bx, _by, dx, _dy, _e in residuals(obs, trial)
                      if dx is not None)
            if best is None or sse < best[0]:
                best = (sse, h, theta, cam_x)
            h += 0.001
        _sse, h_m, theta, cam_x = best
        print(f"[solve] fitted H = {h_m:.4f} m (a ruler measurement is more "
              f"trustworthy over this small a band)")
    else:
        theta, cam_x = fit_forward(obs, h_m)

    sign_y, cam_y, lateral_spread = fit_lateral(obs, theta, h_m)

    pose = acg.ArmCamPose(
        name=pose_name,
        arm_deg=tuple(float(v) for v in arm_deg) or (0.0,) * 6,
        theta_deg=theta, h_m=h_m, cam_x_m=cam_x, cam_y_m=cam_y, sign_y=sign_y,
        distance_model_measured=True, base_offset_measured=True,
        source="solve_arm_cam_extrinsics.py",
    )
    return pose, lateral_spread


def report(obs: Sequence[Obs], pose: acg.ArmCamPose, lateral_spread: float) -> bool:
    print("\n== solved extrinsics ==")
    print(f"  theta = {pose.theta_deg:.3f} deg   (measured towards base +X; "
          f">90 means past vertical)")
    print(f"  H     = {pose.h_m:.4f} m")
    print(f"  cam_x = {pose.cam_x_m:+.4f} m")
    print(f"  cam_y = {pose.cam_y_m:+.4f} m")
    print(f"  sign_y= {pose.sign_y:+.0f}")

    print("\n== residuals (solved minus measured) ==")
    worst = 0.0
    ok = True
    for base_x, base_y, dx, dy, err in residuals(obs, pose):
        if dx is None:
            print(f"  base=({base_x:+.3f},{base_y:+.3f})  UNUSABLE: {err}")
            ok = False
            continue
        worst = max(worst, abs(dx), abs(dy))
        flag = "  " if max(abs(dx), abs(dy)) <= WARN_TOL_M else (
            "! " if max(abs(dx), abs(dy)) <= PASS_TOL_M else "X ")
        print(f"{flag}base=({base_x:+.3f},{base_y:+.3f})  "
              f"dx={dx*1000:+6.1f} mm  dy={dy*1000:+6.1f} mm")

    print(f"\n  worst residual {worst*1000:.1f} mm "
          f"(pass <= {PASS_TOL_M*1000:.0f} mm)")
    if lateral_spread < 0.02:
        print("  WARNING: every observation is within 2 cm of the image centre "
              "laterally, so sign_y and cam_y are barely constrained. Add points "
              "further left and right.")
        ok = False
    if worst > PASS_TOL_M:
        print("  FAIL: residuals exceed the gate. If the error grows with distance, "
              "H is wrong; if it is a constant shift, re-check the base-frame "
              "reference mark (CALIBRATION_PLAN.md Phase 3 step 1).")
        ok = False

    print("""
== read this before comparing theta against the URDF prediction ==
  Over the narrow ground band a near-vertical camera sees, theta and cam_x are
  strongly correlated: raising theta and shortening cam_x cancel out almost
  exactly. So the two numbers above are NOT separately identifiable from this
  data, and the fit is free to land far from the physical truth while still
  predicting positions correctly.

  In simulation, data with 1 px / 2 mm of noise recovers theta only to about
  +-5 deg and cam_x to about +-2 cm -- yet predicts held-out points to 0.9 mm
  mean, 2.2 mm p95. One fit returned theta=79.8 where the truth was 89.3, and
  still placed every point within 1.3 mm.

  Therefore:
    * judge this calibration by its RESIDUALS and by held-out points, never by
      how close theta lands to 89.7 or cam_x to 0.276;
    * do not extrapolate outside the band you measured -- that is the one place
      the correlated error stops cancelling;
    * keep a couple of placements out of the fit and check them afterwards.""")

    print("\n== paste into integration/arm_cam_geometry.py ==")
    print(f"""    theta_deg={pose.theta_deg:.3f},
    h_m={pose.h_m:.4f},
    cam_x_m={pose.cam_x_m:.4f},
    cam_y_m={pose.cam_y_m:.4f},
    sign_y={pose.sign_y:.1f},
    distance_model_measured=True,
    base_offset_measured=True,""")
    print("\n== or pass straight to the bridge, no code edit ==")
    print(f"  python3 integration/vision_grasp_bridge.py \\\n"
          f"    --cam-theta {pose.theta_deg:.3f} --cam-h {pose.h_m:.4f} \\\n"
          f"    --cam-x {pose.cam_x_m:.4f} --cam-y {pose.cam_y_m:.4f} "
          f"--sign-y {pose.sign_y:.0f}")
    return ok


def _selftest() -> int:
    """Round-trip synthetic observations generated from a known pose."""
    print("solve_arm_cam_extrinsics self-test")
    truth = acg.V21_C3_GRASP_HOME.replace(theta_deg=89.30, h_m=0.2170,
                                          cam_x_m=0.2600, cam_y_m=-0.0050)
    # Points spread across the visible band and well off-centre laterally.
    obs: List[Obs] = []
    for u, v in [(212, 60), (212, 200), (212, 340), (212, 460),
                 (340, 200), (90, 200), (360, 400), (70, 400)]:
        hit = acg.ground_hit_from_raw(float(u), float(v), truth)
        obs.append((hit.obj_x, hit.obj_y, float(u), float(v)))

    pose, spread = solve(obs, truth.h_m, arm_deg=truth.arm_deg,
                         pose_name="selftest")
    errs = {
        "theta": abs(pose.theta_deg - truth.theta_deg),
        "cam_x": abs(pose.cam_x_m - truth.cam_x_m),
        "cam_y": abs(pose.cam_y_m - truth.cam_y_m),
    }
    print(f"  recovered theta={pose.theta_deg:.4f} (truth {truth.theta_deg}) "
          f"err {errs['theta']:.5f} deg")
    print(f"  recovered cam_x={pose.cam_x_m:.5f} (truth {truth.cam_x_m}) "
          f"err {errs['cam_x']*1000:.4f} mm")
    print(f"  recovered cam_y={pose.cam_y_m:.5f} (truth {truth.cam_y_m}) "
          f"err {errs['cam_y']*1000:.4f} mm")
    print(f"  recovered sign_y={pose.sign_y:+.0f} (truth {truth.sign_y:+.0f})")

    bad = []
    if errs["theta"] > 0.01:
        bad.append("theta")
    if errs["cam_x"] > 0.0005 or errs["cam_y"] > 0.0005:
        bad.append("cam_x/cam_y")
    if pose.sign_y != truth.sign_y:
        bad.append("sign_y")

    # A wrong sign_y must be detected, not absorbed into cam_y.
    flipped = truth.replace(sign_y=-1.0)
    obs_flipped = []
    for u, v in [(340, 200), (90, 200), (360, 400), (70, 400), (212, 300)]:
        hit = acg.ground_hit_from_raw(float(u), float(v), flipped)
        obs_flipped.append((hit.obj_x, hit.obj_y, float(u), float(v)))
    pose_f, _ = solve(obs_flipped, flipped.h_m, pose_name="selftest-flipped")
    if pose_f.sign_y != -1.0:
        bad.append("sign_y flip not detected")
    print(f"  mirrored data recovers sign_y={pose_f.sign_y:+.0f} (expected -1)")

    print(f"\n{'PASS' if not bad else 'FAIL: ' + ', '.join(bad)}")
    return 1 if bad else 0


def main() -> int:
    p = argparse.ArgumentParser(
        description="Solve arm-camera extrinsics from base-frame observations")
    p.add_argument("--obs", action="append", type=parse_obs, default=[],
                   help="BASE_X,BASE_Y,PIXEL_U,PIXEL_V (raw pixels). Repeatable.")
    p.add_argument("--h", type=float, default=None,
                   help="ruler-measured camera height above the floor (m)")
    p.add_argument("--object-height", type=float, default=0.0,
                   help="height of the object used for the observations (m). The "
                        "pixels are its SILHOUETTE CENTRE, which sits at half "
                        "height, so the solve runs against a plane h/2 above the "
                        "floor -- this does that subtraction for you. Leave at 0 "
                        "only when the observations are of a flat marker.")
    p.add_argument("--solve-h", action="store_true",
                   help="fit H too, starting from --h. Less trustworthy than a ruler.")
    p.add_argument("--pose-name", default="v21_c3_grasp_home")
    p.add_argument("--arm-deg", default="90,67.08,9.79,9.79,90,30",
                   help="the arm pose these observations were taken at")
    p.add_argument("--selftest", action="store_true")
    args = p.parse_args()

    if args.selftest:
        return _selftest()
    if not args.obs:
        p.error("give at least three --obs, or --selftest")
    if args.h is None:
        p.error("--h is required (measure the lens centre to the floor with a ruler)")
    if not math.isfinite(args.h) or args.h <= 0.0:
        p.error("--h must be finite and positive")

    if not math.isfinite(args.object_height) or args.object_height < 0.0:
        p.error("--object-height must be finite and non-negative")
    if args.object_height >= args.h:
        p.error("--object-height must be below the camera height")
    h_effective = args.h - args.object_height / 2.0
    if args.object_height:
        print(f"[solve] observations are silhouette centres of a "
              f"{args.object_height*100:.1f} cm object, so the fit runs against the "
              f"plane {args.object_height/2*100:.2f} cm above the floor: effective "
              f"camera height {h_effective:.4f} m (ruler {args.h:.4f}).")

    arm_deg = [float(x) for x in args.arm_deg.split(",") if x.strip()]
    pose, spread = solve(args.obs, h_effective, solve_h=args.solve_h,
                         pose_name=args.pose_name, arm_deg=arm_deg)
    return 0 if report(args.obs, pose, spread) else 1


if __name__ == "__main__":
    raise SystemExit(main())
