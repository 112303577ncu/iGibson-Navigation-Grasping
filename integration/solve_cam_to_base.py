#!/usr/bin/env python3
"""Solve Phase-3 camera-to-base constants from latch measurements.

Every ``--pair`` is BASE_X,BASE_Y,LATCH_X,LATCH_Y in metres.  BASE_X/Y are
absolute coordinates in the PPO/URDF ``base_link`` frame, not distances from
the temporary ground mark.  If the mark is at ``(ref_x, ref_y)``, convert a
ruler placement ``(forward, left)`` before entering it:

    base_x = ref_x + forward
    base_y = ref_y + left

Example: reference TCP=(0.167, 0.018), object 25 cm ahead of the mark:

    --pair 0.417,0.018,<latch_x>,<latch_y>

The latch output may already include non-default CAM_TO_BASE_X/Y or SIGN_Y;
pass those values with ``--applied-*`` to solve the underlying raw estimate.
"""

from __future__ import annotations

import argparse
from typing import List, Sequence, Tuple


PASS_TOL_M = 0.02
DRIFT_WARN_M = 0.01
SIGN_GAP_M = 0.01   # min spread difference before SIGN_Y is considered decided
Pair = Tuple[float, float, float, float]


def parse_pair(raw: str) -> Pair:
    try:
        values = tuple(float(value) for value in raw.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid --pair: {raw}") from exc
    if len(values) != 4:
        raise argparse.ArgumentTypeError(
            "--pair needs BASE_X,BASE_Y,LATCH_X,LATCH_Y"
        )
    return values  # type: ignore[return-value]


def solve(pairs: Sequence[Pair], applied_cam_x: float = 0.0,
          applied_cam_y: float = 0.0, applied_sign_y: float = 1.0):
    """Return solved constants, report lines, and whether every residual passes."""
    if len(pairs) < 3:
        raise ValueError("need at least three base-frame measurements")

    # Undo constants that were active when the latch was recorded.  The raw
    # lateral estimate is right-positive; base +Y is determined by SIGN_Y.
    raw = []
    for base_x, base_y, latch_x, latch_y in pairs:
        raw.append((
            base_x,
            base_y,
            latch_x - applied_cam_x,
            (latch_y - applied_cam_y) / applied_sign_y,
        ))

    # SIGN_Y: the correct sign makes (base_y - sign*right) a constant (= cam_y)
    # across points, while the wrong sign spreads it by ~2x the lateral
    # placement range - so pick the sign with the smaller spread.  A per-point
    # vote on sign(base_y) is NOT usable here: base_y is absolute, so a camera
    # mounted off-centre (cam_y) flips votes for points with |base_y| ~ |cam_y|.
    lines: List[str] = []

    def _y_spread(sign: float) -> float:
        offsets = [base_y - sign * right for _bx, base_y, _fwd, right in raw]
        return max(offsets) - min(offsets)

    spread_pos, spread_neg = _y_spread(1.0), _y_spread(-1.0)
    if abs(spread_pos - spread_neg) < SIGN_GAP_M:
        sign_y = applied_sign_y
        lines.append(
            "WARN: placements have no usable lateral spread - SIGN_Y cannot be "
            f"determined, keeping {applied_sign_y:+.0f}. Add clear left/right points."
        )
    else:
        sign_y = 1.0 if spread_pos < spread_neg else -1.0

    x_offsets = [base_x - forward for base_x, _base_y, forward, _right in raw]
    y_offsets = [base_y - sign_y * right for _base_x, base_y, _forward, right in raw]
    cam_x = sum(x_offsets) / len(x_offsets)
    cam_y = sum(y_offsets) / len(y_offsets)

    lines.append("point-by-point check (all coordinates are base_link frame):")
    passed = True
    for base_x, base_y, forward, right_offset in raw:
        residual_x = base_x - (forward + cam_x)
        residual_y = base_y - (sign_y * right_offset + cam_y)
        ok = abs(residual_x) < PASS_TOL_M and abs(residual_y) < PASS_TOL_M
        passed = passed and ok
        lines.append(
            f"  base=({base_x:+.3f},{base_y:+.3f}) raw=(d={forward:.3f}, "
            f"right={right_offset:+.3f}) residual=({residual_x:+.4f},"
            f"{residual_y:+.4f}) m {'OK' if ok else '** > 2cm **'}"
        )

    forwards = [forward for _x, _y, forward, _right in raw]
    if max(forwards) - min(forwards) > 0.05:
        x_spread = max(x_offsets) - min(x_offsets)
        y_spread = max(y_offsets) - min(y_offsets)
        if x_spread > DRIFT_WARN_M:
            lines.append(f"WARN: forward offset drifts {x_spread * 100:.1f}cm; redo Phase 2 theta")
        if y_spread > DRIFT_WARN_M:
            lines.append(f"WARN: lateral offset drifts {y_spread * 100:.1f}cm; check cx/geometry")
    else:
        lines.append("note: add a point at a different distance to check theta/cx drift")
    return cam_x, cam_y, sign_y, lines, passed


def run_selftest() -> None:
    def synth(placements, cam_x, cam_y, sign):
        return [(bx, by, bx - cam_x, (by - cam_y) / sign) for bx, by in placements]

    true_x, true_y, true_sign = 0.055, -0.012, -1.0
    centred = ((0.25, 0.0), (0.25, 0.05), (0.25, -0.05), (0.35, 0.0))
    pairs = synth(centred, true_x, true_y, true_sign)
    cam_x, cam_y, sign_y, _lines, passed = solve(pairs)
    assert abs(cam_x - true_x) < 1e-9
    assert abs(cam_y - true_y) < 1e-9
    assert sign_y == true_sign and passed

    applied = (0.02, 0.01, -1.0)
    applied_pairs = []
    for base_x, base_y, forward, right in pairs:
        applied_pairs.append((base_x, base_y, forward + applied[0], applied[2] * right + applied[1]))
    cam_x, cam_y, sign_y, _lines, passed = solve(applied_pairs, *applied)
    assert abs(cam_x - true_x) < 1e-9
    assert abs(cam_y - true_y) < 1e-9
    assert sign_y == true_sign and passed

    # Absolute-coordinate regression: mark at ref_y=+0.018 and camera 4cm to
    # the RIGHT of base origin.  A sign(base_y) vote flips here (the -0.032
    # point sits between 0 and cam_y); the spread method must not.
    absolute = ((0.417, 0.018), (0.417, 0.068), (0.417, -0.032), (0.517, 0.018))
    cam_x, cam_y, sign_y, _lines, passed = solve(synth(absolute, 0.03, -0.04, -1.0))
    assert abs(cam_x - 0.03) < 1e-9
    assert abs(cam_y - (-0.04)) < 1e-9
    assert sign_y == -1.0 and passed

    # Forward error growing with distance must fire the theta drift warning.
    drifty = []
    for i, (bx, by) in enumerate(((0.25, 0.0), (0.35, 0.05), (0.45, -0.05))):
        drifty.append((bx, by, bx - true_x - 0.012 * i, (by - true_y) / true_sign))
    _x, _y, _s, lines, _p = solve(drifty)
    assert any("theta" in line for line in lines), lines

    # A 5cm-off point must fail the 2cm gate.
    bad = list(pairs)
    bx, by, forward, right = bad[0]
    bad[0] = (bx, by, forward - 0.05, right)
    _x, _y, _s, _lines, passed = solve(bad)
    assert not passed

    # No lateral spread at all -> SIGN_Y undecidable, applied sign kept.
    front_only = synth(((0.25, 0.0), (0.30, 0.0), (0.35, 0.0)), true_x, 0.0, 1.0)
    _x, _y, sign_y, lines, _p = solve(front_only, applied_sign_y=-1.0)
    assert sign_y == -1.0
    assert any("lateral spread" in line for line in lines), lines
    print("[selftest] OK")


def main() -> int:
    parser = argparse.ArgumentParser(description="Solve CAM_TO_BASE_X/Y and SIGN_Y from base-frame latch data")
    parser.add_argument("--pair", action="append", default=[], type=parse_pair,
                        help="BASE_X,BASE_Y,LATCH_X,LATCH_Y metres; repeat at least 3 times")
    parser.add_argument("--applied-cam-x", type=float, default=0.0)
    parser.add_argument("--applied-cam-y", type=float, default=0.0)
    parser.add_argument("--applied-sign-y", choices=(-1.0, 1.0), type=float, default=1.0)
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        run_selftest()
        return 0
    if len(args.pair) < 3:
        parser.error("need at least three --pair values (front, left, right)")

    cam_x, cam_y, sign_y, lines, passed = solve(
        args.pair, args.applied_cam_x, args.applied_cam_y, args.applied_sign_y
    )
    print("\n".join(lines))
    print("\n" + "=" * 60)
    print(f"CAM_TO_BASE_X = {cam_x:+.4f}")
    print(f"CAM_TO_BASE_Y = {cam_y:+.4f}")
    print(f"SIGN_Y        = {sign_y:+.0f}")
    print("=" * 60)
    if passed:
        print("PASS: verify each base-frame placement once more with these constants applied.")
        return 0
    print("FAIL: re-measure the failing placement(s) before updating constants.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
