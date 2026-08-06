#!/usr/bin/env python3
"""Map-frame goal source for the patrol / deliver states.

This is the piece Route B lists as missing (``MapGoalProvider``). It turns
Route C's ``route.yaml`` (117 waypoints, 0.049 m/cell map, origin [0,0,0]) plus
an AMCL pose into the same ``(dist, bearing)`` contract the RL nav policy
already consumes, so ``nav_rl.build_nav_obs()`` does not change at all:

    obs[0] = dist, obs[1] = sin(bearing), obs[2] = cos(bearing)

Bearing convention is identical to ``nav_rl.GoalTracker``: robot frame, x
forward, y LEFT, ``bearing = wrap(atan2(dy, dx) - yaw)``. That matches Route
B's reconstructed formula (``map_goal_provider_formula_reconstructed.py``) and
the sim's ``atan2(rel_y, rel_x) - yaw``.

The provider is deliberately ignorant of ROS and of the policy: it takes poses
in, hands goal fixes out. ``mission_pipeline`` feeds it AMCL poses; the offline
selftest feeds it synthetic ones.

Run:
    python3 integration/map_goal_provider.py --selftest
    python3 integration/map_goal_provider.py --route <route.yaml> --validate
"""
from __future__ import annotations

import argparse
import dataclasses
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

# Default location of the Route C handoff. The route package lives outside this
# repo, so the path is a hint for --validate, never an import-time dependency.
DEFAULT_ROUTE_HINT = (
    Path(__file__).resolve().parents[2]
    / "Navigation" / "02_route_c_external_map_handoff" / "config" / "routes" / "route.yaml"
)

# SETMOTOR_ODOM_INTEGRATION.md section 7.1. Route C's waypoint spacing is
# 0.75 m, so 0.25 leaves the required 2*r < spacing margin with room to spare.
DEFAULT_ARRIVAL_RADIUS_M = 0.25

# An AMCL pose older than this cannot be trusted to place a map goal. The nav
# loop stops the chassis when a fix goes invalid (INTEGRATION_CONTRACT.md s8).
DEFAULT_POSE_MAX_AGE_S = 1.0


# ════════════════════════════════════════════════════════════════════════════
# Data
# ════════════════════════════════════════════════════════════════════════════

@dataclasses.dataclass(frozen=True)
class Waypoint:
    id: str
    x: float
    y: float
    yaw: Optional[float] = None


@dataclasses.dataclass(frozen=True)
class MapPose:
    """Robot pose in the map frame (from AMCL)."""
    x: float
    y: float
    yaw: float
    stamp: float

    def is_finite(self) -> bool:
        return all(math.isfinite(v) for v in (self.x, self.y, self.yaw))


@dataclasses.dataclass(frozen=True)
class GoalFix:
    """What the nav loop needs to build obs[0:3], plus why it may be unusable."""
    valid: bool
    dist: float
    bearing: float
    source_stamp: float
    reason: str = ""
    goal_id: str = ""
    goal_yaw: Optional[float] = None


@dataclasses.dataclass(frozen=True)
class Rect:
    name: str
    min_x: float
    min_y: float
    max_x: float
    max_y: float

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (self.min_x - margin <= x <= self.max_x + margin
                and self.min_y - margin <= y <= self.max_y + margin)


@dataclasses.dataclass
class RouteSpec:
    waypoints: List[Waypoint]
    loop: bool = True
    map_frame: str = "map"
    bin_center: Optional[Tuple[float, float]] = None
    bin_approach: Optional[Tuple[float, float, float]] = None
    deliver_interrupt_id: Optional[str] = None
    forbidden: List[Rect] = dataclasses.field(default_factory=list)
    source_path: str = ""

    def index_of(self, waypoint_id: str) -> int:
        for i, wp in enumerate(self.waypoints):
            if wp.id == waypoint_id:
                return i
        raise KeyError(f"no waypoint with id {waypoint_id!r}")

    def min_spacing_m(self) -> float:
        if len(self.waypoints) < 2:
            return float("inf")
        pairs = list(zip(self.waypoints, self.waypoints[1:]))
        if self.loop:
            pairs.append((self.waypoints[-1], self.waypoints[0]))
        return min(math.hypot(b.x - a.x, b.y - a.y) for a, b in pairs)


# ════════════════════════════════════════════════════════════════════════════
# Loading + validation
# ════════════════════════════════════════════════════════════════════════════

def _as_xy(value: Any, name: str) -> Tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        raise ValueError(f"{name} must be a [x, y] sequence, got {value!r}")
    x, y = float(value[0]), float(value[1])
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return x, y


def parse_route(doc: Mapping[str, Any], *, source_path: str = "",
                annotations: Optional[Mapping[str, Any]] = None,
                check_spacing: bool = True) -> RouteSpec:
    """Build a RouteSpec from an already-parsed route.yaml mapping.

    Accepts Route C's nested layout (``patrol.waypoints``) and the flatter
    minimal contract from SETMOTOR_ODOM_INTEGRATION.md section 7.1
    (``waypoints`` at the top level).
    """
    if not isinstance(doc, Mapping):
        raise ValueError("route document must be a mapping")

    map_frame = str(doc.get("map_frame") or doc.get("frame_id") or "map")
    if map_frame != "map":
        raise ValueError(f"route frame must be 'map', got {map_frame!r}")

    patrol = doc.get("patrol")
    if isinstance(patrol, Mapping) and "waypoints" in patrol:
        raw_wps = patrol.get("waypoints")
        loop = bool(patrol.get("loop", True))
        declared_count = patrol.get("waypoint_count")
    else:
        raw_wps = doc.get("waypoints")
        loop = str(doc.get("patrol_mode", "loop")).lower() == "loop"
        declared_count = None

    if not isinstance(raw_wps, Sequence) or not raw_wps:
        raise ValueError("route contains no waypoints")

    waypoints: List[Waypoint] = []
    seen: Dict[str, int] = {}
    for i, raw in enumerate(raw_wps):
        if not isinstance(raw, Mapping):
            raise ValueError(f"waypoint[{i}] must be a mapping, got {raw!r}")
        wid = str(raw.get("id", f"wp_{i:03d}"))
        if wid in seen:
            raise ValueError(
                f"duplicate waypoint id {wid!r} at index {i} (first seen at {seen[wid]})"
            )
        seen[wid] = i
        frame = raw.get("frame_id", map_frame)
        if frame is not None and str(frame) != "map":
            raise ValueError(f"waypoint {wid!r} frame must be 'map', got {frame!r}")
        try:
            x, y = float(raw["x"]), float(raw["y"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"waypoint {wid!r} needs finite x/y: {exc}") from exc
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f"waypoint {wid!r} coordinates must be finite, got ({x}, {y})")
        raw_yaw = raw.get("yaw")
        yaw = None if raw_yaw is None else float(raw_yaw)
        if yaw is not None and not math.isfinite(yaw):
            raise ValueError(f"waypoint {wid!r} yaw must be finite or null, got {raw_yaw!r}")
        waypoints.append(Waypoint(wid, x, y, yaw))

    if declared_count is not None and int(declared_count) != len(waypoints):
        raise ValueError(
            f"route declares waypoint_count={declared_count} but lists {len(waypoints)}"
        )

    spec = RouteSpec(
        waypoints=waypoints, loop=loop, map_frame=map_frame, source_path=source_path
    )

    # ── trash bin: route.yaml (Route C) or the flat minimal contract ──
    bin_doc = doc.get("trash_bin")
    if isinstance(bin_doc, Mapping):
        center = bin_doc.get("center")
        if isinstance(center, Mapping):
            spec.bin_center = (float(center["x"]), float(center["y"]))
        elif center is not None:
            spec.bin_center = _as_xy(center, "trash_bin.center")
        approach = bin_doc.get("approach") or bin_doc.get("place_pose")
        if isinstance(approach, Mapping):
            spec.bin_approach = (
                float(approach["x"]), float(approach["y"]), float(approach.get("yaw", 0.0))
            )
        elif approach is not None:
            ax, ay = _as_xy(approach, "trash_bin.approach")
            ayaw = float(approach[2]) if len(approach) > 2 else 0.0
            spec.bin_approach = (ax, ay, ayaw)

    deliver = doc.get("deliver_chain")
    if isinstance(deliver, Mapping):
        interrupt = deliver.get("patrol_interrupt_waypoint_id")
        if interrupt is not None:
            spec.deliver_interrupt_id = str(interrupt)

    # ── forbidden areas come from map_annotations.yaml, not route.yaml ──
    if annotations:
        spec.forbidden = _parse_forbidden(annotations)
        ann_bin = (annotations.get("anchors") or {}) if isinstance(annotations, Mapping) else {}
        approach = ann_bin.get("trash_bin_approach") if isinstance(ann_bin, Mapping) else None
        if isinstance(approach, Mapping):
            ann_approach = (
                float(approach["x"]), float(approach["y"]), float(approach.get("yaw") or 0.0)
            )
            if spec.bin_approach is not None and not _poses_match(spec.bin_approach, ann_approach):
                raise ValueError(
                    "trash_bin approach point disagrees between route.yaml "
                    f"{spec.bin_approach} and map_annotations.yaml {ann_approach}; "
                    "resolve the stale copy before driving to the bin"
                )
            spec.bin_approach = ann_approach

    _validate_geometry(spec, check_spacing=check_spacing)
    return spec


def _poses_match(a: Tuple[float, float, float], b: Tuple[float, float, float],
                 tol_m: float = 0.02, tol_rad: float = 0.02) -> bool:
    return (math.hypot(a[0] - b[0], a[1] - b[1]) <= tol_m
            and abs(wrap_angle(a[2] - b[2])) <= tol_rad)


def _parse_forbidden(annotations: Mapping[str, Any]) -> List[Rect]:
    rects: List[Rect] = []
    areas = annotations.get("areas")
    if not isinstance(areas, Mapping):
        return rects
    for key in ("forbidden_areas", "patrol_only_restrictions"):
        entries = areas.get(key)
        if not isinstance(entries, Sequence):
            continue
        for entry in entries:
            if not isinstance(entry, Mapping):
                continue
            name = str(entry.get("name", key))
            bounds = entry.get("bounds")
            if isinstance(bounds, Mapping):
                rects.append(Rect(
                    name,
                    float(bounds["min_x"]), float(bounds["min_y"]),
                    float(bounds["max_x"]), float(bounds["max_y"]),
                ))
                continue
            polygon = entry.get("polygon")
            if isinstance(polygon, Sequence) and polygon:
                xs = [float(p[0]) for p in polygon]
                ys = [float(p[1]) for p in polygon]
                rects.append(Rect(name, min(xs), min(ys), max(xs), max(ys)))
    return rects


def _validate_spacing(spec: RouteSpec,
                      arrival_radius_m: float = DEFAULT_ARRIVAL_RADIUS_M) -> None:
    """Waypoints must be further apart than two arrival radii.

    Checked separately from the structural rules because re-sampling can fix a
    spacing failure, while a duplicate id or a waypoint inside a no-go zone is
    a defect in the route that no post-processing should paper over.
    """
    spacing = spec.min_spacing_m()
    if spacing <= 2.0 * arrival_radius_m:
        raise ValueError(
            f"minimum waypoint spacing {spacing:.3f} m must exceed 2 x arrival "
            f"radius ({2.0 * arrival_radius_m:.3f} m) or the robot can 'arrive' "
            f"at two waypoints at once — re-sample the route "
            f"(map_goal_provider.resample_waypoints / --resample-m)"
        )


def _validate_geometry(spec: RouteSpec,
                       arrival_radius_m: float = DEFAULT_ARRIVAL_RADIUS_M,
                       *, check_spacing: bool = True) -> None:
    """SETMOTOR_ODOM_INTEGRATION.md section 7.1 load-time checks.

    Anything wrong here must fail at BOOT, never mid-patrol.
    """
    if check_spacing:
        _validate_spacing(spec, arrival_radius_m)
    for wp in spec.waypoints:
        for rect in spec.forbidden:
            if rect.contains(wp.x, wp.y):
                raise ValueError(
                    f"waypoint {wp.id!r} at ({wp.x:.3f}, {wp.y:.3f}) lies inside "
                    f"forbidden area {rect.name!r}"
                )
    if spec.bin_approach is not None and spec.bin_center is not None:
        ax, ay, _ = spec.bin_approach
        cx, cy = spec.bin_center
        reach = math.hypot(cx - ax, cy - ay)
        # The arm's verified forward reach envelope; see CLAUDE.md / docs/calibration/arm_pose.md.
        if not 0.10 <= reach <= 0.60:
            raise ValueError(
                f"bin approach point is {reach:.3f} m from the bin centre, "
                f"outside the plausible arm placing range [0.10, 0.60] m"
            )
    if spec.deliver_interrupt_id is not None:
        spec.index_of(spec.deliver_interrupt_id)  # raises KeyError if unknown


def resample_waypoints(waypoints: List[Waypoint], spacing_m: float,
                       loop: bool = True,
                       min_gap_m: Optional[float] = None) -> List[Waypoint]:
    """Re-sample a waypoint polyline at uniform arc length.

    Route C generated route.yaml with a 4-neighbour orthogonal A* on the 0.049
    m/cell grid, so the path is a staircase: it declares 0.75 m spacing but the
    real minimum gap is 0.049 m -- one cell -- and 45 of its 117 gaps are under
    0.5 m. With a 0.25 m arrival radius the robot satisfies several waypoints at
    once and the patrol index races ahead of the robot. ``patrol_001`` and
    ``patrol_116`` are even at identical coordinates.

    Corner-preserving decimation does not help: on a staircase every cell is a
    corner (53 of 117 here), and consecutive corners are 0.049 m apart.

    Re-sampling by arc length does, and is geometrically safe: every emitted
    point lies ON a segment of the original polyline, so the route cannot leave
    the 0.35 m safe corridor it was planned inside. It only removes the
    staircase's redundant sampling.

    Arc length alone is not enough, though. Arrival is judged on STRAIGHT-LINE
    distance, and two points ``s`` apart along the path are closer than ``s``
    across a corner -- s/sqrt(2) at a right angle. So a second pass drops any
    point within ``min_gap_m`` of the last one kept (default 0.7 * spacing),
    which makes the Euclidean guarantee true by construction instead of by
    luck.
    """
    if spacing_m <= 0.0 or not math.isfinite(spacing_m):
        raise ValueError(f"resample spacing must be finite and > 0, got {spacing_m}")
    if min_gap_m is None:
        min_gap_m = 0.7 * spacing_m
    if min_gap_m <= 0.0 or not math.isfinite(min_gap_m) or min_gap_m > spacing_m:
        raise ValueError(
            f"min_gap_m must be finite and in (0, spacing_m], got {min_gap_m}")
    pts = [(w.x, w.y) for w in waypoints]
    if len(pts) < 2:
        return list(waypoints)
    if loop:
        pts.append(pts[0])

    out: List[Tuple[float, float, float]] = []
    carry = 0.0
    for i in range(1, len(pts)):
        (ax, ay), (bx, by) = pts[i - 1], pts[i]
        seg = math.hypot(bx - ax, by - ay)
        if seg <= 0.0:
            continue                       # duplicate point: nothing to walk
        heading = math.atan2(by - ay, bx - ax)
        if not out:
            out.append((ax, ay, heading))
        travelled = 0.0
        while carry + (seg - travelled) >= spacing_m:
            travelled += spacing_m - carry
            out.append((ax + (bx - ax) * travelled / seg,
                        ay + (by - ay) * travelled / seg,
                        heading))
            carry = 0.0
        carry += seg - travelled

    # Second pass: enforce the Euclidean gap that arrival actually tests.
    kept: List[Tuple[float, float, float]] = []
    for p in out:
        if kept and math.hypot(p[0] - kept[-1][0], p[1] - kept[-1][1]) < min_gap_m:
            continue
        kept.append(p)

    # On a loop the last sample can land on top of the first; drop it rather
    # than recreating the duplicate we came here to remove.
    while loop and len(kept) > 2 and \
            math.hypot(kept[-1][0] - kept[0][0], kept[-1][1] - kept[0][1]) < min_gap_m:
        kept.pop()

    width = max(3, len(str(max(len(kept) - 1, 1))))
    return [Waypoint(f"wp_{i:0{width}d}", x, y, yaw)
            for i, (x, y, yaw) in enumerate(kept)]


def route_spacing_report(spec: RouteSpec) -> dict:
    """Gap statistics, for logging what a route really looks like."""
    wps = spec.waypoints
    if len(wps) < 2:
        return {"count": len(wps), "min": float("inf"), "max": 0.0, "below_half": 0}
    pairs = list(zip(wps, wps[1:]))
    if spec.loop:
        pairs.append((wps[-1], wps[0]))
    gaps = [math.hypot(b.x - a.x, b.y - a.y) for a, b in pairs]
    return {
        "count": len(wps),
        "min": min(gaps),
        "max": max(gaps),
        "total_m": sum(gaps),
        "below_half": sum(1 for g in gaps if g < 0.5),
    }


def load_route(route_path: str, *, annotations_path: Optional[str] = None,
               arrival_radius_m: float = DEFAULT_ARRIVAL_RADIUS_M,
               resample_m: Optional[float] = None) -> RouteSpec:
    """Read route.yaml (and optionally map_annotations.yaml) from disk.

    ``resample_m`` re-spaces the patrol polyline; see ``resample_waypoints`` for
    why Route C's raw route needs it. Without it a route whose waypoints are
    closer together than 2x the arrival radius is rejected outright rather than
    driven badly.
    """
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise RuntimeError(
            "PyYAML is required to load route.yaml; run "
            "'python3 -m pip install pyyaml' inside grasp_venv"
        ) from exc

    path = Path(route_path).expanduser().resolve()
    with open(path, "r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh)

    annotations = None
    if annotations_path:
        ann_path = Path(annotations_path).expanduser().resolve()
        with open(ann_path, "r", encoding="utf-8") as fh:
            annotations = yaml.safe_load(fh)

    spec = parse_route(doc, source_path=str(path), annotations=annotations,
                       check_spacing=resample_m is None)
    if resample_m is not None:
        before = route_spacing_report(spec)
        spec.waypoints = resample_waypoints(spec.waypoints, resample_m, spec.loop)
        after = route_spacing_report(spec)
        print(f"[map-goal] re-sampled route at {resample_m:.2f} m: "
              f"{before['count']} -> {after['count']} waypoints; "
              f"min gap {before['min']:.3f} -> {after['min']:.3f} m "
              f"({before['below_half']} gaps under 0.5 m -> {after['below_half']}); "
              f"loop length {after['total_m']:.1f} m")
        if spec.deliver_interrupt_id is not None:
            # The named interrupt waypoint no longer exists; the bin approach
            # point is what actually matters and is unaffected.
            spec.deliver_interrupt_id = None
    _validate_geometry(spec, arrival_radius_m)
    return spec


# ════════════════════════════════════════════════════════════════════════════
# Math (shared convention with nav_rl.GoalTracker)
# ════════════════════════════════════════════════════════════════════════════

def wrap_angle(a: float) -> float:
    """Wrap to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def goal_features(pose: MapPose, gx: float, gy: float) -> Tuple[float, float]:
    """(dist, bearing) of a map point relative to the robot. Bearing is LEFT-positive."""
    dx, dy = gx - pose.x, gy - pose.y
    return math.hypot(dx, dy), wrap_angle(math.atan2(dy, dx) - pose.yaw)


# ════════════════════════════════════════════════════════════════════════════
# Provider
# ════════════════════════════════════════════════════════════════════════════

class MapGoalProvider:
    """AMCL pose + route waypoints -> the GoalProvider contract.

    Contract (SETMOTOR_ODOM_INTEGRATION.md section 2.5)::

        get(now) -> GoalFix{dist, bearing, source_stamp, valid, reason}
        reset()

    Two target modes share one provider so the nav loop never has to care which
    state it is in:

    * patrol   -- walk ``route.waypoints``, advancing on arrival, looping.
    * override -- an ad-hoc map target (the bin approach point during DELIVER).
      ``set_target()`` parks the patrol index so ``resume()`` continues from the
      waypoint *after* the interrupted one.
    """

    def __init__(self, route: RouteSpec, *,
                 arrival_radius_m: float = DEFAULT_ARRIVAL_RADIUS_M,
                 pose_max_age_s: float = DEFAULT_POSE_MAX_AGE_S,
                 start_index: int = 0):
        if not route.waypoints:
            raise ValueError("route has no waypoints")
        if not math.isfinite(arrival_radius_m) or arrival_radius_m <= 0.0:
            raise ValueError(f"arrival_radius_m must be > 0, got {arrival_radius_m}")
        if not math.isfinite(pose_max_age_s) or pose_max_age_s <= 0.0:
            raise ValueError(f"pose_max_age_s must be > 0, got {pose_max_age_s}")
        _validate_geometry(route, arrival_radius_m)

        self.route = route
        self.arrival_radius_m = float(arrival_radius_m)
        self.pose_max_age_s = float(pose_max_age_s)
        self._index = int(start_index) % len(route.waypoints)
        self._pose: Optional[MapPose] = None
        self._override: Optional[Tuple[float, float, Optional[float], str]] = None
        self._interrupted_index: Optional[int] = None
        self._laps = 0

    # ── pose input ──
    def set_pose(self, pose: MapPose) -> None:
        if not isinstance(pose, MapPose):
            raise TypeError(f"expected MapPose, got {type(pose).__name__}")
        if not pose.is_finite():
            raise ValueError(f"AMCL pose must be finite, got {pose}")
        self._pose = pose

    @property
    def pose(self) -> Optional[MapPose]:
        return self._pose

    def pose_age(self, now: Optional[float] = None) -> float:
        if self._pose is None:
            return float("inf")
        return (time.time() if now is None else now) - self._pose.stamp

    # ── target selection ──
    @property
    def index(self) -> int:
        return self._index

    @property
    def laps(self) -> int:
        return self._laps

    @property
    def interrupted_index(self) -> Optional[int]:
        return self._interrupted_index

    @property
    def in_override(self) -> bool:
        return self._override is not None

    def current_waypoint(self) -> Waypoint:
        return self.route.waypoints[self._index]

    def target(self) -> Tuple[float, float, Optional[float], str]:
        """(x, y, yaw_or_None, label) of whatever is currently being driven to."""
        if self._override is not None:
            return self._override
        wp = self.current_waypoint()
        return wp.x, wp.y, wp.yaw, wp.id

    def set_target(self, x: float, y: float, yaw: Optional[float] = None,
                   label: str = "override") -> None:
        """Drive to an arbitrary map point; remember where patrol was."""
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f"override target must be finite, got ({x}, {y})")
        if yaw is not None and not math.isfinite(yaw):
            raise ValueError(f"override yaw must be finite or None, got {yaw}")
        if self._override is None:
            self._interrupted_index = self._index
        self._override = (float(x), float(y), None if yaw is None else float(yaw), str(label))

    def set_bin_target(self) -> None:
        """Override to Route C's confirmed bin approach point (DELIVER entry)."""
        if self.route.bin_approach is None:
            raise RuntimeError(
                "route has no trash_bin.approach; cannot enter DELIVER without a "
                "confirmed approach pose"
            )
        bx, by, byaw = self.route.bin_approach
        self.set_target(bx, by, byaw, label="trash_bin_approach")

    def clear_target(self) -> None:
        """Drop the override without touching the patrol index."""
        self._override = None

    def resume_patrol(self) -> int:
        """Leave override mode and continue at the waypoint AFTER the interrupt.

        Section 7.2: re-walking the interrupted waypoint would drive the robot
        back over the spot it just cleared.
        """
        self._override = None
        if self._interrupted_index is not None:
            self._index = self._interrupted_index
            self._interrupted_index = None
        self._advance_index()
        return self._index

    def _advance_index(self) -> None:
        nxt = self._index + 1
        if nxt >= len(self.route.waypoints):
            if not self.route.loop:
                self._index = len(self.route.waypoints) - 1
                return
            nxt = 0
            self._laps += 1
        self._index = nxt

    def advance(self) -> int:
        """Mark the current target reached and select the next one."""
        if self._override is not None:
            self._override = None
            return self._index
        self._advance_index()
        return self._index

    def reset(self, start_index: Optional[int] = None) -> None:
        """Clear all target state. Callers must also reset ActionDelay/tracker."""
        self._override = None
        self._interrupted_index = None
        self._laps = 0
        if start_index is not None:
            self._index = int(start_index) % len(self.route.waypoints)

    def nearest_index(self, x: float, y: float) -> int:
        """Closest waypoint to a map point — used to pick the patrol start."""
        best, best_d = 0, float("inf")
        for i, wp in enumerate(self.route.waypoints):
            d = math.hypot(wp.x - x, wp.y - y)
            if d < best_d:
                best, best_d = i, d
        return best

    # ── the GoalProvider contract ──
    def get(self, now: Optional[float] = None) -> GoalFix:
        now = time.time() if now is None else now
        gx, gy, gyaw, label = self.target()

        if self._pose is None:
            return GoalFix(False, 0.0, 0.0, 0.0, "no AMCL pose received yet", label, gyaw)
        age = self.pose_age(now)
        if age > self.pose_max_age_s:
            return GoalFix(
                False, 0.0, 0.0, self._pose.stamp,
                f"AMCL pose stale ({age:.2f}s > {self.pose_max_age_s:.2f}s)", label, gyaw,
            )

        dist, bearing = goal_features(self._pose, gx, gy)
        if not (math.isfinite(dist) and math.isfinite(bearing)):
            return GoalFix(False, 0.0, 0.0, self._pose.stamp,
                           "goal features non-finite", label, gyaw)
        return GoalFix(True, dist, bearing, self._pose.stamp, "", label, gyaw)

    def arrived(self, fix: Optional[GoalFix] = None,
                now: Optional[float] = None) -> bool:
        fix = self.get(now) if fix is None else fix
        return bool(fix.valid and fix.dist <= self.arrival_radius_m)

    def in_forbidden(self, margin_m: float = 0.0) -> Optional[str]:
        """Name of the forbidden area the robot is currently inside, if any."""
        if self._pose is None:
            return None
        for rect in self.route.forbidden:
            if rect.contains(self._pose.x, self._pose.y, margin_m):
                return rect.name
        return None


# ════════════════════════════════════════════════════════════════════════════
# Selftest
# ════════════════════════════════════════════════════════════════════════════

def _toy_route(n: int = 6, spacing: float = 0.75) -> RouteSpec:
    wps = [Waypoint(f"p{i}", i * spacing, 0.0, 0.0) for i in range(n)]
    return RouteSpec(
        waypoints=wps, loop=True,
        bin_center=(1.0, 1.0), bin_approach=(1.0, 0.6, math.pi / 2.0),
        deliver_interrupt_id="p2",
    )


def run_selftest() -> None:
    def approx(a, b, tol=1e-6):
        assert abs(a - b) <= tol, f"{a} != {b} (tol {tol})"

    # ── bearing convention matches nav_rl.GoalTracker (left positive) ──
    pose = MapPose(0.0, 0.0, 0.0, stamp=100.0)
    d, b = goal_features(pose, 2.0, 0.0)
    approx(d, 2.0); approx(b, 0.0)
    _, b = goal_features(pose, 0.0, 1.0)
    approx(b, math.pi / 2.0)          # goal to the LEFT -> +bearing
    _, b = goal_features(pose, 0.0, -1.0)
    approx(b, -math.pi / 2.0)         # goal to the RIGHT -> -bearing
    # rotating the robot left by 90 deg puts a left-side goal straight ahead
    _, b = goal_features(MapPose(0.0, 0.0, math.pi / 2.0, 100.0), 0.0, 1.0)
    approx(b, 0.0)
    print("[map-goal] bearing convention OK")

    route = _toy_route()
    gp = MapGoalProvider(route, arrival_radius_m=0.25, pose_max_age_s=1.0)

    # ── no pose -> invalid, never a silent zero goal ──
    fix = gp.get(now=100.0)
    assert not fix.valid and "no AMCL pose" in fix.reason, fix
    # ── stale pose -> invalid ──
    gp.set_pose(MapPose(0.0, 0.0, 0.0, stamp=100.0))
    assert not gp.get(now=102.0).valid
    assert gp.get(now=100.5).valid
    print("[map-goal] pose freshness gating OK")

    # ── patrol advance + loop ──
    gp.set_pose(MapPose(0.0, 0.0, 0.0, stamp=200.0))
    assert gp.current_waypoint().id == "p0"
    assert gp.arrived(now=200.0)                       # sitting on p0
    gp.advance()
    assert gp.current_waypoint().id == "p1"
    for _ in range(5):
        gp.advance()
    assert gp.current_waypoint().id == "p0" and gp.laps == 1, (gp.index, gp.laps)
    print("[map-goal] patrol advance + loop OK")

    # ── interrupt / resume continues at the NEXT waypoint (section 7.2) ──
    gp.reset(start_index=2)
    assert gp.current_waypoint().id == "p2"
    gp.set_bin_target()
    assert gp.in_override and gp.target()[3] == "trash_bin_approach"
    assert gp.interrupted_index == 2
    gp.set_pose(MapPose(1.0, 0.0, 0.0, stamp=300.0))
    fix = gp.get(now=300.0)
    approx(fix.dist, 0.6, 1e-9)                        # bin approach at (1.0, 0.6)
    approx(fix.bearing, math.pi / 2.0, 1e-9)
    assert gp.resume_patrol() == 3, gp.index
    assert not gp.in_override and gp.interrupted_index is None
    print("[map-goal] interrupt/resume OK")

    # ── nearest waypoint selection ──
    assert gp.nearest_index(3.1, 0.4) == 4, gp.nearest_index(3.1, 0.4)
    print("[map-goal] nearest_index OK")

    # ── load-time validation must reject bad routes ──
    def rejects(build, needle):
        try:
            build()
        except (ValueError, KeyError) as exc:
            assert needle in str(exc), f"wrong error for {needle!r}: {exc}"
            return
        raise AssertionError(f"expected rejection containing {needle!r}")

    rejects(lambda: parse_route({"map_frame": "odom", "waypoints": [{"id": "a", "x": 0, "y": 0}]}),
            "frame must be 'map'")
    rejects(lambda: parse_route({"waypoints": [{"id": "a", "x": 0, "y": 0},
                                               {"id": "a", "x": 1, "y": 0}]}),
            "duplicate waypoint id")
    rejects(lambda: parse_route({"waypoints": [{"id": "a", "x": 0, "y": 0},
                                               {"id": "b", "x": 0.10, "y": 0}]}),
            "must exceed 2 x arrival")
    rejects(lambda: parse_route({"waypoints": [{"id": "a", "x": 0, "y": 0},
                                               {"id": "b", "x": float("nan"), "y": 0}]}),
            "finite")
    rejects(lambda: parse_route({"patrol": {"waypoint_count": 9,
                                            "waypoints": [{"id": "a", "x": 0, "y": 0},
                                                          {"id": "b", "x": 1, "y": 0}]}}),
            "waypoint_count")
    # a stale bin approach in annotations must be caught, not silently preferred
    rejects(lambda: parse_route(
        {"waypoints": [{"id": "a", "x": 0, "y": 0}, {"id": "b", "x": 1, "y": 0}],
         "trash_bin": {"center": [4.77, 13.0], "approach": [4.35, 13.0, 0.0]}},
        annotations={"anchors": {"trash_bin_approach": {"x": 5.3074, "y": 12.8906,
                                                        "yaw": 2.8896}}}),
        "disagrees between route.yaml")
    print("[map-goal] route validation OK")

    # ── forbidden areas ──
    route2 = _toy_route()
    route2.forbidden = [Rect("no_go", 2.0, -0.5, 3.0, 0.5)]
    rejects(lambda: _validate_geometry(route2), "forbidden area")
    print("[map-goal] forbidden-area rejection OK")

    # ── re-sampling a staircase route (Route C's real defect) ──
    # An L of 0.049 m steps: what a 4-neighbour orthogonal A* emits on the
    # 0.049 m/cell map. 0.75 m declared, 0.049 m actual.
    cell = 0.049
    stair = ([Waypoint(f"s{i}", i * cell, 0.0) for i in range(41)]
             + [Waypoint(f"t{i}", 40 * cell, i * cell) for i in range(1, 41)])
    raw = RouteSpec(waypoints=stair, loop=False)
    approx(raw.min_spacing_m(), cell, 1e-9)
    rejects(lambda: _validate_spacing(raw), "must exceed 2 x arrival")

    rs = resample_waypoints(stair, 0.75, loop=False)
    out = RouteSpec(waypoints=rs, loop=False)
    _validate_spacing(out, 0.25)                        # must now pass
    assert len(rs) < len(stair), (len(rs), len(stair))
    # the guarantee is Euclidean, and it must hold ACROSS the corner
    assert out.min_spacing_m() >= 0.7 * 0.75 - 1e-9, out.min_spacing_m()
    # every emitted point must lie on the original polyline (never cut a corner)
    for w in rs:
        on_leg = (abs(w.y) < 1e-9 and -1e-9 <= w.x <= 40 * cell + 1e-9) or \
                 (abs(w.x - 40 * cell) < 1e-9 and -1e-9 <= w.y <= 40 * cell + 1e-9)
        assert on_leg, f"resampled point {w} left the original path"
    assert len({w.id for w in rs}) == len(rs), "resampled ids must stay unique"
    print(f"[map-goal] staircase re-sample OK ({len(stair)} -> {len(rs)}, "
          f"min gap {raw.min_spacing_m():.3f} -> {out.min_spacing_m():.3f} m, "
          f"all points on the original path)")

    # a loop must not resample back into a duplicate first/last pair
    square = []
    for i in range(20):
        square.append(Waypoint(f"a{i}", i * 0.1, 0.0))
    for i in range(20):
        square.append(Waypoint(f"b{i}", 2.0, i * 0.1))
    for i in range(20):
        square.append(Waypoint(f"c{i}", 2.0 - i * 0.1, 2.0))
    for i in range(20):
        square.append(Waypoint(f"d{i}", 0.0, 2.0 - i * 0.1))
    loop_rs = resample_waypoints(square, 0.75, loop=True)
    ls = RouteSpec(waypoints=loop_rs, loop=True)
    assert ls.min_spacing_m() >= 0.7 * 0.75 - 1e-9, ls.min_spacing_m()
    _validate_spacing(ls, 0.25)      # 0.7 * 0.75 = 0.525 > 2 * 0.25
    print(f"[map-goal] loop re-sample keeps the wrap-around gap "
          f"({ls.min_spacing_m():.3f} m) OK")

    # the Euclidean guarantee is what makes 0.75 m safe with a 0.25 m radius;
    # a spacing whose corner chord would fall under 2r must still be caught
    tight = RouteSpec(waypoints=resample_waypoints(square, 0.5, loop=True), loop=True)
    rejects(lambda: _validate_spacing(tight, 0.25), "must exceed 2 x arrival")
    print(f"[map-goal] a too-tight re-sample is still rejected "
          f"({tight.min_spacing_m():.3f} m) OK")

    for bad in (0.0, -1.0, float("nan")):
        try:
            resample_waypoints(stair, bad)
        except ValueError:
            continue
        raise AssertionError(f"resample spacing {bad} should have been rejected")
    print("[map-goal] re-sample argument validation OK")

    print("[map-goal] SELFTEST PASSED")


def _validate_cli(args) -> int:
    ann = args.annotations
    if ann is None:
        guess = Path(args.route).resolve().parents[1] / "annotations" / "map_annotations.yaml"
        ann = str(guess) if guess.exists() else None
    spec = load_route(args.route, annotations_path=ann,
                      arrival_radius_m=args.arrival_radius,
                      resample_m=args.resample_m)
    print(f"[map-goal] route      : {spec.source_path}")
    print(f"[map-goal] annotations: {ann or '(none)'}")
    print(f"[map-goal] waypoints  : {len(spec.waypoints)}  loop={spec.loop}")
    print(f"[map-goal] spacing min: {spec.min_spacing_m():.3f} m "
          f"(arrival radius {args.arrival_radius:.3f} m)")
    print(f"[map-goal] bin center : {spec.bin_center}")
    print(f"[map-goal] bin approach: {spec.bin_approach}")
    print(f"[map-goal] interrupt wp: {spec.deliver_interrupt_id}")
    for rect in spec.forbidden:
        print(f"[map-goal] forbidden  : {rect.name} "
              f"x[{rect.min_x:.2f},{rect.max_x:.2f}] y[{rect.min_y:.2f},{rect.max_y:.2f}]")
    print("[map-goal] route VALID")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true", help="offline logic tests")
    ap.add_argument("--validate", action="store_true", help="load and check a route.yaml")
    ap.add_argument("--route", default=str(DEFAULT_ROUTE_HINT))
    ap.add_argument("--annotations", default=None,
                    help="map_annotations.yaml (default: guessed next to route.yaml)")
    ap.add_argument("--arrival-radius", type=float, default=DEFAULT_ARRIVAL_RADIUS_M)
    ap.add_argument("--resample-m", type=float, default=None,
                    help="re-space the patrol polyline at this arc length "
                         "(Route C's raw route needs ~0.75)")
    args = ap.parse_args()

    if args.selftest:
        run_selftest()
        return 0
    if args.validate:
        return _validate_cli(args)
    ap.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
