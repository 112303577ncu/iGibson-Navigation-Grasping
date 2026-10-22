"""Reversible placement poses, independent of ROS/models/actuators on import."""
import base64
import json
import math
from pathlib import Path
import time

MAX_BACK = .60
TURN_RING = .07
WALL_MARGIN = .08


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def half_depth(box, yaw, fallback):
    """Support of a rotated rectangular box along the approach direction."""
    names = ('orientation_rad', 'length_m', 'width_m')
    if all(name in box for name in names):
        angle, length, width = [box[name] for name in names]
        if (not all(type(v) in (float, int) and math.isfinite(v) for v in (angle, length, width))
                or not 0 < length <= .5 or not 0 < width <= .5):
            raise ValueError('invalid placement-box dimensions/orientation')
        return (abs(math.cos(yaw - angle)) * length + abs(math.sin(yaw - angle)) * width) / 2
    return fallback


def turn_ready(mask, pose):
    return mask.allowed(*pose[:2]) and all(mask.allowed(
        pose[0] + radius * math.cos(k * math.pi / 8),
        pose[1] + radius * math.sin(k * math.pi / 8))
        for radius in (.06, TURN_RING) for k in range(16))


class Grid:
    def __init__(self, record):
        self.width, self.height = record['width'], record['height']
        self.resolution, self.origin = record['resolution'], record['origin']
        self.cells = base64.b64decode(record['cells'], validate=True)
        if (not isinstance(self.width, int) or not isinstance(self.height, int)
                or not 0 < self.width * self.height <= 4000000
                or len(self.cells) != self.width * self.height or self.resolution <= 0):
            raise ValueError('invalid placement mask')

    def allowed(self, x, y):
        col = int(math.floor((x - self.origin[0]) / self.resolution))
        row = self.height - 1 - int(math.floor((y - self.origin[1]) / self.resolution))
        return (0 <= col < self.width and 0 <= row < self.height
                and self.cells[row * self.width + col] > 250)


def encode_mask(mask):
    return dict(width=mask.width, height=mask.height, resolution=mask.resolution,
                origin=list(mask.origin), cells=base64.b64encode(bytes(mask.cells)).decode('ascii'))


class Geometry:
    def __init__(self, mask, walls, obstacles, dock):
        if not walls or not obstacles:
            raise ValueError('placement needs room walls and a fixed-bin envelope')
        self.mask, self.walls, self.obstacles, self.dock = mask, walls, obstacles, dock

    def wall_clearance(self, pose):
        return min(self.dock.wall_distance(self.dock.to_map(pose, (x, y)), self.walls)
                   for x1, y1, x2, y2 in self.dock.BODY_RECTANGLES
                   for x, y in ((x1, y1), (x1, y2), (x2, y1), (x2, y2)))

    def exit(self, pose):
        """Find an entire directional return path, not merely a legal endpoint."""
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            raise ValueError('invalid placement pose')
        if self.wall_clearance(pose) < WALL_MARGIN:
            return None
        for i in range(61):
            d = i * .01
            point = [pose[0] - d * math.cos(pose[2]), pose[1] - d * math.sin(pose[2]), pose[2]]
            if turn_ready(self.mask, point) and not self.dock.back_path_problem(
                    pose, d, self.walls, obstacles=self.obstacles):
                return dict(distance_m=d, point=point)
        return None

    def choose(self, box, current, final_x, aim_y, exclude=()):
        """Safest sampled heading first, then shorter reverse/transit distances.

        This is a 5-degree search, not a claim of a continuous global optimum.
        Vision arrival/release checks remain responsible for the actual box.
        """
        candidates = []
        for i in range(72):
            yaw = -math.pi + i * math.pi / 36
            if any(abs(wrap(yaw - bad)) < math.radians(10) for bad in exclude):
                continue
            c, s = math.cos(yaw), math.sin(yaw)
            depth = half_depth(box, yaw, self.dock.BOX_DEPTH_M / 2)
            def at(distance):
                return [box['x'] - distance * c + aim_y * s,
                        box['y'] - distance * s - aim_y * c, yaw]
            release, staging = at(final_x + depth), at(1.0)
            if not turn_ready(self.mask, staging):
                continue
            result = self.exit(release)
            if result is None:
                continue
            # Do not rely on one exact stop or one exact yaw. These local
            # perturbations supplement, not replace, measured error bounds.
            samples = [[release[0] + dx, release[1] + dy, yaw + angle]
                       for dx in (-.04, 0., .04) for dy in (-.04, 0., .04)
                       for angle in (-math.radians(8), 0., math.radians(8))]
            exits = [self.exit(p) for p in samples]
            if any(e is None for e in exits):
                continue
            # Every point from the acquisition station to the stop must retain
            # a valid straight return. A later camera correction is checked live.
            length = math.hypot(staging[0] - release[0], staging[1] - release[1])
            corridor = [at(final_x + depth + length * k / 20) for k in range(21)]
            if any(self.exit(p) is None for p in corridor):
                continue
            clearance = min(self.wall_clearance(p) for p in samples)
            record = dict(staging=staging, release=release, exit=result,
                          wall_clearance_m=clearance, yaw_rad=yaw,
                          worst_retreat_m=max(e['distance_m'] for e in exits),
                          projected_box_half_depth_m=depth,
                          sampled_stop_error_m=.04, sampled_yaw_error_deg=8.,
                          hardware_validated=False)
            transit = math.hypot(staging[0] - current[0], staging[1] - current[1])
            candidates.append(((-clearance, record['worst_retreat_m'], transit, i), record))
        if not candidates:
            raise ValueError('NO_REVERSIBLE_DOCK: no sampled placement direction has a turn-clear return')
        return [item[1] for item in sorted(candidates, key=lambda item: item[0])]

    def command_problem(self, plan, pose, vx, wz, measured_vx=0., measured_wz=0.):
        if not all(math.isfinite(v) for v in list(pose) + [vx, wz]):
            return 'nonfinite command/pose'
        if not -.08 <= vx <= .15 or abs(wz) > 1.0:
            return 'placement speed limit'
        if self.exit(pose) is None:
            return 'current pose has no certified straight return'
        if not turn_ready(self.mask, pose):
            if abs(wz) > .02:
                return 'no turning in the close placement corridor'
            if abs(wrap(pose[2] - plan['yaw_rad'])) > math.radians(8):
                return 'placement heading left its return corridor'
        # Include a 0.6s command/watchdog horizon. Check arcs as well as forward
        # projections; every future pose must retain a bounded way back.
        x, y, yaw = pose
        # Match the native patrol's conservative translation bound, rather
        # than assuming requested m/s equals floor speed before calibration.
        predicted_vx = math.copysign(max(2.4 * abs(vx), abs(measured_vx)), vx) if vx else measured_vx
        predicted_wz = math.copysign(max(abs(wz), abs(measured_wz)), wz) if wz else measured_wz
        for _ in range(12):
            x += predicted_vx * .05 * math.cos(yaw + predicted_wz * .025)
            y += predicted_vx * .05 * math.sin(yaw + predicted_wz * .025)
            yaw = wrap(yaw + predicted_wz * .05)
            if self.exit([x, y, yaw]) is None:
                return 'predicted motion would lose the return corridor'
            if not turn_ready(self.mask, [x, y, yaw]) and (
                    abs(predicted_wz) > .02 or abs(wrap(yaw - plan['yaw_rad'])) > math.radians(8)):
                return 'predicted turn outside the patrol turning region'
        return None


class Permit:
    """Worker-side read-only gate; zero commands always pass, stale data pauses."""
    def __init__(self, path, dock=None):
        if dock is None:
            from integration import placement_dock as dock
        self.path = Path(path)
        contract = json.loads(self.path.read_text(encoding='utf-8'))
        if contract.get('schema') != 1:
            raise ValueError('invalid placement contract')
        self.contract = contract
        self.geometry = Geometry(Grid(contract['mask']), contract['walls'], contract['obstacles'], dock)
        self.reason = None

    def allows(self, vx, wz, clock=time.monotonic):
        if vx == 0. and wz == 0.:
            return True
        try:
            state = json.loads(Path(self.contract['state_file']).read_text(encoding='utf-8'))
            age = clock() - state['received_at'] + state['source_age_s']
            if (state.get('ready') is not True or not 0 <= age <= .4
                    or state.get('context') != self.contract['context']):
                raise ValueError('placement localization unavailable/stale/context changed')
            measured, angular = state.get('measured_vx'), state.get('measured_wz')
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in (measured, angular)):
                raise ValueError('invalid measured placement velocity')
            if measured * vx < -.0001:
                raise ValueError('stop before reversing placement motion')
            self.reason = self.geometry.command_problem(self.contract['plan'], state['pose'], vx, wz, measured, angular)
            return self.reason is None
        except (OSError, ValueError, KeyError, TypeError, OverflowError) as exc:
            self.reason = str(exc)
            return False

    def limit(self, vx, wz):
        if self.allows(vx, wz):
            return vx, wz
        # Reduce only forward speed; never invent a turn, reverse direction or
        # increase an action to force progress. A stale/context fault admits none.
        if vx > 0:
            for factor in (.75, .5, .25, .125):
                slower = vx * factor
                if slower >= .015 and self.allows(slower, wz):
                    return slower, wz
        return 0., 0.
