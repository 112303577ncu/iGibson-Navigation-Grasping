"""Offline patrol/pickup geometry; writes proposals, never connects to hardware.

Room coordinates start at the wall corner opposite the placement box.  A
conservative circular travel outline handles every heading. Proposals use the
raw known-free map, fitted walls and bin envelope; active files stay untouched.
"""
import argparse
import hashlib
import json
import math
from functools import lru_cache
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / 'maps/demo_box_20261018_patrol'
OUTPUT = ROOT / 'docs/planning/patrol_workspace_2026-10-22'
KOALA_SHA = 'e8766f03d0de3e48419a488f330b62e3412afab3'
RADIUS = 0.25  # operator correction, 2026-10-22; replaces the earlier 35cm bound
# Proposed upper bounds, NOT measured guarantees or AMCL covariance estimates.
BUDGET = {'localization': .04, 'wall_fit': .015,
          'grid_half_diagonal': math.sqrt(2) * .02 / 2, 'tracking_and_stop': .03}
BUFFER = .10  # round sum(BUDGET.values()) = 0.099142... upwards
TURN_DRIFT = .06
FINAL_X = .34  # latest koala, chassis-centre to sugarbox
STEP = .005
SUGARBOX_RADIUS = .04  # placement proposal assumes the box fits an 8cm diameter disc


def samples(a, b, step=STEP):
    n = max(1, int(math.ceil(math.dist(a, b) / step)))
    return [tuple(a[j] + (b[j] - a[j]) * k / n for j in range(2))
            for k in range(n + 1)]


def rect_distance(p, rect):
    x0, y0, x1, y1 = rect
    return math.hypot(max(x0 - p[0], 0., p[0] - x1),
                      max(y0 - p[1], 0., p[1] - y1))


def wall_clearance(p, size):
    return min(p[0], p[1], size[0] - p[0], size[1] - p[1])


def segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0., min(1., ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)))
    return math.dist(p, (a[0] + t * dx, a[1] + t * dy))


def inset_convex(polygon, margin):
    """Clip a CCW polygon by the inward offset of each original edge."""
    output = list(polygon)
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        distance = lambda p: (-dy * (p[0] - a[0]) + dx * (p[1] - a[1])) / length - margin
        clipped = []
        for p, q in zip(output, output[1:] + output[:1]):
            dp, dq = distance(p), distance(q)
            if dp >= -1e-10:
                clipped.append(p)
            if (dp >= 0) != (dq >= 0):
                t = dp / (dp - dq)
                clipped.append(tuple(p[j] + t * (q[j] - p[j]) for j in range(2)))
        output = clipped
    return output


class Workspace:
    def __init__(self, scene=SCENE):
        import cv2
        import numpy as np
        import yaml
        from integration.g2_patrol_policy import Policy, read_pgm
        self.scene = Path(scene)
        self.policy = Policy(str(scene))
        self.layout = self.policy.layout
        self.angle = self.layout['room_heading_rad']
        self.c, self.s = math.cos(self.angle), math.sin(self.angle)
        uv = [self._uv(p) for p in self.layout['room_polygon']]
        self.origin_uv = (min(p[0] for p in uv), min(p[1] for p in uv))
        self.size = tuple(self.layout['room_size_m'])
        # Use the entire moved-box envelope, not its unverified midpoint pose.
        u0, u1, v0, v1 = self.layout['navigation_mask_validation']['envelope_room_uv']
        ou, ov = self.origin_uv
        self.bin_rect = (u0 - ou, v0 - ov, u1 - ou, v1 - ov)
        self.route = [(.45, .45), (self.size[0] - .45, .45),
                      (self.size[0] - .45, 1.05), (1.00, 1.50), (.45, 1.50)]
        self.target_polygon = inset_convex(self.route, RADIUS + BUFFER + SUGARBOX_RADIUS)
        meta = yaml.safe_load((self.scene / 'demo_box.yaml').read_text())
        if (meta['resolution'] != self.policy.resolution
                or tuple(meta['origin']) != self.policy.origin or meta.get('negate', 0) != 0):
            raise ValueError('raw occupancy map and centre mask geometry differ')
        width, height, raw = read_pgm(self.scene / meta['image'])
        if (width, height) != (self.policy.width, self.policy.height):
            raise ValueError('raw occupancy map and centre mask dimensions differ')
        # Unknown cells remain obstacles, including under the body, not just at its centre.
        free = (np.array(raw).reshape(height, width) > 250).astype(np.uint8)
        self.map_clearance = cv2.distanceTransform(free, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        self.map_clearance *= self.policy.resolution

    def _uv(self, p):
        return (p[0] * self.c + p[1] * self.s,
                -p[0] * self.s + p[1] * self.c)

    def to_room(self, p):
        u, v = self._uv(p)
        return (u - self.origin_uv[0], v - self.origin_uv[1])

    def to_map(self, p):
        u, v = p[0] + self.origin_uv[0], p[1] + self.origin_uv[1]
        return (u * self.c - v * self.s, u * self.s + v * self.c)

    @lru_cache(maxsize=200000)
    def clear(self, p, turning=False):
        if len(p) != 2 or not all(math.isfinite(x) for x in p):
            return False
        required = RADIUS + BUFFER + (TURN_DRIFT if turning else 0)
        mapped = self.to_map(p)
        col = math.floor((mapped[0] - self.policy.origin[0]) / self.policy.resolution)
        row = self.policy.height - 1 - math.floor(
            (mapped[1] - self.policy.origin[1]) / self.policy.resolution)
        if not (0 <= col < self.policy.width and 0 <= row < self.policy.height):
            return False
        return (wall_clearance(p, self.size) >= required
                and rect_distance(p, self.bin_rect) >= required
                and self.map_clearance[row, col] >= required)

    def corridor(self, start, end):
        return all(self.clear(p) for p in samples(start, end))


class Camera:
    def __init__(self):
        import cv2
        import numpy as np
        from integration.sugarbox_ground_calibration import load_ground_calibration
        self.cv2, self.np = cv2, np
        self.H, self.offset, self.hull = load_ground_calibration(
            ROOT / 'detection/rear_ground_homography.json')
        self.inverse = np.linalg.inv(self.H)

    def inside(self, forward, left):
        from integration.sugarbox_ground_calibration import inside_calibration_hull
        # Same centre -> camera-ground origin conversion as mission_loop.
        pixel = self.inverse @ self.np.array([forward + self.offset - .135, left, 1.])
        if not self.np.isfinite(pixel).all() or abs(pixel[2]) < 1e-9:
            return False
        pixel /= pixel[2]
        return (0 <= pixel[0] < 640 and 0 <= pixel[1] < 480
                and inside_calibration_hull(pixel[0], pixel[1], self.hull, margin_px=10))


def pickup(workspace, camera, target, station):
    """An inward-facing observation, straight approach and exact reverse corridor.

    Validate all target positions in the proposed zone, rather than assuming
    an unconstrained avoidance policy will stay in this corridor.
    """
    if not workspace.clear(station, turning=True):
        return None
    dx, dy = target[0] - station[0], target[1] - station[1]
    distance = math.hypot(dx, dy)
    if not .75 <= distance <= .94:
        return None
    inward = math.atan2(workspace.size[1] / 2 - station[1], workspace.size[0] / 2 - station[0])
    bearing = math.atan2(dy, dx)
    scans = [inward + math.radians(a) for a in (-24, 0, 24)]
    scan = min(scans, key=lambda a: abs((bearing - a + math.pi) % (2 * math.pi) - math.pi))
    forward = dx * math.cos(scan) + dy * math.sin(scan)
    left = -dx * math.sin(scan) + dy * math.cos(scan)
    if abs(math.atan2(left, forward)) > math.radians(12):
        return None
    if not camera.inside(forward, left) or not camera.inside(distance, 0):
        return None
    travel = distance - FINAL_X
    if not 0 < travel <= .60:
        return None
    stop = tuple(target[j] - FINAL_X * (target[j] - station[j]) / distance for j in range(2))
    if not workspace.corridor(station, stop):
        return None
    return {'station_room_m': station, 'stop_room_m': stop,
            'station_map_m': workspace.to_map(station), 'stop_map_m': workspace.to_map(stop),
            'heading_map_rad': workspace.angle + math.atan2(dy, dx),
            'observation_heading_map_rad': workspace.angle + scan,
            'acquisition_range_m': distance, 'reverse_to_station_m': travel,
            'turn_after_pickup': False}


def build(workspace, camera):
    # CCW perimeter with the placement corner clipped; all turns occur at stations.
    for p in workspace.route:
        if not workspace.clear(p, turning=True):
            raise ValueError('observation station lacks turn clearance: ' + str(p))
    if not all(workspace.corridor(a, b) for a, b in zip(
            workspace.route, workspace.route[1:] + workspace.route[:1])):
        raise ValueError('patrol corridor is not safe')
    import cv2
    import numpy as np
    poly = np.array(workspace.target_polygon, dtype=np.float32)
    x0, x1 = min(p[0] for p in workspace.target_polygon), max(p[0] for p in workspace.target_polygon)
    y0, y1 = min(p[1] for p in workspace.target_polygon), max(p[1] for p in workspace.target_polygon)
    nx, ny = math.ceil((x1 - x0) / .01), math.ceil((y1 - y0) / .01)
    targets = list(workspace.target_polygon)
    for a, b in zip(workspace.target_polygon,
                    workspace.target_polygon[1:] + workspace.target_polygon[:1]):
        targets.extend(samples(a, b, .01))
    for i in range(nx + 1):
        for j in range(ny + 1):
            target = (x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny)
            if all((b[0] - a[0]) * (target[1] - a[1])
                   - (b[1] - a[1]) * (target[0] - a[0]) >= -1e-12
                   for a, b in zip(workspace.target_polygon,
                                   workspace.target_polygon[1:] + workspace.target_polygon[:1])):
                targets.append(target)
    plans = []
    for target in sorted(set(tuple(p) for p in targets)):
        choices = [pickup(workspace, camera, target, station) for station in workspace.route]
        choices = [p for p in choices if p is not None]
        if not choices:
            raise ValueError('no safe pickup for target ' + str(target))
        chosen = min(choices, key=lambda p: p['reverse_to_station_m'])
        plans.append(dict(chosen, target_room_m=target, target_map_m=workspace.to_map(target)))
    files = ['scene_layout.json', 'navigation_center_mask.pgm', 'navigation_center_mask.yaml',
             'demo_box.yaml', 'demo_box.pgm', 'aruco_corner_landmarks.json']
    hashes = {f: hashlib.sha256((workspace.scene / f).read_bytes()).hexdigest() for f in files}
    return {
        'schema': 1, 'status': 'offline_proposal', 'hardware_validated': False,
        'runtime_enforced': False, 'koala_commit': KOALA_SHA, 'frame_id': 'map',
        'source_sha256': hashes, 'room_size_m': workspace.size,
        'calibration_sha256': hashlib.sha256(
            (ROOT / 'detection/rear_ground_homography.json').read_bytes()).hexdigest(),
        'room_origin_map_m': workspace.to_map((0., 0.)), 'room_heading_rad': workspace.angle,
        'error_budget_m': BUDGET, 'error_bounds_verified': False,
        'travel_radius_m': RADIUS, 'translation_required_clearance_m': RADIUS + BUFFER,
        'turn_required_clearance_m': RADIUS + BUFFER + TURN_DRIFT,
        'sampling_step_m': STEP, 'target_grid_m': .01,
        'target_zone_guarantee': 'sampled_only; admit each observed target again at runtime',
        'lap_indices': [0, 1, 2, 3, 4, 0],
        'stations': [{'name': 'ABCDE'[i], 'room_m': p, 'map_m': workspace.to_map(p),
                      'observation_heading_map_rad': workspace.angle + math.atan2(
                          workspace.size[1] / 2 - p[1], workspace.size[0] / 2 - p[0]),
                      'observation_fan_offsets_deg': [-24, 0, 24]}
                     for i, p in enumerate(workspace.route)],
        'target_centres_room_polygon_m': workspace.target_polygon,
        'target_centres_map_polygon_m': [workspace.to_map(p) for p in workspace.target_polygon],
        'sugarbox_max_radius_m': SUGARBOX_RADIUS,
        'placement_zone_area_m2': cv2.contourArea(poly),
        'bin_envelope_room_rectangle_m': workspace.bin_rect,
        'sugarbox_final_chassis_distance_m': FINAL_X,
        'pickup_cases_checked': len(plans),
        'acquisition_range_m': [min(p['acquisition_range_m'] for p in plans),
                                max(p['acquisition_range_m'] for p in plans)],
        'reverse_range_m': [min(p['reverse_to_station_m'] for p in plans),
                           max(p['reverse_to_station_m'] for p in plans)],
        'pickup_cases': plans,
        'requirements_before_runtime': [
            'same swept-geometry gate on every TCP velocity owner, including PPO and E1 creep',
            'verified localization/wall/stop error bounds at the selected speed',
            'travel and holding footprints within the radius; extended-arm sweep separately checked',
            'observe at stops facing into field; continuous tangent camera view is insufficient',
            'corridor tracking with fresh pose; retreat uses recorded path and confirmed arm travel',
            'registered placement box and separately validated directional docking corridor',
        ],
    }


def write_masks(workspace, report, folder):
    import numpy as np
    import cv2
    masks = []
    policy = workspace.policy
    for turning, name in ((False, 'navigation_center_mask.proposal'),
                           (True, 'navigation_turn_mask.proposal')):
        mask = np.zeros((policy.height, policy.width), dtype=np.uint8)
        for row in range(policy.height):
            for col in range(policy.width):
                mapped = (policy.origin[0] + (col + .5) * policy.resolution,
                          policy.origin[1] + (policy.height - row - .5) * policy.resolution)
                if workspace.clear(workspace.to_room(mapped), turning):
                    mask[row, col] = 255
        _, labels = cv2.connectedComponents((mask > 0).astype(np.uint8), connectivity=8)
        route_labels = []
        for p in workspace.route:
            x, y = workspace.to_map(p)
            row = policy.height - 1 - math.floor((y - policy.origin[1]) / policy.resolution)
            col = math.floor((x - policy.origin[0]) / policy.resolution)
            route_labels.append(int(labels[row, col]))
        if len(set(route_labels)) != 1 or route_labels[0] == 0:
            raise ValueError('stations are not in the same connected mask component')
        mask[labels != route_labels[0]] = 0
        data = 'P5\n{} {}\n255\n'.format(policy.width, policy.height).encode() + mask.tobytes()
        (folder / (name + '.pgm')).write_bytes(data)
        (folder / (name + '.yaml')).write_text(
            'image: {}.pgm\nresolution: {}\norigin: {}\nnegate: 0\n'
            'occupied_thresh: 0.65\nfree_thresh: 0.196\n'.format(
                name, policy.resolution, list(policy.origin)), encoding='utf-8')
        masks.append(mask)
    old = np.array(policy.cells).reshape(policy.height, policy.width) > 250
    new = masks[0] > 250
    cell_area = policy.resolution ** 2
    report['mask_comparison'] = {
        'old_centre_area_m2': float(old.sum()) * cell_area,
        'new_centre_area_m2': float(new.sum()) * cell_area,
        'added_area_m2': float((new & ~old).sum()) * cell_area,
        'removed_area_m2': float((old & ~new).sum()) * cell_area,
        'turn_area_m2': float((masks[1] > 250).sum()) * cell_area,
        'active_files_modified': False,
        'mask_scope': 'robot centres; never use an inflated centre mask for AMCL',
    }
    return masks[0]


def plot(workspace, report, mask, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import cv2
    from matplotlib.patches import Rectangle, Circle, Polygon
    plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei', 'DejaVu Sans']
    fig, ax = plt.subplots(figsize=(9, 9))
    w, h = workspace.size
    ax.add_patch(Rectangle((0, 0), w, h, facecolor='#fce5e5', edgecolor='black', lw=2))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    res = workspace.policy.resolution
    for contour in contours:
        points = []
        for col, row in contour.reshape(-1, 2):
            points.append(workspace.to_room((workspace.policy.origin[0] + (int(col) + .5) * res,
                workspace.policy.origin[1] + (workspace.policy.height - int(row) - .5) * res)))
        ax.add_patch(Polygon(points, facecolor='#f8fafc', edgecolor='#aa4545',
                             label='新中心可行區：35cm＋原始地圖／紙盒包絡'))
    ax.add_patch(Rectangle((.4, .4), w - .8, h - .8, fill=False,
                           edgecolor='#b48838', linestyle='--', label='原距牆40cm參考線'))
    x0, y0, x1, y1 = workspace.bin_rect
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color='#b17d4a', label='投放盒保守位置包絡'))
    ax.add_patch(Polygon(workspace.target_polygon, facecolor='#85d6a5',
                         edgecolor='#187347', lw=2, label='糖果盒中心區（距巡航線至少39cm）'))
    route = workspace.route
    ax.plot(*zip(*(route + route[:1])), 'o-', color='#2259b5', lw=3, label='巡航 A→B→C→D→E→A')
    for i, p in enumerate(route):
        ax.text(p[0], p[1] - .065, 'ABCDE'[i], ha='center', weight='bold')
        bearing = report['stations'][i]['observation_heading_map_rad'] - workspace.angle
        ax.annotate('', (p[0] + .16 * math.cos(bearing), p[1] + .16 * math.sin(bearing)),
                    p, arrowprops={'arrowstyle': '->', 'color': '#2259b5'})
    example = min(report['pickup_cases'], key=lambda p: math.dist(p['target_room_m'], (.95, .95)))
    ax.plot(*zip(example['station_room_m'], example['stop_room_m']), '--', color='#753fa1',
            lw=2, label='接近／原路倒退；夾取點禁止原地轉向')
    ax.add_patch(Circle(example['stop_room_m'], RADIUS, fill=False, color='#753fa1', lw=1.5,
                        label='示例停靠車體半徑25cm'))
    ax.plot(*example['target_room_m'], '*', color='#187347', ms=12)
    ax.set(xlim=(-.06, w + .06), ylim=(-.06, h + .06), aspect='equal',
           xlabel='沿牆 u（m；左下角起算）', ylabel='沿牆 v（m）',
           title='25cm車體：離線巡航／夾取配置提案\n紅色為中心禁航區；停站向內掃視，投放近接區另驗')
    ax.set_xticks([i / 10 for i in range(0, 21, 2)])
    ax.set_yticks([i / 10 for i in range(0, 21, 2)])
    ax.grid(alpha=.2)
    ax.legend(loc='lower left', fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    args = parser.parse_args()
    workspace = Workspace()
    report = build(workspace, Camera())
    args.output.mkdir(parents=True, exist_ok=True)
    mask = write_masks(workspace, report, args.output)
    (args.output / 'plan.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    plot(workspace, report, mask, args.output / 'layout.png')
    print(json.dumps({k: report[k] for k in ('status', 'pickup_cases_checked',
                     'acquisition_range_m', 'reverse_range_m', 'runtime_enforced')}, indent=2))


if __name__ == '__main__':
    import sys
    sys.path.insert(0, str(ROOT))
    main()
