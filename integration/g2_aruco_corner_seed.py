#!/usr/bin/env python3
"""Select a LiDAR hypothesis using coarse wall-marker bearings; never move.

Default is offline selection. --real, on the Jetson only, revalidates the
resident and submits one seed, then waits for acknowledged healthy AMCL.
No OpenCV, intrinsics, Serial, arm, velocity output, or model inference.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from integration import v23_task_resume_guard as guard


class Rejected(ValueError):
    def __init__(self, status, reason):
        self.status = status
        super().__init__(reason)


def require(condition, status, reason):
    if not condition:
        raise Rejected(status, reason)


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def vector(value, n):
    require(isinstance(value, list) and len(value) == n and all(finite(v) for v in value),
            'INVALID_INPUT', 'invalid coordinate vector')
    return value


def load(path, limit=262144):
    require(Path(path).stat().st_size <= limit, 'INVALID_INPUT', 'input too large')
    def bad(_):
        raise Rejected('INVALID_INPUT', 'nonfinite JSON number')
    value = json.loads(Path(path).read_text(encoding='utf-8'), parse_constant=bad)
    require(isinstance(value, dict), 'INVALID_INPUT', 'JSON object required')
    return value


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def map_hash(path, image_path=None):
    # Identical byte contract to the resident AMCL supervisor.
    import yaml
    path = Path(path)
    meta = yaml.safe_load(path.read_text(encoding='utf-8'))
    image = Path(image_path) if image_path is not None else path.parent / meta['image']
    digest = hashlib.sha256()
    digest.update(path.read_bytes()); digest.update(image.read_bytes())
    digest.update(b'lidar_forward_m=0.10')
    return digest.hexdigest()


def checked_registry(registry):
    require(registry.get('schema') == 1 and registry.get('dictionary') == 'DICT_4X4_50',
            'INVALID_INPUT', 'wrong landmark schema/dictionary')
    require(registry.get('registration_verified') is True and registry.get('registration_id'),
            'LANDMARKS_UNVERIFIED', 'confirm actual wall ID positions before enabling')
    camera = registry['camera']
    require(camera.get('orientation_verified') is True and camera.get('fov_bound_verified') is True,
            'CAMERA_DIRECTION_UNVERIFIED', 'confirm camera direction, mirror and coarse FOV bound')
    require(isinstance(camera.get('image_mirrored'), bool) and camera.get('camera_id'),
            'INVALID_INPUT', 'camera identity/mirror missing')
    vector(camera['offset_xy_m'], 2)
    for key, lo, hi in [('horizontal_fov_bound_deg', 30, 140),
                        ('yaw_uncertainty_deg', 0, 15), ('offset_uncertainty_m', 0, .25)]:
        require(finite(camera.get(key)) and lo <= camera[key] <= hi,
                'INVALID_INPUT', 'invalid camera bound: ' + key)
    require(finite(camera.get('yaw_rad')), 'INVALID_INPUT', 'invalid camera yaw')
    markers = registry['markers']
    require(isinstance(markers, dict) and 1 <= len(markers) <= 50, 'INVALID_INPUT', 'missing marker table')
    for key, marker in markers.items():
        require(key == str(int(key)) and 0 <= int(key) < 50, 'INVALID_INPUT', 'invalid marker ID')
        vector(marker['point_map'], 2)
        require(finite(marker.get('normal_yaw_rad')) and finite(marker.get('position_uncertainty_m'))
                and 0 <= marker['position_uncertainty_m'] <= .25,
                'INVALID_INPUT', 'invalid marker location/normal bounds')
    return camera, markers


def observations(packet, snapshot, registry):
    for key in ('capture_id', 'boot_id', 'owner_pid', 'map_hash'):
        require(packet.get(key) == snapshot.get(key) and snapshot.get(key) is not None,
                'CONTEXT_MISMATCH', 'observation binding mismatch: ' + key)
    require(packet.get('schema') == 1 and packet.get('clock') == 'jetson',
            'INVALID_INPUT', 'observations require Jetson source-image timestamps')
    require(packet.get('registration_id') == registry['registration_id']
            and packet.get('camera_id') == registry['camera']['camera_id'],
            'CONTEXT_MISMATCH', 'landmark registration/camera changed')
    frames = packet['frames']
    require(isinstance(frames, list) and 3 <= len(frames) <= 10,
            'NEED_MORE_FRAMES', 'need 3..10 distinct stopped-camera frames')
    last_seq, last_stamp = -1, -float('inf')
    sets, seen = [], {}
    for frame in frames:
        seq, stamp = frame['seq'], frame['stamp']
        require(type(seq) is int and seq > last_seq and finite(stamp) and stamp > last_stamp,
                'INVALID_INPUT', 'repeated/out-of-order image sequence or timestamp')
        require(abs(stamp-snapshot['time']) <= 1.0,
                'UNSYNCHRONIZED_INPUT', 'image and stationary LiDAR capture differ by >1s')
        last_seq, last_stamp = seq, stamp
        detections = frame['markers']
        require(isinstance(detections, list) and 1 <= len(detections) <= 50,
                'INVALID_INPUT', 'invalid marker detections')
        ids = set()
        for item in detections:
            marker_id = item['id']; side = item['side']
            require(type(marker_id) is int and str(marker_id) in registry['markers']
                    and marker_id not in ids and side in ('left', 'center', 'right'),
                    'INVALID_INPUT', 'unknown/duplicate marker or invalid image side')
            ids.add(marker_id); seen.setdefault(marker_id, []).append(side)
        sets.append(ids)
    require(frames[-1]['stamp']-frames[0]['stamp'] <= 1.5,
            'UNSYNCHRONIZED_INPUT', 'image batch spans >1.5s')
    stable = set.intersection(*sets)
    require(bool(stable), 'NEED_MORE_FRAMES', 'no marker present in every source image')
    return {marker_id: seen[marker_id] for marker_id in sorted(stable)}


def bearing_matches(pose, marker, camera, side, pose_uncertainty_m=.10):
    x, y, yaw = pose
    cx, cy = camera['offset_xy_m']
    px = x + math.cos(yaw)*cx - math.sin(yaw)*cy
    py = y + math.sin(yaw)*cx + math.cos(yaw)*cy
    mx, my = marker['point_map']; dx, dy = mx-px, my-py
    distance = math.hypot(dx, dy)
    # Overlapping uncertainty intervals retain competing hypotheses. They never
    # serve as a score advantage for the nearest nominal marker location.
    uncertainty = marker['position_uncertainty_m'] + camera['offset_uncertainty_m'] + pose_uncertainty_m
    normal = marker['normal_yaw_rad']
    if -dx*math.cos(normal)-dy*math.sin(normal) < -uncertainty:
        return False
    half = math.radians(camera['horizontal_fov_bound_deg']/2)
    tolerance = math.radians(10)
    if camera['image_mirrored']:
        side = {'left': 'right', 'right': 'left', 'center': 'center'}[side]
    lo, hi = {'left': (-tolerance, half+tolerance),
              'right': (-half-tolerance, tolerance),
              'center': (-math.radians(15), math.radians(15))}[side]
    error = (math.pi if distance <= uncertainty else math.asin(uncertainty/distance))
    error += math.radians(camera['yaw_uncertainty_deg'])
    angle = guard.wrap(math.atan2(dy, dx)-yaw-camera['yaw_rad'])
    return any(angle+shift+error >= lo and angle+shift-error <= hi
               for shift in (-2*math.pi, 0, 2*math.pi))


def joint_bearings_possible(pose, markers, camera, observed, budget=100000):
    """Conservative interval cover of ONE shared camera position/yaw.

    Independent ray bounds can invent a different camera offset for every ID.
    Cover the whole permitted camera disk/yaw interval with small cells instead.
    Each cell still OVERestimates marker and position uncertainty; only discard
    a hypothesis when every covering cell is impossible. Budget exhaustion
    preserves ambiguity rather than selecting a pose on an incomplete search.
    """
    radius = camera['offset_uncertainty_m']+.10
    cells = max(1, int(math.ceil(2*radius/.05)))
    step = 2*radius/cells; cell_radius = step/math.sqrt(2)
    yaw_bound = camera['yaw_uncertainty_deg']
    yaw_cells = max(1, int(math.ceil(2*yaw_bound/2)))
    yaw_step = 2*yaw_bound/yaw_cells
    c, s = math.cos(pose[2]), math.sin(pose[2])
    cx, cy = camera['offset_xy_m']
    origin = [pose[0]+c*cx-s*cy, pose[1]+s*cx+c*cy]
    expanded = {mid: dict(markers[str(mid)], position_uncertainty_m=
                         markers[str(mid)]['position_uncertainty_m']+cell_radius) for mid in observed}
    constraints = [(mid, side) for mid, sides in observed.items() for side in set(sides)]
    view = dict(camera, offset_xy_m=[0., 0.], offset_uncertainty_m=0.,
                yaw_uncertainty_deg=yaw_step/2)
    evaluations = 0
    for ix in range(cells):
        dx = -radius+(ix+.5)*step
        for iy in range(cells):
            dy = -radius+(iy+.5)*step
            if math.hypot(dx, dy)-cell_radius > radius:
                continue
            for iz in range(yaw_cells):
                yaw = pose[2]+math.radians(-yaw_bound+(iz+.5)*yaw_step)
                candidate = [origin[0]+dx, origin[1]+dy, yaw]
                possible = True
                for mid, side in constraints:
                    evaluations += 1
                    if evaluations > budget:
                        return True
                    if not bearing_matches(candidate, expanded[mid], view, side, 0.):
                        possible = False
                        break
                if possible:
                    return True
    return False


def select(snapshot, candidates, packet, registry, snapshot_digest):
    camera, markers = checked_registry(registry)
    for key in ('capture_id', 'boot_id', 'owner_pid', 'map_hash'):
        require(snapshot.get(key) is not None, 'UNBOUND_SNAPSHOT', 'new bounded capture required: ' + key)
    require(finite(snapshot.get('time')) and candidates.get('source_stamp') == snapshot['time']
            and candidates.get('snapshot_sha256') == snapshot_digest
            and candidates.get('map_hash') == snapshot['map_hash'] == registry.get('map_hash')
            and candidates.get('lidar_forward_m') == .10,
            'CONTEXT_MISMATCH', 'scan hypotheses do not belong to this capture/map/geometry')
    require(snapshot.get('lidar_forward_m') == .10 and finite(snapshot.get('lidar_yaw_rad'))
            and abs(guard.wrap(snapshot['lidar_yaw_rad']-math.pi)) <= math.radians(1),
            'CONTEXT_MISMATCH', 'bounded capture must verify LiDAR TF geometry')
    pose_odom = vector(snapshot['odom']['pose'], 3)
    observed = observations(packet, snapshot, registry)
    rows = candidates['hypotheses']
    require(isinstance(rows, list) and 1 <= len(rows) <= 12 and type(candidates.get('points')) is int
            and candidates['points'] >= 100, 'INVALID_INPUT', 'insufficient/invalid wall hypotheses')
    remaining, evaluated = [], []
    for index, row in enumerate(rows):
        pose = vector(row['pose'], 3)
        require(all(finite(row.get(k)) for k in ('score_m', 'wall_p90_m', 'within_5cm_fraction')),
                'INVALID_INPUT', 'invalid LiDAR quality metrics')
        # The fitter scores its best 85% of wall observations. A bootstrap
        # hypothesis must actually place at least that fraction within 5cm;
        # a low trimmed mean alone can conceal a large mismatching wall tail.
        quality = (0 <= row['score_m'] <= .04 and 0 <= row['wall_p90_m'] <= .10
                   and .85 <= row['within_5cm_fraction'] <= 1)
        compatible = all(bearing_matches(pose, markers[str(mid)], camera, side)
                         for mid, sides in observed.items() for side in sides)
        if compatible:
            compatible = joint_bearings_possible(pose, markers, camera, observed)
        evaluated.append(dict(index=index, wall_quality_ok=quality, landmark_compatible=compatible))
        if compatible and quality:
            remaining.append(index)
    require(bool(remaining), 'NO_MATCH', 'no hypothesis agrees with stable markers and wall quality')
    # Multiple fits within one already-distinguished pose are harmless duplicates;
    # keep truly separated alternatives ambiguous, including wrap at +/-pi.
    best = min(remaining, key=lambda i: rows[i]['score_m'])
    pose = rows[best]['pose']
    require(all(guard.distance(pose, rows[i]['pose']) <= .05
                and abs(guard.wrap(pose[2]-rows[i]['pose'][2])) <= math.radians(3) for i in remaining),
            'AMBIGUOUS', 'multiple distinct corner/heading hypotheses remain; observe another stopped view')
    seed = dict(stamp=snapshot['time'], pose=pose, odom=pose_odom,
                boot_id=snapshot['boot_id'], owner_pid=snapshot['owner_pid'], map_hash=snapshot['map_hash'],
                source='lidar_with_coarse_aruco_disambiguation', capture_id=snapshot['capture_id'],
                registration_id=registry['registration_id'], camera_id=camera['camera_id'])
    return dict(ok=True, status='CANDIDATE_SELECTED', selected_index=best, seed=seed,
                stable_marker_ids=list(observed), candidates=evaluated, localized=False,
                arm_or_motion_executed=False)


def current_snapshot():
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    health = guard.resident_health()
    owners = subprocess_owners()
    return health, boot, guard.available_mb(), owners


def subprocess_owners():
    import subprocess
    result = subprocess.run(['fuser', '/dev/myserial'], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            timeout=2, universal_newlines=True)
    return result.stdout.split() if result.returncode == 0 else []


def check_live(capture, health, boot, ram, owners, now):
    require(boot == capture['boot_id'] and health.get('service_pid') == capture['owner_pid']
            and owners == [str(capture['owner_pid'])], 'CONTEXT_CHANGED', 'boot/sole Serial owner changed')
    require(0 <= now-capture['time'] <= 120, 'CAPTURE_EXPIRED', 'new stopped capture required')
    require(ram >= 1024, 'WAIT_MEMORY', 'need 1024 MB available before localization')
    require(health.get('ok') is True and health.get('chassis_tcp_ready') is True
            and guard.task_ready(health), 'WAIT_STOPPED_TRAVEL', 'resident must be stopped, travel and unclaimed')
    chassis, odom = health['chassis'], health['odom']
    require(finite(chassis.get('board_rx_age_s')) and 0 <= chassis['board_rx_age_s'] <= .2
            and odom.get('valid') is True and odom.get('failures') == 0
            and all(abs(v) <= .01 for v in vector(odom['twist'], 3)),
            'STALE_FEEDBACK', 'fresh stationary board/odom feedback required')
    old, pose = vector(capture['odom']['pose'], 3), vector(odom['pose'], 3)
    require(chassis['stopped_for_s'] >= now-capture['time']-.1
            and guard.distance(old, pose) <= .01 and abs(guard.wrap(pose[2]-old[2])) <= math.radians(1),
            'CAPTURE_MOVED', 'robot moved after capture; recapture both image and LiDAR')


def wait_applied(seed, report, loc_path, seconds, output, snapshot_fn=current_snapshot,
                 clock=time.monotonic, wall=time.time, sleep=time.sleep):
    checkpoint = dict(schema=1, phase='idle', task_id=seed['capture_id'], next_task='patrol',
                      stamp=seed['stamp'], boot_id=seed['boot_id'], owner_pid=seed['owner_pid'],
                      map_hash=seed['map_hash'], start_pose=seed['pose'], start_odom=seed['odom'])
    gate = guard.ResumeGate(checkpoint); deadline = clock()+seconds
    while clock() < deadline:
        health, boot, ram, owners = snapshot_fn(); now = wall()
        require(boot == seed['boot_id'] and health.get('service_pid') == seed['owner_pid']
                and owners == [str(seed['owner_pid'])], 'CONTEXT_CHANGED', 'owner/boot changed after seed')
        require(ram >= 768, 'WAIT_MEMORY', 'localization RAM pressure')
        require(guard.task_ready(health), 'WAIT_STOPPED_TRAVEL', 'robot claimed/moved while applying seed')
        require(health['chassis']['stopped_for_s'] >= now-seed['stamp']-.1,
                'CAPTURE_MOVED', 'robot moved since the selected stationary capture')
        try:
            loc = load(loc_path)
            if loc.get('accepted_reference_stamp') == seed['stamp']:
                require(loc.get('map_hash') == seed['map_hash'], 'CONTEXT_CHANGED', 'AMCL map changed')
                result = gate.poll(health, loc, boot, now)
                if result['ok']:
                    pose = result['pose']
                    expected = seed['pose']
                    require(guard.distance(pose, expected) <= .08
                            and abs(guard.wrap(pose[2]-expected[2])) <= math.radians(8),
                            'POSE_MISMATCH', 'AMCL did not settle near selected LiDAR pose')
                    return dict(report, status='LOCALIZED', localized=True, pose=pose,
                                valid_until=result['valid_until'], next_action='plan from current pose')
            else:
                gate.reset()
        except (OSError, KeyError, TypeError, ValueError) as exc:
            if isinstance(exc, Rejected):
                raise
            gate.reset()
        sleep(.1)
    return dict(report, ok=False, status='AMCL_TIMEOUT', localized=False,
                next_action='inspect health reason; do not resubmit this capture or start patrol')


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('snapshot', 'candidates', 'observations', 'landmarks', 'output'):
        p.add_argument('--'+name, required=True)
    p.add_argument('--real', action='store_true')
    p.add_argument('--trusted-capture', default='/dev/shm/g2_corner_capture.json')
    p.add_argument('--map', default=str(ROOT/'maps/demo_box_20261017/demo_box.yaml'))
    p.add_argument('--seed', default='/dev/shm/g2_localization_seed.json')
    p.add_argument('--health', default='/dev/shm/g2_localization_health.json')
    p.add_argument('--receipt', default='/dev/shm/g2_corner_seed_receipt.json')
    p.add_argument('--seconds', type=float, default=20)
    args = p.parse_args(argv)
    require_paths = [getattr(args, name) for name in ('snapshot', 'candidates', 'observations', 'landmarks',
                                                    'trusted_capture', 'map', 'seed', 'health', 'receipt')]
    if Path(args.output).resolve() in {Path(v).resolve() for v in require_paths}:
        p.error('output must not overwrite an input/seed/receipt')
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 30:
        p.error('seconds must be 1..30')
    immutable = {Path(getattr(args, name)).resolve() for name in
                 ('snapshot', 'candidates', 'observations', 'landmarks', 'trusted_capture', 'map', 'health')}
    if (Path(args.seed).resolve() in immutable or Path(args.receipt).resolve() in immutable
            or Path(args.seed).resolve() == Path(args.receipt).resolve()):
        p.error('seed/receipt must be distinct from input and health files')
    report = dict(ok=False, localized=False, seed_written=False, arm_or_motion_executed=False)
    lock = None
    try:
        snapshot = load(args.snapshot); candidates = load(args.candidates)
        packet = load(args.observations); registry = load(args.landmarks)
        report.update(select(snapshot, candidates, packet, registry, file_hash(args.snapshot)))
        if not args.real:
            report.update(status='SELECTED_DRY_RUN', next_action='no seed written; real apply requires live Jetson checks')
        else:
            require(registry.get('simulation_only') is not True,
                    'SIMULATION_ONLY', 'replay/test landmark tables cannot submit a real seed')
            import fcntl
            lock = open('/tmp/g2_corner_seed.lock', 'a')
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            require(file_hash(args.trusted_capture) == file_hash(args.snapshot),
                    'CONTEXT_CHANGED', 'only the latest Jetson-owned capture may be submitted')
            require(map_hash(args.map) == snapshot['map_hash'], 'CONTEXT_CHANGED', 'live raw-map file changed')
            health, boot, ram, owners = current_snapshot()
            check_live(snapshot, health, boot, ram, owners, time.time())
            seed_digest = hashlib.sha256(json.dumps(report['seed'], sort_keys=True, allow_nan=False).encode('utf-8')).hexdigest()
            submitted = False
            if Path(args.receipt).exists():
                receipt = load(args.receipt)
                if receipt.get('capture_id') == snapshot['capture_id']:
                    require(receipt.get('seed_digest') == seed_digest and Path(args.seed).exists()
                            and load(args.seed) == report['seed'],
                            'SUBMISSION_UNCERTAIN', 'previous submission is not confirmed on disk; never replay it')
                    submitted = True
                    report['seed_already_submitted'] = True
            # Record intent first. If writing the seed fails, this capture stays
            # consumed instead of risking an unknown duplicate after a restart.
            if not submitted:
                guard.atomic(args.receipt, dict(capture_id=snapshot['capture_id'], boot_id=boot,
                                               stamp=snapshot['time'], seed_digest=seed_digest,
                                               state='submission_started'))
                guard.atomic(args.seed, report['seed']); report['seed_written'] = True
            report.update(ok=False, status='WAIT_AMCL_ACK')
            guard.atomic(args.output, report)
            report = wait_applied(report['seed'], report, args.health, args.seconds, args.output)
            report['ok'] = report.get('localized') is True
    except (Exception, KeyboardInterrupt) as exc:
        report.update(ok=False, localized=False, status=getattr(exc, 'status', 'CHECK_FAILED'),
                      reason=str(exc) or 'interrupted', next_action='keep stopped; inspect reason or acquire another view')
    finally:
        if lock:
            lock.close()
    guard.atomic(args.output, report); print(json.dumps(report, allow_nan=False))
    return 0 if report['ok'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
