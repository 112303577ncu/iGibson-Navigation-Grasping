import copy
import json
import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from integration import g2_aruco_corner_seed as tool


def fixture():
    snapshot = dict(time=100., capture_id='capture', boot_id='boot', owner_pid=7,
                    map_hash='map', odom={'pose': [0., 0., 0.]}, lidar_forward_m=.10, lidar_yaw_rad=math.pi)
    camera = dict(camera_id='rear', orientation_verified=True, fov_bound_verified=True,
                  image_mirrored=False, yaw_rad=0., yaw_uncertainty_deg=0.,
                  offset_xy_m=[0., 0.], offset_uncertainty_m=0., horizontal_fov_bound_deg=90.)
    registry = dict(schema=1, dictionary='DICT_4X4_50', registration_verified=True,
                    registration_id='measured-walls', map_hash='map', camera=camera,
                    markers={'0': dict(point_map=[2., 1.5], normal_yaw_rad=math.pi, position_uncertainty_m=0.)})
    packet = dict(schema=1, capture_id='capture', boot_id='boot', owner_pid=7, map_hash='map',
                  clock='jetson', registration_id='measured-walls', camera_id='rear',
                  frames=[dict(seq=i, stamp=99.8+i*.1, markers=[{'id': 0, 'side': 'left'}]) for i in range(3)])
    candidates = dict(source_stamp=100., snapshot_sha256='digest', map_hash='map',
                      lidar_forward_m=.1, points=200, hypotheses=[
                          dict(pose=[1., 1., yaw], score_m=.01, wall_p90_m=.03,
                               within_5cm_fraction=.95) for yaw in (0., math.pi/2, math.pi, -math.pi/2)])
    return snapshot, candidates, packet, registry


def health():
    return dict(ok=True, chassis_tcp_ready=True, service_pid=7,
                chassis=dict(motors=[0, 0, 0, 0], moving=False, arm_busy=False,
                             arm_pose='travel', client=None, stopped_for_s=10., board_rx_age_s=.01),
                odom=dict(pose=[0., 0., 0.], twist=[0., 0., 0.], valid=True, failures=0))


def test_selects_landmark_compatible_pose_instead_of_best_score():
    snap, candidates, packet, registry = fixture()
    candidates['hypotheses'][1]['score_m'] = .001
    result = tool.select(snap, candidates, packet, registry, 'digest')
    assert result['selected_index'] == 0 and not result['localized']
    assert result['seed']['stamp'] == 100. and result['seed']['odom'] == [0., 0., 0.]


def test_more_than_four_hypotheses_is_supported_without_hardcoded_corner_count():
    s, c, p, r = fixture()
    c['hypotheses'].append(dict(c['hypotheses'][2], pose=[.7, 1., math.pi]))
    assert tool.select(s, c, p, r, 'digest')['selected_index'] == 0


@pytest.mark.parametrize('key,value', [('registration_verified', False), ('registration_id', None)])
def test_unregistered_installation_diagram_is_not_an_absolute_reference(key, value):
    s, c, p, r = fixture(); r[key] = value
    with pytest.raises(tool.Rejected, match='actual wall'):
        tool.select(s, c, p, r, 'digest')


@pytest.mark.parametrize('field,value', [('orientation_verified', False), ('fov_bound_verified', False),
                                        ('horizontal_fov_bound_deg', 200), ('image_mirrored', None)])
def test_camera_direction_bounds_required_without_requiring_intrinsics(field, value):
    s, c, p, r = fixture(); r['camera'][field] = value
    with pytest.raises(tool.Rejected):
        tool.select(s, c, p, r, 'digest')


def test_mirrored_image_and_camera_backwards_mount_are_explicit():
    s, c, p, r = fixture(); r['camera']['image_mirrored'] = True
    for frame in p['frames']:
        frame['markers'][0]['side'] = 'right'
    assert tool.select(s, c, p, r, 'digest')['selected_index'] == 0
    r['camera']['image_mirrored'] = False; r['camera']['yaw_rad'] = math.pi
    for frame in p['frames']:
        frame['markers'][0]['side'] = 'left'
    assert tool.select(s, c, p, r, 'digest')['selected_index'] == 2


def test_distinct_plausible_locations_remain_ambiguous():
    s, c, p, r = fixture()
    c['hypotheses'].append(dict(c['hypotheses'][0], pose=[.8, 1., 0.]))
    with pytest.raises(tool.Rejected) as caught:
        tool.select(s, c, p, r, 'digest')
    assert caught.value.status == 'AMBIGUOUS'


def test_same_pose_duplicate_fit_and_yaw_wrap_are_one_solution():
    s, c, p, r = fixture()
    c['hypotheses'].append(dict(c['hypotheses'][0], pose=[1.01, 1., 2*math.pi+.01]))
    assert tool.select(s, c, p, r, 'digest')['selected_index'] == 0


def test_large_near_marker_uncertainty_preserves_alternatives():
    s, c, p, r = fixture()
    for row in c['hypotheses']:
        row['pose'][:2] = [1.9, 1.5]
    r['markers']['0']['position_uncertainty_m'] = .25
    r['camera']['offset_uncertainty_m'] = .25
    with pytest.raises(tool.Rejected) as caught:
        tool.select(s, c, p, r, 'digest')
    assert caught.value.status == 'AMBIGUOUS'


def test_shared_camera_interval_cover_does_not_discard_feasible_observations():
    import random
    rng = random.Random(19)
    _, _, _, registry = fixture()
    view = dict(registry['camera'], offset_uncertainty_m=.2, yaw_uncertainty_deg=10)
    nominal = [1., 1., 0.]
    for _ in range(30):
        camera_x, camera_y = 1+rng.uniform(-.1, .1), 1+rng.uniform(-.1, .1)
        yaw = rng.uniform(-math.radians(10), math.radians(10))
        markers, seen = {}, {}
        for mid, bearing in enumerate([-.4, 0., .4]):
            x, y = camera_x+math.cos(bearing+yaw), camera_y+math.sin(bearing+yaw)
            markers[str(mid)] = dict(point_map=[x+rng.uniform(-.1, .1), y+rng.uniform(-.1, .1)],
                                     position_uncertainty_m=.2, normal_yaw_rad=math.pi)
            seen[mid] = ['right' if bearing < -.1 else ('left' if bearing > .1 else 'center')]
        assert tool.joint_bearings_possible(nominal, markers, view, seen)


def test_shared_camera_budget_exhaustion_keeps_ambiguity():
    _, _, _, registry = fixture()
    assert tool.joint_bearings_possible([1., 1., math.pi], registry['markers'], registry['camera'],
                                        {0: ['left']}, budget=0)


@pytest.mark.parametrize('key,value', [('capture_id', 'old'), ('owner_pid', 8), ('boot_id', 'old'),
                                     ('map_hash', 'old'), ('camera_id', 'other'),
                                     ('registration_id', 'old'), ('clock', 'pc')])
def test_observation_bindings_never_mix_boots_owners_cameras_or_clock_domains(key, value):
    s, c, p, r = fixture(); p[key] = value
    with pytest.raises(tool.Rejected):
        tool.select(s, c, p, r, 'digest')


@pytest.mark.parametrize('key,value', [('snapshot_sha256', 'old'), ('source_stamp', 99.),
                                     ('map_hash', 'old'), ('lidar_forward_m', .13), ('points', 50)])
def test_old_match_files_or_wrong_geometry_are_rejected(key, value):
    s, c, p, r = fixture(); c[key] = value
    with pytest.raises(tool.Rejected):
        tool.select(s, c, p, r, 'digest')


@pytest.mark.parametrize('change', ['few', 'same_seq', 'same_stamp', 'future', 'unknown', 'duplicate', 'bad_side'])
def test_requires_actual_distinct_fresh_consistent_source_frames(change):
    s, c, p, r = fixture()
    if change == 'few': p['frames'].pop()
    if change == 'same_seq': p['frames'][1]['seq'] = 0
    if change == 'same_stamp': p['frames'][1]['stamp'] = p['frames'][0]['stamp']
    if change == 'future': p['frames'][-1]['stamp'] = 102.
    if change == 'unknown': p['frames'][0]['markers'][0]['id'] = 9
    if change == 'duplicate': p['frames'][0]['markers'].append({'id': 0, 'side': 'left'})
    if change == 'bad_side': p['frames'][0]['markers'][0]['side'] = 'up'
    with pytest.raises(tool.Rejected):
        tool.select(s, c, p, r, 'digest')


def test_ids_must_persist_across_all_three_frames():
    s, c, p, r = fixture()
    r['markers']['1'] = copy.deepcopy(r['markers']['0'])
    p['frames'][-1]['markers'][0]['id'] = 1
    with pytest.raises(tool.Rejected) as caught:
        tool.select(s, c, p, r, 'digest')
    assert caught.value.status == 'NEED_MORE_FRAMES'


def test_bad_wall_fit_never_becomes_a_seed():
    s, c, p, r = fixture(); c['hypotheses'][0]['within_5cm_fraction'] = .4
    with pytest.raises(tool.Rejected) as caught:
        tool.select(s, c, p, r, 'digest')
    assert caught.value.status == 'NO_MATCH'


def test_low_trimmed_score_does_not_hide_a_seventeen_percent_wall_mismatch():
    s, c, p, r = fixture()
    c['hypotheses'].append(dict(c['hypotheses'][0], pose=[.8, 1., 0.],
                                score_m=.001, within_5cm_fraction=.83))
    assert tool.select(s, c, p, r, 'digest')['selected_index'] == 0


@pytest.mark.parametrize('change', ['owner', 'boot', 'two_owners', 'ram', 'expired', 'future',
                                    'moving', 'arm_busy', 'client', 'stale_board', 'odom',
                                    'twist', 'translation', 'angle', 'moved_returned'])
def test_real_submission_checks_live_resident_not_just_packet(change):
    s, _, _, _ = fixture(); h = health(); boot = 'boot'; ram = 1500; owners = ['7']; now = 101.
    if change == 'owner': h['service_pid'] = 8
    if change == 'boot': boot = 'new'
    if change == 'two_owners': owners.append('8')
    if change == 'ram': ram = 900
    if change == 'expired': now = 221.
    if change == 'future': now = 99.
    if change == 'moving': h['chassis']['moving'] = True
    if change == 'arm_busy': h['chassis']['arm_busy'] = True
    if change == 'client': h['chassis']['client'] = 'other'
    if change == 'stale_board': h['chassis']['board_rx_age_s'] = 1.
    if change == 'odom': h['odom']['valid'] = False
    if change == 'twist': h['odom']['twist'][0] = .2
    if change == 'translation': h['odom']['pose'][0] = .02
    if change == 'angle': h['odom']['pose'][2] = .03
    if change == 'moved_returned': now = 102.; h['chassis']['stopped_for_s'] = .5
    with pytest.raises(tool.Rejected):
        tool.check_live(s, h, boot, ram, owners, now)


def write_inputs(tmp_path):
    s, c, p, r = fixture(); paths = []
    for name, value in [('snapshot', s), ('candidates', c), ('observations', p), ('landmarks', r)]:
        path = tmp_path/(name+'.json'); path.write_text(json.dumps(value)); paths += ['--'+name, str(path)]
    c['snapshot_sha256'] = tool.file_hash(tmp_path/'snapshot.json')
    (tmp_path/'candidates.json').write_text(json.dumps(c))
    return paths + ['--output', str(tmp_path/'output.json')]


def test_default_cli_does_not_read_robot_or_write_seed(tmp_path, monkeypatch):
    args = write_inputs(tmp_path)
    monkeypatch.setattr(tool, 'current_snapshot', lambda: pytest.fail('offline means no robot RPC'))
    assert tool.main(args) == 0
    result = json.loads((tmp_path/'output.json').read_text())
    assert result['status'] == 'SELECTED_DRY_RUN' and not result['seed_written'] and not result['localized']


def test_simulation_registry_cannot_be_used_for_real_submission(tmp_path, monkeypatch):
    args = write_inputs(tmp_path)
    path = tmp_path/'landmarks.json'; registry = json.loads(path.read_text())
    registry['simulation_only'] = True; path.write_text(json.dumps(registry))
    monkeypatch.setattr(tool, 'current_snapshot', lambda: pytest.fail('simulation must stop before live RPC'))
    assert tool.main(args+['--real']) == 2
    assert json.loads((tmp_path/'output.json').read_text())['status'] == 'SIMULATION_ONLY'


def test_output_and_seed_cannot_overwrite_inputs(tmp_path):
    args = write_inputs(tmp_path); before = (tmp_path/'snapshot.json').read_bytes()
    with pytest.raises(SystemExit):
        tool.main(args + ['--seed', str(tmp_path/'snapshot.json')])
    with pytest.raises(SystemExit):
        tool.main(args + ['--output', str(tmp_path/'snapshot.json')])
    assert (tmp_path/'snapshot.json').read_bytes() == before


def test_apply_once_and_preserve_source_timestamp(tmp_path, monkeypatch):
    args = write_inputs(tmp_path); seed_path = tmp_path/'seed.json'; receipt = tmp_path/'receipt.json'
    monkeypatch.setattr(tool, 'current_snapshot', lambda: (health(), 'boot', 1500, ['7']))
    monkeypatch.setattr(tool, 'map_hash', lambda _: 'map')
    monkeypatch.setattr(tool.time, 'time', lambda: 101.)
    monkeypatch.setattr(tool, 'wait_applied', lambda seed, report, *_: dict(report, localized=True, status='LOCALIZED'))
    import tempfile
    monkeypatch.setitem(sys.modules, 'fcntl', type('Fcntl', (), dict(LOCK_EX=1, LOCK_NB=2, flock=lambda *_: None)))
    def fake_open(path, *_args, **_kwargs):
        assert path == '/tmp/g2_corner_seed.lock'; return tempfile.TemporaryFile()
    monkeypatch.setattr(tool, 'open', fake_open, raising=False)
    real = ['--real', '--trusted-capture', str(tmp_path/'snapshot.json'), '--seed', str(seed_path),
            '--receipt', str(receipt)]
    assert tool.main(args+real) == 0
    assert json.loads(seed_path.read_text())['stamp'] == 100.
    saved = seed_path.read_bytes()
    assert tool.main(args+real) == 0
    report = json.loads((tmp_path/'output.json').read_text())
    assert report['seed_already_submitted'] and not report['seed_written']
    assert seed_path.read_bytes() == saved
    seed_path.unlink()  # A crash after receipt and before seed is ambiguous.
    assert tool.main(args+real) == 2
    assert json.loads((tmp_path/'output.json').read_text())['status'] == 'SUBMISSION_UNCERTAIN'
    assert not seed_path.exists()


def test_old_or_wrong_ack_cannot_report_localized(tmp_path):
    clock = [0.]; _, _, _, _ = fixture()
    def sleep(dt): clock[0] += dt
    loc = tmp_path/'health.json'; loc.write_text(json.dumps(dict(accepted_reference_stamp=99.)))
    seed = dict(stamp=100., capture_id='c', boot_id='boot', owner_pid=7, map_hash='map',
                pose=[1., 1., 0.], odom=[0., 0., 0.])
    report = tool.wait_applied(seed, {}, loc, 1, None, lambda: (health(), 'boot', 1500, ['7']),
                               clock=lambda: clock[0], wall=lambda: 101., sleep=sleep)
    assert report['status'] == 'AMCL_TIMEOUT' and not report['localized']


def test_ack_requires_five_distinct_healthy_odom_frames(tmp_path):
    clock = [0.]; loc_path = tmp_path/'health.json'; samples = []
    def sleep(dt): clock[0] += dt
    def snapshot():
        now = 101.+clock[0]; samples.append(now)
        loc = dict(stamp=now, ready=True, boot_id='boot', owner_pid=7, map_hash='map',
                   accepted_reference_stamp=100., state=dict(pose=[1., 1., 0.], odom=[0., 0., 0.],
                       cov=[.001, .001, .001], scan_age=.01, tf_age=.01, odom_stamp=now-.01))
        loc_path.write_text(json.dumps(loc))
        return health(), 'boot', 1500, ['7']
    seed = dict(stamp=100., capture_id='c', boot_id='boot', owner_pid=7, map_hash='map',
                pose=[1., 1., 0.], odom=[0., 0., 0.])
    report = tool.wait_applied(seed, {}, loc_path, 1, None, snapshot, clock=lambda: clock[0],
                               wall=lambda: 101.+clock[0], sleep=sleep)
    assert report['localized'] and report['status'] == 'LOCALIZED' and len(samples) == 5


def test_historical_random_start_disambiguation_replay_is_not_hardware_registration():
    folder = ROOT/'docs/calibration/dropoff_filter_2026-10-18'
    snap = tool.load(folder/'fresh_reference_scan.json')
    candidates = tool.load(folder/'fresh_reference_hypotheses.json')
    registry = tool.load(ROOT/'maps/demo_box_20261018_patrol/aruco_corner_landmarks.example.json')
    # The saved old capture predates this protocol. These bindings and camera
    # confirmations are test-only, never a deployable registration or seed.
    registry.update(registration_verified=True, registration_id='SIMULATION_ONLY')
    registry['camera'].update(orientation_verified=True, fov_bound_verified=True)
    snap.update(capture_id='replay', boot_id='replay', owner_pid=8117, map_hash=registry['map_hash'],
                lidar_forward_m=.10, lidar_yaw_rad=math.pi)
    candidates.update(snapshot_sha256='replay', map_hash=registry['map_hash'])
    packet = dict(schema=1, clock='jetson', capture_id='replay', boot_id='replay', owner_pid=8117,
                  map_hash=registry['map_hash'], registration_id='SIMULATION_ONLY',
                  camera_id=registry['camera']['camera_id'], frames=[dict(seq=i, stamp=snap['time']-.2+.1*i,
                  markers=[{'id': 2, 'side': 'left'}, {'id': 7, 'side': 'right'}]) for i in range(3)])
    assert tool.select(snap, candidates, packet, registry, 'replay')['selected_index'] == 3


def test_real_default_landmark_table_is_disabled():
    registry = tool.load(ROOT/'maps/demo_box_20261018_patrol/aruco_corner_landmarks.example.json')
    assert set(registry['markers']) == set(map(str, range(8)))
    with pytest.raises(tool.Rejected) as exc:
        tool.checked_registry(registry)
    assert exc.value.status == 'LANDMARKS_UNVERIFIED'


def test_scan_match_keeps_nonacceptance_and_adds_input_hash_contract():
    source = (ROOT/'integration/g2_scan_match.py').read_text(encoding='utf-8')
    assert "'accepted':False" in source and "'snapshot_sha256':file_hash(args.snapshot)" in source
