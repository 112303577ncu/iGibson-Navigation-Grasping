import json
import math
import runpy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from integration import g2_placement_corridor as corridor
from integration import g2_placement_supervisor as supervisor
from integration import g2_prepare_koala as prepare


class Mask:
    def allowed(self, x, y):
        return .4 <= x <= 1.6 and .4 <= y <= 1.6


class Dock:
    BODY_RECTANGLES = ((-.15, -.14, .15, .14),)
    BOX_DEPTH_M = .135
    PLACEMENT_FINAL_X_M = .253
    PLACEMENT_AIM_Y_M = -.028

    @staticmethod
    def to_map(pose, point):
        x, y, a = pose
        return (x + math.cos(a) * point[0] - math.sin(a) * point[1],
                y + math.sin(a) * point[0] + math.cos(a) * point[1])

    @staticmethod
    def wall_distance(point, walls):
        x, y = point
        return min(x, y, 2. - x, 2. - y)

    @staticmethod
    def back_path_problem(pose, metres, walls, obstacles=()):
        for x, y in ((-.15, -.14), (-.15, .14), (.15, -.14), (.15, .14)):
            start = Dock.to_map(pose, (x, y))
            end = Dock.to_map(pose, (x - metres, y))
            if Dock.wall_distance(end, walls) <= 0:
                return 'wall'
            for ox, oy, radius in obstacles:
                if math.hypot(end[0] - ox, end[1] - oy) < radius and (
                        math.hypot(end[0] - ox, end[1] - oy) < math.hypot(start[0] - ox, start[1] - oy)):
                    return 'bin'

    @staticmethod
    def box_from_layout(layout, source):
        return dict(x=1.7, y=1.7, verified=False)


def geometry():
    return corridor.Geometry(Mask(), [1], [(1.9, 1.9, .1)], Dock)


def test_diagonal_approach_accounts_for_rotated_box_depth():
    box = dict(orientation_rad=0., length_m=.18, width_m=.135)
    assert corridor.half_depth(box, 0., .01) == pytest.approx(.09)
    assert corridor.half_depth(box, math.pi / 2, .01) == pytest.approx(.0675)
    assert corridor.half_depth(box, math.pi / 4, .01) == pytest.approx((.18 + .135) / math.sqrt(8))


def test_corner_heading_maximizes_wall_clearance_before_shortness():
    plans = geometry().choose(dict(x=1.7, y=1.7), [1., 1., 0.], .253, -.028)
    assert plans and 25 <= math.degrees(plans[0]['yaw_rad']) <= 65
    assert plans[0]['worst_retreat_m'] <= .6
    assert plans[0]['hardware_validated'] is False
    assert corridor.turn_ready(Mask(), plans[0]['exit']['point'])


def test_no_exit_is_reported_before_motion_or_release():
    g = corridor.Geometry(SimpleNamespace(allowed=lambda x, y: False), [1], [(1.7, 1.7, .1)], Dock)
    with pytest.raises(ValueError, match='NO_REVERSIBLE_DOCK'):
        g.choose(dict(x=1.7, y=1.7), [1., 1., 0.], .253, -.028)


def test_inflated_mask_does_not_forbid_a_checked_straight_exit():
    result = geometry().exit([1.72, 1., 0.])
    assert result and result['distance_m'] == pytest.approx(.19)
    assert geometry().command_problem(dict(yaw_rad=0.), [1.72, 1., 0.], -.08, 0.) is None


@pytest.mark.parametrize('vx,wz,part', [(.15, 0., 'predicted'),
                                      (.2, 0., 'speed'), (-.09, 0., 'speed')])
def test_command_cannot_consume_the_reserved_return(vx, wz, part):
    assert part in geometry().command_problem(dict(yaw_rad=0.), [1.72, 1., 0.], vx, wz)


def test_close_corridor_allows_motor_effective_turn_when_every_pose_retains_exit():
    g = geometry()
    assert not corridor.turn_ready(Mask(), [1.72, 1., 0.])
    assert g.command_problem(dict(yaw_rad=0.), [1.72, 1., 0.], 0., .3) is None
    assert g.command_problem(dict(yaw_rad=0.), [1.72, 1., .2], -.05, 0.) is None


def test_turn_towards_wall_is_stopped_even_when_current_exit_exists():
    g = geometry()
    pose = [1.72, 1., 0.]
    assert g.exit(pose) is not None
    assert 'predicted' in g.command_problem(dict(yaw_rad=pose[2]), pose, 0., 1.)


def test_rotation_cannot_sweep_deeper_into_the_conservative_bin_envelope():
    g = corridor.Geometry(Mask(), [1], [(1.72, 1.3, .15)], Dock)
    assert g.rotation_problem([1.72, 1., 0.], .3)
    assert 'rotation' in g.command_problem(dict(yaw_rad=0.), [1.72, 1., 0.], 0., 1.)


def test_permit_stale_context_and_stop_are_independent(tmp_path):
    state = tmp_path / 'state.json'
    contract = tmp_path / 'contract.json'
    mask = SimpleNamespace(width=100, height=100, resolution=.02, origin=[0., 0.],
                           cells=bytes([255]) * 10000)
    contract.write_text(json.dumps(dict(schema=1, context=['boot', 7, 'map'],
        state_file=str(state), mask=corridor.encode_mask(mask), walls=[1],
        obstacles=[[1.9, 1.9, .1]], plan=dict(yaw_rad=0.))))
    permit = corridor.Permit(contract, dock=Dock)
    assert permit.allows(0., 0.)  # A missing permit must never block stop.
    value = dict(ready=True, received_at=10., source_age_s=.1,
                 context=['boot', 7, 'map'], pose=[1., 1., 0.], measured_vx=0., measured_wz=0.)
    state.write_text(json.dumps(value))
    assert permit.allows(.08, 0., clock=lambda: 10.1)
    assert not permit.allows(.08, 0., clock=lambda: 10.5)
    value['measured_wz'] = -.3
    state.write_text(json.dumps(value))
    assert not permit.allows(0., .3, clock=lambda: 10.1)
    assert 'reversing placement rotation' in permit.reason
    value['context'][1] = 8
    state.write_text(json.dumps(value))
    assert not permit.allows(.08, 0., clock=lambda: 10.1)
    assert permit.allows(0., 0., clock=lambda: 100.)


def test_engine_gate_sends_stop_without_touching_sugarbox_commands():
    # Covered with an executable send stub below rather than importing models.
    env = dict(G2_GUARD_PATH='contract', TARGET_KIND='placement',
               G2Permit=lambda path: SimpleNamespace(limit=lambda vx, wz: (0., 0.)))
    exec('def send(self, msg, vx, wz):\n' + prepare.ENGINE_GATE, env)
    client = SimpleNamespace(_send_json=Mock(), sock=object())
    env['send'](client, {'action': 'velocity', 'vx': .1}, .1, 0.)
    client._send_json.assert_called_with({'action': 'stop'})
    env['TARGET_KIND'] = 'sugarbox'
    env['send'](client, {'action': 'velocity', 'vx': .1}, .1, 0.)
    client._send_json.assert_called_with({'action': 'velocity', 'vx': .1})


def test_motor_reconnect_cannot_replay_a_previously_admitted_velocity():
    env = dict(json=json, G2_GUARD_PATH='contract', TARGET_KIND='placement')
    exec('def connect(self, sock):\n' + prepare.CONNECT_GATE, env)
    client, sock = SimpleNamespace(), SimpleNamespace(sendall=Mock())
    env['connect'](client, sock)
    sock.sendall.assert_not_called()
    with pytest.raises(RuntimeError, match='not replayed'):
        env['connect'](client, sock)
    assert json.loads(sock.sendall.call_args.args[0]) == {'action': 'stop'}


def test_speed_limiter_can_keep_a_safe_slow_approach_and_never_increase_it():
    permit = corridor.Permit.__new__(corridor.Permit)
    permit.allows = lambda vx, wz: 0 < vx <= .04 and wz == 0.
    assert permit.limit(.08, 0.) == (.04, 0.)
    assert permit.limit(.03, 0.) == (.03, 0.)
    assert permit.limit(-.08, 0.) == (0., 0.)
    assert permit.limit(.08, .3) == (0., 0.)


def test_stream_binding_counts_network_backlog_instead_of_refreshing_old_pose(monkeypatch, tmp_path):
    import io
    saved = []
    def packet(stamp, now):
        return json.dumps(dict(now=now, record=dict(stamp=stamp, ready=True,
            boot_id='boot', owner_pid=7, map_hash='map',
            state=dict(pose=[1., 1., 0.], cov=[.000025] * 3, vx=0., wz=0.)))) + '\n'
    feed = supervisor.LocalizationFeed.__new__(supervisor.LocalizationFeed)
    feed.path, feed.context = tmp_path / 'state.json', ['boot', 7, 'map']
    feed.proc = SimpleNamespace(stdin=io.StringIO(), stdout=io.StringIO(
        '{"hello":true}\n' + packet(99.9, 100.) + packet(100., 100.1) + packet(102., 102.1)))
    ticks = iter([10., 10.1, 12., 12.2])
    monkeypatch.setattr(supervisor.time, 'monotonic', lambda: next(ticks))
    monkeypatch.setattr(supervisor, 'atomic', lambda path, value: saved.append(value))
    feed._read()
    samples = [s for s in saved if 'pose' in s]
    assert samples[0]['ready']
    assert not samples[1]['ready'] and samples[1]['source_age_s'] == pytest.approx(2.)
    assert samples[2]['ready']
    assert saved[-1] == {'ready': False}


def test_windows_reader_lock_retries_are_bounded_without_changing_sample_time(tmp_path, monkeypatch):
    original, calls = Path.replace, []
    def replace(path, target):
        calls.append(1)
        if len(calls) < 3:
            raise PermissionError('brief reader lock')
        return original(path, target)
    monkeypatch.setattr(Path, 'replace', replace)
    monkeypatch.setattr(supervisor.time, 'sleep', lambda seconds: None)
    target = tmp_path / 'state.json'
    supervisor.atomic(target, dict(received_at=10.))
    assert len(calls) == 3 and json.loads(target.read_text()) == {'received_at': 10.}


def test_release_checks_after_wall_correction_before_the_actual_arm_rpc(tmp_path):
    calls = []
    def original(host, **kw):
        kw['drive'](host, .02)
        return kw['read'](host, 'release')
    runner = SimpleNamespace(place=original, drive=lambda *a: calls.append('drive'),
                             ctl=lambda h, c: calls.append(c), start_engine=Mock())
    s = supervisor.Supervisor(SimpleNamespace(runner=runner), {}, 'scene', tmp_path)
    room = SimpleNamespace(_snapshot=lambda **kw: dict(pose=[1., 1., 0.]))
    g = SimpleNamespace(command_problem=Mock(return_value=None), exit=lambda pose: {'distance_m': .2})
    s.active = dict(room=room, geometry=g, plan=dict(release=[1., 1., 0.]))
    s.place('host')
    assert calls == ['drive', 'release']
    assert s.events[-1]['status'] == 'RELEASE_EXIT_CONFIRMED'
    g.command_problem.return_value = 'no exit'
    with pytest.raises(RuntimeError):
        s.place('host')
    assert calls == ['drive', 'release']


def test_release_cannot_succeed_at_the_acquisition_station(tmp_path):
    def original(host, **kw):
        return kw['read'](host, 'release')
    runner = SimpleNamespace(place=original, drive=Mock(), ctl=Mock(), start_engine=Mock())
    s = supervisor.Supervisor(SimpleNamespace(runner=runner), {}, 'scene', tmp_path)
    s.active = dict(room=SimpleNamespace(_snapshot=lambda **kw: dict(pose=[1., 1., 0.])),
                    geometry=Mock(), plan=dict(release=[1.6, 1., 0.]))
    with pytest.raises(RuntimeError, match='RELEASE_POSE_REFUSED'):
        s.place('host')
    runner.ctl.assert_not_called()


def test_absolute_parking_heading_wraps_without_an_unnecessary_turn():
    import importlib.util
    spec = importlib.util.spec_from_file_location('corridor_patrol_yaw',
        Path(__file__).resolve().parents[1] / 'deploy/ros/g2_map_patrol.py')
    patrol = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(patrol)
    runner = patrol.Runner.__new__(patrol.Runner)
    runner.report = {'recoveries': []}
    runner.ready = lambda: dict(pose=[1., 1., math.pi - .02])
    runner.pulse = Mock()
    runner.face_yaw(-math.pi + .03)
    runner.pulse.assert_not_called()
    assert runner.report['park_yaw_error_deg'] == pytest.approx(math.degrees(.05))


def execute_fixture(tmp_path, monkeypatch, states, codes):
    plan = dict(staging=[1., 1., 0.], yaw_rad=0.)
    g = SimpleNamespace(choose=lambda *a: [plan], command_problem=lambda *a: None)
    monkeypatch.setattr(supervisor.corridor, 'Geometry', lambda *a: g)
    monkeypatch.setattr(supervisor.corridor, 'encode_mask', lambda *a: {})
    monkeypatch.setattr(supervisor.corridor, 'turn_ready', lambda *a: True)
    monkeypatch.setattr(supervisor, 'LocalizationFeed', Mock())
    runner = SimpleNamespace(place=Mock(), drive=Mock(), ctl=Mock(), start_engine=Mock(), stop=Mock())
    room = SimpleNamespace(mask=Mask(), walls=[1], obstacles=[1],
        _snapshot=Mock(return_value=dict(pose=[1., 1., 0.], context=('boot', 7, 'map'))),
        _leave_ready=Mock())
    mission = SimpleNamespace(runner=runner, dock=Dock, _runtime=SimpleNamespace(usable=lambda: True),
        stage_options=lambda args: [], run_stage=Mock(side_effect=codes),
        arm_state=Mock(side_effect=states), unlock=Mock(), to_travel=Mock())
    s = supervisor.Supervisor(mission, {'host': room}, 'scene', tmp_path)
    s.transit = Mock(return_value={'ok': True})
    layout = tmp_path / 'layout.json'
    layout.write_text('{}')
    args = SimpleNamespace(host='host', wall_m=None)
    return s, room, mission, (args, layout, tmp_path, 1, Mock(), Mock())


def test_known_held_failure_exits_before_one_reapproach(tmp_path, monkeypatch):
    s, room, mission, args = execute_fixture(tmp_path, monkeypatch,
        ['holding', 'holding', 'empty'], [2, 0])
    s.execute(*args)
    assert mission.run_stage.call_count == 2
    room._leave_ready.assert_called_once_with(0., holding=True)
    assert s.events[-1]['status'] == 'PLACED'
    live_state = supervisor.LocalizationFeed.call_args.args[1]
    assert live_state.parent != tmp_path and not live_state.exists()


def test_unknown_release_result_never_replays_placement(tmp_path, monkeypatch):
    s, room, mission, args = execute_fixture(tmp_path, monkeypatch, ['unknown', 'unknown'], [2])
    with pytest.raises(RuntimeError, match='ACTION_OUTCOME_UNKNOWN'):
        s.execute(*args)
    assert mission.run_stage.call_count == 1
    room._leave_ready.assert_not_called()


def test_reapproach_budget_is_finite_and_last_failure_exits_to_patrol(tmp_path, monkeypatch):
    s, room, mission, args = execute_fixture(tmp_path, monkeypatch,
        ['holding', 'holding', 'holding', 'holding'], [2, 2])
    with pytest.raises(RuntimeError, match='turn-clear checkpoint'):
        s.execute(*args)
    assert mission.run_stage.call_count == room._leave_ready.call_count == 2


def test_isolated_stage_cannot_bypass_release_guards(tmp_path, monkeypatch):
    s, room, mission, args = execute_fixture(tmp_path, monkeypatch, [], [])
    mission._runtime = None
    with pytest.raises(RuntimeError, match='resident'):
        s.execute(*args)
    s.transit.assert_not_called()


def test_default_demo_mission_routes_to_the_guarded_entry(monkeypatch):
    import subprocess
    import sys
    call = Mock(return_value=0)
    monkeypatch.setattr(subprocess, 'call', call)
    monkeypatch.setattr(sys, 'argv', ['demo.py', 'mission', '--host', '127.0.0.1'])
    with pytest.raises(SystemExit):
        runpy.run_path(str(Path(__file__).resolve().parents[1] / 'grasp/v23/demo.py'), run_name='__main__')
    assert call.call_args.args[0][1].endswith('v23_mission_entry.py')


def observation_fixture(tmp_path, monkeypatch):
    calls = []
    def original(host, report, final_x, aim_y, empty_test=False, **kw):
        if kw.get('wall_m') is not None:
            kw['drive'](host, .02)
        return kw['read'](host, 'release-preview' if empty_test else 'release')
    def arrival_problem(report, final_x, aim_y):
        if not report or report.get('seeded') is not False or not report.get('arrived'):
            return 'not a visual arrival'
        if not report.get('reason', '').startswith('PURSUIT_DONE'):
            return 'not at destination'
        if abs(report['x'] - final_x) > .03 or abs(report['y'] - aim_y) > .04:
            return 'outside arrival tolerance'
    dock = SimpleNamespace(arrival_problem=arrival_problem, to_map=Dock.to_map,
                           BOX_DEPTH_M=.135, BIN_HALF_DIAGONAL_M=.112)
    runner = SimpleNamespace(place=original, drive=lambda *a: calls.append('drive'),
                             ctl=lambda h, c: calls.append(c), start_engine=Mock())
    s = supervisor.Supervisor(SimpleNamespace(runner=runner, dock=dock), {}, 'scene', tmp_path)
    room = SimpleNamespace(mask=Mask(), walls=[1], obstacles=[(1.7, 1., .312)],
        _snapshot=Mock(return_value=dict(pose=[1.2, 1., 0.], context=('boot', 7, 'map'))))
    g = SimpleNamespace(exit=Mock(return_value=dict(distance_m=.2)), command_problem=Mock(return_value=None))
    monkeypatch.setattr(supervisor.corridor, 'Geometry', lambda mask, walls, obstacles, dock:
                        SimpleNamespace(exit=g.exit, command_problem=g.command_problem, obstacles=obstacles))
    s.active = dict(room=room, context=('boot', 7, 'map'), geometry=g,
                    box=dict(x=1.7, y=1., verified=False), plan=dict(release=[1.38, 1., 0.]))
    report = dict(x=.253, y=-.028, seeded=False, arrived=True, reason='PURSUIT_DONE',
                  t=100., visual_age_s=3., target_kind='placement', blocked=False)
    monkeypatch.setattr(supervisor.time, 'time', lambda: 101.)
    return s, runner, room, g, calls, report


def test_unverified_bin_uses_visual_stop_and_keeps_both_obstacle_envelopes(tmp_path, monkeypatch):
    s, runner, room, g, calls, report = observation_fixture(tmp_path, monkeypatch)
    s.place('host', report, .253, -.028, wall_m=.5)
    assert calls == ['drive', 'release']
    assert len(room.obstacles) == 2
    assert room.obstacles[0] == (1.7, 1., .312)
    assert room.obstacles[1] == pytest.approx((1.5205, .972, .152))
    assert s.active['observed_release'] == pytest.approx([1.2, 1., 0.])
    assert s.events[0]['registration_verified'] is False
    assert s.events[-1]['status'] == 'RELEASE_EXIT_CONFIRMED'


@pytest.mark.parametrize('bad', ['seeded', 'stale', 'old_visual', 'wrong_target', 'blocked',
                                  'outside_candidate', 'context', 'no_exit', 'wall_correction_exit'])
def test_bad_bin_observation_never_sends_release_or_changes_registration(tmp_path, monkeypatch, bad):
    s, runner, room, g, calls, report = observation_fixture(tmp_path, monkeypatch)
    if bad == 'seeded': report['seeded'] = True
    if bad == 'stale': report['t'] = 90.
    if bad == 'old_visual': report['visual_age_s'] = 13.
    if bad == 'wrong_target': report['target_kind'] = 'sugarbox'
    if bad == 'blocked': report['blocked'] = True
    if bad == 'outside_candidate': s.active['box']['x'] = 1.9
    if bad == 'context': room._snapshot.return_value['context'] = ('boot', 8, 'map')
    if bad == 'no_exit': g.exit.return_value = None
    if bad == 'wall_correction_exit': g.command_problem.return_value = 'lost exit'
    with pytest.raises(RuntimeError):
        s.place('host', report, .253, -.028, wall_m=.5)
    assert calls == []
    assert s.active['box']['verified'] is False


def test_verified_bin_still_requires_the_reserved_release_point(tmp_path, monkeypatch):
    s, runner, room, g, calls, report = observation_fixture(tmp_path, monkeypatch)
    s.active['box']['verified'] = True
    with pytest.raises(RuntimeError, match='RELEASE_POSE_REFUSED'):
        s.place('host', report, .253, -.028)
    assert calls == [] and len(room.obstacles) == 1



def test_retry_planning_keeps_the_observed_bin_envelope(tmp_path, monkeypatch):
    s, room, mission, args = execute_fixture(tmp_path, monkeypatch,
        ['holding', 'holding', 'empty'], [])
    seen = []
    g = supervisor.corridor.Geometry(None, None, None, None)
    def factory(mask, walls, obstacles, dock):
        seen.append(list(obstacles))
        return g
    monkeypatch.setattr(supervisor.corridor, 'Geometry', factory)
    def stage(*a):
        if len(seen) == 1:
            room.obstacles = list(room.obstacles) + [(1.5, 1.5, .152)]
            return 2
        return 0
    mission.run_stage.side_effect = stage
    s.execute(*args)
    assert seen == [[1], [1, (1.5, 1.5, .152)]]
