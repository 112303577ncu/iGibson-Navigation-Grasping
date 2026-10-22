import copy
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from integration import g2_patrol_reentry as reentry
from integration import v23_mission_entry as entry


class Mask:
    def allowed(self, x, y):
        return .4 <= x <= 1.6 and .4 <= y <= 1.6


class Plant:
    def __init__(self, x=1.8, overshoot=0.):
        self.pose = [x, 1., 0.]
        self.odom = [x + 3., 5., 0.]
        self.stamp = 100.
        self.context = ('boot', 7, 'map')
        self.commands = []
        self.overshoot = overshoot

    def read(self):
        self.stamp += 1.
        return {'pose': list(self.pose), 'odom': list(self.odom),
                'stamp': self.stamp, 'context': self.context}

    def back(self, metres):
        self.commands.append(metres)
        self.pose[0] -= metres + self.overshoot
        self.odom[0] -= metres + self.overshoot


def test_exit_ack_checks_native_six_centimetre_ring_as_well_as_extra_margin():
    mask = SimpleNamespace(allowed=lambda x, y: not (abs(x - 1.06) < .001 and abs(y - 1.) < .001))
    assert mask.allowed(1., 1.) and mask.allowed(1.07, 1.)
    assert not reentry.turn_ready(mask, (1., 1.))


def run(plant, **kwargs):
    return reentry.retreat_to_patrol(Mask(), plant.read, plant.back,
                                    kwargs.pop('path_problem', lambda pose, metres: None),
                                    kwargs.pop('stop', Mock()), clock=lambda: 0., **kwargs)


def test_fixed_backout_finishes_inside_mask_but_extra_retreat_is_needed_to_turn():
    plant = Plant(1.8)
    assert Mask().allowed(1.6, 1.)
    assert not reentry.turn_ready(Mask(), (1.6, 1.))
    report = run(plant)
    assert report['ok'] and report['next_stage'] == 'PATROL_REENTRY_READY'
    assert len(plant.commands) == 3
    assert report['distance_m'] == pytest.approx(.27)
    assert reentry.turn_ready(Mask(), report['final_pose'])
    assert len(set(t['stamp'] for t in report['trace'][-3:])) == 3


def test_measured_overshoot_prevents_unnecessary_extra_backing():
    plant = Plant(1.8, overshoot=.04)
    report = run(plant)
    assert len(plant.commands) == 2
    assert report['distance_m'] == pytest.approx(.28)
    assert reentry.turn_ready(Mask(), report['final_pose'])


def test_ready_robot_with_no_minimum_does_not_keep_backing():
    plant = Plant(1.4)
    assert run(plant, minimum=0.)['ok']
    assert plant.commands == []


def test_farther_retreat_required_for_failed_grasp_remains_a_minimum():
    plant = Plant(1.8)
    report = run(plant, minimum=.45)
    assert report['distance_m'] >= .445


def test_no_turn_clear_exit_does_not_fall_back_to_a_fixed_unsafe_distance():
    plant = Plant(1.8)
    report = {}
    stop = Mock()
    with pytest.raises(RuntimeError, match='no straight exit'):
        run(plant, minimum=0., limit=.1, report=report, stop=stop)
    assert plant.commands == [] and stop.call_count >= 2
    assert report['ok'] is False and 'error' in report


def test_physical_back_path_refusal_prevents_movement_even_if_mask_endpoint_is_good():
    plant = Plant()
    with pytest.raises(RuntimeError, match='wall'):
        run(plant, path_problem=lambda p, d: 'would sweep a wall')
    assert plant.commands == []


def test_context_change_after_move_aborts_before_another_command():
    plant = Plant()
    back = plant.back
    def changed(metres):
        back(metres)
        plant.context = ('other-boot', 7, 'map')
    plant.back = changed
    with pytest.raises(RuntimeError, match='owner/boot/map'):
        run(plant)
    assert len(plant.commands) == 1


def test_repeated_timestamp_is_not_a_final_reentry_acknowledgement():
    plant = Plant(1.4)
    read = plant.read
    plant.read = lambda: dict(read(), stamp=100.)
    with pytest.raises(RuntimeError, match='new localization'):
        run(plant, minimum=0.)
    assert not plant.commands


def test_amcl_correction_is_not_counted_as_actual_odometry_progress():
    plant = Plant()
    def no_actual_movement(metres):
        plant.commands.append(metres)
        plant.pose[0] = 1.4
    plant.back = no_actual_movement
    with pytest.raises(RuntimeError, match='no measured progress'):
        run(plant)
    assert len(plant.commands) == 3


def test_excess_lateral_drift_does_not_trigger_a_blind_rotation():
    plant = Plant()
    back = plant.back
    def drift(metres):
        back(metres)
        plant.odom[1] += .05
    plant.back = drift
    with pytest.raises(RuntimeError, match='deviated'):
        run(plant)
    assert len(plant.commands) == 1


def fake_mission(plant):
    health = {'ok': True, 'holding': False, 'service_pid': 7,
              'chassis': {'arm_pose': 'travel', 'arm_busy': False, 'moving': False,
                          'motors': [0, 0, 0, 0], 'client': None}}
    def locate(host):
        state = plant.read()
        return {'ready': True, 'age': .1, 'boot_id': 'boot', 'owner_pid': 7, 'map_hash': 'map',
                'state': {'pose': state['pose'], 'odom': state['odom'], 'odom_stamp': state['stamp'],
                          'cov': [.000025] * 3, 'vx': 0., 'wz': 0.}}
    class Room:
        def __init__(self, host, walls=None, back=None, obstacles=(), mask=None, locate=None):
            self.host, self.walls, self.mask, self.backed = host, walls, mask, None
            self.obstacles = obstacles
        def _back(self, metres):
            plant.back(metres)
        def reset(self):
            self.backed = None
    mission = SimpleNamespace(Room=Room, BACK_OUT_M=.20, place_held=Mock(),
        dock=SimpleNamespace(fetch_nav_mask=Mock(), back_path_problem=Mock(return_value=None),
                             read_localization=locate,
                             localization_pose=lambda loc: tuple(loc['state']['pose'])),
        runner=SimpleNamespace(back_straight=Mock(), stop=Mock()))
    return mission, health


def test_entry_adapter_gates_next_loop_on_measured_turn_clear_completion(tmp_path, monkeypatch):
    plant = Plant()
    mission, health = fake_mission(plant)
    monkeypatch.setattr(entry, 'rpc', lambda *args: copy.deepcopy(health))
    info = entry.install_patrol_reentry(mission, tmp_path, with_placement=False)
    room = mission.Room('host', walls=[1], mask=Mask())
    next_loop = Mock()
    mission.place_held()
    room.leave()
    next_loop()
    result = json.loads((tmp_path / 'mission/post_place_reentry.json').read_text())[-1]
    assert result['ok'] and result['distance_m'] == pytest.approx(.27)
    assert info['turn_margin_m'] == .07 and len(info['source_sha256']) == 64
    next_loop.assert_called_once()
    # A retry at the same already-ready spot must not command another fixed 10cm.
    assert room.make() is False
    assert len(plant.commands) == 3


def test_successful_placement_has_no_mandatory_fixed_retreat_when_already_ready(tmp_path):
    plant = Plant(1.4)
    mission, health = fake_mission(plant)
    reentry.bind_mission(mission, 'scene', tmp_path, lambda host: health)
    room = mission.Room('host', walls=[1], mask=Mask())
    mission.place_held()
    result = room.leave()
    assert result['ok'] and result['minimum_back_m'] == 0.
    assert plant.commands == []


def test_measured_coast_past_nominal_budget_can_finish_without_another_command():
    plant = Plant(1.77, overshoot=.04)
    report = run(plant, minimum=.20, limit=.25)
    assert report['ok'] and report['distance_m'] == pytest.approx(.28)
    assert len(plant.commands) == 2


@pytest.mark.parametrize('fault', ['holding', 'moving', 'client', 'covariance', 'stale', 'owner', 'nan_velocity'])
def test_adapter_never_backs_with_invalid_release_or_feedback(tmp_path, fault):
    plant = Plant()
    mission, health = fake_mission(plant)
    if fault == 'holding': health['holding'] = True
    if fault == 'moving': health['chassis']['moving'] = True
    if fault == 'client': health['chassis']['client'] = 'other'
    original = mission.dock.read_localization
    def locate(host):
        loc = original(host)
        if fault == 'covariance': loc['state']['cov'][0] = .01
        if fault == 'stale': loc['age'] = 2.
        if fault == 'owner': loc['owner_pid'] = 8
        if fault == 'nan_velocity': loc['state']['vx'] = float('nan')
        return loc
    mission.dock.read_localization = locate
    reentry.bind_mission(mission, 'scene', tmp_path, lambda host: health)
    room = mission.Room('host', walls=[1], mask=Mask())
    with pytest.raises(RuntimeError):
        room.leave()
    assert not plant.commands
    assert json.loads((tmp_path / 'post_place_reentry.json').read_text())[-1]['ok'] is False


def test_incompatible_upstream_cannot_silently_keep_fixed_backout(tmp_path):
    with pytest.raises(RuntimeError, match='loop-opt'):
        reentry.bind_mission(SimpleNamespace(dock=SimpleNamespace(), runner=SimpleNamespace()),
                             'scene', tmp_path, Mock())
