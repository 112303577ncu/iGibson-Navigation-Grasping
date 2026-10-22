"""Startup ordering: no mission velocity owner before a live ack and new video."""
import copy
from pathlib import Path
import sys
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from integration import v23_mission_entry as entry


def parked():
    return {'ok': True, 'holding': False, 'chassis': {
        'motors': [0, 0, 0, 0], 'moving': False, 'arm_pose': 'travel',
        'arm_busy': False, 'client': None}}


def localized():
    return {'ok': True, 'localized': True, 'status': 'LOCALIZED'}


@pytest.mark.parametrize('reply', [
    {'ok': True, 'localized': False, 'status': 'CANDIDATE_SELECTED'},
    {'ok': True, 'localized': True, 'status': 'AMCL_TIMEOUT'},
    {'ok': False, 'localized': True, 'status': 'LOCALIZED'}, {},
])
def test_no_stream_or_mission_on_partial_or_failed_initialization(reply):
    stream, mission = Mock(), Mock()
    with pytest.raises(RuntimeError):
        entry.run_stages(lambda: reply, stream, mission)
    stream.assert_not_called()
    mission.assert_not_called()


def test_camera_failure_never_starts_the_upstream_mission():
    mission = Mock()
    with pytest.raises(RuntimeError, match='camera'):
        entry.run_stages(localized, Mock(side_effect=RuntimeError('camera stale')), mission)
    mission.assert_not_called()


def test_success_runs_one_mission_after_the_two_startup_gates():
    events = []
    def initialize():
        events.append('AMCL ack')
        return localized()
    def video():
        events.append('new video')
    def mission():
        events.append('mission')
        return 2  # Preserve a real upstream failure, never turn it into success.
    assert entry.run_stages(initialize, video, mission) == 2
    assert events == ['AMCL ack', 'new video', 'mission']


@pytest.mark.parametrize('field,value', [
    ('motors', [1, 1, 1, 1]), ('moving', True), ('arm_pose', 'e1'),
    ('arm_busy', True), ('client', 'other-controller'),
])
def test_initialization_refuses_a_nonparked_or_owned_state(field, value):
    health = parked()
    health['chassis'][field] = value
    with pytest.raises(RuntimeError):
        entry.require_parked(health)


def test_held_or_unknown_object_blocks_reinitialization():
    for value in (True, None):
        health = parked()
        health['holding'] = value
        with pytest.raises(RuntimeError):
            entry.require_parked(health)
    entry.require_parked(parked())


def test_formal_eight_id_registration_is_the_startup_source():
    raw, registry = entry.registry_bytes()
    assert raw == (ROOT / entry.REGISTRY).read_bytes()
    assert set(registry['markers']) == set(map(str, range(8)))
    assert registry['registration_id'] == 'coarse_wall_layout_20261021_v3'


def reference_records():
    _, registry = entry.registry_bytes()
    seed = {'stamp': 1., 'registration_id': registry['registration_id'],
            'map_hash': registry['map_hash'], 'boot_id': 'boot', 'owner_pid': 7}
    rows = []
    for i in range(5):
        health = dict(parked(), service_pid=7)
        rows.append({'health': health, 'reference': dict(seed), 'now': 100. + i,
                     'boot_id': 'boot', 'owners': ['7'], 'localization': {
                         'ready': True, 'stamp': 100. + i, 'boot_id': 'boot', 'owner_pid': 7,
                         'map_hash': registry['map_hash'], 'accepted_reference_stamp': 1.,
                         'state': {'pose': [0., 0., 0.], 'cov': [.001] * 3, 'odom_stamp': 100. + i}}})
    return rows, registry


def test_long_running_healthy_localization_does_not_need_a_new_seed():
    rows, registry = reference_records()
    result = entry.existing_reference(rows, registry)
    assert result['localized'] and result['seed_written'] is False
    assert result['accepted_reference_stamp'] == 1.


@pytest.mark.parametrize('kind', ['stale', 'unacknowledged', 'wrong_registry', 'duplicate',
                                  'wrong_owner', 'uncertain', 'negative_covariance', 'moving_pose'])
def test_reusing_localization_keeps_freshness_identity_and_quality_gates(kind):
    rows, registry = reference_records()
    row = rows[-1]
    if kind == 'stale': row['now'] += 2
    if kind == 'unacknowledged': row['localization']['accepted_reference_stamp'] = 0
    if kind == 'wrong_registry': row['reference']['registration_id'] = 'old'
    if kind == 'duplicate': row['localization']['state']['odom_stamp'] = 100
    if kind == 'wrong_owner': row['owners'] = ['8']
    if kind == 'uncertain': row['localization']['state']['cov'][0] = .1
    if kind == 'negative_covariance': row['localization']['state']['cov'][0] = -1e-18
    if kind == 'moving_pose': row['localization']['state']['pose'][0] = .04
    with pytest.raises(RuntimeError):
        entry.existing_reference(rows, registry)


def test_existing_reference_requires_five_distinct_samples():
    rows, registry = reference_records()
    with pytest.raises(RuntimeError, match='five'):
        entry.existing_reference(rows[:4], registry)


def test_raw_capture_download_preserves_bytes_and_checks_remote_digest(tmp_path, monkeypatch):
    import hashlib
    from types import SimpleNamespace
    raw = b'{ "original_spacing": true }\n'
    replies = iter([SimpleNamespace(stdout=raw), SimpleNamespace(
        stdout=(hashlib.sha256(raw).hexdigest() + '  capture.json\n').encode())])
    monkeypatch.setattr(entry, 'remote', lambda *a, **k: next(replies))
    path = tmp_path/'capture.json'
    entry.download('host', '/capture.json', path)
    assert path.read_bytes() == raw


def test_truncated_capture_download_is_not_written(tmp_path, monkeypatch):
    from types import SimpleNamespace
    replies = iter([SimpleNamespace(stdout=b'partial'), SimpleNamespace(stdout=b'0'*64)])
    monkeypatch.setattr(entry, 'remote', lambda *a, **k: next(replies))
    path = tmp_path/'capture.json'
    with pytest.raises(RuntimeError, match='truncated'):
        entry.download('host', '/capture.json', path)
    assert not path.exists()
