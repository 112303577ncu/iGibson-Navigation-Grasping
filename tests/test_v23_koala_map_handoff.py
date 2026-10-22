import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from integration import v23_koala_map_handoff as bridge
from integration import mission_loop as mission

SCENE = 'maps/demo_box_20261018_patrol'


def verified_config(tmp_path):
    config = json.loads(bridge.DEFAULT_CONFIG.read_text())
    config.update(verified=True, camera_handoff_verified=True,
                  box_registration_verified=True, calibration_id='test-measured-camera')
    path = tmp_path/'handoff.json'
    path.write_text(json.dumps(config))
    return path, config


def test_default_does_not_authorize_unmeasured_box_or_camera():
    with pytest.raises(ValueError, match='WAIT_HANDOFF_CALIBRATION'):
        bridge.load_config(bridge.DEFAULT_CONFIG, SCENE)


@pytest.mark.parametrize('change,match', [
    ({'scene': 'wrong'}, 'scene mismatch'),
    ({'map_hash': None}, 'identity missing'),
    ({'camera_handoff_verified': False}, 'WAIT_HANDOFF'),
    ({'point_map': [True, 1]}, 'finite map point'),
    ({'point_map': [float('nan'), 1]}, 'finite map point'),
    ({'point_map': [1.02558, .66380]}, '0.90..1.40'),
    ({'point_map': [1, 2], 'box_center_map': [1, 1]}, 'outside navigation'),
])
def test_invalid_handoff_never_becomes_motion(tmp_path, change, match):
    path, cfg = verified_config(tmp_path)
    cfg.update(change); path.write_text(json.dumps(cfg))
    with pytest.raises(ValueError, match=match):
        bridge.load_config(path, SCENE)


def test_config_hash_binds_windows_and_jetson(tmp_path):
    path, cfg = verified_config(tmp_path)
    assert bridge.load_config(path, SCENE, hashlib.sha256(path.read_bytes()).hexdigest()) == cfg
    with pytest.raises(ValueError, match='configuration mismatch'):
        bridge.load_config(path, SCENE, '0'*64)


def test_map_transit_faces_box_and_keeps_native_ros_and_boundary_checks():
    cfg = json.loads(bridge.DEFAULT_CONFIG.read_text())
    cmd = bridge.navigation_command(SCENE, cfg, '/tmp/nav.json', True)
    assert '/usr/bin/python "$@"' in cmd[2]
    assert '--point' in cmd and '--face-point' in cmd and '--boundary-micro' in cmd
    assert '--real' in cmd and '--keep-heading' not in cmd
    assert '--real' not in bridge.navigation_command(SCENE, cfg, '/tmp/nav.json', False)


@pytest.mark.parametrize('holding,ram,map_id', [(False, 1500, 'm'), (True, 700, 'm'), (True, 1500, 'wrong')])
def test_holding_ram_or_map_failure_never_starts_resume(monkeypatch, holding, ram, map_id):
    monkeypatch.setattr(bridge, 'snapshot', lambda: ({'holding': holding}, {'map_hash': map_id}, 'boot', ram))
    monkeypatch.setattr(bridge.guard, 'begin_checkpoint', lambda *_a, **_k: pytest.fail('must stop before checkpoint'))
    with pytest.raises(ValueError):
        bridge.ready({'map_hash': 'm'})


def test_resume_failure_does_not_imply_transit_permission(monkeypatch):
    monkeypatch.setattr(bridge, 'snapshot', lambda: ({'holding': True}, {'map_hash': 'm'}, 'boot', 1500))
    monkeypatch.setattr(bridge.guard, 'begin_checkpoint', lambda *_a, **_k: {})
    monkeypatch.setattr(bridge.guard, 'run_resume', lambda *_a, **_k: {'ok': False, 'status': 'RESUME_TIMEOUT'})
    with pytest.raises(ValueError, match='RESUME_TIMEOUT'):
        bridge.ready({'map_hash': 'm'})


@pytest.mark.parametrize('nav_ok,nav_real', [(False, True), (True, False)])
def test_failed_or_dry_navigation_never_permits_real_visual_stage(tmp_path, monkeypatch, nav_ok, nav_real):
    path, cfg = verified_config(tmp_path); out = tmp_path/'result.json'
    monkeypatch.setattr(bridge, 'ready', lambda *_a, **_k: {'ok': True})
    def run(cmd):
        Path(cmd[cmd.index('--output')+1]).write_text(json.dumps({'ok': nav_ok, 'real': nav_real}))
        return 0
    monkeypatch.setattr(bridge, 'run_navigation', run)
    assert bridge.main(['--config', str(path), '--scene', SCENE, '--output', str(out), '--real']) == 2
    assert not json.loads(out.read_text())['ok']


def test_success_still_never_claims_release_or_mission_completion(tmp_path, monkeypatch):
    path, cfg = verified_config(tmp_path); out = tmp_path/'result.json'; checks = []
    monkeypatch.setattr(bridge, 'ready', lambda *_a, **_k: checks.append('stable') or
                        {'ok': True, 'boot_id': 'b', 'owner_pid': 1, 'map_hash': 'm'})
    def run(cmd):
        Path(cmd[cmd.index('--output')+1]).write_text(json.dumps({'ok': True, 'real': True}))
        return 0
    monkeypatch.setattr(bridge, 'run_navigation', run)
    assert bridge.main(['--config', str(path), '--scene', SCENE, '--output', str(out), '--real']) == 0
    report = json.loads(out.read_text())
    assert checks == ['stable', 'stable']
    assert report['next_stage'] == 'KOALA_VISUAL_APPROACH'
    assert not report['release_ready'] and not report['arm_or_release_executed']


def test_controller_restart_during_map_transit_never_authorizes_placement(tmp_path, monkeypatch):
    path, cfg = verified_config(tmp_path); out = tmp_path/'result.json'
    contexts = iter([{'boot_id': 'b', 'owner_pid': 1, 'map_hash': 'm'},
                     {'boot_id': 'b', 'owner_pid': 2, 'map_hash': 'm'}])
    monkeypatch.setattr(bridge, 'ready', lambda *_a, **_k: next(contexts))
    def run(cmd):
        Path(cmd[cmd.index('--output')+1]).write_text(json.dumps({'ok': True, 'real': True}))
        return 0
    monkeypatch.setattr(bridge, 'run_navigation', run)
    assert bridge.main(['--config', str(path), '--scene', SCENE, '--output', str(out), '--real']) == 2
    assert 'context changed' in json.loads(out.read_text())['error']


@pytest.mark.parametrize('pose,ok', [([.663, .521, 0], True), ([1.55, .83, 0], False)])
def test_postrelease_reentry_requires_empty_jaw_and_legal_centre(tmp_path, monkeypatch, pose, ok):
    path, cfg = verified_config(tmp_path); out = tmp_path/'result.json'; holding = []
    def ready(*_a, **kwargs):
        holding.append(kwargs['expected_holding']); return {'pose': pose}
    monkeypatch.setattr(bridge, 'ready', ready)
    monkeypatch.setattr(bridge, 'run_navigation', lambda *_a: pytest.fail('reentry is read-only'))
    code = bridge.main(['--config', str(path), '--scene', SCENE, '--output', str(out), '--reentry-only'])
    assert (code == 0) == ok
    assert holding == [False]


def test_output_cannot_overwrite_handoff_config(tmp_path):
    path, cfg = verified_config(tmp_path); before = path.read_bytes()
    with pytest.raises(SystemExit):
        bridge.main(['--config', str(path), '--scene', SCENE, '--output', str(path)])
    assert path.read_bytes() == before


def test_remote_failure_never_unlocks_for_vision(tmp_path, monkeypatch):
    monkeypatch.setattr(mission.map_handoff, 'load_config', lambda *_a: {})
    monkeypatch.setattr(mission, 'ssh', lambda *_a, **_k: SimpleNamespace(returncode=2, stdout='{"ok":false}', stderr=''))
    monkeypatch.setattr(mission.runner, 'service_command', lambda *_a: pytest.fail('must not unlock'))
    args = SimpleNamespace(map_handoff_config=bridge.DEFAULT_CONFIG, scene=SCENE, host='10.0.0.1')
    with pytest.raises(mission.MissionError, match='handoff failed'):
        mission.transit_to_visual_handoff(args)


def test_map_failure_keeps_held_object_and_never_starts_release(tmp_path, monkeypatch):
    from integration import rear_stream
    stages = []
    monkeypatch.setattr(mission, 'preflight', lambda *_: [])
    monkeypatch.setattr(mission, 'pc_address', lambda *_: '10.0.0.2')
    monkeypatch.setattr(mission, 'stream_running', lambda *_: True)
    monkeypatch.setattr(mission, 'SugarboxWatcher', lambda *_: None)
    monkeypatch.setattr(rear_stream, 'StreamReader', lambda *_: SimpleNamespace(close=lambda: None))
    monkeypatch.setattr(mission.map_handoff, 'load_config', lambda *_: {'box_center_map': [1.56, .83]})
    monkeypatch.setattr(mission, 'patrol_until_sugarbox', lambda *_: None)
    monkeypatch.setattr(mission.runner, 'service_command', lambda *_: None)
    monkeypatch.setattr(mission.runner, 'ctl', lambda *_: {'holding': True})
    monkeypatch.setattr(mission.runner, 'stop', lambda *_: None)
    monkeypatch.setattr(mission, 'run_stage', lambda host, argv: stages.append(argv[1]) or 0)
    def failed(*_):
        raise mission.MissionError('AMCL not ready')
    monkeypatch.setattr(mission, 'transit_to_visual_handoff', failed)
    monkeypatch.setattr(mission, 'back_out', lambda *_: pytest.fail('no back out without release'))
    assert mission.main(['--host', '10.0.0.1', '--real', '--i-am-beside-the-robot']) == 2
    assert stages == ['sugarbox']
