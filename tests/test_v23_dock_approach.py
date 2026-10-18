import math
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from unittest.mock import Mock
import pytest
from integration.v23_dock_approach import DockController
from integration.resident_motion import ResidentMotion


def config():
    return dict(role='dropoff',target_id='aruco:7',calibration_id='camera-v1',calibration_verified=True,standoff_m=.30)


def observation(**over):
    o=dict(valid=True,role='dropoff',target_id='aruco:7',calibration_id='camera-v1',
           frame='base_footprint',coordinate_convention='x_forward_y_left',stamp=100.,seq=1,x_m=.5,y_m=0.)
    o.update(over);return o


@pytest.mark.parametrize('over',[dict(calibration_verified=False),dict(role='grasp'),dict(standoff_m=.01),dict(target_id='')])
def test_unverified_configuration_cannot_drive(over):
    c=config();c.update(over)
    with pytest.raises(ValueError):DockController(c)


@pytest.mark.parametrize('over',[dict(role='grasp'),dict(target_id='sugarbox'),dict(calibration_id='other'),
 dict(stamp=99.),dict(stamp=101.),dict(valid=False),dict(x_m=float('nan')),
 dict(frame='camera'),dict(coordinate_convention='x_forward_y_right'),dict(seq=True)])
def test_wrong_target_stale_or_wrong_frame_stops(over):
    result=DockController(config()).step(observation(**over),100.)
    assert result['vx']==result['wz']==0 and not result['done']


def test_left_target_turns_left_and_no_forward_while_aligning():
    result=DockController(config()).step(observation(y_m=.15),100.)
    assert result['wz']==1 and result['vx']==0


def test_same_frame_cannot_republish_old_motion():
    c=DockController(config());o=observation(y_m=.15)
    assert c.step(o,100.)['wz']>0
    assert c.step(o,100.1)['wz']==0


def test_arrival_requires_three_distinct_fresh_frames():
    c=DockController(config())
    for i in range(3):
        r=c.step(observation(x_m=.30,seq=i+1,stamp=100.+i*.1),100.+i*.1)
        assert r['done']==(i==2)
        assert r['vx']==r['wz']==0


def test_target_loss_resets_arrival_confirmation():
    c=DockController(config());c.step(observation(x_m=.3),100.)
    c.step(None,100.1)
    assert not c.step(observation(x_m=.3,seq=2,stamp=100.2),100.2)['done']


def test_closer_than_standoff_never_reverses_blindly():
    with pytest.raises(ValueError,match='too close'):DockController(config()).step(observation(x_m=.25),100.)


def test_transport_routes_to_resident_and_stops_on_guard_error():
    def reject():raise RuntimeError('stale odom')
    sock=Mock();m=ResidentMotion(guard=reject);m.sock=sock
    with pytest.raises(RuntimeError):m.set_car_motion(.08,0,0)
    assert all(b'velocity' not in args[0] for args,_ in sock.sendall.call_args_list)
    assert sock.sendall.call_args[0][0]==b'{"action":"stop"}\n'


def test_transport_has_no_unguarded_live_mode_or_lateral_command():
    m=ResidentMotion();m.sock=Mock()
    with pytest.raises(RuntimeError):m.set_car_motion(.08,0,0)
    with pytest.raises(ValueError):m.set_car_motion(0,.1,0)


def test_legacy_navigator_rejects_serial_handle_before_camera_or_model_load():
    from integration.vision_grasp_pipeline import Navigator
    with pytest.raises(ValueError,match='resident TCP'):Navigator(Mock(resident_transport=False),None)


def test_legacy_real_nav_cannot_open_serial():
    from pathlib import Path
    import ast
    tree=ast.parse((Path(__file__).resolve().parents[1]/'integration/nav_rl_grasp_pipeline.py').read_text(encoding='utf-8'))
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='run_pipeline')
    first=function.body[0]
    assert isinstance(first,ast.If) and isinstance(first.body[0],ast.Raise)
