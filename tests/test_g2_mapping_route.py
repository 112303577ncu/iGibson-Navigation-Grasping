import math
import json
import time
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from integration import g2_mapping_route as route


def pose(yaw=0, x=0, y=0):
    return route.nav.Odom(x,y,yaw,0,0,0,0,'odom','base_footprint')


def test_left_rotation_across_wrap():
    assert route.bounded_pose(pose(math.radians(170)),pose(math.radians(-100))) == pytest.approx(90)


@pytest.mark.parametrize('end',[pose(math.radians(-2)),pose(math.radians(96)),pose(x=.051)])
def test_bad_rotation_stops(end):
    with pytest.raises(RuntimeError):
        route.bounded_pose(pose(),end)


def test_live_pulse_closes_client_on_sensor_failure(monkeypatch):
    calls = []
    sample = SimpleNamespace(points=[],stamp=1)
    reads = iter([(sample,pose()),(sample,pose())])
    def check(*args):
        try:
            return next(reads)
        except StopIteration:
            raise RuntimeError('stale')
    class Client:
        def __init__(self,direction):
            calls.append(('open',direction))
        def velocity(self):
            calls.append('velocity')
        def close(self):
            calls.append('close')
    monkeypatch.setattr(route,'checked',check)
    monkeypatch.setattr(route.turn,'TurnClient',Client)
    with pytest.raises(RuntimeError,match='stale'):
        route.left(None,{},True)
    assert calls == [('open',1),'close']


def test_forward_wall_is_kept_in_session_self_override():
    cfg=route.config()
    assert cfg.lidar_forward_offset_m == .10
    assert route.nav.nr.front_min_brake([(179.8,.30),(180,.30),(180.2,.30)],cfg) < cfg.safety_brake_dist


def test_native_snapshot_read_does_not_refresh_stale_data(tmp_path):
    wall=time.time()
    stamp={'secs':int(wall),'nsecs':int((wall-int(wall))*1e9)}
    data={'written_at':wall-2,'scan':{'points':[[180,1]],'stamp':wall,'frame':'laser'},
          'odom':{'header':{'stamp':stamp,'frame_id':'odom'},'child_frame_id':'base_footprint',
                  'pose':{'pose':{'position':{'x':0,'y':0},'orientation':{'x':0,'y':0,'z':0,'w':1}}},
                  'twist':{'twist':{'linear':{'x':0},'angular':{'z':0}}}}}
    path=tmp_path/'snapshot.json';path.write_text(json.dumps(data))
    scan,odom,_,error=route.NativeSensors(path).snapshot()
    assert not error
    assert route.nav.health(scan,odom,route.config())[0]=='stale_arrival'


def test_native_missing_snapshot_fails_closed(tmp_path):
    scan,odom,_,error=route.NativeSensors(tmp_path/'missing').snapshot()
    assert scan is None and odom is None and error


def test_resume_refuses_robot_moved_since_failure(monkeypatch):
    monkeypatch.setattr(route,'checked',lambda *args:(None,pose(x=.1)))
    report={'resume_origin':route.dataclasses.asdict(pose()),'resume_expected':[0,0,0]}
    with pytest.raises(RuntimeError,match='robot moved'):
        route.left(None,report,True)


def test_parking_waits_for_board_settle_then_records_status(monkeypatch):
    calls=[]
    states=iter([True,False])
    def rpc(command):
        calls.append(command)
        if command=='health':
            return {'chassis':{'motors':[0,0,0,0],'moving':next(states)}}
        return {'ok':True}
    monkeypatch.setattr(route,'rpc',rpc)
    monkeypatch.setattr(route.time,'sleep',lambda _:None)
    report={};route.park_report(report)
    assert calls==['health','health','maintenance-lock','status']
    assert report['lock_verified'] and report['final_status']['ok']
