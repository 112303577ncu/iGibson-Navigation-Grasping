import importlib.util
import math
from pathlib import Path
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'integration'))
from g2_patrol_policy import Policy,read_pgm,wrap
spec=importlib.util.spec_from_file_location('map_patrol',ROOT/'deploy/ros/g2_map_patrol.py')
patrol=importlib.util.module_from_spec(spec);spec.loader.exec_module(patrol)


def test_real_scene_opposite_corner_route_stays_in_mask():
    p=Policy(str(ROOT/'maps/demo_box_20261017'))
    goals=p.patrol_points()
    assert len(goals)==4
    assert all(p.segment_allowed(a,b) for a,b in zip(goals,goals[1:]+goals[:1]))
    box=p.layout['bin']['pose_map']
    distances=[math.hypot(x-box['x'],y-box['y']) for x,y in goals]
    assert distances.index(max(distances))==0


def test_moved_box_envelope_requires_inward_patrol_route():
    p=Policy(str(ROOT/'maps/demo_box_20261018_patrol'))
    with pytest.raises(ValueError,match='patrol segment'):
        p.patrol_points(.60)
    points=p.patrol_points(.70)
    assert all(p.segment_allowed(a,b) for a,b in zip(points,points[1:]+points[:1]))
    runner=patrol.Runner.__new__(patrol.Runner);runner.policy=p
    assert all(runner.rotation_allowed(point) for point in points)


def test_uncertain_box_envelope_does_not_enable_exact_docking():
    p=Policy(str(ROOT/'maps/demo_box_20261018_patrol'))
    p.require_motion_ready(.10)
    with pytest.raises(ValueError,match='physical box registration'):
        p.enable_dock_zone(.10)
    u0,u1,v0,v1=p.layout['navigation_mask_validation']['envelope_room_uv']
    angle=p.layout['room_heading_rad'];c,s=math.cos(angle),math.sin(angle)
    for u in (u0,(u0+u1)/2,u1):
        for v in (v0,(v0+v1)/2,v1):
            assert not p.allowed(c*u-s*v,s*u+c*v)


@pytest.mark.parametrize('inset',['nan','.59','.91'])
def test_patrol_cli_rejects_inset_before_hardware(monkeypatch,inset):
    monkeypatch.setattr(sys,'argv',['g2_map_patrol.py','--scene','unused',
                                   '--output','unused','--inset',inset])
    with pytest.raises(SystemExit) as exc:
        patrol.main()
    assert exc.value.code==2


def test_final_facing_rotates_without_approaching_point():
    runner=patrol.Runner.__new__(patrol.Runner);runner.report={'recoveries':[]}
    pose=[0,0,0];calls=[]
    runner.ready=lambda:{'pose':pose}
    def pulse(vx,wz,duration):
        calls.append((vx,wz,duration));pose[2]=math.pi/2
    runner.pulse=pulse
    runner.face_point([0,10])
    assert calls and all(vx==0 and wz==1 for vx,wz,_ in calls)
    assert runner.report['park_facing']=='requested_point'
    assert runner.report['park_yaw_error_deg']==pytest.approx(0)


def test_parking_cannot_reuse_arrival_before_turn_drift():
    runner=patrol.Runner.__new__(patrol.Runner);runner.report={
        'arrivals':[{'error_m':.044}]}
    runner.settle=lambda:{'pose':[.08,0,0]}
    with pytest.raises(RuntimeError,match='final turn displaced'):
        runner.verify_parked_goal([0,0])
    assert runner.report['parked_goal_valid'] is False
    assert runner.report['final_goal_error_m']==pytest.approx(.08)
    runner.settle=lambda:{'pose':[.04,0,0]}
    runner.verify_parked_goal([0,0])
    assert runner.report['parked_goal_valid'] is True


def test_outer_route_retains_wall_box_and_turn_limits():
    import json
    p=Policy(str(ROOT/'maps/demo_box_20261018_patrol'))
    goals=json.loads((ROOT/'docs/calibration/nav_outer_2026-10-18/outer_goals.json').read_text())
    points=[g['point'] for g in goals]
    runner=patrol.Runner.__new__(patrol.Runner);runner.policy=p
    assert all(runner.rotation_allowed(point) for point in points)
    assert all(p.segment_allowed(a,b) for a,b in zip(points,points[1:]))
    # Forward braking projection remains legal even near each outward corner.
    for a,b in zip(points,points[1:]):
        yaw=math.atan2(b[1]-a[1],b[0]-a[0])
        before=[b[0]-.045*math.cos(yaw),b[1]-.045*math.sin(yaw),yaw]
        assert p.stopping_path_allowed(before,.074)


def test_final_heading_modes_cannot_conflict(monkeypatch):
    monkeypatch.setattr(sys,'argv',['g2_map_patrol.py','--scene','unused',
        '--output','unused','--keep-heading','--face-point','0','1'])
    with pytest.raises(SystemExit) as exc:patrol.main()
    assert exc.value.code==2


@pytest.mark.parametrize('front,duration,allowed',[(.40,.04,True),(.40,.15,False),(.34,.04,False)])
def test_boundary_micro_keeps_long_pulse_and_close_wall_brakes(front,duration,allowed):
    runner=patrol.Runner.__new__(patrol.Runner);runner.boundary_micro=True
    runner.report={'pulses':0,'trace':[]};calls=[]
    state={'odom':[0,0,0],'pose':[0,0,0],'front':front}
    runner.ready=lambda:state;runner.state=lambda:state;runner.settle=lambda:state
    runner.stop=lambda:calls.append('stop')
    class PolicyStub:
        def stopping_path_allowed(self,*args):return True
    class SocketStub:
        def sendall(self,payload):calls.append(payload)
    runner.policy=PolicyStub();runner.sock=SocketStub()
    if allowed:
        runner.pulse(.15,0,duration)
        assert any(isinstance(item,bytes) and b'velocity' in item for item in calls)
    else:
        with pytest.raises(patrol.Transient,match='front brake'):runner.pulse(.15,0,duration)
        assert calls==['stop']


def test_map_outside_and_paper_box_are_forbidden():
    p=Policy(str(ROOT/'maps/demo_box_20261017'))
    assert not p.allowed(-100,100)
    assert not p.allowed(float('nan'),0)
    b=p.layout['bin']['pose_map'];assert not p.allowed(b['x'],b['y'])


def test_segment_checks_middle_not_only_endpoints():
    p=Policy.__new__(Policy);p.resolution=.02
    p.allowed=lambda x,y:not .45<x<.55
    assert not p.segment_allowed((0,0),(1,0))


def test_predicted_stopping_path_cannot_cross_limit():
    p=Policy.__new__(Policy);p.resolution=.02;p.allowed=lambda x,y:x<.5
    assert p.stopping_path_allowed((.1,0,0),.2)
    assert not p.stopping_path_allowed((.4,0,0),.2)
    assert not p.stopping_path_allowed((0,0,0),float('nan'))


def test_pgm_rejects_incomplete_data(tmp_path):
    f=tmp_path/'mask.pgm';f.write_bytes(b'P5\n2 2\n255\n\xff')
    with pytest.raises(ValueError):read_pgm(str(f))


def test_front_wall_retained_while_body_and_side_points_excluded():
    wall=[(math.pi+x,.3) for x in (-.003,0,.003)]
    assert patrol.front_distance(wall)<.5
    assert math.isinf(patrol.front_distance([(0,.13),(math.pi/2,.4)]))


def test_live_command_stops_on_sensor_exception():
    r=patrol.Runner.__new__(patrol.Runner);r.report={'pulses':0};r.ready=lambda:{'odom':[0,0,0]}
    r.state=lambda:(_ for _ in ()).throw(patrol.Transient('stale scan'))
    calls=[];r.stop=lambda:calls.append('stop')
    with pytest.raises(patrol.Transient):r.pulse(.15,0,.1)
    assert calls==['stop']


def test_wrapped_yaw_crosses_pi_short_way():
    assert wrap(math.radians(-358))==pytest.approx(math.radians(2))


def test_front_brake_prevents_any_velocity_write():
    r=patrol.Runner.__new__(patrol.Runner);r.report={'pulses':0}
    state={'odom':[0,0,0],'pose':[0,0,0],'front':.49}
    r.ready=lambda:state;r.state=lambda:state
    calls=[];r.stop=lambda:calls.append('stop')
    with pytest.raises(patrol.Transient,match='front brake'):r.pulse(.15,0,.1)
    assert calls==['stop']


def test_turn_requires_mask_reserve_and_stops_without_sending():
    r=patrol.Runner.__new__(patrol.Runner);r.report={'pulses':0}
    state={'odom':[0,0,0],'pose':[0,0,0],'front':1}
    r.ready=lambda:state;r.state=lambda:state;r.rotation_allowed=lambda _:False
    calls=[];r.stop=lambda:calls.append('stop')
    with pytest.raises(RuntimeError,match='turn drift reserve'):r.pulse(0,1,.1)
    assert calls==['stop']

def test_planner_routes_around_obstacle_without_crossing():
    p=Policy.__new__(Policy);p.resolution=.1;p.origin=(0,0,0)
    p.allowed=lambda x,y:0<=x<1 and 0<=y<1 and not (.4<=x<.6 and y<.7)
    route=p.plan((.15,.15),(.85,.15))
    assert len(route)>2
    assert all(p.segment_allowed(a,b) for a,b in zip(route,route[1:]))


def test_planner_rejects_disconnected_goal():
    p=Policy.__new__(Policy);p.resolution=.1;p.origin=(0,0,0)
    p.allowed=lambda x,y:0<=x<1 and 0<=y<1 and not .4<=x<.6
    with pytest.raises(ValueError,match='no legal route'):p.plan((.15,.15),(.85,.15))


def test_heading_corrects_cross_track_toward_original_line():
    from g2_patrol_policy import guided_heading
    assert guided_heading((.5,.06),(0,0),(1,0))<0
    assert guided_heading((.5,-.06),(0,0),(1,0))>0


def test_all_requested_goals_validated_before_motion():
    p=Policy(str(ROOT/'maps/demo_box_20261017'));points=p.patrol_points()
    assert patrol.resolve_goals(['home'],p,points)[0][1]==list(points[0])
    with pytest.raises(ValueError):patrol.resolve_goals(['home',{'point':[100,100]}],p,points)
    with pytest.raises(ValueError,match='not yet validated'):patrol.resolve_goals(['dropoff'],p,points)

def test_dock_stage_keeps_turn_room_before_close_approach():
    sys.path.insert(0,str(ROOT/'deploy/ros'))
    from g2_dock_probe import dock_staging
    p=Policy(str(ROOT/'maps/demo_box_20261017'));stage,goal,yaw=dock_staging(p)
    r=patrol.Runner.__new__(patrol.Runner);r.policy=p
    assert p.segment_allowed(stage,goal)
    assert r.rotation_allowed(stage)
    assert not r.rotation_allowed(goal)
    assert math.hypot(stage[0]-goal[0],stage[1]-goal[1])==pytest.approx(.18)

def test_special_dock_mode_rejects_long_forward_pulse_and_stops():
    r=patrol.Runner.__new__(patrol.Runner);r.report={'pulses':0};r.max_forward_duration=.08
    r.ready=lambda:{'odom':[0,0,0]}
    calls=[];r.stop=lambda:calls.append('stop')
    with pytest.raises(RuntimeError,match='duration limit'):r.pulse(.15,0,.15)
    assert calls==['stop']


def test_special_dock_mode_retains_its_front_brake():
    r=patrol.Runner.__new__(patrol.Runner);r.report={'pulses':0};r.front_brake_m=.35;r.max_forward_duration=.08
    state={'odom':[0,0,0],'pose':[0,0,0],'front':.34};r.ready=lambda:state;r.state=lambda:state
    calls=[];r.stop=lambda:calls.append('stop')
    with pytest.raises(patrol.Transient,match='front brake'):r.pulse(.15,0,.08)
    assert calls==['stop']

def test_final_dock_approach_refuses_sideways_goal_without_motion():
    from g2_dock_probe import approach_without_turn
    r=patrol.Runner.__new__(patrol.Runner);r.ready=lambda:{'pose':[0,0,0]}
    calls=[];r.pulse=lambda *a:calls.append(a)
    with pytest.raises(RuntimeError,match='alignment'):approach_without_turn(r,[.1,.05])
    assert calls==[]


def test_final_dock_geometry_preserves_forward_and_lateral_axes():
    from g2_dock_probe import final_approach_geometry
    d,a,l=final_approach_geometry([0,0,math.pi/2],[.01,.1])
    assert a==pytest.approx(.1)
    assert l==pytest.approx(-.01)
    assert d==pytest.approx(math.hypot(.01,.1))

def test_known_physical_box_mismatch_blocks_further_dock_commands():
    from g2_dock_probe import require_box_registration
    p=Policy(str(ROOT/'maps/demo_box_20261017'))
    with pytest.raises(ValueError,match='re-register'):require_box_registration(p)

def test_corrected_lidar_offset_and_tf_agree():
    import xml.etree.ElementTree as ET
    from g2_geometry import LIDAR_FORWARD_M
    root=ET.parse(str(ROOT/'deploy/ros/x3plus_tg30_navigation.launch')).getroot()
    node=next(n for n in root.findall('node') if n.attrib['name']=='base_link_to_laser')
    assert float(node.attrib['args'].split()[0])==LIDAR_FORWARD_M==.10
    assert patrol.front_distance([(math.pi,.3)]*3)==pytest.approx(.4)


def test_old_map_geometry_blocks_motion_until_revalidated():
    p=Policy(str(ROOT/'maps/demo_box_20261017'))
    with pytest.raises(ValueError,match='geometry'):p.require_geometry(.10)
    p.layout['sensor_geometry']['validation']={'verified':True,'lidar_forward_m':.10}
    p.require_geometry(.10)


def test_local_dock_zone_cannot_activate_without_box_registration():
    p=Policy(str(ROOT/'maps/demo_box_20261017'))
    p.layout['sensor_geometry']['validation']={'verified':True,'lidar_forward_m':.10}
    with pytest.raises(ValueError,match='registration'):p.enable_dock_zone(.10)


def test_draft_dock_zone_preserves_box_keepout_and_general_margin():
    p=Policy(str(ROOT/'maps/demo_box_20261017'));normal=list(p.cells)
    p.layout['sensor_geometry']['validation']={'verified':True,'lidar_forward_m':.10}
    p.layout['bin']['registration_verified']=True;p.layout['bin']['dock_zone']['enabled']=True
    p.enable_dock_zone(.10)
    assert list(p.cells)==normal
    assert sum(a<=250 and b>250 for a,b in zip(p.cells,p.dock_cells))==34
    b=p.layout['bin']['pose_map'];assert not p.allowed(b['x'],b['y'])
    # Local wall relaxation must not erase the separate paper-box keepout.
    assert not p.stopping_path_allowed([1.3096433981950129,.7760871813481001,.19371492425441295],.074)

def test_moved_box_blocks_old_route_even_after_geometry_validation():
    p=Policy(str(ROOT/'maps/demo_box_20261017'))
    p.layout['sensor_geometry']['validation']={'verified':True,'lidar_forward_m':.10}
    with pytest.raises(ValueError,match='box moved'):p.require_motion_ready(.10)


def test_transient_sensor_recovery_requires_five_distinct_odom_frames(monkeypatch):
    from unittest.mock import Mock
    r=patrol.Runner.__new__(patrol.Runner);r.stop=Mock();r.report={}
    r.end=1000.  # normal Runner construction supplies the task deadline
    stamps=[1,1,2,3,4,5];calls=[patrol.Transient('stale AMCL')]+[{'odom_stamp':n} for n in stamps]
    def state():
        value=calls.pop(0)
        if isinstance(value,Exception):raise value
        return value
    r.state=state;monkeypatch.setattr(patrol.time,'sleep',lambda _:None)
    assert r.ready()['odom_stamp']==5
    assert r.stop.call_count==1 and r.recovery_generation==1
    assert r.report['sensor_wait_elapsed_s']==0.
    assert r.report['last_sensor_wait_elapsed_s']>=0.


@pytest.mark.parametrize('degraded,stamp,kind', [(True,100,RuntimeError),(False,100,patrol.Transient),(True,90,patrol.Transient)])
def test_supervisor_deadline_is_not_followed_by_another_navigation_wait(monkeypatch,degraded,stamp,kind):
    import io,json
    r=patrol.Runner.__new__(patrol.Runner);r.end=200;r.pid=7
    report=dict(owner_pid=7,ready=False,stamp=stamp,degraded=degraded,
                reason='stale scan',reason_code='SCAN_STALE',next_action='wait_for_fresh_scan')
    monkeypatch.setattr(patrol,'open',lambda *_:io.StringIO(json.dumps(report)),raising=False)
    monkeypatch.setattr(patrol.time,'time',lambda:100)
    with pytest.raises(kind) as failure:r.state()
    if degraded and stamp==100:
        assert type(failure.value) is RuntimeError
        assert 'SCAN_STALE' in str(failure.value) and 'wait_for_fresh_scan' in str(failure.value)


@pytest.mark.parametrize('corruption', [None, 'mask', 'margin', 'reserve', 'bin'])
def test_35cm_scene_requires_the_matching_approved_mask(tmp_path, corruption):
    import json
    import shutil
    source = ROOT / 'maps/demo_box_20261018_patrol'
    for name in ('navigation_center_mask.yaml', 'navigation_center_mask.pgm', 'scene_layout.json'):
        shutil.copyfile(source / name, tmp_path / name)
    path = tmp_path / 'scene_layout.json'
    layout = json.loads(path.read_text())
    if corruption == 'mask':
        mask = tmp_path / 'navigation_center_mask.pgm'
        data = bytearray(mask.read_bytes()); data[-1] ^= 255; mask.write_bytes(data)
    if corruption == 'margin': layout['center_wall_margin_m'] = .349
    if corruption == 'reserve': layout['navigation_mask_reduction_20261021']['turn_reserve_m'] = .01
    if corruption == 'bin': layout['bin']['center_keepout_buffer_m'] = .1
    path.write_text(json.dumps(layout))
    if corruption:
        with pytest.raises(ValueError): Policy(str(tmp_path))
    else:
        policy = Policy(str(tmp_path))
        assert policy.layout['center_wall_margin_m'] == .35
        assert sum(v > 250 for v in policy.cells) == 3262
