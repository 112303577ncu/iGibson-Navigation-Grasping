import importlib.util
import math
from pathlib import Path
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'deploy/ros'))
spec=importlib.util.spec_from_file_location('relative_dock',ROOT/'deploy/ros/g2_relative_dock_probe.py')
dock=importlib.util.module_from_spec(spec);spec.loader.exec_module(dock)


def test_measured_gap_bounds_advance_without_changing_map_box_registration():
    target,offset=dock.approach_target([0,0,0],[.04,.01,0],.35,.30)
    assert target==pytest.approx(.26)
    assert offset==pytest.approx(.04)
    assert .35-offset-target>=.04


@pytest.mark.parametrize('pose',[[0,.07,0],[.09,0,0],[0,0,math.radians(7)]])
def test_unaligned_or_relocated_measurement_rejected(pose):
    with pytest.raises(ValueError,match='not aligned'):
        dock.approach_target([0,0,0],pose,.35,.30)


@pytest.mark.parametrize('gap,advance',[(.32,.30),(.35,.31),(float('nan'),.30)])
def test_long_or_unknown_gap_command_rejected(gap,advance):
    with pytest.raises(ValueError):dock.approach_target([0,0,0],[0,0,0],gap,advance)


def test_progress_uses_start_heading_and_wraps_yaw():
    along,side,angle=dock.components([0,0,math.pi/2],[0,.1,math.pi/2])
    assert along==pytest.approx(.1)
    assert side==pytest.approx(0,abs=1e-12)
    assert angle==pytest.approx(0)
    assert dock.components([0,0,math.radians(179)],[0,0,math.radians(-179)])[2]==pytest.approx(math.radians(2))


def test_raw_unknown_cell_in_body_disk_is_not_cleared_by_operator_measurement():
    disk=dock.FreeDisk.__new__(dock.FreeDisk)
    disk.w=disk.h=50;disk.resolution=.04;disk.origin=(-1,-1,0)
    disk.cells=bytearray([255]*2500);disk.cache={}
    assert disk.allowed(0,0)
    disk.cells[24*50+31]=205;disk.cache={}
    assert not disk.allowed(0,0)


def test_wall_margin_not_relaxed_by_relative_docking():
    room=[[0,0],[2,0],[2,2],[0,2]]
    assert dock.wall_clearance(room,[.5,.5])==pytest.approx(.5)
    assert dock.wall_clearance(room,[.39,1])<.4
    assert dock.wall_clearance(room,[-.1,1])<0


def interrupted_reference():
    reference={'pose_map':[0,0,0],'operator_front_gap_range_m':[.35,.40],
               'requested_advance_m':.30,'service_pid':123}
    previous={'ok':False,'real':True,'initial':{'pose':[0,0,0],'odom':[0,0,0]},
              'target_from_start_m':.30,
              'final_health':{'service_pid':123,'odom':{'pose':[.26,0,0]}}}
    return previous,reference


def test_resume_retains_original_30cm_target_instead_of_another_30cm():
    previous,reference=interrupted_reference()
    origin,target,_=dock.resume_parameters(previous,reference,[.26,0,0])
    assert target-dock.components(origin,[.26,0,0])[0]==pytest.approx(.04)


def test_resume_rejects_moved_robot_owner_or_measurement():
    previous,reference=interrupted_reference()
    with pytest.raises(ValueError,match='robot moved'):
        dock.resume_parameters(previous,reference,[.30,0,0])
    previous['final_health']['service_pid']=456
    with pytest.raises(ValueError,match='owner changed'):
        dock.resume_parameters(previous,reference,[.26,0,0])
    previous,reference=interrupted_reference();reference['requested_advance_m']=.25
    with pytest.raises(ValueError,match='measurement changed'):
        dock.resume_parameters(previous,reference,[.26,0,0])
