import math
import sys
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'integration'))
from g2_localization_policy import propagated_seed,restart_pose,RestartBudget,turn_clear,FaultDelay


def seed():
    return dict(stamp=100,boot_id='boot',owner_pid=7,map_hash='map',pose=[1,2,math.pi/2],odom=[0,0,0])


def test_restart_propagates_odom_in_map_frame():
    assert propagated_seed(seed(),[.05,0,.02],105,'boot',7,'map')==pytest.approx([1,2.05,math.pi/2+.02])


@pytest.mark.parametrize('boot,owner,digest', [('new',7,'map'),('boot',8,'map'),('boot',7,'new')])
def test_reboot_owner_or_map_invalidates_seed(boot,owner,digest):
    with pytest.raises(ValueError):propagated_seed(seed(),[0,0,0],105,boot,owner,digest)


@pytest.mark.parametrize('odom,now', [([0,0,0],116),([0,0,0],99),([.11,0,0],105),([0,0,.2],105),([float('nan'),0,0],105)])
def test_old_or_moved_reference_never_restarts(odom,now):
    with pytest.raises(ValueError):propagated_seed(seed(),odom,now,'boot',7,'map')


def test_restart_has_cooldown_and_window_budget():
    b=RestartBudget()
    assert b.take(0)
    assert not b.take(1)
    assert b.take(10) and b.take(20)
    assert not b.take(30)
    assert b.take(601)


def test_new_independent_reference_survives_processing_time_but_not_expiry():
    assert restart_pose(seed(),[0,0,0],160,'boot',7,'map',True)==pytest.approx(seed()['pose'])
    with pytest.raises(ValueError):restart_pose(seed(),[0,0,0],221,'boot',7,'map',True)


def test_internal_recovery_keeps_short_expiry_after_localization_accepted():
    with pytest.raises(ValueError):restart_pose(seed(),[0,0,0],116,'boot',7,'map',False)


@pytest.mark.parametrize('odom,boot,owner,digest', [([.11,0,0],'boot',7,'map'),([0,0,.2],'boot',7,'map'),([0,0,0],'new',7,'map'),([0,0,0],'boot',8,'map'),([0,0,0],'boot',7,'new')])
def test_external_reference_does_not_relax_motion_or_identity(odom,boot,owner,digest):
    with pytest.raises(ValueError):restart_pose(seed(),odom,160,boot,owner,digest,True)


def test_service_has_no_wall_clock_expiry_or_serial_access():
    unit=(ROOT/'deploy/systemd/x3plus-g2-localization.service').read_text()
    assert 'RuntimeMaxSec' not in unit
    assert 'Restart=on-failure' in unit and 'MemoryMax=256M' in unit
    source=(ROOT/'deploy/ros/g2_localization_service.py').read_text()
    assert 'socket.create_connection' not in source and 'Rosmaster(' not in source


def test_turn_clearance_keeps_close_cluster_but_rejects_single_noise():
    points=[(math.pi+x,.15) for x in (-.003,0,.003)]
    assert not turn_clear(points)
    assert turn_clear(points[:1])
    assert turn_clear([(math.pi,.8)]*3)


def test_short_sensor_outage_does_not_restart_localization():
    delay=FaultDelay(8)
    assert not delay.update(True,100)
    assert not delay.update(True,100.4)
    assert not delay.update(False,101)
    assert not delay.update(True,200)
    assert delay.update(True,208)
