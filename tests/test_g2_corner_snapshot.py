"""Execute the native capture with fake ROS callbacks, no AMCL or hardware."""
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'deploy/ros'))
sys.path.insert(0, str(ROOT/'integration'))
spec = importlib.util.spec_from_file_location('corner_capture', ROOT/'deploy/ros/g2_corner_snapshot.py')
capture = importlib.util.module_from_spec(spec); spec.loader.exec_module(capture)


def runtime(monkeypatch, tmp_path, bad=None):
    clock = [0.]
    health = dict(ok=True, chassis_tcp_ready=True, service_pid=7,
        chassis=dict(motors=[0, 0, 0, 0], moving=False, arm_busy=False, arm_pose='travel',
                     client=None, stopped_for_s=10., board_rx_age_s=.01),
        odom=dict(pose=[0., 0., 0.], twist=[0., 0., 0.], valid=True, failures=0))
    if bad == 'busy': health['chassis']['arm_busy'] = True
    stamp = NS(to_sec=lambda: 100.)
    scan = NS(header=NS(frame_id='laser', stamp=stamp), ranges=[1.]*500, range_min=.1,
              range_max=8., angle_min=-math.pi, angle_increment=.01)
    pos = NS(x=0., y=0.); quat = NS(x=0., y=0., z=0., w=1.)
    odom = NS(header=NS(frame_id='odom', stamp=stamp), child_frame_id='base_footprint',
              pose=NS(pose=NS(position=pos, orientation=quat)),
              twist=NS(twist=NS(linear=NS(x=0., y=0.), angular=NS(z=0.))))
    if bad == 'frames': scan.header.frame_id = 'wrong'
    def subscribe(topic, _type, callback, **_kwargs):
        callback(scan if topic == '/scan' else odom)
    rospy = NS(init_node=lambda *_a, **_k: None, Subscriber=subscribe,
               Time=lambda x: x, is_shutdown=lambda: False)
    transform = NS(lookupTransform=lambda *_: ([.13 if bad == 'geometry' else .10, 0., .095], [0., 0., 1., 0.]))
    tf = NS(TransformListener=lambda: transform, LookupException=LookupError,
            ConnectivityException=ConnectionError, ExtrapolationException=ArithmeticError,
            transformations=NS(euler_from_quaternion=lambda q: [0., 0., math.pi if q[2] == 1 else 0.]))
    monkeypatch.setitem(sys.modules, 'rospy', rospy); monkeypatch.setitem(sys.modules, 'tf', tf)
    monkeypatch.setitem(sys.modules, 'sensor_msgs', NS()); monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(LaserScan=object))
    monkeypatch.setitem(sys.modules, 'nav_msgs', NS()); monkeypatch.setitem(sys.modules, 'nav_msgs.msg', NS(Odometry=object))
    monkeypatch.setattr(capture, 'rpc', lambda _: health)
    monkeypatch.setattr(capture, 'available_mb', lambda: 700 if bad == 'ram' else 1500)
    monkeypatch.setattr(capture, 'map_hash', lambda _: 'raw-map')
    fake = NS(returncode=0, communicate=lambda: (b'7 8' if bad == 'owners' else b'7', b'/dev/myserial:'))
    monkeypatch.setattr(capture.subprocess, 'Popen', lambda *_a, **_k: fake)
    monkeypatch.setattr(capture.time, 'time', lambda: 100.+clock[0])
    monkeypatch.setattr(capture.time, 'sleep', lambda dt: clock.__setitem__(0, clock[0]+dt))
    def open_file(path, *args, **kwargs):
        if str(path) == '/proc/uptime': return io.StringIO(str(clock[0])+' 0')
        if str(path) == '/proc/sys/kernel/random/boot_id': return io.StringIO('boot')
        return open(path, *args, **kwargs)
    monkeypatch.setattr(capture, 'open', open_file, raising=False)
    out = tmp_path/'capture.json'
    monkeypatch.setattr(sys, 'argv', ['capture', '--map', str(tmp_path/'raw.yaml'), '--output', str(out)])
    return out


def test_stopped_capture_needs_no_amcl_and_binds_real_geometry(monkeypatch, tmp_path):
    out = runtime(monkeypatch, tmp_path)
    assert capture.main() == 0
    data = json.loads(out.read_text())
    assert data['map_hash'] == 'raw-map' and data['owner_pid'] == 7 and data['boot_id'] == 'boot'
    assert data['capture_id'] and data['scan']['frame'] == 'laser' and len(data['scan']['ranges']) == 500
    assert data['lidar_forward_m'] == .10 and data['lidar_yaw_rad'] == math.pi


@pytest.mark.parametrize('bad', ['busy', 'ram', 'owners', 'frames', 'geometry'])
def test_invalid_capture_never_replaces_trusted_file(monkeypatch, tmp_path, bad):
    out = runtime(monkeypatch, tmp_path, bad)
    out.write_text('previous trusted capture')
    with pytest.raises((ValueError, RuntimeError)):
        capture.main()
    assert out.read_text() == 'previous trusted capture'
