import importlib.util
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('boot_chassis', ROOT/'grasp/v23/chassis_server.py')
cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)


def test_never_driven_boot_reports_finite_observed_stop_without_changing_arm_gate():
    now = [100.]
    board = SimpleNamespace(set_motor=lambda *_: None)
    ch = cs.Chassis(board, lambda *_: (0, 0, 0, 0), clock=lambda: now[0], log=lambda _: None)
    ch.set_arm_pose('travel')
    ch.stop()
    assert ch.status()['stopped_for_s'] == 0.
    assert ch.begin_arm() is None  # Existing never-driven boot permission stays unchanged.
    ch.end_arm(); now[0] += 2; ch.stop()
    assert ch.status()['stopped_for_s'] == 2.


def test_raw_map_hash_can_bind_exact_robot_yaml_with_absolute_linux_image_path(tmp_path):
    from integration.g2_aruco_corner_seed import map_hash
    image = tmp_path/'map.pgm'; image.write_bytes(b'actual raster')
    meta = tmp_path/'robot.yaml'; meta.write_text('image: /home/jetson/map.pgm\n')
    import hashlib
    digest = hashlib.sha256(meta.read_bytes()+image.read_bytes()+b'lidar_forward_m=0.10').hexdigest()
    assert map_hash(meta, image) == digest
