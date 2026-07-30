from __future__ import annotations

import importlib
import math
import os
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np

from grasp import x3plus_real_grasp as real_grasp
from grasp.x3plus_real_grasp import (
    DeployConfig,
    DetectionReceiver,
    GraspController,
    JointMapper,
    ServoController,
    arm_limit_margin_violations,
    validate_grasp_home_reach,
)
from integration.nav_rl import (
    ActionDelay,
    GoalTracker,
    NavRLConfig,
    RosLaserScanSource,
    laser_scan_to_points,
    make_lidar,
    scan_to_rays,
)
from integration import vision_grasp_pipeline as vgp
from integration import nav_rl_grasp_pipeline as nrgp
from startup_device_check import camera_identity
from stream_cam import CameraState


class _FailingServoDevice:
    def set_uart_servo_angle_array(self, **_kwargs):
        raise OSError("simulated UART failure")


class _ClosedCapture:
    def isOpened(self):
        return False

    def release(self):
        pass


class _StaleDetection:
    def snapshot(self):
        return np.array([0.25, 0.0, 0.02], dtype=np.float32), None, False


class SafetyGuardTests(unittest.TestCase):
    def test_jetson_host_environment_override(self):
        with mock.patch.dict(os.environ, {"X3PLUS_JETSON_HOST": "robot-test.local"}):
            importlib.reload(vgp)
            self.assertEqual(vgp.JETSON_IP, "robot-test.local")
            self.assertIn("robot-test.local:8080", vgp.URL_ARM)
            self.assertIn("robot-test.local:8080", vgp.URL_REAR)
        importlib.reload(vgp)

    def test_detection_payload_rejects_non_finite_and_negative_width(self):
        pos, width = DetectionReceiver._parse_payload(
            b'{"x": 0.25, "y": 0.01, "z": 0.02, "w": 0.03}'
        )
        np.testing.assert_allclose(pos, [0.25, 0.01, 0.02])
        self.assertAlmostEqual(width, 0.03)
        with self.assertRaises(ValueError):
            DetectionReceiver._parse_payload(b'{"x": NaN, "y": 0, "z": 0.02}')
        with self.assertRaises(ValueError):
            DetectionReceiver._parse_payload(b'{"x": 0.25, "y": 0, "w": -0.01}')

    def test_servo_state_is_not_advanced_when_uart_write_raises(self):
        controller = ServoController(DeployConfig(), dry_run=True)
        original = controller._last_deg.copy()
        controller.dry_run = False
        controller._device = _FailingServoDevice()
        with self.assertRaises(RuntimeError):
            controller.send_degrees([93, 137, 3, 3, 93, 33], run_time_ms=100)
        self.assertEqual(controller._last_deg, original)
        controller._device = None

    def test_emergency_stop_holds_instead_of_moving_home(self):
        controller = ServoController(DeployConfig(), dry_run=True)
        current = [100.0, 120.0, 20.0, 30.0, 80.0, 60.0]
        controller._last_deg = current.copy()
        controller.emergency_stop()
        self.assertEqual(controller._last_deg, current)

    def test_long_glide_is_split_at_board_duration_limit(self):
        controller = ServoController(DeployConfig(), dry_run=True)
        target = controller._last_deg.copy()
        target[0] += 60.0
        with mock.patch("grasp.x3plus_real_grasp.time.sleep") as sleep:
            segments = controller.glide_to(target, 15.0, 800, 3000)
        self.assertEqual(segments, 2)
        self.assertEqual(controller._last_deg, target)
        self.assertTrue(all(call.args[0] <= 2.0 for call in sleep.call_args_list))

    def test_joint_mapper_rejects_nan_policy_action(self):
        mapper = JointMapper(DeployConfig())
        action = np.zeros(6, dtype=np.float32)
        action[2] = math.nan
        with self.assertRaises(ValueError):
            mapper.norm_action_to_sim_angles(action)

    def test_negative_action_delay_is_rejected(self):
        with self.assertRaises(ValueError):
            ActionDelay(-1)

    def test_goal_tracker_fix_age_honours_explicit_zero_time(self):
        tracker = GoalTracker()
        tracker._last_fix = -2.0
        self.assertEqual(tracker.fix_age(now=0.0), 2.0)

    def test_ros_laserscan_conversion_filters_invalid_ranges(self):
        points = laser_scan_to_points({
            "angle_min": -math.pi / 2,
            "angle_increment": math.pi / 4,
            "range_min": 0.1,
            "range_max": 10.0,
            "ranges": [1.0, math.inf, math.nan, 0.05, 2.0],
        })
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0][0], -90.0)
        self.assertAlmostEqual(points[1][0], 90.0)
        self.assertEqual([p[1] for p in points], [1.0, 2.0])
        with self.assertRaises(ValueError):
            laser_scan_to_points({
                "angle_min": 0.0,
                "angle_increment": 0.0,
                "range_min": 0.1,
                "range_max": 10.0,
                "ranges": [1.0],
            })

    def test_ros_scan_left_right_mapping_is_not_mirrored(self):
        cfg = NavRLConfig(lidar_angle_dir=1.0)
        right = scan_to_rays([(-85.0, 1.0)], cfg)
        left = scan_to_rays([(85.0, 1.0)], cfg)
        self.assertLessEqual(int(np.argmin(right)), 3)
        self.assertGreaterEqual(int(np.argmin(left)), 44)

    def test_ros_scan_source_updates_freshness_even_when_all_ranges_are_inf(self):
        source = object.__new__(RosLaserScanSource)
        source._points = []
        source._ts = 0.0
        source._ros_stamp = None
        source._lock = threading.Lock()
        source._last_error = ""
        source._on_scan({
            "header": {"stamp": {"secs": 12, "nsecs": 500000000}},
            "angle_min": -1.0,
            "angle_increment": 0.1,
            "range_min": 0.01,
            "range_max": 50.0,
            "ranges": [math.inf, math.inf],
        })
        self.assertEqual(source.get_points(), [])
        self.assertTrue(math.isfinite(source.age()))
        self.assertEqual(source.ros_stamp(), 12.5)

    def test_ros_scan_source_uses_latest_only_subscription_and_closes(self):
        state = SimpleNamespace(queue_length=None, callback=None, unsubscribed=False,
                                terminated=False)

        class FakeRos:
            def __init__(self, host, port):
                self.host, self.port, self.is_connected = host, port, False

            def run(self, timeout):
                self.is_connected = True

            def terminate(self):
                state.terminated = True

        class FakeTopic:
            def __init__(self, _client, _name, _type, queue_length):
                state.queue_length = queue_length

            def subscribe(self, callback):
                state.callback = callback

            def unsubscribe(self):
                state.unsubscribed = True

        fake_roslibpy = SimpleNamespace(Ros=FakeRos, Topic=FakeTopic)
        with mock.patch.dict(sys.modules, {"roslibpy": fake_roslibpy}):
            source = RosLaserScanSource("127.0.0.1", 9090, "/scan")
            self.assertEqual(state.queue_length, 1)
            self.assertIsNotNone(state.callback)
            source.close()
        self.assertTrue(state.unsubscribed)
        self.assertTrue(state.terminated)

    def test_legacy_rplidar_backend_requires_an_explicit_port(self):
        with self.assertRaises(ValueError):
            make_lidar(NavRLConfig(), "rplidar")

    def test_camera_capture_failure_is_reported_to_main_thread(self):
        state = CameraState("missing", 640, 480, 15.0, None)
        with mock.patch.object(state, "open_capture", return_value=_ClosedCapture()):
            state.capture_loop()
        self.assertTrue(state.startup.is_set())
        self.assertIn("cannot open camera", state.capture_error)

    def test_camera_identity_groups_video_indices(self):
        first = camera_identity({"ID_PATH": "usb-port-a-video-index0"})
        second = camera_identity({"ID_PATH": "usb-port-a-video-index1"})
        self.assertEqual(first, second)
        self.assertIsNone(camera_identity({}))

    def test_hardware_smoke_modules_are_import_safe(self):
        for name in (
            "detection.debug_tools.test",
            "grasp.servo_test",
            "grasp.fk_test",
            "grasp.workspace_scan",
        ):
            importlib.import_module(name)

    def test_camera_to_base_mapping_is_explicit_and_validated(self):
        nav = vgp.Navigator(
            None,
            None,
            dry_run=True,
            cam_to_base_x=0.12,
            cam_to_base_y=-0.01,
            sign_y=-1.0,
        )
        nav._last_arm = {
            "dist": 0.22,
            "offset": 0.03,
            "box_w_px": 90,
            "class_name": None,
        }
        pos, width = nav.latch_arm_object()
        self.assertEqual(pos, [0.34, -0.04, 0.02])
        self.assertGreater(width, 0.0)
        with self.assertRaises(ValueError):
            vgp.Navigator(None, None, sign_y=0.0)

    def test_pitched_arm_camera_uses_optical_depth_for_lateral_and_width(self):
        dist_m = 0.2613
        old_ground_scaled_offset = 0.0394
        cx_u = vgp.CX_ARM + old_ground_scaled_offset * vgp.FX_ARM / dist_m
        corrected = vgp.estimate_pitched_offset_x(
            cx_u,
            dist_m,
            vgp.FX_ARM,
            vgp.CX_ARM,
            vgp.THETA_ARM,
            vgp.H_ARM,
        )
        self.assertAlmostEqual(corrected, 0.0614, delta=0.002)
        self.assertGreater(corrected, old_ground_scaled_offset * 1.5)

        width = vgp.estimate_pitched_width(
            90, dist_m, vgp.FX_ARM, vgp.THETA_ARM, vgp.H_ARM
        )
        optical_depth = vgp.optical_depth_from_ground_distance(
            dist_m, vgp.THETA_ARM, vgp.H_ARM
        )
        self.assertAlmostEqual(width, 90 * optical_depth / vgp.FX_ARM)

    def test_real_smooth_lock_pose_joint_limit_margin(self):
        limits = ((0.0, 180.0),) * 4 + ((0.0, 270.0),)
        self.assertEqual(
            arm_limit_margin_violations(
                [90.0, 2.7, 61.2, 74.1, 110.2], limits, 1.0
            ),
            [],
        )
        self.assertEqual(
            arm_limit_margin_violations(
                [89.3, 0.0, 65.0, 75.0, 111.3], limits, 1.0
            ),
            ["S2=0.0° (limit 0.0..180.0°)"],
        )
        with self.assertRaises(ValueError):
            arm_limit_margin_violations([90.0], ((0.0, 180.0),), -1.0)

    def test_grasp_home_reach_guard_accepts_15cm_and_rejects_far_target(self):
        home = np.array([0.1668, 0.0182, 0.0101])
        at_limit = home + np.array([0.15, 0.0, 0.0])
        self.assertAlmostEqual(
            validate_grasp_home_reach(at_limit, home, 0.15),
            0.15,
        )
        with self.assertRaisesRegex(RuntimeError, "beyond the 0.150m reachable radius"):
            validate_grasp_home_reach(home + [0.151, 0.0, 0.0], home, 0.15)
        with self.assertRaises(ValueError):
            validate_grasp_home_reach([math.nan, 0.0, 0.0], home, 0.15)

    def test_latch_detection_is_taken_after_one_grasp_home_move(self):
        events = []

        class FakeServo:
            def move_to_grasp_home(self):
                events.append("move_grasp_home")

            def move_to_home(self):
                raise AssertionError("navigation home must not be used for grasp detection")

        class FakeDetection:
            def discard_pending(self):
                events.append("discard_pending")

        controller = object.__new__(GraspController)
        controller.cfg = SimpleNamespace(
            grasp_home_deg=(90.0, 32.704, 9.786, 32.704, 90.0, 30.0),
            preposition_wait_sec=0.0,
            latch_obj=True,
        )
        controller.servo = FakeServo()
        controller.detection = FakeDetection()
        controller._obj_latched = True
        controller._sync_joint_state_from_servos = lambda: events.append("sync")
        controller._latch_object = lambda: events.append("latch")
        with mock.patch("grasp.x3plus_real_grasp.time.sleep"):
            controller._prepare_grasp_home()

        self.assertFalse(controller._obj_latched)
        self.assertEqual(
            events,
            ["move_grasp_home", "sync", "discard_pending", "latch"],
        )

    def test_pose_only_bypasses_grasp_controller_run(self):
        argv = ["x3plus_real_grasp.py", "--pose-only", "grasp-home"]
        with mock.patch.object(sys, "argv", argv), \
                mock.patch.object(real_grasp, "run_pose_only") as pose_only, \
                mock.patch.object(real_grasp, "GraspController") as controller_cls:
            real_grasp.main()

        controller_cls.assert_not_called()
        pose_only.assert_called_once()
        cfg = pose_only.call_args.args[0]
        self.assertEqual(cfg.grasp_home_deg, DeployConfig().grasp_home_deg)
        self.assertFalse(pose_only.call_args.kwargs["real_servo"])
        self.assertEqual(pose_only.call_args.kwargs["pose_name"], "grasp-home")

    def test_real_pipeline_refuses_unconfirmed_camera_frame(self):
        args = SimpleNamespace(real=True, i_confirm_camera_frame=False)
        with self.assertRaises(SystemExit):
            vgp.run_pipeline(args)

    def test_real_integrated_grasp_refuses_legacy_nav_home_latch(self):
        with self.assertRaisesRegex(SystemExit, "grasp-home homography"):
            vgp.run_pipeline(SimpleNamespace(
                real=True,
                i_confirm_camera_frame=True,
            ))

        with self.assertRaisesRegex(SystemExit, "grasp-home homography"):
            nrgp.run_pipeline(SimpleNamespace(
                no_lidar=False,
                lidar_backend="ros",
                real=True,
                nav_only=False,
                i_confirm_camera_frame=True,
            ))

    def test_legacy_motor_calibration_blocks_nonstop_without_real_flag(self):
        module = importlib.import_module("detection.calibration.calibrate_mecanum_setmotor")
        if not module.MOTION_ALLOWED:
            with self.assertRaises(RuntimeError):
                module.send_motor_action(object(), "forward", 30)

    def test_real_socket_latch_refuses_default_position(self):
        controller = object.__new__(GraspController)
        controller.cfg = SimpleNamespace(latch_wait_sec=0.0)
        controller.detection = _StaleDetection()
        controller.real_servo = True
        with self.assertRaises(RuntimeError):
            controller._latch_object()


if __name__ == "__main__":
    unittest.main()
