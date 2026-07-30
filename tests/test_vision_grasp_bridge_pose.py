import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from integration import vision_grasp_bridge as bridge
from integration.grasp_home_homography import make_calibration, save_calibration


class FakeCapture:
    def __init__(self, opened, reads=()):
        self.opened = opened
        self.reads = list(reads)
        self.released = False

    def isOpened(self):
        return self.opened and not self.released

    def read(self):
        if self.reads:
            return self.reads.pop(0)
        return False, None

    def release(self):
        self.released = True


def geometry_args(**overrides):
    values = {
        "camera_pose": "grasp-home",
        "calibration_only": False,
        "homography": None,
        "camera_height": None,
        "camera_theta": None,
        "cam_x": None,
        "cam_y": None,
        "sign_x": None,
        "sign_y": None,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class VisionGraspBridgePoseTests(unittest.TestCase):
    def test_camera_open_is_confirmed_only_after_a_real_frame(self):
        frame = SimpleNamespace(size=3, shape=(1, 1, 3))
        cannot_open = FakeCapture(False)
        opens_and_reads = FakeCapture(True, [(False, None), (True, frame)])
        with patch.object(bridge, "open_capture",
                          side_effect=[cannot_open, opens_and_reads]):
            cap, first_frame, error = bridge.open_capture_checked(
                "mock", attempts=2, probe_reads=2,
                retry_delay=0.0, probe_delay=0.0,
            )
        self.assertIs(cap, opens_and_reads)
        self.assertIs(first_frame, frame)
        self.assertEqual(error, "")
        self.assertTrue(cannot_open.released)
        self.assertFalse(opens_and_reads.released)

    def test_camera_open_with_no_frames_fails_after_finite_retries(self):
        first = FakeCapture(True)
        second = FakeCapture(True)
        with patch.object(bridge, "open_capture", side_effect=[first, second]):
            cap, frame, error = bridge.open_capture_checked(
                "mock", attempts=2, probe_reads=2,
                retry_delay=0.0, probe_delay=0.0,
            )
        self.assertIsNone(cap)
        self.assertIsNone(frame)
        self.assertIn("produced no frame", error)
        self.assertTrue(first.released)
        self.assertTrue(second.released)

    def test_grasp_home_runtime_requires_homography(self):
        with self.assertRaisesRegex(SystemExit, "--homography"):
            bridge.resolve_camera_geometry(geometry_args())

    def test_calibration_only_never_needs_a_homography(self):
        args = geometry_args(calibration_only=True)
        bridge.resolve_camera_geometry(args)
        self.assertIsNone(args.homography_matrix)

    def test_calibration_only_rejects_nav_home(self):
        args = geometry_args(camera_pose="nav-home", calibration_only=True)
        with self.assertRaisesRegex(SystemExit, "only supported"):
            bridge.resolve_camera_geometry(args)

    def test_nav_home_resolves_legacy_measured_defaults(self):
        args = geometry_args(camera_pose="nav-home")
        bridge.resolve_camera_geometry(args)
        self.assertEqual(args.sign_x, 1.0)
        self.assertEqual(args.sign_y, -1.0)
        self.assertAlmostEqual(args.camera_height, bridge.H)

    def test_verified_homography_maps_inside_and_rejects_outside_hull(self):
        pixels = [(u, v) for v in (100.0, 300.0, 450.0)
                  for u in (100.0, 300.0, 500.0)]
        bases = [(0.30 - 0.0002 * v, 0.00025 * (u - 300.0))
                 for u, v in pixels]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "grasp_home.json"
            save_calibration(path, make_calibration(pixels, bases))
            args = geometry_args(homography=str(path))
            bridge.resolve_camera_geometry(args)
            x, y = bridge.apply_grasp_home_mapping(args, 300.0, 300.0)
            self.assertAlmostEqual(x, 0.24, places=8)
            self.assertAlmostEqual(y, 0.0, places=8)
            edge_x, edge_y = bridge.apply_grasp_home_mapping(args, 100.0, 100.0)
            self.assertAlmostEqual(edge_x, 0.28, places=8)
            self.assertAlmostEqual(edge_y, -0.05, places=8)
            with self.assertRaisesRegex(ValueError, "outside calibration hull"):
                bridge.apply_grasp_home_mapping(args, 50.0, 50.0)

    def test_calibration_batch_uses_medians(self):
        rows = []
        for u in (100.0, 102.0, 500.0):
            rows.append({
                "class": "bottle-cap", "u": u, "v": 200.0,
                "left_u": u - 10.0, "left_v": 200.0,
                "right_u": u + 10.0, "right_v": 200.0,
                "raw_u": u + 1.0, "raw_v": 201.0,
            })
        result = bridge.median_calibration_record(rows)
        self.assertEqual(result["u"], 102.0)
        self.assertEqual(result["samples"], 3)


if __name__ == "__main__":
    unittest.main()
