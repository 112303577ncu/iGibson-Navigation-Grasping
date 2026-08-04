from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from integration.grasp_home_homography import (
    HomographyCalibrationError,
    SCHEMA,
    apply_homography,
    fit_homography,
    load_calibration,
    load_homography,
    main,
    make_calibration,
    reprojection_errors,
    save_calibration,
)


class GraspHomeHomographyTests(unittest.TestCase):
    def setUp(self):
        self.true_h = np.array([
            [3.1e-4, -2.0e-5, 0.075],
            [1.5e-5, -2.8e-4, 0.112],
            [8.0e-5, -5.0e-5, 1.0],
        ])
        self.pixels = np.array([
            [150.0, 180.0], [320.0, 160.0], [510.0, 190.0],
            [140.0, 350.0], [330.0, 330.0], [520.0, 370.0],
        ])
        self.bases = apply_homography(self.true_h, self.pixels)

    def test_dlt_recovers_projective_mapping(self):
        solved = fit_homography(self.pixels, self.bases)
        errors = reprojection_errors(solved, self.pixels, self.bases)
        self.assertLess(float(np.max(errors)), 1e-10)
        probe = np.array([275.0, 285.0])
        np.testing.assert_allclose(
            apply_homography(solved, probe),
            apply_homography(self.true_h, probe),
            atol=1e-10,
        )

    def test_exactly_four_pairs_are_supported(self):
        indexes = [0, 2, 3, 5]
        solved = fit_homography(self.pixels[indexes], self.bases[indexes])
        np.testing.assert_allclose(
            apply_homography(solved, self.pixels), self.bases, atol=1e-9
        )

    def test_apply_preserves_single_and_batch_shapes(self):
        self.assertEqual(apply_homography(self.true_h, [200, 250]).shape, (2,))
        self.assertEqual(
            apply_homography(self.true_h, [[200, 250], [300, 350]]).shape,
            (2, 2),
        )

    def test_rejects_too_few_mismatched_and_nonfinite_points(self):
        with self.assertRaises(HomographyCalibrationError):
            fit_homography(self.pixels[:3], self.bases[:3])
        with self.assertRaises(HomographyCalibrationError):
            fit_homography(self.pixels, self.bases[:4])
        bad = self.pixels.copy()
        bad[0, 0] = np.nan
        with self.assertRaises(HomographyCalibrationError):
            fit_homography(bad, self.bases)

    def test_rejects_collinear_or_duplicate_correspondences(self):
        line = np.array([[0, 0], [1, 0], [2, 0], [3, 0]], dtype=float)
        with self.assertRaises(HomographyCalibrationError):
            fit_homography(line, line)
        duplicates = np.array([[0, 0], [0, 0], [1, 0], [0, 1]], dtype=float)
        with self.assertRaises(HomographyCalibrationError):
            fit_homography(duplicates, duplicates)

    def test_rejects_singular_matrix_and_point_at_infinity(self):
        with self.assertRaises(HomographyCalibrationError):
            apply_homography(np.zeros((3, 3)), [1, 2])
        sends_u_one_to_infinity = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 0.0, -1.0],
        ])
        with self.assertRaises(HomographyCalibrationError):
            apply_homography(sends_u_one_to_infinity, [1, 2])

    def test_json_round_trip_and_schema_validation(self):
        calibration = make_calibration(self.pixels, self.bases)
        self.assertEqual(calibration["schema"], SCHEMA)
        self.assertEqual(calibration["fit"]["point_count"], len(self.pixels))
        self.assertLess(calibration["fit"]["max_error_m"], 1e-10)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.json"
            save_calibration(path, calibration)
            loaded = load_calibration(path)
            self.assertEqual(loaded["input_frame"], calibration["input_frame"])
            np.testing.assert_allclose(load_homography(path), calibration["homography"])

            loaded["version"] = 999
            path.write_text(json.dumps(loaded), encoding="utf-8")
            with self.assertRaises(HomographyCalibrationError):
                load_homography(path)

    def test_loader_can_enforce_stricter_runtime_quality_gate(self):
        calibration = make_calibration(self.pixels, self.bases)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.json"
            save_calibration(path, calibration)
            load_calibration(path, min_points=6, max_error_m=0.02)
            with self.assertRaises(HomographyCalibrationError):
                load_calibration(path, min_points=7)

            calibration["points"][0]["x"] += 0.03
            path.write_text(json.dumps(calibration), encoding="utf-8")
            with self.assertRaises(HomographyCalibrationError):
                load_calibration(path)

    def test_cli_accepts_point_json_and_writes_calibration(self):
        rows = [
            {"u": float(p[0]), "v": float(p[1]),
             "x": float(b[0]), "y": float(b[1])}
            for p, b in zip(self.pixels, self.bases)
        ]
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "points.json"
            output = Path(directory) / "result.json"
            source.write_text(json.dumps({"points": rows}), encoding="utf-8")
            self.assertEqual(
                main(["--points-json", str(source), "--output", str(output)]), 0
            )
            self.assertTrue(output.is_file())
            self.assertLess(load_calibration(output)["fit"]["rmse_m"], 1e-10)


if __name__ == "__main__":
    unittest.main()
