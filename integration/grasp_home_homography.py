#!/usr/bin/env python3
"""Calibrate grasp-home undistorted pixels to ``base_link`` ground XY.

The arm camera moves with the arm, so a mapping measured at navigation home is
not valid at grasp home.  This module fits a planar projective transform from
at least four measured correspondences::

    undistorted pixel (u, v) -> base_link (x, y) metres

Only NumPy is required.  In particular, the solver does not depend on OpenCV,
which keeps it usable in the reduced Jetson deployment environment.

Input JSON example::

    {
      "points": [
        {"u": 180, "v": 220, "x": 0.24, "y": 0.06},
        {"u": 460, "v": 220, "x": 0.24, "y": -0.06},
        {"u": 180, "v": 420, "x": 0.12, "y": 0.06},
        {"u": 460, "v": 420, "x": 0.12, "y": -0.06}
      ]
    }

The pixel coordinates supplied here must already be undistorted with the same
camera intrinsics used at runtime.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple, Union

import numpy as np


SCHEMA = "x3plus.grasp_home_homography"
SCHEMA_VERSION = 1
INPUT_FRAME = "grasp_home_arm_camera_undistorted_pixels"
OUTPUT_FRAME = "base_xy_m"
_EPS = 1e-12


class HomographyCalibrationError(ValueError):
    """Raised when homography calibration data is invalid or degenerate."""


def _as_points(values: Sequence[Sequence[float]], name: str,
               minimum: int = 1) -> np.ndarray:
    try:
        points = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise HomographyCalibrationError(f"{name} must be numeric Nx2 points") from exc
    if points.ndim == 1 and points.shape == (2,):
        points = points.reshape(1, 2)
    if points.ndim != 2 or points.shape[1] != 2:
        raise HomographyCalibrationError(
            f"{name} must have shape (N, 2), got {points.shape}"
        )
    if points.shape[0] < minimum:
        raise HomographyCalibrationError(
            f"{name} needs at least {minimum} point(s), got {points.shape[0]}"
        )
    if not np.all(np.isfinite(points)):
        raise HomographyCalibrationError(f"{name} contains NaN or infinity")
    return points


def _validate_spread(points: np.ndarray, name: str) -> None:
    # Four copies of one coordinate, or points on one image/ground line, do
    # not constrain a planar projective transform.
    if np.unique(points, axis=0).shape[0] < 4:
        raise HomographyCalibrationError(f"{name} needs at least four distinct points")
    centred = points - np.mean(points, axis=0)
    if np.linalg.matrix_rank(centred) < 2:
        raise HomographyCalibrationError(f"{name} points are collinear or coincident")


def _normalise_points(points: np.ndarray, name: str) -> Tuple[np.ndarray, np.ndarray]:
    """Return Hartley-normalised points and their 3x3 normalisation matrix."""
    centre = np.mean(points, axis=0)
    shifted = points - centre
    mean_distance = float(np.mean(np.linalg.norm(shifted, axis=1)))
    if not math.isfinite(mean_distance) or mean_distance <= _EPS:
        raise HomographyCalibrationError(f"{name} points have no usable spread")
    scale = math.sqrt(2.0) / mean_distance
    transform = np.array([
        [scale, 0.0, -scale * centre[0]],
        [0.0, scale, -scale * centre[1]],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    homogeneous = np.column_stack((points, np.ones(points.shape[0])))
    normalised = (transform @ homogeneous.T).T[:, :2]
    return normalised, transform


def _validate_homography_matrix(homography: Sequence[Sequence[float]]) -> np.ndarray:
    try:
        matrix = np.asarray(homography, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise HomographyCalibrationError("homography must be a numeric 3x3 matrix") from exc
    if matrix.shape != (3, 3):
        raise HomographyCalibrationError(
            f"homography must have shape (3, 3), got {matrix.shape}"
        )
    if not np.all(np.isfinite(matrix)):
        raise HomographyCalibrationError("homography contains NaN or infinity")
    norm = float(np.linalg.norm(matrix))
    if norm <= _EPS or np.linalg.matrix_rank(matrix) < 3:
        raise HomographyCalibrationError("homography is singular or all zero")
    return matrix


def fit_homography(pixel_points: Sequence[Sequence[float]],
                   base_points: Sequence[Sequence[float]]) -> np.ndarray:
    """Fit an undistorted-pixel to base-XY homography using normalised DLT.

    Args:
        pixel_points: ``N x 2`` undistorted ``(u, v)`` pixels.
        base_points: Matching ``N x 2`` ``base_link (x, y)`` points in metres.

    Returns:
        A finite, nonsingular 3x3 NumPy matrix, normalised so ``H[2, 2]`` is
        one whenever that element is usable.
    """
    pixels = _as_points(pixel_points, "pixel_points", minimum=4)
    bases = _as_points(base_points, "base_points", minimum=4)
    if pixels.shape[0] != bases.shape[0]:
        raise HomographyCalibrationError(
            "pixel_points and base_points must contain the same number of points"
        )
    _validate_spread(pixels, "pixel_points")
    _validate_spread(bases, "base_points")

    pixels_n, pixel_transform = _normalise_points(pixels, "pixel_points")
    bases_n, base_transform = _normalise_points(bases, "base_points")

    rows: List[List[float]] = []
    for (u, v), (x, y) in zip(pixels_n, bases_n):
        rows.append([-u, -v, -1.0, 0.0, 0.0, 0.0, x * u, x * v, x])
        rows.append([0.0, 0.0, 0.0, -u, -v, -1.0, y * u, y * v, y])
    design = np.asarray(rows, dtype=np.float64)
    if np.linalg.matrix_rank(design) < 8:
        raise HomographyCalibrationError(
            "point correspondences are degenerate and do not define one homography"
        )

    # full_matrices=True is intentional: with exactly four pairs A is 8x9,
    # and the null-space vector is the ninth row of Vh.
    _u, _singular_values, vh = np.linalg.svd(design, full_matrices=True)
    normalised_h = vh[-1].reshape(3, 3)
    matrix = np.linalg.inv(base_transform) @ normalised_h @ pixel_transform

    if abs(matrix[2, 2]) > _EPS:
        matrix = matrix / matrix[2, 2]
    else:
        matrix = matrix / np.linalg.norm(matrix)
    return _validate_homography_matrix(matrix)


def apply_homography(homography: Sequence[Sequence[float]],
                     pixels: Sequence[Sequence[float]]) -> np.ndarray:
    """Map one ``(u,v)`` point or an ``N x 2`` array into base XY metres.

    A single input pair returns shape ``(2,)``; a batch returns ``(N, 2)``.
    """
    matrix = _validate_homography_matrix(homography)
    try:
        raw = np.asarray(pixels, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise HomographyCalibrationError("pixels must be numeric Nx2 points") from exc
    single = raw.ndim == 1
    points = _as_points(raw, "pixels")
    homogeneous = np.column_stack((points, np.ones(points.shape[0])))
    projected = (matrix @ homogeneous.T).T
    denominators = projected[:, 2]
    if (not np.all(np.isfinite(projected)) or
            np.any(np.abs(denominators) <= _EPS)):
        raise HomographyCalibrationError(
            "homography maps at least one pixel to infinity"
        )
    result = projected[:, :2] / denominators[:, None]
    if not np.all(np.isfinite(result)):
        raise HomographyCalibrationError("homography produced a non-finite base point")
    return result[0] if single else result


def reprojection_errors(homography: Sequence[Sequence[float]],
                        pixel_points: Sequence[Sequence[float]],
                        base_points: Sequence[Sequence[float]]) -> np.ndarray:
    """Return Euclidean base-plane reprojection error for every pair, in m."""
    pixels = _as_points(pixel_points, "pixel_points")
    bases = _as_points(base_points, "base_points")
    if pixels.shape[0] != bases.shape[0]:
        raise HomographyCalibrationError(
            "pixel_points and base_points must contain the same number of points"
        )
    predicted = apply_homography(homography, pixels)
    return np.linalg.norm(predicted - bases, axis=1)


def make_calibration(pixel_points: Sequence[Sequence[float]],
                     base_points: Sequence[Sequence[float]]) -> Dict[str, Any]:
    """Fit a homography and return its versioned JSON-serialisable document."""
    pixels = _as_points(pixel_points, "pixel_points", minimum=4)
    bases = _as_points(base_points, "base_points", minimum=4)
    homography = fit_homography(pixels, bases)
    predicted = apply_homography(homography, pixels)
    errors = np.linalg.norm(predicted - bases, axis=1)
    point_rows = []
    for pixel, base, estimate, error in zip(pixels, bases, predicted, errors):
        point_rows.append({
            "u": float(pixel[0]),
            "v": float(pixel[1]),
            "x": float(base[0]),
            "y": float(base[1]),
            "x_reprojected": float(estimate[0]),
            "y_reprojected": float(estimate[1]),
            "error_m": float(error),
        })
    rmse = float(math.sqrt(float(np.mean(errors ** 2))))
    return {
        "schema": SCHEMA,
        "version": SCHEMA_VERSION,
        "input_frame": INPUT_FRAME,
        "output_frame": OUTPUT_FRAME,
        "homography": homography.tolist(),
        "fit": {
            "point_count": int(pixels.shape[0]),
            "rmse_m": rmse,
            "mean_error_m": float(np.mean(errors)),
            "max_error_m": float(np.max(errors)),
        },
        "points": point_rows,
    }


def save_calibration(path: Union[str, Path], calibration: Dict[str, Any]) -> None:
    """Validate and save a calibration document as strict (no NaN) JSON."""
    _validate_calibration_document(calibration)
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        json.dump(calibration, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def _validate_calibration_document(document: Any, min_points: int = 4,
                                   max_error_m: float = None) -> Dict[str, Any]:
    if isinstance(min_points, bool) or not isinstance(min_points, int) or min_points < 4:
        raise HomographyCalibrationError("min_points must be an integer of at least four")
    if max_error_m is not None and (
            not math.isfinite(max_error_m) or max_error_m <= 0.0):
        raise HomographyCalibrationError("max_error_m must be finite and greater than zero")
    if not isinstance(document, dict):
        raise HomographyCalibrationError("calibration JSON root must be an object")
    if document.get("schema") != SCHEMA:
        raise HomographyCalibrationError(
            f"unexpected calibration schema: {document.get('schema')!r}"
        )
    if document.get("version") != SCHEMA_VERSION:
        raise HomographyCalibrationError(
            f"unsupported calibration version: {document.get('version')!r}"
        )
    if document.get("input_frame") != INPUT_FRAME:
        raise HomographyCalibrationError(
            f"unexpected input_frame: {document.get('input_frame')!r}"
        )
    if document.get("output_frame") != OUTPUT_FRAME:
        raise HomographyCalibrationError(
            f"unexpected output_frame: {document.get('output_frame')!r}"
        )
    homography = _validate_homography_matrix(document.get("homography"))

    fit = document.get("fit")
    if not isinstance(fit, dict):
        raise HomographyCalibrationError("calibration fit must be an object")
    point_count = fit.get("point_count")
    if (isinstance(point_count, bool) or not isinstance(point_count, int) or
            point_count < min_points):
        raise HomographyCalibrationError(
            f"calibration needs at least {min_points} fitted points"
        )
    metric_names = ("rmse_m", "mean_error_m", "max_error_m")
    for name in metric_names:
        value = fit.get(name)
        if (not isinstance(value, (int, float)) or isinstance(value, bool) or
                not math.isfinite(value) or value < 0.0):
            raise HomographyCalibrationError(f"fit.{name} must be finite and nonnegative")

    rows = document.get("points")
    if not isinstance(rows, list) or len(rows) != point_count:
        raise HomographyCalibrationError(
            "calibration points length must equal fit.point_count"
        )
    required = ("u", "v", "x", "y", "x_reprojected", "y_reprojected", "error_m")
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise HomographyCalibrationError(f"calibration point {index} must be an object")
        for name in required:
            value = row.get(name)
            if (not isinstance(value, (int, float)) or isinstance(value, bool) or
                    not math.isfinite(value)):
                raise HomographyCalibrationError(
                    f"calibration point {index}.{name} must be finite"
                )
        if row["error_m"] < 0.0:
            raise HomographyCalibrationError(
                f"calibration point {index}.error_m must be nonnegative"
            )

    pixels = _as_points([(row["u"], row["v"]) for row in rows], "points pixels", 4)
    bases = _as_points([(row["x"], row["y"]) for row in rows], "points bases", 4)
    _validate_spread(pixels, "points pixels")
    _validate_spread(bases, "points bases")
    predicted = apply_homography(homography, pixels)
    stored_predicted = _as_points(
        [(row["x_reprojected"], row["y_reprojected"]) for row in rows],
        "stored reprojections",
        4,
    )
    errors = np.linalg.norm(predicted - bases, axis=1)
    stored_errors = np.asarray([row["error_m"] for row in rows], dtype=np.float64)
    # JSON float round trips are effectively exact here.  A small absolute
    # tolerance still permits files produced by other standards-compliant
    # encoders while rejecting a matrix/metadata mismatch.
    if not np.allclose(predicted, stored_predicted, rtol=1e-9, atol=1e-12):
        raise HomographyCalibrationError("stored point reprojections do not match homography")
    if not np.allclose(errors, stored_errors, rtol=1e-9, atol=1e-12):
        raise HomographyCalibrationError("stored point errors do not match homography")

    computed_metrics = {
        "rmse_m": float(math.sqrt(float(np.mean(errors ** 2)))),
        "mean_error_m": float(np.mean(errors)),
        "max_error_m": float(np.max(errors)),
    }
    for name, computed in computed_metrics.items():
        if not math.isclose(fit[name], computed, rel_tol=1e-9, abs_tol=1e-12):
            raise HomographyCalibrationError(f"fit.{name} does not match point errors")
    if max_error_m is not None and computed_metrics["max_error_m"] >= max_error_m:
        raise HomographyCalibrationError(
            f"calibration max error {computed_metrics['max_error_m']:.6f}m is not below "
            f"required {max_error_m:.6f}m"
        )
    return document


def load_calibration(path: Union[str, Path], min_points: int = 4,
                     max_error_m: float = None) -> Dict[str, Any]:
    """Load and validate a versioned grasp-home calibration document.

    Runtime callers may request a stricter quality gate than the four-pair
    mathematical minimum, for example ``min_points=6, max_error_m=0.02``.
    """
    try:
        with Path(path).open("r", encoding="utf-8") as stream:
            document = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise HomographyCalibrationError(
            f"cannot load homography calibration {path}: {exc}"
        ) from exc
    return _validate_calibration_document(document, min_points, max_error_m)


def load_homography(path: Union[str, Path], min_points: int = 4,
                    max_error_m: float = None) -> np.ndarray:
    """Load only the validated 3x3 matrix needed by runtime vision code."""
    document = load_calibration(path, min_points, max_error_m)
    return _validate_homography_matrix(document["homography"]).copy()


def _parse_point_spec(raw: str) -> Tuple[float, float, float, float]:
    try:
        values = tuple(float(value.strip()) for value in raw.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"invalid --point {raw!r}; expected U,V,X,Y"
        ) from exc
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise argparse.ArgumentTypeError(
            f"invalid --point {raw!r}; expected four finite values U,V,X,Y"
        )
    return values  # type: ignore[return-value]


def _load_input_points(path: Union[str, Path]) -> Tuple[np.ndarray, np.ndarray]:
    try:
        with Path(path).open("r", encoding="utf-8") as stream:
            document = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise HomographyCalibrationError(f"cannot load point file {path}: {exc}") from exc
    rows = document.get("points") if isinstance(document, dict) else document
    if not isinstance(rows, list):
        raise HomographyCalibrationError(
            "point JSON must be a list or an object containing a 'points' list"
        )
    pixels: List[Tuple[float, float]] = []
    bases: List[Tuple[float, float]] = []
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise HomographyCalibrationError(f"point {index} must be an object")
        try:
            pixels.append((row["u"], row["v"]))
            bases.append((row["x"], row["y"]))
        except KeyError as exc:
            raise HomographyCalibrationError(
                f"point {index} is missing key {exc.args[0]!r}"
            ) from exc
    return (_as_points(pixels, "pixel_points", minimum=4),
            _as_points(bases, "base_points", minimum=4))


def run_selftest() -> None:
    true_h = np.array([
        [3.1e-4, -2.0e-5, 0.075],
        [1.5e-5, -2.8e-4, 0.112],
        [8.0e-5, -5.0e-5, 1.0],
    ])
    pixels = np.array([
        [150.0, 180.0], [320.0, 160.0], [510.0, 190.0],
        [140.0, 350.0], [330.0, 330.0], [520.0, 370.0],
    ])
    bases = apply_homography(true_h, pixels)
    solved = fit_homography(pixels, bases)
    errors = reprojection_errors(solved, pixels, bases)
    assert float(np.max(errors)) < 1e-10, errors

    document = make_calibration(pixels, bases)
    assert document["schema"] == SCHEMA
    assert document["fit"]["point_count"] == 6
    assert document["fit"]["max_error_m"] < 1e-10

    try:
        fit_homography(
            [[0, 0], [1, 0], [2, 0], [3, 0]],
            [[0, 0], [1, 0], [2, 0], [3, 0]],
        )
    except HomographyCalibrationError:
        pass
    else:
        raise AssertionError("collinear points were not rejected")
    print("[selftest] OK")


def main(argv: Sequence[str] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Fit grasp-home UNDISTORTED pixel (u,v) -> base_link (x,y metres) "
            "homography without OpenCV"
        )
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument(
        "--points-json",
        help="JSON list (or {'points': [...]}) containing u,v,x,y fields",
    )
    source.add_argument(
        "--point",
        action="append",
        default=[],
        type=_parse_point_spec,
        metavar="U,V,X,Y",
        help="one correspondence; repeat at least four times",
    )
    parser.add_argument(
        "--output",
        default="grasp_home_homography.json",
        help="output calibration JSON (default: %(default)s)",
    )
    parser.add_argument(
        "--max-rmse-cm",
        type=float,
        default=None,
        help="optional nonzero exit when fit RMSE exceeds this threshold",
    )
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args(argv)

    if args.selftest:
        run_selftest()
        return 0
    if args.points_json:
        pixels, bases = _load_input_points(args.points_json)
    elif args.point:
        pixels = np.asarray([(p[0], p[1]) for p in args.point], dtype=np.float64)
        bases = np.asarray([(p[2], p[3]) for p in args.point], dtype=np.float64)
    else:
        parser.error("provide --points-json or at least four --point values")
    if len(pixels) < 4:
        parser.error("need at least four point correspondences")
    if args.max_rmse_cm is not None and (
            not math.isfinite(args.max_rmse_cm) or args.max_rmse_cm <= 0.0):
        parser.error("--max-rmse-cm must be finite and greater than zero")

    try:
        calibration = make_calibration(pixels, bases)
        save_calibration(args.output, calibration)
    except HomographyCalibrationError as exc:
        parser.error(str(exc))

    fit = calibration["fit"]
    print(f"[homography] points: {fit['point_count']}")
    print(f"[homography] RMSE:   {fit['rmse_m'] * 100.0:.3f} cm")
    print(f"[homography] mean:   {fit['mean_error_m'] * 100.0:.3f} cm")
    print(f"[homography] max:    {fit['max_error_m'] * 100.0:.3f} cm")
    print(f"[homography] saved:  {Path(args.output)}")
    if (args.max_rmse_cm is not None and
            fit["rmse_m"] * 100.0 > args.max_rmse_cm):
        print(
            f"[homography] FAIL: RMSE exceeds {args.max_rmse_cm:.3f} cm threshold"
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
