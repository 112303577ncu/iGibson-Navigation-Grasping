#!/usr/bin/env python3
"""Static camera-vs-grasp frame verifier for X3Plus.

This script does not use pybullet, cv2, ultralytics, or hardware. It parses the
URDF kinematic chain from base_link to mono_link and prints the arm-camera pose
for the configured navigation/grasp homes.

Use it to check whether the vision mapping assumptions

    obj_x = camera_ground_distance + cam_x
    obj_y = sign_y * camera_lateral_offset + cam_y

are consistent with the PPO grasp frame, which is the URDF/PyBullet base frame.
"""
from __future__ import annotations

import argparse
import ast
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Iterable, List, Tuple


ROOT = Path(__file__).resolve().parent.parent
URDF = ROOT / "grasp" / "x3plus" / "yahboomcar.urdf"
BRIDGE = ROOT / "integration" / "vision_grasp_bridge.py"
PIPELINE = ROOT / "integration" / "vision_grasp_pipeline.py"


def parse_deg_csv(value: str) -> Tuple[float, float, float, float, float, float]:
    vals = [float(x.strip()) for x in value.split(",") if x.strip()]
    if len(vals) != 6:
        raise argparse.ArgumentTypeError(f"expected 6 comma-separated degrees, got {len(vals)}")
    return tuple(vals)  # type: ignore[return-value]


def matmul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def eye() -> List[List[float]]:
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def trans(x: float, y: float, z: float) -> List[List[float]]:
    out = eye()
    out[0][3], out[1][3], out[2][3] = x, y, z
    return out


def rotx(a: float) -> List[List[float]]:
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]


def roty(a: float) -> List[List[float]]:
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]


def rotz(a: float) -> List[List[float]]:
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def rpy(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    return matmul(matmul(rotz(yaw), roty(pitch)), rotx(roll))


def axisrot(axis: Iterable[float], angle: float) -> List[List[float]]:
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    if n <= 1e-12:
        return eye()
    x, y, z = x / n, y / n, z / n
    c, s, cc = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [
        [c + x * x * cc, x * y * cc - z * s, x * z * cc + y * s, 0],
        [y * x * cc + z * s, c + y * y * cc, y * z * cc - x * s, 0],
        [z * x * cc - y * s, z * y * cc + x * s, c + z * z * cc, 0],
        [0, 0, 0, 1],
    ]


def read_joints(urdf: Path):
    root = ET.parse(urdf).getroot()
    by_child = {}
    for elem in root.findall("joint"):
        origin = elem.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rot = [0.0, 0.0, 0.0]
        if origin is not None:
            xyz = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()]
            rot = [float(v) for v in origin.attrib.get("rpy", "0 0 0").split()]
        axis_elem = elem.find("axis")
        axis = [0.0, 0.0, 1.0]
        if axis_elem is not None:
            axis = [float(v) for v in axis_elem.attrib.get("xyz", "0 0 1").split()]
        joint = {
            "name": elem.attrib["name"],
            "type": elem.attrib.get("type", "fixed"),
            "parent": elem.find("parent").attrib["link"],
            "child": elem.find("child").attrib["link"],
            "xyz": xyz,
            "rpy": rot,
            "axis": axis,
        }
        by_child[joint["child"]] = joint
    return by_child


def chain_to(by_child, target: str, root: str = "base_link"):
    chain = []
    link = target
    while link != root:
        joint = by_child[link]
        chain.append(joint)
        link = joint["parent"]
    return list(reversed(chain))


def origin_tf(joint) -> List[List[float]]:
    return matmul(trans(*joint["xyz"]), rpy(*joint["rpy"]))


def hw_to_sim_rad(hw_deg6: Tuple[float, ...]) -> List[float]:
    # Matches DeployConfig.arm_hw_invert=(False,...): sim_deg = hw_api - 90.
    return [math.radians(v - 90.0) for v in hw_deg6[:5]]


def link_tf(urdf: Path, hw_deg6: Tuple[float, ...], target: str = "mono_link") -> List[List[float]]:
    by_child = read_joints(urdf)
    q = {f"arm_joint{i + 1}": val for i, val in enumerate(hw_to_sim_rad(hw_deg6))}
    tf = eye()
    for joint in chain_to(by_child, target):
        tf = matmul(tf, origin_tf(joint))
        if joint["type"] in ("revolute", "continuous"):
            tf = matmul(tf, axisrot(joint["axis"], q.get(joint["name"], 0.0)))
    return tf


def axis_info(tf: List[List[float]], col: int):
    vec = [tf[i][col] for i in range(3)]
    yaw = math.degrees(math.atan2(vec[1], vec[0]))
    elev = math.degrees(math.atan2(vec[2], math.hypot(vec[0], vec[1])))
    return vec, yaw, elev


def get_constant(path: Path, name: str):
    text = path.read_text(encoding="utf-8", errors="ignore")
    match = re.search(rf"^{name}\s*=\s*([^#\n]+)", text, flags=re.MULTILINE)
    if not match:
        return None
    raw = match.group(1).strip()
    try:
        return ast.literal_eval(raw)
    except Exception:
        try:
            return float(raw)
        except ValueError:
            return raw


def print_pose(label: str, tf: List[List[float]]) -> None:
    pos = [tf[i][3] for i in range(3)]
    print(f"\n== {label} ==")
    print(f"mono_link position in URDF base: x={pos[0]:+.4f} y={pos[1]:+.4f} z={pos[2]:+.4f} m")
    for name, col in [("+X", 0), ("+Y", 1), ("+Z", 2)]:
        vec, yaw, elev = axis_info(tf, col)
        print(
            f"mono_link {name}: [{vec[0]:+.4f}, {vec[1]:+.4f}, {vec[2]:+.4f}] "
            f"yaw_xy={yaw:+.1f} deg elev={elev:+.1f} deg"
        )
    z_vec, z_yaw, z_elev = axis_info(tf, 2)
    print("Assuming mono_link +Z is the optical axis:")
    print(f"  suggested cam_x={pos[0]:+.4f}, cam_y={pos[1]:+.4f}, H={pos[2]:.4f}, theta_down={-z_elev:.1f} deg")
    print(f"  forward axis yaw should be near 0 deg for obj_x=distance+cam_x; actual {z_yaw:+.1f} deg")


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify static camera-vs-grasp frame assumptions.")
    parser.add_argument("--urdf", type=Path, default=URDF)
    parser.add_argument("--nav-home-deg", type=parse_deg_csv, default=(90.0, 140.0, 0.0, 0.0, 90.0, 30.0))
    # Default matches DeployConfig.grasp_home_deg (v17 training home under the
    # calibrated all-False mapping); keep the two in sync.
    parser.add_argument("--grasp-home-deg", type=parse_deg_csv, default=(90.0, 32.704, 9.786, 32.704, 90.0, 30.0))
    args = parser.parse_args()

    print("PPO object frame: URDF/PyBullet base_link frame used by x3plus_real_grasp.py.")
    print("Vision mapping assumes camera optical forward projects to base +X and lateral projects to base +/-Y.")

    print_pose("navigation home", link_tf(args.urdf, args.nav_home_deg))
    print_pose("PPO grasp home", link_tf(args.urdf, args.grasp_home_deg))

    print("\n== configured vision constants ==")
    print(
        "bridge:",
        {k: get_constant(BRIDGE, k) for k in ("H", "FIXED_THETA", "FX", "FY", "CX_CAM", "CY")},
    )
    print(
        "pipeline:",
        {k: get_constant(PIPELINE, k) for k in ("H_ARM", "THETA_ARM", "FX_ARM", "FY_ARM", "CX_ARM", "CY_ARM", "CAM_TO_BASE_X", "CAM_TO_BASE_Y", "SIGN_Y")},
    )
    print("\nIf the suggested URDF values and configured constants differ, the camera XY is not yet proven to be in the PPO frame.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
