#!/usr/bin/env python3
"""Joint direction calibration helper for X3Plus sim-to-real deployment.

This script compares the training/FK expectation with the real servo command
for a small positive simulated joint motion. Use it to verify whether each
hardware joint direction matches the URDF/PPO convention.

Default mode is dry-run. Add --real on the Jetson to move one joint at a time.
"""
from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

p = None


@dataclass
class CalibrationConfig:
    urdf_path: str = "x3plus/yahboomcar.urdf"
    serial_port: str = "/dev/myserial"
    # S6=30 keeps the gripper open (and constant) through every home/test cycle;
    # 180 would slam it closed at each return-to-home under the current convention.
    home_deg: Tuple[float, ...] = (90.0, 90.0, 90.0, 90.0, 90.0, 30.0)
    arm_sim_limits: Tuple[Tuple[float, float], ...] = (
        (-1.5708, 1.5708),
        (-1.5708, 1.5708),
        (-1.5708, 1.5708),
        (-1.5708, 1.5708),
        (-1.5708, 3.14159),
    )
    arm_hw_center: Tuple[float, ...] = (90.0, 90.0, 90.0, 90.0, 90.0)
    arm_hw_range: Tuple[Tuple[float, float], ...] = (
        (0.0, 180.0),
        (0.0, 180.0),
        (0.0, 180.0),
        (0.0, 180.0),
        (0.0, 270.0),
    )
    # Must match DeployConfig: Rosmaster API's S2/S3/S4 mirror already
    # accounts for the physical reversal, so all API-space mappings are +.
    arm_hw_invert: Tuple[bool, ...] = (False, False, False, False, False)
    gripper_home: float = 30.0  # current deployment convention: fingers open


class JointMapper:
    def __init__(self, cfg: CalibrationConfig):
        self.cfg = cfg

    def sim_arm_to_hw_deg(self, sim_angles_rad: Sequence[float]) -> List[float]:
        values = np.asarray(sim_angles_rad, dtype=np.float64).reshape(-1)
        if values.shape != (5,) or not np.all(np.isfinite(values)):
            raise ValueError("sim arm angles must contain exactly five finite values")
        hw: List[float] = []
        for rad, invert, center, (hw_lo, hw_hi) in zip(
            values,
            self.cfg.arm_hw_invert,
            self.cfg.arm_hw_center,
            self.cfg.arm_hw_range,
        ):
            sim_deg = math.degrees(float(rad))
            deg = center - sim_deg if invert else center + sim_deg
            hw.append(float(np.clip(deg, hw_lo, hw_hi)))
        return hw

    def hw_deg_to_sim_arm(self, hw_degs: Sequence[float]) -> np.ndarray:
        values = np.asarray(hw_degs, dtype=np.float64).reshape(-1)
        if values.shape != (5,) or not np.all(np.isfinite(values)):
            raise ValueError("hardware arm angles must contain exactly five finite values")
        sim: List[float] = []
        for deg, invert, center, (sim_lo, sim_hi) in zip(
            values,
            self.cfg.arm_hw_invert,
            self.cfg.arm_hw_center,
            self.cfg.arm_sim_limits,
        ):
            sim_deg = center - deg if invert else deg - center
            sim.append(float(np.clip(math.radians(sim_deg), sim_lo, sim_hi)))
        return np.array(sim, dtype=np.float32)


class FKComputer:
    def __init__(self, urdf_path: str):
        global p
        if p is None:
            import pybullet as pybullet

            p = pybullet
        self.client = p.connect(p.DIRECT)
        if self.client < 0:
            raise RuntimeError("failed to create PyBullet DIRECT client")
        self._closed = False
        try:
            urdf_abs = Path(__file__).parent / urdf_path
            if not urdf_abs.exists():
                raise FileNotFoundError(f"URDF not found: {urdf_abs}")
            self.body = p.loadURDF(str(urdf_abs), useFixedBase=True, physicsClientId=self.client)

            wanted = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
            name_to_id = {}
            short_to_full = {}
            for j in range(p.getNumJoints(self.body, physicsClientId=self.client)):
                info = p.getJointInfo(self.body, j, physicsClientId=self.client)
                full = info[1].decode("utf-8")
                name_to_id[full] = j
                for target in wanted:
                    if full.endswith(target):
                        short_to_full[target] = full

            missing = [name for name in wanted if name not in short_to_full]
            if missing:
                raise RuntimeError(f"Missing URDF arm joints: {missing}")
            self.arm_indices = [name_to_id[short_to_full[name]] for name in wanted]
            self.ee_link = self.arm_indices[-1]
        except Exception:
            self.close()
            raise

    def compute_tcp(self, arm_rads: Sequence[float]) -> np.ndarray:
        values = np.asarray(arm_rads, dtype=np.float64).reshape(-1)
        if values.shape != (5,) or not np.all(np.isfinite(values)):
            raise ValueError("FK arm angles must contain exactly five finite values")
        for idx, rad in zip(self.arm_indices, values):
            p.resetJointState(self.body, idx, float(rad), physicsClientId=self.client)
        p.stepSimulation(physicsClientId=self.client)
        state = p.getLinkState(
            self.body,
            self.ee_link,
            computeForwardKinematics=True,
            physicsClientId=self.client,
        )
        return np.array(state[0], dtype=np.float32)

    def close(self) -> None:
        if not self._closed:
            p.disconnect(self.client)
            self._closed = True


class ServoWriter:
    def __init__(self, port: str, dry_run: bool):
        self.dry_run = dry_run
        self.bot = None
        if dry_run:
            return
        from Rosmaster_Lib import Rosmaster

        try:
            self.bot = Rosmaster(com=port)
            self.bot.create_receive_threading()
            time.sleep(0.5)
        except Exception:
            self.close()
            raise

    def send(self, deg6: Sequence[float], run_time_ms: int, label: str) -> None:
        values = np.asarray(deg6, dtype=np.float64).reshape(-1)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            raise ValueError("servo command must contain exactly six finite angles")
        if run_time_ms < 0:
            raise ValueError("servo run time must be non-negative")
        actual_run_time_ms = min(int(run_time_ms), 2000)
        text = " ".join(f"S{i + 1}={deg:.1f}" for i, deg in enumerate(values))
        prefix = "DRY" if self.dry_run else "REAL"
        print(f"[{prefix}] {label}: {text} t={actual_run_time_ms}ms")
        if self.dry_run:
            return
        self.bot.set_uart_servo_angle_array(angle_s=values.tolist(), run_time=actual_run_time_ms)

    def close(self) -> None:
        if self.bot is not None:
            try:
                self.bot.cancel_receive_threading()
            except Exception:
                pass
            serial_port = getattr(self.bot, "ser", None)
            if serial_port is not None:
                try:
                    serial_port.close()
                except Exception:
                    pass
            self.bot = None


def parse_invert_flags(value: str) -> Tuple[bool, ...]:
    parts = [part.strip() for part in value.split(",") if part.strip()]
    if len(parts) != 5:
        raise argparse.ArgumentTypeError("Need 5 comma-separated flags, e.g. 0,0,0,0,0")
    out = []
    for part in parts:
        if part not in {"0", "1", "false", "true", "False", "True"}:
            raise argparse.ArgumentTypeError(f"Invalid invert flag: {part}")
        out.append(part in {"1", "true", "True"})
    return tuple(out)  # type: ignore[return-value]


def dominant_axis(delta: np.ndarray) -> str:
    labels = ["X", "Y", "Z"]
    idx = int(np.argmax(np.abs(delta)))
    sign = "+" if delta[idx] >= 0 else "-"
    return f"{sign}{labels[idx]}"


def format_delta(delta: np.ndarray) -> str:
    return f"dx={delta[0]:+.4f} dy={delta[1]:+.4f} dz={delta[2]:+.4f} m"


def ask_match() -> Optional[bool]:
    while True:
        ans = input("  Real motion same as expected dominant direction? [y/n/s] ").strip().lower()
        if ans in {"y", "yes"}:
            return True
        if ans in {"n", "no"}:
            return False
        if ans in {"s", "skip", ""}:
            return None
        print("  Please enter y, n, or s.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Verify X3Plus joint direction against URDF/FK convention.")
    parser.add_argument("--real", action="store_true", help="Actually move servos. Default is dry-run.")
    parser.add_argument("--port", default="/dev/myserial", help="Rosmaster serial port.")
    parser.add_argument("--delta-deg", type=float, default=10.0, help="Positive sim perturbation per joint.")
    parser.add_argument("--run-time", type=int, default=1200, help="Servo move time in milliseconds.")
    parser.add_argument("--settle", type=float, default=1.0, help="Seconds to wait after each command.")
    parser.add_argument("--joint", type=int, choices=[1, 2, 3, 4, 5], help="Only test one joint.")
    parser.add_argument("--interactive", action="store_true", help="Ask whether real motion matched and summarize.")
    parser.add_argument("--invert", type=parse_invert_flags, default=(False, False, False, False, False),
                        help="Current API-space invert flags, default 0,0,0,0,0")
    args = parser.parse_args()
    if not math.isfinite(args.delta_deg) or args.delta_deg <= 0.0:
        parser.error("--delta-deg must be finite and > 0")
    if args.run_time < 0:
        parser.error("--run-time must be >= 0")
    if not math.isfinite(args.settle) or args.settle < 0.0:
        parser.error("--settle must be finite and >= 0")

    cfg = CalibrationConfig(serial_port=args.port, arm_hw_invert=args.invert)
    mapper = JointMapper(cfg)
    fk: Optional[FKComputer] = None
    servo: Optional[ServoWriter] = None
    try:
        fk = FKComputer(cfg.urdf_path)
        servo = ServoWriter(args.port, dry_run=not args.real)

        home_arm = mapper.hw_deg_to_sim_arm(cfg.home_deg[:5])
        home_tcp = fk.compute_tcp(home_arm)
        home_deg6 = list(cfg.home_deg)
        joints = [args.joint] if args.joint else [1, 2, 3, 4, 5]
        recommendations: List[str] = []

        print("\nX3Plus joint direction calibration")
        print(f"  invert flags S1-S5: {cfg.arm_hw_invert}")
        print(f"  home TCP in training/URDF frame: {home_tcp.round(4).tolist()}")
        print("  Rule: if real dominant motion is opposite, toggle that joint's invert flag.\n")

        for joint in joints:
            j = joint - 1
            test_arm = home_arm.copy()
            lo, hi = cfg.arm_sim_limits[j]
            test_arm[j] = float(np.clip(test_arm[j] + math.radians(args.delta_deg), lo, hi))
            test_tcp = fk.compute_tcp(test_arm)
            delta_tcp = test_tcp - home_tcp
            target_hw = mapper.sim_arm_to_hw_deg(test_arm)
            target_deg6 = target_hw + [cfg.gripper_home]
            hw_delta = target_deg6[j] - home_deg6[j]

            print(f"S{joint}: sim +{args.delta_deg:.1f} deg")
            print(f"  expected TCP delta: {format_delta(delta_tcp)}  dominant={dominant_axis(delta_tcp)}")
            print(f"  servo command: S{joint} {home_deg6[j]:.1f} -> {target_deg6[j]:.1f} ({hw_delta:+.1f} deg)")

            servo.send(home_deg6, args.run_time, "home")
            time.sleep(args.settle)
            servo.send(target_deg6, args.run_time, f"S{joint} test")
            time.sleep(args.settle)

            if args.real and args.interactive:
                matched = ask_match()
                if matched is False:
                    toggled = list(cfg.arm_hw_invert)
                    toggled[j] = not toggled[j]
                    recommendations.append(
                        f"S{joint}: likely invert should be {toggled[j]} "
                        f"(candidate flags: {tuple(toggled)})"
                    )
                elif matched is True:
                    recommendations.append(f"S{joint}: current invert looks OK")

            servo.send(home_deg6, args.run_time, "home")
            time.sleep(args.settle)
            print()

        if recommendations:
            print("Summary:")
            for item in recommendations:
                print(f"  - {item}")
    finally:
        if servo is not None:
            servo.close()
        if fk is not None:
            fk.close()


if __name__ == "__main__":
    main()
