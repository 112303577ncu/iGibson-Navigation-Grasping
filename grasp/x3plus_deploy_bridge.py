"""Bridge policy action outputs to X3 Plus UART servo commands.

WARNING: This file is NOT used by x3plus_real_grasp.py.
The main deploy script uses JointMapper / ServoController internally.
Its default linear ranges now match the current all-False API-space mapping,
but x3plus_real_grasp.py remains the source of truth for real angle tests.

Pipeline (if you do extend this class):
1) normalized action [-1, 1] -> simulated joint range
2) simulated joint range -> hardware degree range
3) safety layer: hard clipping + per-step max delta
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class JointMap:
    sim_low: float
    sim_high: float
    hw_deg_low: float
    hw_deg_high: float
    max_delta_deg_per_step: float


def _map_linear(value: float, src_low: float, src_high: float, dst_low: float, dst_high: float) -> float:
    if src_high <= src_low:
        return float(dst_low)
    ratio = (value - src_low) / (src_high - src_low)
    return float(dst_low + ratio * (dst_high - dst_low))


class X3PlusServoBridge:
    """Convert policy action to safe servo-angle array for real robot execution."""

    def __init__(self, maps: Sequence[JointMap], emergency_pose_deg: Optional[Sequence[float]] = None):
        if len(maps) != 6:
            raise ValueError(f"Expected 6 joint maps, got {len(maps)}")
        self.maps = list(maps)
        self.last_cmd_deg: Optional[np.ndarray] = None
        self.emergency_pose_deg = np.array(emergency_pose_deg, dtype=np.float32) if emergency_pose_deg is not None else None

    @classmethod
    def default(cls) -> "X3PlusServoBridge":
        # NOT USED by main deploy script. Values below match DeployConfig's
        # all-False API-space mapping (hw(API) = 90 + sim_deg).
        # Confirmed hw ranges (from DeployConfig in x3plus_real_grasp.py):
        #   S1-S4: sim=(-π/2,π/2), hw=(0,180) → hw = 90 + sim_deg
        #   S5:    sim=(-π/2,π),   hw=(0,270) → hw = 90 + sim_deg
        #   S6: gripper open=30°, closed=180° (not a linear sim mapping)
        default_maps = [
            JointMap(sim_low=-1.5708, sim_high=1.5708, hw_deg_low=0.0, hw_deg_high=180.0, max_delta_deg_per_step=3.0),
            JointMap(sim_low=-1.5708, sim_high=1.5708, hw_deg_low=0.0, hw_deg_high=180.0, max_delta_deg_per_step=3.0),
            JointMap(sim_low=-1.5708, sim_high=1.5708, hw_deg_low=0.0, hw_deg_high=180.0, max_delta_deg_per_step=3.0),
            JointMap(sim_low=-1.5708, sim_high=1.5708, hw_deg_low=0.0, hw_deg_high=180.0, max_delta_deg_per_step=3.0),
            JointMap(sim_low=-1.5708, sim_high=3.14159, hw_deg_low=0.0, hw_deg_high=270.0, max_delta_deg_per_step=3.0),
            JointMap(sim_low=-1.5, sim_high=0.0, hw_deg_low=30.0, hw_deg_high=180.0, max_delta_deg_per_step=10.0),
        ]
        # Safe home keeps S6 open. Under the current convention 180° is CLOSED
        # and must not be used as an emergency/default pose.
        emergency_pose = [90.0, 90.0, 90.0, 90.0, 90.0, 30.0]
        return cls(default_maps, emergency_pose_deg=emergency_pose)

    def action_to_servo_deg(self, action_norm: Sequence[float]) -> List[float]:
        action = np.asarray(action_norm, dtype=np.float32)
        if action.shape != (6,):
            raise ValueError(f"Expected normalized action shape (6,), got {action.shape}")
        if not np.all(np.isfinite(action)):
            raise ValueError(f"Normalized action contains NaN or infinity: {action}")

        action = np.clip(action, -1.0, 1.0)
        cmd_deg = np.zeros(6, dtype=np.float32)

        for i, m in enumerate(self.maps):
            sim_angle = _map_linear(float(action[i]), -1.0, 1.0, m.sim_low, m.sim_high)
            target_deg = _map_linear(sim_angle, m.sim_low, m.sim_high, m.hw_deg_low, m.hw_deg_high)
            target_deg = float(np.clip(target_deg, m.hw_deg_low, m.hw_deg_high))

            if self.last_cmd_deg is None:
                safe_deg = target_deg
            else:
                prev = float(self.last_cmd_deg[i])
                safe_deg = prev + float(np.clip(target_deg - prev, -m.max_delta_deg_per_step, m.max_delta_deg_per_step))
                safe_deg = float(np.clip(safe_deg, m.hw_deg_low, m.hw_deg_high))

            cmd_deg[i] = safe_deg

        self.last_cmd_deg = cmd_deg.copy()
        return cmd_deg.tolist()

    def emergency_command(self) -> List[float]:
        if self.emergency_pose_deg is None:
            raise RuntimeError("No emergency pose configured")
        cmd = self.emergency_pose_deg.astype(np.float32).copy()
        for i, m in enumerate(self.maps):
            cmd[i] = float(np.clip(cmd[i], m.hw_deg_low, m.hw_deg_high))
        self.last_cmd_deg = cmd.copy()
        return cmd.tolist()

    def send(
        self,
        action_norm: Sequence[float],
        run_time: int,
        sender: Callable[..., object],
    ) -> List[float]:
        """Convert and send command to `set_uart_servo_angle_array`-like callable."""
        angles = self.action_to_servo_deg(action_norm)
        try:
            sender(angle_s=angles, run_time=run_time)
        except TypeError:
            sender(angles, run_time)
        return angles


def _parse_action_csv(action_csv: str) -> List[float]:
    vals = [float(x.strip()) for x in action_csv.split(",") if x.strip()]
    if len(vals) != 6:
        raise ValueError(f"Need exactly 6 values, got {len(vals)}")
    if not all(np.isfinite(v) for v in vals):
        raise ValueError(f"Action values must be finite, got {vals}")
    return vals


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert normalized 6D action to X3 Plus servo command")
    parser.add_argument("--action", type=str, required=True, help="CSV action, e.g. 0.1,-0.2,0,0.3,-0.1,0.8")
    parser.add_argument("--run-time", type=int, default=120, help="Servo run time in ms")
    parser.add_argument("--emergency", action="store_true", help="Emit emergency pose command")
    args = parser.parse_args()

    bridge = X3PlusServoBridge.default()
    if args.emergency:
        cmd = bridge.emergency_command()
        print(f"EMERGENCY_ANGLE_S={cmd}")
    else:
        action = _parse_action_csv(args.action)
        cmd = bridge.action_to_servo_deg(action)
        print(f"ANGLE_S={cmd}")
    print(f"RUN_TIME={args.run_time}")


if __name__ == "__main__":
    main()
