#!/usr/bin/env python3
"""Monte-Carlo FK workspace scan, side-effect free when imported."""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pybullet as p


def main() -> int:
    parser = argparse.ArgumentParser(description="X3Plus FK workspace scan")
    parser.add_argument("--samples", type=int, default=80000)
    args = parser.parse_args()
    if args.samples <= 0:
        parser.error("--samples must be > 0")

    client = p.connect(p.DIRECT)
    if client < 0:
        raise RuntimeError("PyBullet DIRECT connection failed")
    try:
        urdf = Path(__file__).resolve().parent / "x3plus" / "yahboomcar.urdf"
        robot = p.loadURDF(str(urdf), useFixedBase=True, physicsClientId=client)
        arm_joints = [8, 9, 10, 11, 12]
        limits = [(-1.5708, 1.5708)] * 4 + [(-1.5708, 3.14159)]
        rng = np.random.default_rng(42)
        reachable = []

        for _ in range(args.samples):
            angles = [rng.uniform(lo, hi) for lo, hi in limits]
            for joint, angle in zip(arm_joints, angles):
                p.resetJointState(robot, joint, angle, physicsClientId=client)
            p.stepSimulation(physicsClientId=client)
            reachable.append(p.getLinkState(robot, 12, physicsClientId=client)[0])

        arr = np.asarray(reachable)
        print(f"X range: {arr[:, 0].min():.3f} ~ {arr[:, 0].max():.3f}")
        print(f"Y range: {arr[:, 1].min():.3f} ~ {arr[:, 1].max():.3f}")
        print(f"Z range: {arr[:, 2].min():.3f} ~ {arr[:, 2].max():.3f}")
        front_low = arr[(arr[:, 0] > 0.15) & (arr[:, 2] < 0.25)]
        if len(front_low):
            print(f"\nfront-low reachable samples: {len(front_low)}")
            print(f"  X: {front_low[:, 0].min():.3f} ~ {front_low[:, 0].max():.3f}")
            print(f"  Z: {front_low[:, 2].min():.3f} ~ {front_low[:, 2].max():.3f}")
        else:
            print("\nNo samples with x>0.15 and z<0.25")
    finally:
        p.disconnect(client)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
