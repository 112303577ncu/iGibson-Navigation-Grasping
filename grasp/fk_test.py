#!/usr/bin/env python3
"""Print two FK reference poses without running work at import time."""
from pathlib import Path

import pybullet as p


def main() -> int:
    client = p.connect(p.DIRECT)
    if client < 0:
        raise RuntimeError("PyBullet DIRECT connection failed")
    try:
        urdf = Path(__file__).resolve().parent / "x3plus" / "yahboomcar.urdf"
        robot = p.loadURDF(str(urdf), useFixedBase=True, physicsClientId=client)

        for joint, angle in zip([8, 9, 10, 11, 12], [0, -1.5708, -1.5708, -1.5708, 0]):
            p.resetJointState(robot, joint, angle, physicsClientId=client)
        p.stepSimulation(physicsClientId=client)
        state = p.getLinkState(robot, 12, physicsClientId=client)
        print("TCP at max-down:", [round(x, 4) for x in state[0]])

        for joint in [8, 9, 10, 11, 12]:
            p.resetJointState(robot, joint, 0, physicsClientId=client)
        p.stepSimulation(physicsClientId=client)
        state = p.getLinkState(robot, 12, physicsClientId=client)
        print("TCP at home:    ", [round(x, 4) for x in state[0]])
    finally:
        p.disconnect(client)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
