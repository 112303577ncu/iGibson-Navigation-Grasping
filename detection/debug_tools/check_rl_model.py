#!/usr/bin/env python3
"""Inspect the current 55D navigation policy with correctly ordered observations."""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]


def load_nav_rl_module():
    """Load the active navigation module without changing sys.path on import."""
    import importlib.util
    import sys

    module_path = ROOT / "integration" / "nav_rl.py"
    spec = importlib.util.spec_from_file_location("x3plus_active_nav_rl", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load navigation module: {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> int:
    nav_rl = load_nav_rl_module()
    parser = argparse.ArgumentParser(description="Check X3Plus 55D PPO navigation actions")
    parser.add_argument("--model", default=None)
    parser.add_argument("--vecnorm", default=None)
    args = parser.parse_args()
    if (args.model is None) != (args.vecnorm is None):
        parser.error("--model and --vecnorm must be supplied together")

    cfg = nav_rl.NavRLConfig()
    if args.model is not None:
        cfg.model_path = args.model
        cfg.vecnorm_path = args.vecnorm
    policy = nav_rl.NavPolicy(cfg)
    rays = np.full(cfg.lidar_num_rays, cfg.lidar_max_dist, dtype=np.float32)

    print("goal_dist | bearing | action[0] | action[1]")
    print("-------------------------------------------")
    for goal_dist in (0.9, 0.5, 0.2):
        for bearing in (-0.8, -0.4, 0.0, 0.4, 0.8):
            obs = nav_rl.build_nav_obs(
                goal_dist,
                bearing,
                0.0,
                0.0,
                np.zeros(2, dtype=np.float32),
                rays,
            )
            action = policy.predict(obs)
            print(
                f"{goal_dist:8.2f} | {bearing:7.2f} | "
                f"{float(action[0]):8.3f} | {float(action[1]):8.3f}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
