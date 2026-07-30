#!/usr/bin/env python3
"""Verify an installed X3Plus model package and optionally load SB3 artifacts."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from model_tools.model_package import deployment_paths, verify_package_directory


def _deep_verify(package_dir: Path, manifest):
    try:
        import gymnasium as gym
        import numpy as np
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
    except ImportError as exc:
        raise RuntimeError(f"--deep requires deployment dependencies: {exc}") from exc

    model_path, vecnorm_path = deployment_paths(package_dir, manifest)
    model = PPO.load(str(model_path), device="cpu")
    obs_shape = tuple(model.observation_space.shape)
    action_shape = tuple(model.action_space.shape)
    expected_obs = (manifest["contract"]["observation_dim"],)
    expected_action = (manifest["contract"]["action_dim"],)
    if obs_shape != expected_obs or action_shape != expected_action:
        raise RuntimeError(
            f"PPO spaces {obs_shape}/{action_shape} do not match manifest "
            f"{expected_obs}/{expected_action}"
        )

    obs_dim = expected_obs[0]
    action_dim = expected_action[0]

    class MockEnv(gym.Env):
        observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(obs_dim,), dtype=np.float32)
        action_space = gym.spaces.Box(-1.0, 1.0, shape=(action_dim,), dtype=np.float32)

        def reset(self, *, seed=None, options=None):
            return np.zeros(obs_dim, dtype=np.float32), {}

        def step(self, action):
            return np.zeros(obs_dim, dtype=np.float32), 0.0, False, False, {}

    loaded = VecNormalize.load(str(vecnorm_path), DummyVecEnv([MockEnv]))
    stats_shape = tuple(np.asarray(loaded.obs_rms.mean).shape)
    if stats_shape != expected_obs:
        raise RuntimeError(f"VecNormalize stats {stats_shape} do not match manifest {expected_obs}")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package_dir", type=Path)
    parser.add_argument("--deep", action="store_true",
                        help="load PPO and VecNormalize using deployment dependencies")
    parser.add_argument("--require-status", choices=("candidate", "sim-approved", "hardware-approved"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    manifest = verify_package_directory(args.package_dir)
    if args.require_status and manifest["status"] != args.require_status:
        raise SystemExit(f"required status {args.require_status}, got {manifest['status']}")
    if args.deep:
        _deep_verify(args.package_dir.resolve(), manifest)
    print(f"OK {manifest['model_id']} status={manifest['status']} "
          f"contract={manifest['contract']['observation_dim']}D/"
          f"{manifest['contract']['action_dim']}D deep={args.deep}")


if __name__ == "__main__":
    main()

