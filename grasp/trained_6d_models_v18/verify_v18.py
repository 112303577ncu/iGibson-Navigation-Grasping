#!/usr/bin/env python3
"""Verify the staged grasp-v18 package against its manifest.

Basic mode (stdlib only, safe on the Jetson):
  * both artifacts present and SHA256 matches the manifest
  * the local URDF matches the URDF the policy was trained against
    (compares LF-normalised content, so Windows CRLF checkout does not
    produce a spurious mismatch)

Deep mode (--deep, needs stable-baselines3):
  * PPO observation space == (28,)
  * PPO action space == (6,)
  * VecNormalize obs_rms.mean.shape == (28,)

Exit code 0 = all checks passed.

    python verify_v18.py
    python verify_v18.py --deep
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
MANIFEST = HERE / "manifest.json"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def sha256_text_lf(path: Path) -> str:
    """Hash text content with line endings normalised to LF."""
    data = path.read_bytes().replace(b"\r\n", b"\n")
    return hashlib.sha256(data).hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--deep", action="store_true",
                    help="also load the model with stable-baselines3 and check the 28D/6D contract")
    args = ap.parse_args()

    if not MANIFEST.exists():
        print(f"FAIL  manifest not found: {MANIFEST}")
        return 1
    m = json.loads(MANIFEST.read_text(encoding="utf-8"))
    failures: list[str] = []

    print(f"package : {m['package']}  status={m['status']}")
    print(f"source  : {m['source']['repo']} @ {m['source']['commit'][:8]}")
    print()

    # ── artifacts ────────────────────────────────────────────────────────────
    paths = {}
    for key in ("model", "vecnormalize"):
        art = m["artifacts"][key]
        p = HERE / art["file"]
        paths[key] = p
        if not p.exists():
            failures.append(f"{key}: missing file {art['file']}")
            print(f"FAIL  {key:<13} missing: {art['file']}")
            continue
        got = sha256_file(p)
        ok = got == art["sha256"]
        if not ok:
            failures.append(f"{key}: SHA256 mismatch")
        print(f"{'OK  ' if ok else 'FAIL'}  {key:<13} sha256 {got[:16]}...  ({p.stat().st_size} B)")

    # both-or-neither pairing rule
    if (HERE / m["artifacts"]["model"]["file"]).exists() != \
       (HERE / m["artifacts"]["vecnormalize"]["file"]).exists():
        failures.append("pairing: model and VecNormalize must both be present")
        print("FAIL  pairing       model and VecNormalize must always ship together")

    # ── URDF (FK base must match what the policy trained against) ────────────
    urdf_rel = m["contract"]["urdf"]["laptop_path"]
    repo_root = HERE.parents[3]           # .../x3plus/grasp/trained_6d_models_v18 -> repo root
    urdf = repo_root / urdf_rel
    if not urdf.exists():
        alt = HERE.parent / "x3plus" / "yahboomcar.urdf"
        urdf = alt if alt.exists() else urdf
    if not urdf.exists():
        failures.append(f"urdf: not found ({urdf_rel})")
        print(f"FAIL  urdf          not found: {urdf_rel}")
    else:
        got = sha256_text_lf(urdf)
        want = m["contract"]["urdf"]["sha256_lf_normalized"]
        ok = got == want
        if not ok:
            failures.append("urdf: content differs from the training URDF -> FK would not match")
        print(f"{'OK  ' if ok else 'FAIL'}  urdf          sha256(LF) {got[:16]}...  {urdf}")

    # ── contract summary (informational) ─────────────────────────────────────
    gh = m["contract"]["grasp_home"]
    print()
    print(f"contract: obs {m['contract']['observation_dim']}D / action {m['contract']['action_dim']}D")
    print(f"grasp home API deg : {gh['api_deg']}")
    print(f"gripper            : open={m['contract']['gripper']['open_deg']}  "
          f"closed={m['contract']['gripper']['closed_deg']}")

    # ── deep check ───────────────────────────────────────────────────────────
    if args.deep:
        print()
        try:
            from stable_baselines3 import PPO
            import pickle
        except Exception as e:
            print(f"SKIP  deep          stable-baselines3 unavailable ({e})")
        else:
            try:
                model = PPO.load(str(paths["model"]), device="cpu")
                obs_shape = tuple(model.observation_space.shape)
                act_shape = tuple(model.action_space.shape)
                ok_obs = obs_shape == (m["contract"]["observation_dim"],)
                ok_act = act_shape == (m["contract"]["action_dim"],)
                if not ok_obs:
                    failures.append(f"deep: observation space {obs_shape}")
                if not ok_act:
                    failures.append(f"deep: action space {act_shape}")
                print(f"{'OK  ' if ok_obs else 'FAIL'}  obs space     {obs_shape}")
                print(f"{'OK  ' if ok_act else 'FAIL'}  action space  {act_shape}")

                with open(paths["vecnormalize"], "rb") as f:
                    vec = pickle.load(f)
                shape = tuple(vec.obs_rms.mean.shape)
                ok_v = shape == (m["contract"]["observation_dim"],)
                if not ok_v:
                    failures.append(f"deep: VecNormalize obs_rms {shape}")
                print(f"{'OK  ' if ok_v else 'FAIL'}  vecnorm shape {shape}")
            except Exception as e:
                failures.append(f"deep: load failed ({e})")
                print(f"FAIL  deep          {e}")

    # ── verdict ──────────────────────────────────────────────────────────────
    print()
    if failures:
        print(f"RESULT: FAIL ({len(failures)})")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("RESULT: PASS — package integrity and contract verified.")
    print(f"NOTE  : status is '{m['status']}'. {m['status_note']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
