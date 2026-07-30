#!/usr/bin/env python3
"""Verify and atomically install a local X3Plus model-package archive."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from model_tools.model_package import deployment_paths, install_archive


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--install-root", type=Path,
                        default=Path("grasp/model_packages"))
    parser.add_argument("--replace", action="store_true",
                        help="preserve the existing package as a timestamped backup")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.archive.is_file():
        raise SystemExit(f"archive does not exist: {args.archive}")
    target, manifest, backup = install_archive(args.archive, args.install_root, args.replace)
    model, vecnorm = deployment_paths(target, manifest)
    print(f"installed {manifest['model_id']} ({manifest['status']}) at {target}")
    if backup is not None:
        print(f"previous version preserved at {backup}")
    print("dry-run command:")
    home = ",".join(str(value) for value in manifest["contract"]["grasp_home_api_deg"])
    print(f"  python3 grasp/x3plus_real_grasp.py --model {model} --vecnorm {vecnorm} "
          f"--grasp-home-deg {home}")


if __name__ == "__main__":
    main()

