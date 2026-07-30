#!/usr/bin/env python3
"""Download a private GitHub Release asset and install the model package."""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request
from pathlib import Path
from typing import Optional

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from model_tools.model_package import deployment_paths, install_archive


API_ROOT = "https://api.github.com"


def _token() -> Optional[str]:
    value = os.getenv("GITHUB_TOKEN") or os.getenv("GH_TOKEN")
    if value:
        return value.strip()
    try:
        result = subprocess.run(
            ["gh", "auth", "token"], check=True, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, universal_newlines=True,
        )
        return result.stdout.strip() or None
    except (OSError, subprocess.CalledProcessError):
        return None


def _request(url: str, token: Optional[str], accept: str) -> urllib.request.Request:
    headers = {"Accept": accept, "User-Agent": "x3plus-model-fetch/1"}
    if token:
        headers["Authorization"] = "Bearer " + token
    return urllib.request.Request(url, headers=headers)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", required=True, help="owner/name")
    parser.add_argument("--tag", required=True)
    parser.add_argument("--asset", default=None,
                        help="exact asset name; defaults to the only .tar.gz asset")
    parser.add_argument("--install-root", type=Path,
                        default=Path("grasp/model_packages"))
    parser.add_argument("--replace", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    token = _token()
    if token is None:
        raise SystemExit("private repository access requires GITHUB_TOKEN/GH_TOKEN or an authenticated gh CLI")
    release_url = f"{API_ROOT}/repos/{args.repo}/releases/tags/{args.tag}"
    try:
        with urllib.request.urlopen(_request(release_url, token, "application/vnd.github+json")) as response:
            release = json.loads(response.read().decode("utf-8"))
    except (urllib.error.URLError, ValueError) as exc:
        raise SystemExit(f"could not read release {args.repo}@{args.tag}: {exc}")

    assets = release.get("assets", [])
    if args.asset:
        matches = [asset for asset in assets if asset.get("name") == args.asset]
    else:
        matches = [asset for asset in assets if str(asset.get("name", "")).endswith(".tar.gz")]
    if len(matches) != 1:
        names = ", ".join(str(asset.get("name")) for asset in assets) or "none"
        raise SystemExit(f"expected exactly one matching asset, found {len(matches)}; release assets: {names}")

    asset = matches[0]
    asset_url = asset.get("url")
    with tempfile.TemporaryDirectory(prefix="x3plus-release-") as temp:
        archive = Path(temp) / asset["name"]
        try:
            with urllib.request.urlopen(_request(asset_url, token, "application/octet-stream")) as response:
                with archive.open("wb") as stream:
                    while True:
                        chunk = response.read(1024 * 1024)
                        if not chunk:
                            break
                        stream.write(chunk)
        except urllib.error.URLError as exc:
            raise SystemExit(f"could not download {asset['name']}: {exc}")
        target, manifest, backup = install_archive(archive, args.install_root, args.replace)

    model, vecnorm = deployment_paths(target, manifest)
    print(f"installed {manifest['model_id']} ({manifest['status']}) at {target}")
    if backup is not None:
        print(f"previous version preserved at {backup}")
    print(f"model: {model}")
    print(f"vecnorm: {vecnorm}")


if __name__ == "__main__":
    main()

