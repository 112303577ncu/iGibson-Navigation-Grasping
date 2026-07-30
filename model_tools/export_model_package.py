#!/usr/bin/env python3
"""Create a deterministic X3Plus model package from a manifest template."""
from __future__ import annotations

import argparse
import gzip
import json
import shutil
import sys
import tarfile
import tempfile
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from model_tools.model_package import sha256_file, validate_manifest


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest-template", required=True, type=Path)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--vecnorm", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    for path, label in ((args.manifest_template, "manifest template"),
                        (args.model, "model"), (args.vecnorm, "VecNormalize")):
        if not path.is_file():
            raise SystemExit(f"{label} does not exist: {path}")
    try:
        manifest = json.loads(args.manifest_template.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise SystemExit(f"invalid manifest template: {exc}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="x3plus-model-export-") as temp:
        package_dir = Path(temp) / manifest.get("model_id", "model-package")
        package_dir.mkdir()
        model_target = package_dir / "model.zip"
        vecnorm_target = package_dir / "vecnormalize.pkl"
        shutil.copyfile(str(args.model), str(model_target))
        shutil.copyfile(str(args.vecnorm), str(vecnorm_target))
        manifest["files"] = {
            "model": {"path": model_target.name, "sha256": sha256_file(model_target)},
            "vecnormalize": {"path": vecnorm_target.name, "sha256": sha256_file(vecnorm_target)},
        }
        validate_manifest(manifest)
        (package_dir / "manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        # Fix gzip/tar timestamps and ownership so exporting the same inputs and
        # manifest produces byte-identical archives on the desktop and laptop.
        with args.output.open("wb") as raw_output:
            with gzip.GzipFile(filename="", fileobj=raw_output, mode="wb", mtime=0) as compressed:
                with tarfile.open(fileobj=compressed, mode="w", format=tarfile.PAX_FORMAT) as archive:
                    for path in sorted(item for item in package_dir.rglob("*") if item.is_file()):
                        info = archive.gettarinfo(
                            str(path), arcname=str(path.relative_to(package_dir.parent))
                        )
                        info.mtime = 0
                        info.uid = 0
                        info.gid = 0
                        info.uname = ""
                        info.gname = ""
                        info.mode = 0o644
                        with path.open("rb") as stream:
                            archive.addfile(info, stream)

    print(f"created: {args.output}")
    print(f"model sha256: {manifest['files']['model']['sha256']}")
    print(f"vecnorm sha256: {manifest['files']['vecnormalize']['sha256']}")


if __name__ == "__main__":
    main()
