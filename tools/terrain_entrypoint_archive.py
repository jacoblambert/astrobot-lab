#!/usr/bin/env python3
"""Preserve live-node source provenance while its ROS interface evolves.

Only the listed ROS/advisory adapters are eligible: raw map replay does not execute them.
All reconstruction, geometry, library and input bytes still require their exact
recorded hashes. A historical entrypoint never substitutes for changed mapper
code or bypasses exact replayed layer/association comparison.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run

NON_MAPPER_SOURCES = ("live_local_terrain.py", "local_terrain/contact_advisory.py", "local_terrain/contact_runtime.py")


def source_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def historical_entrypoint(path, digest, archive):
    path, archive = Path(path), Path(archive).resolve(strict=True)
    if path.name not in {Path(p).name for p in NON_MAPPER_SOURCES} or not re.fullmatch("[a-f0-9]{64}", digest):
        raise ValueError("only non-replayed live ROS entrypoint may use historical source")
    target = (archive / "terrain_entrypoints" / (digest + ".py")).resolve(strict=True)
    if not target.is_relative_to(archive) or target.name != path.name or source_hash(target) != digest:
        raise ValueError("historical live entrypoint missing, escaped or changed")
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument("--source", choices=NON_MAPPER_SOURCES, default="live_local_terrain.py")
    args = parser.parse_args()
    source = Path(__file__).parent / args.source
    digest = source_hash(source)
    output = allocate_run(args.archive_root, limit=1024**2)
    target = output / source.name
    shutil.copyfile(source, target)
    with (output / "manifest.json").open("x") as stream:
        json.dump(
            dict(source=str(source), sha256=digest, scope="historical non-replayed ROS entrypoint", acceptance=False),
            stream,
            indent=2,
        )
    index = args.archive_root / "terrain_entrypoints"
    index.mkdir(exist_ok=True)
    link = index / (digest + ".py")
    if not link.exists():
        link.symlink_to(Path("..") / output.name / source.name)
    historical_entrypoint(source, digest, args.archive_root)
    check_run(output)
    print(json.dumps(dict(run=str(output), sha256=digest)), flush=True)


if __name__ == "__main__":
    main()
