#!/usr/bin/env python3
"""Exact per-map cost/mask comparison alongside unchanged raw/advisory replay."""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys

import numpy as np

from archive_guard import allocate_run, check_run
from local_terrain.navigation_cost import navigation_costs
from replay_contact_advisory import evaluate
from replay_local_terrain import sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--reference-cost-file", type=Path, required=True)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    check_run(args.source)
    if not args.reference_cost_file.resolve().is_relative_to(args.archive_root.resolve()):
        raise ValueError("archived reference required")
    spec = importlib.util.spec_from_file_location("frozen_navigation_cost", args.reference_cost_file)
    reference = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = reference
    spec.loader.exec_module(reference)
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    paths = [Path(__file__), Path(__file__).parent / "local_terrain/navigation_cost.py", args.reference_cost_file]
    hashes = {str(p): sha(p) for p in paths}
    for i, p in enumerate(paths):
        shutil.copyfile(p, output / f"source_{i}_{p.name}")
    maps = 0
    digest = hashlib.sha256()
    try:

        def observe(runtime, layers, record, actual):
            nonlocal maps
            old = reference.navigation_costs(layers)
            new = navigation_costs(layers)
            for before, after in zip(old, new):
                np.testing.assert_array_equal(before, after)
                digest.update(after.tobytes())
            maps += 1

        exact = evaluate(args.source, args.archive_root, output, observation_callback=observe)
        if not maps or any(sha(Path(p)) != h for p, h in hashes.items()):
            raise ValueError("empty replay or changed sources")
        report = dict(
            costs_and_masks_exact=True,
            maps=maps,
            output_sha256=digest.hexdigest(),
            raw_and_advisory_replay=exact,
            input_sha256=hashes,
            navigation_qualified=False,
        )
        (output / "report.json").write_text(json.dumps(report, indent=2))
        print(
            json.dumps({k: v for k, v in report.items() if k not in ("raw_and_advisory_replay", "input_sha256")}),
            flush=True,
        )
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=repr(exc), maps=maps)))
        raise
    check_run(output)


if __name__ == "__main__":
    main()
