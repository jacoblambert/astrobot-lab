#!/usr/bin/env python3
"""Fail-closed, exact replay regression comparison; timings are not deterministic."""
import argparse
import json
from pathlib import Path

import numpy as np


def compare_replays(baseline, candidate):
    manifests, summaries = [], []
    for run in (baseline, candidate):
        if (run / "failure.json").exists():
            raise ValueError("failed replay is ineligible")
        manifests.append(json.loads((run / "manifest.json").read_text()))
        summaries.append(json.loads((run / "summary.json").read_text()))
    for key in ("input_sha256", "reference_sha256", "variants", "multiview", "visibility_denominator"):
        if key not in manifests[0] or manifests[0][key] != manifests[1].get(key):
            raise ValueError(f"different or missing replay provenance: {key}")
    for key in ("source", "pose_sensitivity"):
        if manifests[0].get(key) != manifests[1].get(key):
            raise ValueError(f"different replay pose/source provenance: {key}")
    default_grid=dict(origin=[-10.,-10.],resolution=.1,width=200,height=200)
    if manifests[0].get("terrain_geometry",default_grid)!=manifests[1].get("terrain_geometry",default_grid):
        raise ValueError("different terrain grid geometry")
    if not manifests[0]["input_sha256"] or not summaries[0].get("frames"):
        raise ValueError("empty fixture cannot establish regression equivalence")
    if summaries[0]["frames"] != summaries[1].get("frames"):
        raise ValueError("per-frame maps, metrics or scan associations differ")
    variants = manifests[0]["variants"]
    if not variants or any(Path(name).name != name for name in variants):
        raise ValueError("invalid replay variant names")
    for name in variants:
        with np.load(baseline / f"{name}.npz") as old, np.load(candidate / f"{name}.npz") as new:
            if not np.array_equal(old["layers"], new["layers"], equal_nan=True):
                raise ValueError(f"final layered output differs: {name}")
    return dict(exact_match=True, frames=len(summaries[0]["frames"]), variants=len(variants),
                timing_comparison=False, acceptance=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    args = parser.parse_args()
    print(json.dumps(compare_replays(args.baseline, args.candidate), indent=2))


if __name__ == "__main__":
    main()
