#!/usr/bin/env python3
"""Evaluate actual coherent publication-time pose pairs; GT is evaluator-only."""
import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run
from local_terrain.evaluation import relative_error
from replay_local_terrain import sha


def evaluate(local_run, accuracy_run, capture):
    paths = [
        local_run / "report.json",
        local_run / "projections.jsonl",
        accuracy_run / "accuracy_report.json",
        accuracy_run / "accuracy_arrays.npz",
        capture / "shadow_link.json",
    ]
    hashes = {str(p): sha(p) for p in paths}
    local = json.loads(paths[0].read_text())
    accuracy = json.loads(paths[2].read_text())
    if (
        not local["diagnostic_completed"]
        or not accuracy["diagnostic_completed"]
        or local["output_sha256"] != hashes[str(paths[1])]
        or accuracy["output_sha256"]["accuracy_arrays.npz"] != hashes[str(paths[3])]
    ):
        raise ValueError("incomplete diagnostic or changed arrays/projections")
    if not any(capture.name in p for p in accuracy["input_sha256"]):
        raise ValueError("accuracy/capture identity mismatch")
    if local["input_sha256"].get(str(paths[4])) != hashes[str(paths[4])]:
        raise ValueError("local/capture provenance mismatch")
    mount = np.array(json.loads(paths[4].read_text())["request"]["base_from_lidar"])
    with np.load(paths[3], allow_pickle=False) as data:
        times = np.rint(data["times"] * 1e9).astype(np.int64)
        truth = data["gt_base"].copy()
        first = {int(i): p for i, p in zip(data["first_published_indices"], data["first_published"])}
    values = {"coherent_available": [], "first_published_same_pairs": []}
    missing, warmup = 0, 0

    def exact_index(stamp):
        k = np.searchsorted(times, stamp)
        candidates = [i for i in (k - 1, k) if 0 <= i < len(times) and abs(int(times[i]) - stamp) <= 2]
        if len(candidates) != 1:
            raise ValueError("no same-native-tick GT snapshot")
        return candidates[0]

    with paths[1].open() as stream:
        for line in stream:
            record = json.loads(line)
            if record["policy"] != "local_latest_available_odom_v1":
                raise ValueError("requires coherent local reconstruction")
            poses = record["associations"]
            if not poses:
                continue
            end = poses[-1]
            candidates = [p for p in poses if abs(end["acquisition_ns"] - p["acquisition_ns"] - 1_000_000_000) <= 1000]
            if len(candidates) != 1:
                warmup += 1
                continue
            start = candidates[0]
            if any(
                p["available_ns"] > record["publication_ns"] or p["output_ns"] != record["publication_ns"]
                for p in poses
            ):
                raise ValueError("future pose leakage")
            i, j = exact_index(start["acquisition_ns"]), exact_index(end["acquisition_ns"])
            if i not in first or j not in first:
                missing += 1
                continue
            a, b = [np.array(p["map_from_lidar"]) @ np.linalg.inv(mount) for p in (start, end)]
            values["coherent_available"].append(relative_error(a, b, truth[i], truth[j]))
            values["first_published_same_pairs"].append(relative_error(first[i], first[j], truth[i], truth[j]))
    if not values["coherent_available"]:
        raise ValueError("no comparable one-second pose pairs")
    results = {
        mode: {
            key: dict(p95=float(np.percentile([r[key] for r in rows], 95)), maximum=max(r[key] for r in rows))
            for key in ("translation_m", "rotation_rad")
        }
        for mode, rows in values.items()
    }
    if any(sha(path) != value for path, value in hashes.items()):
        raise ValueError("diagnostic inputs changed")
    return dict(
        acceptance=False,
        terrain_truth_evaluated=False,
        results=results,
        pairs=len(values["coherent_available"]),
        warmup_or_missing_window=warmup,
        missing_first_published=missing,
        source_failed=local["source_failed"],
        bounded_prefix=local["bounded_prefix"],
        window_s=local["window_s"],
        input_sha256=hashes,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("local_run", type=Path)
    parser.add_argument("accuracy_run", type=Path)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        result = evaluate(args.local_run.resolve(), args.accuracy_run.resolve(), args.capture.resolve())
        result["source_sha256"] = {str(Path(__file__).resolve()): sha(Path(__file__))}
        with (output / "coherent_motion.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps({k: v for k, v in result.items() if k != "input_sha256"}, indent=2))
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise
