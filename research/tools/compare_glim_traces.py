#!/usr/bin/env python3
"""Attribute repeat differences without alignment, filtering or selecting a winner."""
import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from shadow_pose_replay import sha


def indexed(rows, stage, key):
    selected = [r for r in rows if r["stage"] == stage]
    result = {r[key]: r for r in selected}
    if not selected or len(result) != len(selected):
        raise ValueError("missing or duplicate trace records: " + stage)
    return result


def compare(left, right):
    result = {}
    a = indexed(left, "processed_input", "stamp_ns")
    b = indexed(right, "processed_input", "stamp_ns")
    if a.keys() != b.keys():
        raise ValueError("processed input stamps differ")
    result["processed_frames"] = len(a)
    for field in ("points", "xyz_fnv1a", "neighbors_fnv1a", "times_fnv1a", "imu_count", "last_imu_stamp_ns"):
        different = [t for t in a if a[t][field] != b[t][field]]
        result[field] = dict(different_frames=len(different), first_stamp_ns=min(different) if different else None)
    for stage in ("imu_prediction", "optimized"):
        a, b = (indexed(r, stage, "frame_id") for r in (left, right))
        if a.keys() != b.keys():
            raise ValueError("stage frame IDs differ")
        delta = [np.max(abs(np.asarray(a[k]["map_from_imu"])-b[k]["map_from_imu"])) for k in a]
        ids = [k for k, d in zip(a, delta) if d > 1e-10]
        result[stage] = dict(frames=len(a), different_frames=len(ids), first_frame_id=min(ids) if ids else None,
                             max_matrix_difference=float(max(delta)))
    def gps(rows):
        factors = [f for r in rows if r["stage"] == "pre_smoother" for f in r["factors"] if "gps_position" in f]
        keyed = {int(f["keys"][0]): f for f in factors}
        if not factors or len(keyed) != len(factors):
            raise ValueError("missing or repeated GPS pose keys")
        return keyed
    a, b = gps(left), gps(right)
    common = a.keys() & b.keys()
    different = [k for k in common if a[k] != b[k]]
    result["gps"] = dict(left=len(a), right=len(b), unmatched_keys=len(a.keys() ^ b.keys()),
        different_measurements=len(different), first_frame_id=min(k & ((1 << 56)-1) for k in different) if different else None)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("left", type=Path)
    parser.add_argument("right", type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        records, provenance = [], {}
        for run in (args.left, args.right):
            check_run(run)
            report = json.loads((run/"shadow_pair_audit.json").read_text())
            trace = run/"estimator_trace.jsonl"
            if not report["passed"] or (run/"failure.json").exists() or report["input_sha256"].get(str(trace)) != sha(trace):
                raise ValueError("unqualified trace")
            for path, expected in report["input_sha256"].items():
                if sha(Path(path)) != expected: raise ValueError("replay provenance changed")
            provenance[str(run/"shadow_pair_audit.json")] = sha(run/"shadow_pair_audit.json")
            records.append([json.loads(line) for line in trace.read_text().splitlines()])
        result = dict(completed=True, acceptance=False, results=compare(*records), input_sha256=provenance,
                      note="Descriptive comparison; configuration changes must be interpreted separately.")
        with (output/"trace_comparison.json").open("x") as stream: json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps(result["results"], indent=2))
    except Exception as exc:
        with (output/"failure.json").open("x") as stream: json.dump(dict(error=str(exc)), stream)
        raise


if __name__ == "__main__": main()
