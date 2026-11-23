#!/usr/bin/env python3
"""Require unchanged support decisions before crediting an implementation speedup."""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from replay_local_terrain import sha


def same_scores(before, after):
    if before.keys() != after.keys():
        raise ValueError("changed decision fields/candidates")
    for key, value in before.items():
        other = after[key]
        if isinstance(value, dict):
            same_scores(value, other)
        elif key == "new_ground_area_m2":
            if not math.isclose(value, other, rel_tol=0, abs_tol=1e-12):
                raise ValueError("changed uncovered geometry")
        elif value != other:
            raise ValueError("changed support decision: " + key)


def compare_rows(before, after):
    if len(before) != len(after) or not before:
        raise ValueError("missing decision bins")
    for a, b in zip(before, after):
        for field in ("evidence_ns", "elapsed_s", "history_anchors", "missing_pose_anchors"):
            if a[field] != b[field]:
                raise ValueError("changed contact/query exposure")
        same_scores(a["variants"], b["variants"])
        same_scores(a["retrace"], b["retrace"])
    return len(before)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    reports, rows, hashes = [], [], {}
    for run in (args.before, args.after):
        check_run(run)
        report = json.loads((run / "report.json").read_text())
        if not report["completed"] or sha(run / "scores.jsonl") != report["output_sha256"]["scores.jsonl"]:
            raise ValueError("completed unchanged experiment required")
        reports.append(report)
        rows.append([json.loads(line) for line in (run / "scores.jsonl").open()])
        hashes[str(run / "report.json")] = sha(run / "report.json")
    output = allocate_run(args.archive_root, limit=128 * 1024**2)
    print(f"Run: {output}")
    try:
        contracts = [{k: v for k, v in r["contract"].items() if k != "geometry_backend"} for r in reports]
        if contracts[0] != contracts[1] or reports[0]["summary"] != reports[1]["summary"]:
            raise ValueError("changed policy contract or summary")
        if reports[0]["exact_replay"] != reports[1]["exact_replay"]:
            raise ValueError("changed original maps/replay sources")
        count = compare_rows(*rows)
        timing = [
            dict(
                zip(
                    ("median", "p95", "p99"),
                    map(float, np.percentile([r["diagnostic_query_wall_s"] for r in group], [50, 95, 99])),
                )
            )
            for group in rows
        ]
        result = dict(
            decisions_match=True,
            decision_bins=count,
            acceptance=False,
            navigation_authority=False,
            uncovered_area_numeric_tolerance_m2=1e-12,
            input_sha256=hashes,
            diagnostic_query_batch_seconds=timing,
            note="Instrumented offline batches, not live compute qualification; keep all timings.",
        )
        (output / "report.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=str(exc), acceptance=False)))
        raise


if __name__ == "__main__":
    main()
