#!/usr/bin/env python3
"""Explain unknown wheel-support cells without changing any recorded mapper output."""

import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from attribute_live_floor_support import REASONS
from local_terrain.physx_reference import BodyBox
from local_terrain.support_sweep import swept_polygons, touched_cells
from replay_live_local_terrain import replay
from replay_local_terrain import sha


def explain(layers, record, patches):
    indices, _ = touched_cells(swept_polygons([record["local_from_base"]], patches),
                              record["origin"], record["resolution"])
    x, y = indices.T
    inside = (x >= 0) & (y >= 0) & (x < layers.shape[1]) & (y < layers.shape[0])
    cells = layers[y[inside], x[inside]]
    reasons = cells[cells[:, 0] == -1, 1]
    if not np.isin(reasons, np.arange(len(REASONS))).all():
        raise ValueError("unrecognized rejection reason")
    return dict(cells=len(indices), unknown=int((cells[:, 0] == -1).sum()),
                occupied=int((cells[:, 0] == 100).sum()), outside=int((~inside).sum()),
                unknown_reasons={name: int((reasons == i).sum()) for i, name in enumerate(REASONS)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("assessment", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    check_run(args.assessment)
    report = json.loads((args.assessment / "report.json").read_text())
    if not report["diagnostic_completed"] or report["grids"]["missing_projection_stamps"]:
        raise ValueError("completed map assessment required")
    source = args.archive_root / Path(report["grids"]["mapper"]).name
    check_run(source)
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    hashes = {str(p): sha(p) for p in (args.assessment / "report.json", args.assessment / "motion_scores.jsonl",
                                      Path(__file__), Path(__file__).with_name("attribute_live_floor_support.py"))}
    try:
        if hashes[str(args.assessment / "motion_scores.jsonl")] != report["output_sha256"]["motion_scores.jsonl"]:
            raise ValueError("changed source motion scores")
        patches = [BodyBox(p["center"], p["half_extent"], "assessment_assumed_patch")
                   for p in report["contract"]["patches"]]
        sampled = {r["evidence_ns"]: r for r in
                   map(json.loads, (args.assessment / "motion_scores.jsonl").read_text().splitlines())}
        totals, count, compared, terminal = dict.fromkeys(REASONS, 0), 0, 0, 0
        with (output / "reasons.jsonl").open("x") as stream:
            def observer(layers, record, *, published_valid):
                nonlocal count, compared, terminal
                if not published_valid:
                    terminal += 1
                    return  # Terminal all-unknown invalidation is not classifier rejection.
                result = explain(layers, record, patches)
                stamp = record["evidence_ns"]
                if stamp in sampled:
                    expected = sampled[stamp]["motions"]["stationary"]["wheel_support"]
                    if any(result[k] != expected[k] for k in ("unknown", "occupied", "outside")):
                        raise ValueError("replay wheel query disagrees with recorded-map assessment")
                    compared += 1
                for k, v in result["unknown_reasons"].items():
                    totals[k] += v
                stream.write(json.dumps(dict(evidence_ns=stamp, **result)) + "\n")
                count += 1
                if count % 300 == 0:
                    print(f"Explained {count} exact-matched maps.", flush=True)

            exact = replay(source, args.archive_root, projection_observer=observer)
        if count != report["grids"]["recorded_projections"] or compared != len(sampled):
            raise ValueError("incomplete explanation")
        for path, digest in hashes.items():
            if sha(path) != digest:
                raise ValueError("explanation source changed")
        result = dict(completed=True, acceptance=False, maps=count, sampled_compared=compared,
                      terminal_invalidations=terminal, unknown_reasons=totals, exact_replay=exact,
                      input_sha256=hashes, output_sha256={"reasons.jsonl": sha(output / "reasons.jsonl")})
        check_run(output)
        (output / "report.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(totals, indent=2), flush=True)
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=str(exc), acceptance=False)))
        raise


if __name__ == "__main__":
    main()
