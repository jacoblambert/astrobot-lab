#!/usr/bin/env python3
"""Optimistic diagnostic: ignore unknown cells touched by CURRENT wheel patches.

This is not a safe policy: standing on part of a 10 cm cell does not prove its
whole area supports a wheel. Never clears occupied cells or unknown new cells.
Measures whether validating/preserving current support would be worth pursuing.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from assess_navigation_gaps import candidate_motions, cell_counts
from export_gap_examples import export
from local_terrain.physx_reference import BodyBox
from local_terrain.support_sweep import swept_polygons, touched_cells
from replay_local_terrain import sha


def remaining_unknown(classes, sweep, current):
    current = set(map(tuple, current))
    new = np.asarray([xy for xy in sweep if tuple(xy) not in current], dtype=int).reshape(-1, 2)
    return cell_counts(classes, new)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("assessment", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    check_run(args.assessment)
    report = json.loads((args.assessment / "report.json").read_text())
    score_path = args.assessment / "motion_scores.jsonl"
    if sha(score_path) != report["output_sha256"]["motion_scores.jsonl"]:
        raise ValueError("changed original scores")
    expected = {r["evidence_ns"]: r for r in map(json.loads, score_path.read_text().splitlines())}
    recorder = args.archive_root / Path(report["grids"]["recorder"]).name
    mapper = args.archive_root / Path(report["grids"]["mapper"]).name
    for p in (recorder, mapper):
        check_run(p)
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    paths = [args.assessment / "report.json", score_path, Path(__file__),
             Path(__file__).with_name("assess_navigation_gaps.py"), Path(__file__).with_name("export_gap_examples.py"),
             Path(__file__).with_name("local_terrain") / "support_sweep.py",
             Path(__file__).with_name("local_terrain") / "physx_reference.py"]
    hashes = {str(p): sha(p) for p in paths}
    try:
        patches = [BodyBox(p["center"], p["half_extent"], "assumed_wheel_patch") for p in report["contract"]["patches"]]
        rows = []
        with (output / "counterfactual.jsonl").open("x") as sink:
            def observe(classes, record):
                stamp = record["evidence_ns"]
                if stamp not in expected:
                    return
                original = expected[stamp]
                motions = candidate_motions(record["local_from_base"])
                current, _ = touched_cells(swept_polygons(motions["stationary"], patches),
                                           record["origin"], record["resolution"])
                results = {}
                for name, poses in motions.items():
                    cells, _ = touched_cells(swept_polygons(poses, patches), record["origin"], record["resolution"])
                    old = original["motions"][name]
                    if cell_counts(classes, cells) != old["wheel_support"]:
                        raise ValueError("original query changed")
                    remaining = remaining_unknown(classes, cells, current)
                    available = (old["no_known_obstacle"] and old["wheel_support"]["outside"] == 0
                                 and remaining["unknown"] == 0)
                    results[name] = dict(optimistic_available=available, new_ground=remaining,
                                         toward_goal=old["toward_goal"])
                row = dict(evidence_ns=stamp, elapsed_s=original["elapsed_s"], motions=results)
                rows.append(row)
                sink.write(json.dumps(row) + "\n")

            grid_report = export(recorder, mapper, output, grid_observer=observe)
        if len(rows) != len(expected) or grid_report["missing_projection_stamps"]:
            raise ValueError("missing comparison maps")
        totals = {}
        for period, selected in (("all", rows), ("first_15s", [r for r in rows if r["elapsed_s"] < 15]),
                                 ("after_15s", [r for r in rows if r["elapsed_s"] >= 15])):
            progress = turns = combined = 0
            for row in selected:
                move = any(s["optimistic_available"] and s["toward_goal"] for n, s in row["motions"].items()
                           if n.startswith("move"))
                turn = any(s["optimistic_available"] for n, s in row["motions"].items() if n.startswith("turn"))
                progress += move
                turns += turn
                combined += move or turn
            totals[period] = dict(samples=len(selected), progress=progress, turn=turns, progress_or_turn=combined)
        for path, digest in hashes.items():
            if sha(path) != digest:
                raise ValueError("comparison source changed")
        result = dict(completed=True, acceptance=False, navigation_authority=False, assumption=__doc__,
                      summary=totals, source=str(args.assessment), input_sha256=hashes,
                      grids=grid_report, output_sha256={"counterfactual.jsonl": sha(output / "counterfactual.jsonl")})
        check_run(output)
        (output / "report.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(totals, indent=2), flush=True)
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=str(exc), acceptance=False)))
        raise


if __name__ == "__main__":
    main()
