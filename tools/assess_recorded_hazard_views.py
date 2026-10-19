#!/usr/bin/env python3
"""Repeat the fixed hazard bank using recorded XY/yaw; flat-ground model only.

Uses 15-degree crossings plus start/end from a completed turn. Original Z/tilt,
terrain relief, native measurements and SLAM error are intentionally excluded:
this isolates the effect of the real non-circular observation trajectory.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from assess_nearfield_hazards import STARTUP_RADIUS, make_sensor, render, summarize, view_counts


def recorded_views(capture):
    complete = json.loads((capture / "natural_holdout.json").read_text())
    if not complete["complete"]:
        raise ValueError("completed source required")
    rows = [json.loads(line) for line in (capture / "motion.jsonl").open()]
    origin = np.asarray(json.loads((capture / "clearance.json").read_text())["origin"])
    poses = np.asarray([r["world_from_base"] for r in rows])
    start_yaw = np.arctan2(origin[1, 0], origin[0, 0])
    yaw = np.unwrap(np.arctan2(poses[:, 1, 0], poses[:, 0, 0]) - start_yaw)
    if np.degrees(yaw[-1] - yaw[0]) < 350:
        raise ValueError("completed near-full turn required")
    indices = {0, len(rows) - 1}
    for angle in range(15, 360, 15):
        crossings = np.flatnonzero(np.degrees(yaw) >= angle)
        if len(crossings):
            indices.add(int(crossings[0]))
    c, s = np.cos(start_yaw), np.sin(start_yaw)
    world_to_start = np.array([[c, s], [-s, c]])
    return [
        dict(
            stamp=rows[i]["stamp"],
            yaw=float(np.degrees(yaw[i])),
            xy=(world_to_start @ (poses[i, :2, 3] - origin[:2, 3])).tolist(),
        )
        for i in sorted(indices)
    ]


def matched_rows(rows):
    """Common positions for all sizes of each shape; never let size drop hard cases."""
    eligible = {}
    for row in rows:
        key = row["bank"], row["kind"], row["radius"], row["bearing"]
        eligible[key] = eligible.get(key, True) and row["outside_startup"]
    return [r for r in rows if eligible[r["bank"], r["kind"], r["radius"], r["bearing"]]]


def size_sweep(rows):
    """Declared second-stage development sweep, not a held-out qualification."""
    result = []
    for row in rows:
        if row["kind"] != "rounded" or row["height"] != 0.08 or row["width"] != 0.10:
            continue
        for height in (0.08, 0.10, 0.12, 0.16):
            for width in (0.10, 0.12, 0.15, 0.20):
                result.append(
                    dict(
                        row,
                        id=len(result),
                        height=height,
                        width=width,
                        outside_startup=row["radius"] - width / 2 >= STARTUP_RADIUS,
                        protocols={},
                        views=[],
                    )
                )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("screen", type=Path)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    parser.add_argument("--size-sweep", action="store_true", help="separate post-baseline rounded-rock size sweep")
    args = parser.parse_args()
    check_run(args.screen)
    check_run(args.capture)
    previous = json.loads((args.screen / "report.json").read_text())
    bank = args.screen / "placements.jsonl"
    if (
        not previous["completed"]
        or hashlib.sha256(bank.read_bytes()).hexdigest() != previous["output_sha256"][bank.name]
    ):
        raise ValueError("completed unchanged placement bank required")
    output = allocate_run(args.archive_root, limit=128 * 1024**2)
    print(f"Run: {output}", flush=True)
    paths = [
        Path(__file__),
        Path(__file__).with_name("assess_nearfield_hazards.py"),
        bank,
        args.screen / "report.json",
        *[args.capture / n for n in ("natural_holdout.json", "motion.jsonl", "clearance.json")],
    ]
    # Carry the geometry dependencies as well as their original frozen hashes.
    paths += [Path(p) for p in previous["input_sha256"] if Path(p) not in paths]
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    if any(hashes[p] != h for p, h in previous["input_sha256"].items()):
        raise ValueError("original geometry/source changed; declare a separate screen")
    sensor, poses, rows = make_sensor(), recorded_views(args.capture), []
    cases = [json.loads(line) for line in bank.open()]
    if args.size_sweep:
        cases = size_sweep(cases)
    with (output / "placements.jsonl").open("x") as sink:
        for row in cases:
            views = [
                dict(
                    pose,
                    **view_counts(
                        sensor, dict(row, x=row["x"] - pose["xy"][0], y=row["y"] - pose["xy"][1]), pose["yaw"]
                    ),
                )
                for pose in poses
            ]
            row["protocols"]["recorded_turn_planar"] = {
                f"max_{key}": max(v[key] for v in views)
                for key in ("surface_returns", "raised_8cm_returns", "changed_returns", "missing_expected_returns")
            }
            row["recorded_planar_views"] = views
            sink.write(json.dumps(row) + "\n")
            rows.append(row)
            if len(rows) % 288 == 0:
                check_run(output)
                print(f"Scored {len(rows)} placements", flush=True)
    report = dict(
        completed=True,
        acceptance=False,
        size_sweep=args.size_sweep,
        comparison="post-baseline development sweep" if args.size_sweep else "original fixed placement bank",
        description=__doc__,
        source=str(args.screen),
        capture=str(args.capture),
        poses=poses,
        summary=summarize(rows),
        matched_size_summary=summarize(matched_rows(rows)),
        input_sha256=hashes,
        output_sha256={"placements.jsonl": hashlib.sha256((output / "placements.jsonl").read_bytes()).hexdigest()},
    )
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != hashes[str(p)] for p in paths):
        raise ValueError("source changed during assessment")
    (output / "report.json").write_text(json.dumps(report, indent=2))
    html = render(report["summary"]).replace(
        "<h1>",
        "<p><b>Recorded-turn comparison:</b> recorded XY/yaw at 15° crossings, "
        "flat ground and fixed loaded height; NOT actual native returns or full-pose replay.</p><h1>",
    )
    (output / "report.html").write_text(html)
    (output / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    check_run(output)
    print(output / "report.html")


if __name__ == "__main__":
    main()
