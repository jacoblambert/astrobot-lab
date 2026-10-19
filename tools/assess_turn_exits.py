#!/usr/bin/env python3
"""Offline turn-then-drive availability, not a controller or safety certificate.

Keep the original straight-path denominator separately. A fixed additional set
tests whether steering, rather than changing support rules, exposes an exit.
Every path includes its full turn at the current location and subsequent drive;
no future observations, pose extrapolation or GT terrain enters the query.
"""

import argparse
import json
from pathlib import Path
from time import perf_counter

import numpy as np
from shapely import Polygon, difference, union_all

from archive_guard import allocate_run, check_run
from assess_startup_observation import exit_candidates
from local_terrain.contact_support import patch_polygons
from local_terrain.pose_lookup import PublicationPoseLookup
from local_terrain.support_sweep import swept_polygons
from replay_contact_advisory import evaluate
from replay_local_terrain import sha


def turn_candidates():
    result = {}
    for degrees in (-135, -90, -45, 0, 45, 90, 135, 180):
        yaw = np.radians(degrees)
        turn = np.eye(4)
        turn[:2, :2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
        for distance in (0.5, 1.0):
            end = turn.copy()
            end[:2, 3] = distance * turn[:2, 0]
            result[f"turn_{degrees:+d}_drive_{distance:.1f}m"] = [np.eye(4), turn.copy(), end]
    return result


def summarize(rows):
    decisions = [r for r in rows if r["motions"] is not None]
    return dict(
        decision_bins=len(decisions),
        straight_exit_bins=sum(any(m["exit"] for m in r["motions"]["straight"].values()) for r in decisions),
        turn_exit_bins=sum(any(m["exit"] for m in r["motions"]["turn_then_drive"].values()) for r in decisions),
        # A blocked turn is still a tested candidate, never removed from counts.
        tested_turn_paths=sum(len(r["motions"]["turn_then_drive"]) for r in decisions),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    paths = [Path(__file__), Path(__file__).with_name("assess_startup_observation.py")]
    hashes = {str(p): sha(p) for p in paths}
    for path in paths:
        (output / path.name).write_bytes(path.read_bytes())
    rows = []
    try:
        with (output / "decisions.jsonl").open("x") as sink:

            def observe(runtime, layers, record, actual):
                row = dict(query_ns=actual["query_ns"], motions=None)
                if actual["motions"] is not None:
                    started = perf_counter()
                    advisory = runtime.advisory
                    lookup = PublicationPoseLookup(runtime.timeline, actual["query_ns"])
                    startup_pose = (
                        np.asarray(record["local_from_map"])
                        @ lookup.associate(advisory.memory.anchors[0].stamp_ns).map_from_lidar
                        @ np.linalg.inv(runtime.timeline.extrinsic)
                        @ np.linalg.inv(runtime.node.core.base_from_imu)
                    )
                    xyz = np.c_[
                        advisory.memory.footprint, np.full(len(advisory.memory.footprint), advisory.memory.floor_z)
                    ]
                    startup = Polygon((xyz @ startup_pose[:3, :3].T + startup_pose[:3, 3])[:, :2])
                    assumed = union_all(
                        [
                            startup,
                            *[Polygon(p) for p in patch_polygons(advisory.context.local_from_base, advisory.patches)],
                        ]
                    )
                    row["motions"] = {}
                    for group, candidates in (("straight", exit_candidates()), ("turn_then_drive", turn_candidates())):
                        row["motions"][group] = {}
                        for name, relative in candidates.items():
                            decision = advisory.query(relative, query_ns=actual["query_ns"])
                            swept = union_all(
                                [Polygon(p) for p in swept_polygons(advisory.context.poses(relative), advisory.patches)]
                            )
                            outside = float(difference(swept, assumed).area)
                            row["motions"][group][name] = dict(
                                state=decision["state"],
                                reason=decision["reason"],
                                unresolved_m2=decision.get("new_ground_area_m2"),
                                beyond_assumed_m2=outside,
                                exit=decision["state"] == "available" and outside > 0.001,
                            )
                    row["diagnostic_seconds"] = perf_counter() - started
                sink.write(json.dumps(row, allow_nan=False) + "\n")
                rows.append(row)

            exact = evaluate(args.source, args.archive_root, output, observation_callback=observe)
        if any(sha(p) != digest for p, digest in hashes.items()):
            raise ValueError("diagnostic source changed")
        report = dict(
            completed=True,
            acceptance=False,
            navigation_authority=False,
            tracking_and_stopping_evaluated=False,
            hypothesis=__doc__,
            source=str(args.source),
            exact_replay=exact,
            **summarize(rows),
            source_sha256=hashes,
            output_sha256={"decisions.jsonl": sha(output / "decisions.jsonl")},
        )
    except Exception as exc:
        (output / "failure.json").write_text(json.dumps(dict(completed=False, error=repr(exc)), indent=2))
        raise
    (output / "report.json").write_text(json.dumps(report, indent=2))
    check_run(output)
    print(json.dumps({k: v for k, v in report.items() if k != "exact_replay"}, indent=2))


if __name__ == "__main__":
    main()
