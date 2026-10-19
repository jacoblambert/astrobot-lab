#!/usr/bin/env python3
"""Matched four-phase support experiment; exact baseline, no GT mapper inputs."""

import argparse
import json
from pathlib import Path
import time

import numpy as np
from shapely import Polygon, difference, union_all

from archive_guard import allocate_run, check_run
from assess_startup_observation import exit_candidates
from local_terrain.contact_support import patch_polygons
from local_terrain.phase_support import PHASES, PhaseSupport
from local_terrain.pose_lookup import PublicationPoseLookup
from local_terrain.rolling_mapper import RollingTerrainMapper
from local_terrain.self_filter import chassis_returns
from local_terrain.support_sweep import swept_polygons
from raw_return_filter import valid_range_mask
from replay_contact_advisory import evaluate
from replay_local_terrain import decoded_chunk, relocate, sha
from selected_lidar_profile import profile_path, selected_setup


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    check_run(args.source)
    manifest = json.loads((args.source / "manifest.json").read_text())
    config = manifest["config"]
    library = relocate(manifest["library"], args.archive_root)
    setup, _ = selected_setup(profile_path(manifest["request"]["lidar_profile"]["name"]))
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    paths = [Path(__file__), Path(__file__).parent / "local_terrain/phase_support.py", library]
    hashes = {str(p): sha(p) for p in paths}
    for path in paths[:2]:
        (output / path.name).write_bytes(path.read_bytes())
    chunks = {}
    for event in map(json.loads, (args.source / "events.jsonl").open()):
        if event["kind"] == "chunk":
            chunk = event["payload"]
            if chunk["sequence"] != len(chunks):
                raise ValueError("native sequence is incomplete")
            chunks[chunk["sequence"]] = chunk
    mappers, rows, sequence = [], [], -1
    try:
        with (args.source / "native.bin").open("rb") as raw, (output / "decisions.jsonl").open("x") as sink:

            def observe(runtime, layers, record, actual):
                nonlocal sequence
                started = time.perf_counter()
                origins = [np.asarray(record["origin"]) + offset for offset in PHASES[1:]]
                if not mappers:
                    for origin in origins:
                        mappers.append(
                            RollingTerrainMapper(
                                library, width=60, height=60, origin=origin, lifetime=config["evidence_lifetime_s"]
                            )
                        )
                for mapper, origin in zip(mappers, origins):
                    mapper.shift_origin(origin)
                for association in record["associations"]:
                    number, stamp = association["sequence"], association["acquisition_ns"]
                    if number <= sequence:
                        continue
                    if number != sequence + 1:
                        raise ValueError("phase comparison lost integrated input")
                    chunk = decoded_chunk(chunks[number], raw)
                    if chunk["stamp_ns"] != stamp or chunk["capture"] != manifest["request"]["capture"]:
                        raise ValueError("phase input association changed")
                    points = chunk["points"].copy()
                    points[chassis_returns(points, setup) | (chunk["intensities"] == 0)] = 0
                    keep, _ = valid_range_mask(points, config["minimum"], config["maximum"])
                    pose = np.asarray(association["local_from_lidar"])
                    world = points[keep].astype(float) @ pose[:3, :3].T + pose[:3, 3]
                    for mapper in mappers:
                        mapper.integrate(world, stamp / 1e9, pose[:3, 3], np.arctan2(pose[1, 0], pose[0, 0]))
                    sequence = number
                grids = [
                    (m.project(record["evidence_ns"] / 1e9), dict(record, origin=o.tolist()))
                    for m, o in zip(mappers, origins)
                ]
                row = dict(
                    query_ns=actual["query_ns"], phase_compute_ms=1000 * (time.perf_counter() - started), motions=None
                )
                if actual["motions"] is not None:
                    advisory = runtime.advisory
                    candidate = PhaseSupport(advisory.index, grids, record)
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
                    decisions = {}
                    for name, relative in exit_candidates().items():
                        poses = advisory.context.poses(relative)
                        swept = union_all([Polygon(p) for p in swept_polygons(poses, advisory.patches)])
                        beyond = float(difference(swept, assumed).area)
                        baseline = advisory.query(relative, query_ns=actual["query_ns"])
                        shifted = candidate.query(layers, record, poses, advisory.patches, advisory.body)
                        decisions[name] = dict(
                            baseline=baseline["state"],
                            candidate=shifted,
                            baseline_exit=baseline["state"] == "available" and beyond > 0.001,
                            candidate_exit=shifted["state"] == "available" and beyond > 0.001,
                            beyond_assumed_m2=beyond,
                        )
                    row["motions"] = decisions
                sink.write(json.dumps(row) + "\n")
                rows.append(row)

            exact = evaluate(args.source, args.archive_root, output, observation_callback=observe)
        if any(sha(p) != hashes[str(p)] for p in paths):
            raise ValueError("phase comparison input changed")
        batches = [r for r in rows if r["motions"] is not None]
        report = dict(
            completed=True,
            acceptance=False,
            navigation_authority=False,
            phases_m=PHASES,
            decisions=len(batches),
            compared_valid_maps=len(rows),
            exact_replay=exact,
            baseline_exit_decisions=sum(any(d["baseline_exit"] for d in r["motions"].values()) for r in batches),
            candidate_exit_decisions=sum(any(d["candidate_exit"] for d in r["motions"].values()) for r in batches),
            extra_three_grid_ms_p50_p95_p99_max=np.percentile(
                [r["phase_compute_ms"] for r in rows], [50, 95, 99, 100]
            ).tolist(),
            limits=[
                "matched frozen pose associations, same classifier/lifetime",
                "not runtime qualification",
                "same 20 straight paths and query times; no live commands",
                "all phases veto known hazards",
            ],
            input_sha256=hashes,
            output_sha256={"decisions.jsonl": sha(output / "decisions.jsonl")},
        )
        (output / "report.json").write_text(json.dumps(report, indent=2))
        print(json.dumps({k: v for k, v in report.items() if k != "exact_replay"}, indent=2))
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=str(exc), acceptance=False)))
        raise
    finally:
        for mapper in mappers:
            mapper.close()
    check_run(output)


if __name__ == "__main__":
    main()
