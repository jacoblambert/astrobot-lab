#!/usr/bin/env python3
"""Matched terrain-library experiment on every original frozen live association.

No GT pose substitution, rescheduling, input trimming or navigation authority.
The required baseline mirror matches all six output channels byte for byte.
Candidate scores reuse the existing assessment's poses, goals and denominators.
"""

import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from assess_navigation_gaps import candidate_motions, floor_scores, motion_score, summarize
from build_local_terrain import validate_build
from local_terrain.physx_reference import BodyBox
from local_terrain.rolling_mapper import RollingTerrainMapper
from local_terrain.self_filter import chassis_returns
from local_terrain.support_sweep import swept_polygons, touched_cells
from raw_return_filter import valid_range_mask
from replay_local_terrain import decoded_chunk, sha
from selected_lidar_profile import profile_path, selected_setup


def run(assessment, library, archive, output, *, require_exact=False, diagnostics=False, compare_library=None):
    previous = json.loads((assessment / "report.json").read_text())
    source = archive / Path(previous["grids"]["mapper"]).name
    manifest = json.loads((source / "manifest.json").read_text())
    collection = json.loads((source / "collection.json").read_text())
    if (
        not previous["diagnostic_completed"]
        or not collection["experiment_completed"]
        or collection["status"]["failure"]
    ):
        raise ValueError("completed original assessment and mapper required")
    if previous["grids"]["missing_projection_stamps"]:
        raise ValueError("missing original recorded maps")
    capture = archive / manifest["request"]["capture"]
    check_run(source)
    check_run(capture)
    config = manifest["config"]
    baseline = archive / Path(manifest["library"]).parent.name / Path(manifest["library"]).name
    hashes = validate_build(library, archive)
    if compare_library is not None:
        hashes.update(validate_build(compare_library, archive))
    hashes.update({str(source / n): h for n, h in collection["output_sha256"].items()})
    for name in ("motion_scores.jsonl", "floor_scores.jsonl"):
        hashes[str(assessment / name)] = previous["output_sha256"][name]
    for path in [
        assessment / "report.json",
        source / "manifest.json",
        source / "collection.json",
        capture / "clearance_tile.npz",
        capture / "clearance.json",
        baseline,
        *[
            Path(__file__).with_name(n)
            for n in (
                "reassess_local_terrain.py",
                "assess_navigation_gaps.py",
                "raw_return_filter.py",
                "replay_local_terrain.py",
            )
        ],
        *sorted(Path(__file__).with_name("local_terrain").glob("*.py")),
    ]:
        hashes[str(path)] = sha(path)
    clearance = json.loads((capture / "clearance.json").read_text())
    if sha(capture / "clearance_tile.npz") != clearance["tile_sha256"] or not clearance["passed"]:
        raise ValueError("changed independent reference")
    for path, digest in hashes.items():
        if sha(path) != digest:
            raise ValueError("changed input: " + path)
    (output / "manifest.json").write_text(
        json.dumps(
            dict(
                input_sha256=hashes,
                source=str(source),
                assessment=str(assessment),
                library=str(library),
                require_exact=require_exact,
                diagnostics=diagnostics,
                compare_library=str(compare_library) if compare_library else None,
                acceptance=False,
            ),
            indent=2,
        )
    )
    expected_motion = {
        r["evidence_ns"]: r for r in map(json.loads, (assessment / "motion_scores.jsonl").read_text().splitlines())
    }
    expected_floor = {
        r["evidence_ns"]: r for r in map(json.loads, (assessment / "floor_scores.jsonl").read_text().splitlines())
    }
    patches = [BodyBox(p["center"], p["half_extent"], "same_assumed_patch") for p in previous["contract"]["patches"]]
    body = BodyBox((0, 0, 0), (0.85, 0.70, 0.001), "same_body_guard")
    with np.load(capture / "clearance_tile.npz", allow_pickle=False) as tile:
        points = np.c_[tile["xy"], tile["z"]]
    setup, _ = selected_setup(profile_path(manifest["request"]["lidar_profile"]["name"]))
    chunks = {}
    for event in map(json.loads, (source / "events.jsonl").open()):
        if event["kind"] == "chunk":
            chunk = event["payload"]
            if chunk["sequence"] != len(chunks):
                raise ValueError("unordered native chunks")
            chunks[chunk["sequence"]] = chunk
    mappers, rows, totals, timings = {}, [], {}, []
    last_sequence, frames, equal = -1, 0, 0
    attribution = dict(low_return=0, pooled_normal=0, native_boundary=0, untagged=0)
    examples = []
    display_stamps = {e["evidence_ns"] for e in previous["grids"]["examples"]}
    first_ns = min(expected_floor)
    try:
        with (source / "native.bin").open("rb") as raw, (output / "scores.jsonl").open("x") as sink:
            for record in map(json.loads, (source / "projections.jsonl").open()):
                if not mappers:
                    libraries = [("baseline", baseline), ("candidate", library)]
                    if compare_library is not None:
                        libraries.append(("comparison", compare_library))
                    for name, lib in libraries:
                        mappers[name] = RollingTerrainMapper(
                            lib,
                            width=config["cells"],
                            height=config["cells"],
                            resolution=config["resolution"],
                            origin=record["origin"],
                            lifetime=config["evidence_lifetime_s"],
                        )
                for mapper in mappers.values():
                    mapper.shift_origin(record["origin"])
                elapsed = 0.0
                integrated = 0
                for association in record["associations"]:
                    sequence, stamp = association["sequence"], association["acquisition_ns"]
                    if sequence <= last_sequence:
                        continue
                    if sequence != last_sequence + 1:
                        raise ValueError("missing integrated native input")
                    chunk = decoded_chunk(chunks[sequence], raw)
                    if chunk["stamp_ns"] != stamp or chunk["capture"] != capture.name:
                        raise ValueError("native association changed")
                    native = chunk["points"].copy()
                    native[chassis_returns(native, setup) | (chunk["intensities"] == 0)] = 0
                    keep, _ = valid_range_mask(native, config["minimum"], config["maximum"])
                    pose = np.asarray(association["local_from_lidar"])
                    world = native[keep].astype(float) @ pose[:3, :3].T + pose[:3, 3]
                    for name, mapper in mappers.items():
                        start = time.perf_counter()
                        mapper.integrate(world, stamp / 1e9, pose[:3, 3], np.arctan2(pose[1, 0], pose[0, 0]))
                        if name == "candidate":
                            elapsed += time.perf_counter() - start
                    integrated += 1
                    last_sequence = sequence
                if integrated != record["integrated_inputs"]:
                    raise ValueError("changed integration schedule")
                maps = {}
                for name, mapper in mappers.items():
                    start = time.perf_counter()
                    maps[name] = mapper.project(record["evidence_ns"] / 1e9)
                    if name == "candidate":
                        elapsed += time.perf_counter() - start
                timings.append(elapsed * 1000)
                if hashlib.sha256(maps["baseline"].tobytes()).hexdigest() != record["layers_sha256"]:
                    raise ValueError("baseline mirror mismatch at " + str(frames))
                identical = np.array_equal(maps["baseline"], maps["candidate"], equal_nan=True)
                equal += identical
                if require_exact and not identical:
                    raise ValueError("instrumentation changed original layers")
                if compare_library is not None and maps["comparison"].tobytes() != maps["candidate"].tobytes():
                    raise ValueError("candidate diagnostic/eager implementation changed output")
                stamp = record["evidence_ns"]
                result = dict(
                    evidence_ns=stamp,
                    identical=identical,
                    elapsed_s=(stamp - first_ns) / 1e9,
                    candidate_sha256=hashlib.sha256(maps["candidate"].tobytes()).hexdigest(),
                )
                if stamp in display_stamps:
                    name = f"comparison-{stamp}.npz"
                    np.savez_compressed(
                        output / name,
                        baseline=maps["baseline"],
                        candidate=maps["candidate"],
                        origin=record["origin"],
                        local_from_base=record["local_from_base"],
                        local_from_map=record["local_from_map"],
                        resolution=record["resolution"],
                    )
                    examples.append(
                        dict(file=name, sha256=sha(output / name), evidence_ns=stamp, elapsed_s=result["elapsed_s"])
                    )
                if stamp in expected_floor:
                    original = floor_scores(maps["baseline"][:, :, 0], record, points, manifest["request"]["home_xyz"])
                    if original != expected_floor[stamp]["regions"]:
                        raise ValueError("original floor denominator or classification changed")
                    result["floor"] = floor_scores(
                        maps["candidate"][:, :, 0], record, points, manifest["request"]["home_xyz"]
                    )
                    for region, counts in result["floor"].items():
                        total = totals.setdefault(region, dict.fromkeys(counts, 0))
                        for key, value in counts.items():
                            total[key] += value
                if stamp in expected_motion:
                    old = expected_motion[stamp]
                    motions = {}
                    for name, poses in candidate_motions(record["local_from_base"]).items():
                        score = motion_score(maps["candidate"][:, :, 0], record, poses, patches, body, poses[-1][:2, 3])
                        # Fixed geometry/goal from original assessment; no GT-based candidate selection.
                        score.update({k: old["motions"][name][k] for k in ("toward_goal", "progress_m")})
                        motions[name] = score
                        if require_exact and score != old["motions"][name]:
                            raise ValueError("original motion score changed")
                    row = dict(
                        evidence_ns=stamp,
                        elapsed_s=old["elapsed_s"],
                        leg=old["leg"],
                        phase=old["phase"],
                        motions=motions,
                    )
                    rows.append(row)
                    result["motion"] = row
                if diagnostics and stamp in expected_floor:
                    cells, corners = mappers["candidate"].diagnostics()
                    indices, _ = touched_cells(
                        swept_polygons([record["local_from_base"]], patches), record["origin"], 0.1
                    )
                    x, y = indices.T
                    good = (x >= 0) & (y >= 0) & (x < 60) & (y < 60)
                    x, y = x[good], y[good]
                    negative = maps["candidate"][y, x, 1] == 8
                    bits = cells[y[negative], x[negative], 1]
                    detail = dict(
                        low_return=int(((bits & 1) != 0).sum()),
                        pooled_normal=int(((bits & 2) != 0).sum()),
                        native_boundary=int(((bits & 8) != 0).sum()),
                        untagged=int((bits == 0).sum()),
                    )
                    result["dropoff_causes_under_wheels"] = detail
                    unknown = maps["candidate"][y, x, 0] == -1
                    codes, counts = np.unique(maps["candidate"][y[unknown], x[unknown], 1], return_counts=True)
                    result["unknown_wheel_reasons"] = {str(int(k)): int(v) for k, v in zip(codes, counts)}
                    for key, value in detail.items():
                        attribution[key] += value
                    if result["elapsed_s"] in (63, 80, 160, 180, 200):
                        np.savez_compressed(
                            output / f"diagnostic-{stamp}.npz",
                            layers=maps["candidate"],
                            cells=cells,
                            corners=corners,
                            origin=record["origin"],
                            local_from_base=record["local_from_base"],
                        )
                sink.write(json.dumps(result) + "\n")
                frames += 1
                if frames % 300 == 0:
                    print(
                        f"Replayed {frames} projections; exact baseline, {equal} identical candidate maps.", flush=True
                    )
            if raw.read(1) or last_sequence + 1 != len(chunks) or frames != collection["status"]["projected"]:
                raise ValueError("unconsumed native tail or missing projections")
        if len(rows) != len(expected_motion):
            raise ValueError("missing motion exposure")
        for path, digest in hashes.items():
            if sha(path) != digest:
                raise ValueError("changed source during replay: " + path)
        return dict(
            completed=True,
            acceptance=False,
            exact_baseline=True,
            projections=frames,
            native_chunks=len(chunks),
            candidate_identical=equal,
            summary=summarize(rows),
            floor_totals=totals,
            dropoff_causes_under_wheels=attribution if diagnostics else None,
            input_sha256=hashes,
            comparison_library_exact=compare_library is not None,
            examples=examples,
            instrumented=diagnostics,
            timing_is_compute_acceptance=False,
            candidate_core_ms=dict(
                zip(("p50", "p95", "p99", "max"), map(float, np.percentile(timings, [50, 95, 99, 100])))
            ),
            scores_sha256=sha(output / "scores.jsonl"),
        )
    finally:
        for mapper in mappers.values():
            mapper.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("assessment", type=Path)
    parser.add_argument("library", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument("--diagnostics", action="store_true")
    parser.add_argument("--require-exact", action="store_true")
    parser.add_argument(
        "--compare-library", type=Path, help="also require byte-exact equality with this candidate build"
    )
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    try:
        result = run(
            args.assessment,
            args.library,
            args.archive_root,
            output,
            require_exact=args.require_exact,
            diagnostics=args.diagnostics,
            compare_library=args.compare_library,
        )
        check_run(output)
        (output / "report.json").write_text(json.dumps(result, indent=2))
        print(
            json.dumps(
                {k: result[k] for k in ("summary", "dropoff_causes_under_wheels", "candidate_core_ms")}, indent=2
            )
        )
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=str(exc), acceptance=False)))
        raise


if __name__ == "__main__":
    main()
