#!/usr/bin/env python3
"""Completed-live diagnostic at independent PhysX floor samples, NOT safety truth.

Every pre-motion clearance survey point is queried in every exact-replayed map.
Outside-grid/unknown samples stay in the denominator; completion is invalidated
as in live publication. Pre-projection startup publications are not scored here.
Sampled clear floor is not continuous cell/wheel support; this does not measure obstacle recall or
false-safe rate. GT geometry is evaluator-only, never fed into reconstruction.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from local_terrain.evaluation import availability, query_grid
from natural_motion import survey_points, validate_ground_tile
from replay_live_local_terrain import replay
from replay_local_terrain import sha
from revision_evidence import validated_transform


def sample_floor(layers, record, world_points, home):
    """World->map is the declared home translation, never a fitted alignment."""
    points = np.asarray(world_points, dtype=float)
    home = np.asarray(home, dtype=float)
    if home.shape != (3,) or not np.isfinite(home).all():
        raise ValueError("finite declared home required")
    costs, outside = query_grid(
        layers,
        record["origin"],
        validated_transform(record["local_from_map"]),
        points - home,
        resolution=record["resolution"],
    )
    result = availability(costs, outside)
    result.pop("terrain_truth_evaluated")
    return dict(result, evidence_ns=record["evidence_ns"], publication_ns=record["publication_ns"])


def evaluate(source, capture, archive, output):
    paths = [
        source / "manifest.json",
        capture / "shadow_link.json",
        capture / "natural_holdout.json",
        capture / "clearance.json",
        capture / "clearance_tile.npz",
        Path(__file__),
        Path(__file__).with_name("replay_live_local_terrain.py"),
        Path(__file__).with_name("natural_motion.py"),
    ]
    hashes = {str(p): sha(p) for p in paths}
    manifest, link, complete, clearance = [json.loads(p.read_text()) for p in paths[:4]]
    request = link["request"]
    if (
        any((capture / name).exists() for name in ("failure.json", "orchestration_failure.json"))
        or not complete.get("complete")
        or complete["request"] != request
        or {k: v for k, v in manifest["request"].items() if k != "stamp"} != request
        or request["capture"] != capture.name
        or not clearance.get("passed")
        or clearance["pattern"] != request["pattern"]
        or clearance["tile_sha256"] != hashes[str(paths[4])]
    ):
        raise ValueError("incomplete, mismatched or changed independent floor source")
    origin = validated_transform(clearance["origin"])
    expected_xy, shape = survey_points(origin, request["pattern"])
    with np.load(paths[4], allow_pickle=False) as tile:
        xy, z, normals = tile["xy"], tile["z"], tile["normal"]
    if not np.array_equal(xy, expected_xy):
        raise ValueError("floor survey denominator/order changed")
    validate_ground_tile(xy, z, shape, normals)
    points = np.column_stack((xy, z))
    total = dict(queries=0, free=0, occupied=0, unknown=0, outside=0)
    frames, first_ns, last_ns, after_lifetime = 0, None, None, 0
    first_score = last_score = last_valid_score = None
    with (output / "floor_samples.jsonl").open("x") as stream:

        def observe(layers, record, *, published_valid):
            nonlocal frames, first_ns, last_ns, after_lifetime, first_score, last_score, last_valid_score
            if not published_valid:
                layers[:, :, 0] = -1  # Live runtime invalidates completion, not a stale usable map.
            score = sample_floor(layers, record, points, request["home_xyz"])
            score["published_valid"] = published_valid
            if published_valid:
                last_valid_score = score
            if first_ns is None:
                first_ns, first_score = record["evidence_ns"], score
            last_ns, last_score = record["evidence_ns"], score
            after_lifetime += last_ns - first_ns > manifest["config"]["evidence_lifetime_s"] * 10**9
            for key in total:
                total[key] += score[key]
            stream.write(json.dumps(score, allow_nan=False) + "\n")
            frames += 1

        replay_result = replay(source, archive, projection_observer=observe)
    if not frames or total["queries"] != frames * len(points):
        raise ValueError("missing floor-sample exposure")
    if any(sha(path) != digest for path, digest in hashes.items()):
        raise ValueError("floor diagnostic inputs changed")
    return dict(
        diagnostic_completed=True,
        acceptance=False,
        navigation_authority=False,
        continuous_terrain_truth_evaluated=False,
        obstacle_recall_evaluated=False,
        false_safe_evaluated=False,
        pre_projection_startup_scored=False,
        source=str(source),
        capture=str(capture),
        reference="active_PhysX_pre_motion_clearance_samples",
        unique_floor_samples=len(points),
        projections=frames,
        duration_s=(last_ns - first_ns) / 1e9,
        projections_after_evidence_lifetime=int(after_lifetime),
        overall=total,
        sampled_floor_supported_fraction=total["free"] / total["queries"],
        first_projection=first_score,
        last_projection=last_score,
        last_valid_projection=last_valid_score,
        exact_replay=replay_result,
        input_sha256=hashes,
        output_sha256={"floor_samples.jsonl": sha(output / "floor_samples.jsonl")},
        limitation=__doc__,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    args = parser.parse_args()
    archive = args.archive_root.resolve(strict=True)
    source, capture = args.source.resolve(strict=True), args.capture.resolve(strict=True)
    if source.parent != archive or capture.parent != archive:
        raise ValueError("inputs must be directly on the external archive")
    for folder in (source, capture):
        check_run(folder)
    output = allocate_run(archive)
    print(f"Run: {output}", flush=True)
    try:
        result = evaluate(source, capture, archive, output)
        check_run(output)
        with (output / "report.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(
            json.dumps(
                {k: v for k, v in result.items() if k not in ("exact_replay", "input_sha256", "output_sha256")},
                indent=2,
            )
        )
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
