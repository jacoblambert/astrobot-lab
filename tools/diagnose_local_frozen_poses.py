#!/usr/bin/env python3
"""Audit frozen placement versus latest assimilated native base, with evaluator GT.

Reports all retained-window pose opportunities, NOT actual surviving C++ cell
dependencies (cells may be evicted and buckets may expire earlier). No point
cloud labels, fitted alignment, future estimate revision, extrapolation, or
terrain/uncertainty qualification. Age-bin errors are descriptive, not calibrated
uncertainty bounds or independent statistical trials.
The reference endpoint is the latest native acquisition, not the nominal 100 ms
map query stamp. Its offset is reported; no rounded query stamp is retimestamped
to a nearby GT sample. Consumer-time pose error remains a separate gate.
"""

import argparse
import json
from pathlib import Path
import shutil

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from local_terrain.frozen_pose_audit import exact_native_index, registration_errors
from replay_local_terrain import sha
from revision_evidence import validated_transform


def evaluate(local_run, accuracy_run, capture, *, placement="first_local"):
    if placement not in ("first_local", "current_gauge_of_first_map"):
        raise ValueError("unknown registration attribution mode")
    paths = [
        local_run / "report.json",
        local_run / "projections.jsonl",
        accuracy_run / "accuracy_report.json",
        accuracy_run / "accuracy_arrays.npz",
        capture / "shadow_link.json",
    ]
    hashes = {str(p): sha(p) for p in paths}
    local, accuracy, link = [json.loads(paths[i].read_text()) for i in (0, 2, 4)]
    if (
        not local["diagnostic_completed"]
        or not accuracy["diagnostic_completed"]
        or local["output_sha256"] != hashes[str(paths[1])]
        or accuracy["output_sha256"]["accuracy_arrays.npz"] != hashes[str(paths[3])]
    ):
        raise ValueError("incomplete or modified registration inputs")
    if local["observation_policy"] != "frozen_observation" or local["map_policy"] not in (
        "rolling_incremental",
        "fixed_incremental",
    ):
        raise ValueError("requires declared persistent frozen-placement candidate")
    if not any(capture.name in p for p in accuracy["input_sha256"]):
        raise ValueError("accuracy/capture identity mismatch")
    if local["input_sha256"].get(str(paths[4])) != hashes[str(paths[4])]:
        raise ValueError("candidate/capture identity mismatch")
    mount = validated_transform(link["request"]["base_from_lidar"])
    with np.load(paths[3], allow_pickle=False) as data:
        times = np.rint(data["times"] * 1e9).astype(np.int64)
        truth = data["gt_base"].copy()
    if len(times) != len(truth) or not np.all(np.diff(times) > 0):
        raise ValueError("invalid exact GT timeline")
    lifetime = local.get("evidence_lifetime_s", local["window_s"])
    bins = [(0, 2), (2, 5), (5, 15), (15, 30), (30, 60)]
    samples = [[] for _ in bins]
    retained, last_sequence, frames, missing, pairs = {}, -1, 0, 0, 0
    query_offsets = []
    with paths[1].open() as stream:
        for line in stream:
            row = json.loads(line)
            target, publication = row["evidence_ns"], row["publication_ns"]
            new = 0
            for observation in row["associations"]:
                sequence, stamp = observation["sequence"], observation["acquisition_ns"]
                if not stamp <= observation["available_ns"] <= observation["output_ns"] <= publication:
                    raise ValueError("noncausal frozen placement")
                pose = validated_transform(observation["local_from_lidar"])
                if sequence <= last_sequence:
                    if sequence in retained and (
                        retained[sequence][0] != stamp
                        or not np.array_equal(retained[sequence][1], pose)
                        or placement != "first_local"
                        and not np.array_equal(
                            retained[sequence][2], validated_transform(observation["map_from_lidar"])
                        )
                    ):
                        raise ValueError("frozen observation placement changed")
                    continue
                if sequence != last_sequence + 1 or stamp > target:
                    raise ValueError("missing or future integrated observation")
                map_pose = validated_transform(observation["map_from_lidar"]) if placement != "first_local" else pose
                retained[sequence] = (stamp, pose.copy(), map_pose.copy())
                last_sequence = sequence
                new += 1
            if new != row["integrated_inputs"]:
                raise ValueError("integrated input accounting mismatch")
            end = row["associations"][-1]
            if end["output_ns"] != publication:
                raise ValueError("native endpoint was not placed at this publication")
            offset = target - end["acquisition_ns"]
            if not 0 <= offset <= 34_000_000:
                raise ValueError("latest native endpoint exceeds declared 30 Hz cadence bound")
            query_offsets.append(offset)
            current = validated_transform(end["local_from_lidar"]) @ np.linalg.inv(mount)
            retained = {s: value for s, value in retained.items() if value[0] >= target - lifetime * 10**9}
            if len(retained) > 4096:
                raise ValueError("registration audit bounded window exceeded")
            frames += 1
            try:
                now_index = exact_native_index(times, end["acquisition_ns"])
                indices = [exact_native_index(times, value[0]) for value in retained.values()]
            except ValueError:
                missing += 1
                continue
            if not indices:
                raise ValueError("empty pose opportunity window")
            ages = np.array([(end["acquisition_ns"] - value[0]) / 1e9 for value in retained.values()])
            old_poses = np.array([value[1] for value in retained.values()])
            if placement == "current_gauge_of_first_map":
                # Attribution only: same FIRST map poses, transported through
                # today's gauge. No later estimate revision or GT correction.
                # Assumes a stable underlying map gauge; not the stored map.
                gauge = validated_transform(row["local_from_map"])
                old_poses = gauge @ np.array([value[2] for value in retained.values()])
            errors = registration_errors(current, old_poses, truth[now_index], truth[indices] @ mount)
            pairs += len(errors)
            for i, (low, high) in enumerate(bins):
                mask = (ages >= low) & ((ages < high) if high < 60 else (ages <= high))
                if mask.any():
                    samples[i].append(errors[mask])
    if frames != local["projections"]:
        raise ValueError("missing projection exposure")
    results = {}
    names = ["origin_displacement_m", "rotation_rad", "point_bound_at_2m", "point_bound_at_3m"]
    for (low, high), rows in zip(bins, samples):
        values = np.concatenate(rows) if rows else np.empty((0, 4))
        results[f"{low}_{high}s"] = dict(
            pairs=len(values),
            metrics=(
                {
                    name: dict(
                        zip(
                            ("median", "p95", "p99", "maximum"),
                            map(float, np.percentile(values[:, i], [50, 95, 99, 100])),
                        )
                    )
                    for i, name in enumerate(names)
                }
                if len(values)
                else {}
            ),
        )
    if any(sha(path) != digest for path, digest in hashes.items()):
        raise ValueError("registration audit inputs changed")
    return dict(
        diagnostic_completed=True,
        acceptance=False,
        terrain_truth_evaluated=False,
        calibrated_uncertainty=False,
        projections=frames,
        missing_gt_publications=missing,
        pose_pairs=pairs,
        reference_endpoint="latest_assimilated_native_base_at_this_publication",
        consumer_query_error_evaluated=False,
        maximum_query_minus_native_s=max(query_offsets) / 1e9,
        source_failed=local["source_failed"],
        bounded_prefix=local["bounded_prefix"],
        evidence_lifetime_s=lifetime,
        placement=placement,
        attribution_only=placement != "first_local",
        results=results,
        input_sha256=hashes,
        scope=__doc__,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("local_run", type=Path)
    parser.add_argument("accuracy_run", type=Path)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument("--placement", choices=("first_local", "current_gauge_of_first_map"), default="first_local")
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        sources = [
            Path(__file__),
            Path(__file__).with_name("local_terrain") / "frozen_pose_audit.py",
            Path(__file__).with_name("revision_evidence.py"),
            Path(__file__).with_name("archive_guard.py"),
            Path(__file__).with_name("replay_local_terrain.py"),
        ]
        hashes = {str(p.resolve()): sha(p) for p in sources}
        for path in sources:
            target = output / "source" / path.relative_to(Path(__file__).parent)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, target)
        result = evaluate(
            args.local_run.resolve(), args.accuracy_run.resolve(), args.capture.resolve(), placement=args.placement
        )
        if any(sha(path) != digest for path, digest in hashes.items()):
            raise ValueError("registration audit source changed")
        result["source_sha256"] = hashes
        check_run(output)
        with (output / "report.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps({k: v for k, v in result.items() if k not in ("input_sha256", "source_sha256")}, indent=2))
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
