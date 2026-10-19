#!/usr/bin/env python3
"""Offline batch-difference diagnosis. Does not admit revisions to live terrain."""

import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from replay_local_terrain import sha
from revision_evidence import validated_transform


def summarize(events):
    prior, batches, identity = {}, [], None
    previous = 0
    for event in events:
        if event["kind"] != "revision":
            continue
        record = event["payload"]
        current_identity = (record["session"], record["epoch"])
        if identity is None:
            identity = current_identity
        if identity != current_identity or record["parent"] != previous or record["revision"] != previous + 1:
            raise ValueError("revision identity/chain gap")
        previous = record["revision"]
        rows = []
        for frame in record["frames"]:
            current = validated_transform(frame["map_from_imu"])
            extrinsic = validated_transform(frame["imu_from_lidar"])
            old = prior.get(frame["frame_id"])
            rows.append(
                dict(
                    known=old is not None,
                    matrix_delta=float(np.max(np.abs(current - old["map_from_imu"]))) if old else None,
                    xyz_delta_m=(
                        float(np.linalg.norm(current[:3, 3] - np.asarray(old["map_from_imu"])[:3, 3])) if old else None
                    ),
                    extrinsic_delta=float(np.max(np.abs(extrinsic - old["imu_from_lidar"]))) if old else None,
                    changed_stamp=old is not None and old["stamp_ns"] != frame["stamp_ns"],
                    deskew_recomputed=frame["deskew_recomputed"],
                )
            )
        if record["kind"] != 1 or record["snapshot"]:
            batches.append(
                dict(
                    kind=record["kind"],
                    revision=record["revision"],
                    snapshot=record["snapshot"],
                    frames=len(rows),
                    unknown_frames=sum(not r["known"] for r in rows),
                    changed_stamps=sum(r["changed_stamp"] for r in rows),
                    deskew_recomputed=sum(r["deskew_recomputed"] for r in rows),
                    **{
                        f"max_{key}": max((r[key] for r in rows if r["known"]), default=None)
                        for key in ("matrix_delta", "xyz_delta_m", "extrinsic_delta")
                    },
                )
            )
        for frame in record["frames"]:
            prior[frame["frame_id"]] = frame
        if len(prior) > 40000:
            raise ValueError("offline diagnostic frame budget exceeded")
    return dict(
        revisions=previous,
        unique_frames=len(prior),
        non_odom_batches=batches,
        acceptance=False,
        note="Offline comparison with preceding full pose state only; not a live bounded no-op certificate or revision-handling fix.",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mapper", type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    check_run(args.mapper)
    file = args.mapper / "events.jsonl"
    sealed = args.mapper / "collection.json"
    hashes = {str(p): sha(p) for p in (file, sealed, Path(__file__))}
    if hashes[str(file)] != json.loads(sealed.read_text())["output_sha256"][file.name]:
        raise ValueError("changed sealed event log")
    with file.open() as stream:
        report = summarize(map(json.loads, stream))
    if any(sha(p) != digest for p, digest in hashes.items()):
        raise ValueError("diagnostic input changed")
    report["input_sha256"] = hashes
    output = allocate_run(args.archive_root)
    with (output / "report.json").open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(f"Run: {output}", flush=True)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
