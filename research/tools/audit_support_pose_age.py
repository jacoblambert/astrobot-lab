#!/usr/bin/env python3
"""Measure old-map versus latest causal robot pose on every recorded projection.

No replayed maps, fitted GT alignment, new commands or changed scores. Invalid
age/generation cases remain in the denominator. Uses mapper receipt ordering.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial.transform import Rotation

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from assess_contact_support import apply_mapper_revision, compact_pose_history, mapper_revisions
from local_terrain.bookkeeping import BookkeepingLedger
from local_terrain.control_context import current_context
from shadow_terrain import PoseTimeline


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            digest.update(block)
    return digest.hexdigest()


def audit(source, output):
    manifest = json.loads((source / "manifest.json").read_text())
    collection = json.loads((source / "collection.json").read_text())
    if not collection["experiment_completed"] or collection["status"]["failure"]:
        raise ValueError("completed healthy source required")
    hashes = {
        str(source / p): sha(source / p)
        for p in ("manifest.json", "collection.json", "events.jsonl", "projections.jsonl")
    }
    for name in ("events.jsonl", "projections.jsonl"):
        if hashes[str(source / name)] != collection["output_sha256"][name]:
            raise ValueError("changed recorded stream")
    paths = [Path(__file__), Path(__file__).with_name("assess_contact_support.py")]
    paths += [
        Path(__file__).parent / "local_terrain" / n for n in ("control_context.py", "pose_lookup.py", "bookkeeping.py")
    ]
    for path in paths:
        hashes[str(path)] = sha(path)
        target = output / "source" / path.relative_to(Path(__file__).parent)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
    records = iter(map(json.loads, (source / "projections.jsonl").read_text().splitlines()))
    first = next(records)
    timeline = PoseTimeline(first["session"], first["epoch"])
    ledger = BookkeepingLedger(manifest["config"]["max_bookkeeping_frames"])
    revisions = mapper_revisions(source / "events.jsonl")
    pending = next(revisions, None)
    rows = []
    import itertools

    with (output / "poses.jsonl").open("x") as sink:
        for record in itertools.chain([first], records):
            if record["evidence_ns"] == collection["status"]["evidence_ns"]:
                continue  # Explicit completion invalidation, not an available map.
            publication = record["publication_ns"]
            while pending is not None and pending["arrival_ns"] <= publication:
                apply_mapper_revision(timeline, ledger, pending)
                pending = next(revisions, None)
            compact_pose_history(timeline, manifest["config"]["max_bookkeeping_frames"])
            row = dict(evidence_ns=record["evidence_ns"], query_ns=publication, available=False)
            try:
                context = current_context(timeline, record, manifest["request"]["base_from_imu"], publication)
                delta = np.linalg.inv(np.asarray(record["local_from_base"])) @ context.local_from_base
                row.update(
                    available=True,
                    pose_ns=context.pose_ns,
                    pose_age_s=context.pose_age_s,
                    terrain_age_s=context.terrain_age_s,
                    stale_pose_displacement_m=float(np.linalg.norm(delta[:3, 3])),
                    stale_pose_rotation_deg=float(np.rad2deg(Rotation.from_matrix(delta[:3, :3]).magnitude())),
                )
            except ValueError as exc:
                row["reason"] = str(exc)
            rows.append(row)
            sink.write(json.dumps(row) + "\n")
    valid = [r for r in rows if r["available"]]
    if len(rows) != collection["status"]["projected"] - 1:
        raise ValueError("missing recorded decision samples")
    stats = {}
    for field in ("pose_age_s", "terrain_age_s", "stale_pose_displacement_m", "stale_pose_rotation_deg"):
        values = [r[field] for r in valid]
        stats[field] = (
            dict(zip(("p50", "p95", "max"), map(float, np.percentile(values, [50, 95, 100])))) if values else None
        )
    if any(sha(p) != h for p, h in hashes.items()):
        raise ValueError("audit input changed")
    return dict(
        completed=True,
        source=str(source),
        samples=len(rows),
        current_context_available=len(valid),
        unavailable_reasons={
            reason: sum(r.get("reason") == reason for r in rows)
            for reason in sorted({r["reason"] for r in rows if "reason" in r})
        },
        statistics=stats,
        input_sha256=hashes,
        output_sha256={"poses.jsonl": sha(output / "poses.jsonl")},
        acceptance=False,
        navigation_authority=False,
        note="Receipt-causal pose age only; not GT accuracy, stopping calibration or navigation qualification.",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    args = parser.parse_args()
    check_run(args.source)
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    try:
        report = audit(args.source, output)
        check_run(output)
        with (output / "report.json").open("x") as stream:
            json.dump(report, stream, indent=2)
        print(json.dumps({k: v for k, v in report.items() if k != "input_sha256"}, indent=2), flush=True)
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
