#!/usr/bin/env python3
"""Score dimensioned test rocks in exact-replayed live maps; GT is evaluator-only."""

import argparse
import json
from pathlib import Path

import numpy as np
from shapely import MultiPoint, box

from archive_guard import allocate_run, check_run
from demo_rock_profile import PROFILE
from replay_live_local_terrain import replay
from replay_local_terrain import sha


def sampled_hits(capture, objects):
    """Independent audit rays only: zero sampled hits is not proof of zero returns."""
    source = capture / "native_timing_audit.json"
    audit = json.loads(source.read_text())
    counts = {obj["path"]: 0 for obj in objects}
    for frame in audit["frames"]:
        surface = frame["current_pose"]
        name = Path(surface["hit_records_file"])
        if name.name != str(name):
            raise ValueError("audit hit path must be a filename")
        path = capture / "native_chunks" / name
        if sha(path) != surface["hit_records_sha256"]:
            raise ValueError("changed native hit witnesses")
        for record in json.loads(path.read_text()):
            collision = (record.get("first_hit") or {}).get("collision")
            if collision in counts:
                counts[collision] += 1
    return counts, sha(source)


def object_score(layers, record, obj, home):
    points = np.asarray(obj["points"]) - home
    transform = np.asarray(record["local_from_map"])
    local = points @ transform[:3, :3].T + transform[:3, 3]
    shape = MultiPoint(local[:, :2]).convex_hull
    origin, resolution = np.asarray(record["origin"]), record["resolution"]
    low = np.floor((np.asarray(shape.bounds[:2]) - origin) / resolution).astype(int)
    high = np.floor((np.asarray(shape.bounds[2:]) - origin) / resolution).astype(int)
    counts = dict(occupied=0, free=0, unknown=0, outside=0)
    for y in range(low[1], high[1] + 1):
        for x in range(low[0], high[0] + 1):
            xy = origin + [x * resolution, y * resolution]
            if shape.intersection(box(*xy, *(xy + resolution))).area <= 1e-10:
                continue
            if not (0 <= y < layers.shape[0] and 0 <= x < layers.shape[1]):
                counts["outside"] += 1
            else:
                value = layers[y, x, 0]
                counts["occupied" if value == 100 else "free" if value in (0, 20) else "unknown"] += 1
    apex = np.floor((local[0, :2] - origin) / resolution).astype(int)
    apex_cost = -1
    if 0 <= apex[1] < layers.shape[0] and 0 <= apex[0] < layers.shape[1]:
        apex_cost = int(layers[apex[1], apex[0], 0])
    return dict(path=obj["path"], detected=counts["occupied"] > 0, footprint_cell_counts=counts, apex_cost=apex_cost)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mapper", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads((args.mapper / "manifest.json").read_text())
    request = manifest["request"]
    capture = args.archive_root / request["capture"]
    declaration = request["demo_rocks"]
    geometry = capture / "demo_rocks.json"
    if declaration["profile"] != PROFILE or sha(geometry) != declaration["geometry_sha256"]:
        raise ValueError("missing or changed declared demo geometry")
    data = json.loads(geometry.read_text())
    if not data["verified"] or len(data["objects"]) != 8:
        raise ValueError("all eight independently verified test rocks required")
    output = allocate_run(args.archive_root, limit=128 * 1024**2)
    print(f"Run: {output}", flush=True)
    paths = [geometry, Path(__file__), args.mapper / "manifest.json"]
    hashes = {str(p): sha(p) for p in paths}
    hit_counts, native_hash = sampled_hits(capture, data["objects"])
    hashes[str(capture / "native_timing_audit.json")] = native_hash
    paths.append(capture / "native_timing_audit.json")
    rows = []
    with (output / "objects.jsonl").open("x") as stream:

        def observe(layers, record, *, published_valid):
            if not published_valid:
                return
            scores = [object_score(layers, record, obj, np.asarray(request["home_xyz"])) for obj in data["objects"]]
            row = dict(
                evidence_ns=record["evidence_ns"],
                publication_ns=record["publication_ns"],
                objects=scores,
                detected=sum(s["detected"] for s in scores),
            )
            stream.write(json.dumps(row) + "\n")
            rows.append(row)

        exact = replay(args.mapper, args.archive_root, projection_observer=observe)
    if not rows or any(sha(p) != hashes[str(p)] for p in paths):
        raise ValueError("empty or changed assessment")
    summary = []
    for i, obj in enumerate(data["objects"]):
        first = next((r["evidence_ns"] for r in rows if r["objects"][i]["detected"]), None)
        summary.append(
            dict(
                path=obj["path"],
                sampled_native_hits=hit_counts[obj["path"]],
                first_detected_ns=first,
                maps_detected=sum(r["objects"][i]["detected"] for r in rows),
                final=rows[-1]["objects"][i],
            )
        )
    report = dict(
        completed=True,
        acceptance=False,
        exact_replay=exact,
        maps=len(rows),
        objects=8,
        ever_detected=sum(r["first_detected_ns"] is not None for r in summary),
        final_detected=rows[-1]["detected"],
        per_object=summary,
        profile=declaration,
        input_sha256=hashes,
        output_sha256={"objects.jsonl": sha(output / "objects.jsonl")},
        limits=[
            "actual footprint overlap; no inflated matching halo",
            "not all natural-scene obstacles",
            "raw partial-footprint free cells are not continuous collision-free truth",
            "sampled geometry witnesses; independent native first-hit audit remains separate",
            "zero sampled native hits does not prove no sensor returns",
        ],
    )
    (output / "report.json").write_text(json.dumps(report, indent=2))
    (output / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    check_run(output)
    print(
        json.dumps(
            {k: report[k] for k in ("maps", "objects", "ever_detected", "final_detected", "per_object")}, indent=2
        )
    )


if __name__ == "__main__":
    main()
