#!/usr/bin/env python3
"""Observed natural-surface placement, not obstacle recall or terrain acceptance.

Collision identities and eligibility are GT-only evaluator data. No closest-point
fitting, candidate-dependent selection or estimator feedback is performed.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from shadow_pose_replay import load_pose_sensitivity, sha
from revision_evidence import validated_transform
from natural_holdout import native_hit_records


def point_errors(points, hits, map_from_lidar, home):
    transform = validated_transform(map_from_lidar)
    points = np.asarray(points)
    rows = []
    seen = set()
    for record in hits:
        index, hit = record["point_index"], record["first_hit"]
        if type(index) is not int or not 0 <= index < len(points) or index in seen:
            raise ValueError("invalid or duplicate first-hit point index")
        seen.add(index)
        if hit is None or not 0 <= record["residual_m"] <= .001:
            raise ValueError("unqualified native first hit")
        normal = np.asarray(hit["normal"], dtype=float)
        position = np.asarray(hit["position"], dtype=float)
        if (normal.shape != (3,) or position.shape != (3,) or not np.isfinite([normal, position]).all()
                or abs(np.linalg.norm(normal)-1.) > .001 or not np.isfinite(points[index]).all()):
            raise ValueError("invalid first-hit geometry")
        delta = transform[:3, :3] @ points[index] + transform[:3, 3] + np.asarray(home) - position
        rows.append(dict(collision=hit["collision"], displacement_m=float(np.linalg.norm(delta)),
                         normal_error_m=abs(float(delta @ normal))))
    return rows


def summarize(rows, objects):
    def metrics(values):
        return dict(samples=len(values), **{k: dict(p95=float(np.percentile([r[k] for r in values], 95)),
            maximum=max(r[k] for r in values)) for k in ("displacement_m", "normal_error_m")})
    if not rows:
        raise ValueError("no qualified observed surfaces")
    rocks = []
    for obj in objects:
        path = obj["path"]
        selected = [r for r in rows if r["collision"] == path or r["collision"].startswith(path+"/")]
        extent = np.asarray(obj["maximum"]) - np.asarray(obj["minimum"])
        # Object size is NOT height above local terrain. Keep this limitation explicit.
        large = bool((extent >= .3).all())
        rocks.append(dict(path=path, extent_xyz_m=extent.tolist(), large_by_world_bounds=large,
                          eligible_observed_large=large and len(selected) >= 20,
                          metrics=metrics(selected) if selected else None))
    ground = [r for r in rows if r["collision"].startswith("/Lunaryard/Terrain/")]
    return dict(all_observed=metrics(rows), observed_ground=metrics(ground) if ground else None,
                objects=rocks, eligible_observed_large=sum(r["eligible_observed_large"] for r in rocks))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("pair", type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        for path in (args.capture, args.pair): check_run(path)
        if (args.capture / "failure.json").exists(): raise ValueError("failed capture is ineligible")
        native_path = args.capture / "native_timing_audit.json"
        scene_path = args.capture / "natural_scene.json"
        native = json.loads(native_path.read_text())["frames"]
        scene = json.loads(scene_path.read_text())
        report = json.loads((args.pair / "shadow_pair_audit.json").read_text())
        # load_pose_sensitivity verifies the qualified pair and all its input hashes.
        results = {}
        for mode in ("first_published", "final_revised"):
            poses, _ = load_pose_sensitivity(args.pair, args.capture, native, mode)
            rows = []
            for frame, transform in zip(native, poses):
                path = args.capture / "native_chunks" / frame["file"]
                if sha(path) != frame["sha256"]: raise ValueError("native cloud changed")
                hits = native_hit_records(frame, args.capture / "native_chunks")
                with np.load(path) as data:
                    rows.extend(point_errors(data["points"], hits, transform, report["fixed_home"]))
            results[mode] = summarize(rows, scene["objects"])
        result = dict(completed=True, acceptance=False, results=results,
            eligibility="All recorded audited first hits; large = all world AABB dimensions >=0.3 m and >=20 hits",
            limits="Observed surface placement only; no terrain labels, obstacle detection or unobserved recall.",
            stationary_only=report.get("stationary_only", True),
            input_sha256={str(p): sha(p) for p in (native_path, scene_path, args.pair/"shadow_pair_audit.json")})
        with (output / "natural_surface_report.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps({k: dict(all_observed=v["all_observed"], eligible_observed_large=v["eligible_observed_large"])
                          for k, v in results.items()}, indent=2))
    except Exception as exc:
        with (output / "failure.json").open("x") as stream: json.dump(dict(error=str(exc)), stream)
        raise


if __name__ == "__main__":
    main()
