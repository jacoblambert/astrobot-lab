#!/usr/bin/env python3
"""Point-for-point displacement on GT-defined calibration surfaces, not obstacle recall.

The current fixture contains small boxes and one large plateau, NOT natural
rocks. This diagnostic must not be presented as natural-rock acceptance.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from raw_return_filter import valid_range_mask
from shadow_pose_replay import load_pose_sensitivity, sha


def surface_mask(points, center, width, height, tolerance=.001):
    """Observed top/side returns on a known solid box; exclude floor contact."""
    p = np.asarray(points)
    xy = np.abs(p[:, :2] - center)
    inside = (xy <= width/2+tolerance).all(axis=1) & (p[:, 2] > tolerance) & (p[:, 2] <= height+tolerance)
    shell = (np.abs(p[:, 2]-height) <= tolerance) | (np.abs(xy-width/2) <= tolerance).any(axis=1)
    return inside & shell


def displacement_stats(vectors):
    v = np.concatenate(vectors) if vectors else np.empty((0, 3))
    if not len(v): return dict(returns=0, eligible=False)
    norm = np.linalg.norm(v, axis=1)
    return dict(returns=len(v), eligible=len(v)>=20, displacement_median_m=float(np.median(norm)),
        displacement_p95_m=float(np.percentile(norm,95)), displacement_max_m=float(norm.max()),
        signed_mean_xyz_m=v.mean(axis=0).tolist(), p95_pass=bool(np.percentile(norm,95)<=.1))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("pair", type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    out = allocate_run(args.archive_root)
    print(f"Run: {out}", flush=True)
    try:
        for p in (args.capture,args.pair): check_run(p)
        read = lambda name: json.loads((args.capture/name).read_text())
        frames = read("native_timing_audit.json")["frames"]
        spec, manifest = read("calibration_spec.json"), read("manifest.json")
        attrs = manifest["sensor"]["sensor_attributes"]
        objects = list(spec["boxes"])+[dict(spec["plateau"], name="plateau")]
        if spec["center"] != [-20.,-20.]: raise ValueError("unsupported calibration origin")
        poses = {m: load_pose_sensitivity(args.pair, args.capture, frames, m)[0]
                 for m in ("first_published","final_revised")}
        values = {m: {b["name"]: [] for b in objects} for m in poses}
        ground = {m: [] for m in poses}
        counts = []
        for i,f in enumerate(frames):
            path = args.capture/"native_chunks"/f["file"]
            if sha(path) != f["sha256"]: raise ValueError("native chunk changed")
            with np.load(path) as a:
                points = a["points"].reshape(-1,3)
                if abs(float(a["observed_stamp"])-f["observed_stamp"])>1e-9: raise ValueError("native time mismatch")
                if not np.array_equal(a["world_from_lidar"], f["world_from_lidar"]): raise ValueError("native pose mismatch")
            keep, count = valid_range_mask(points, float(attrs["minRange"]), float(attrs["maxRange"]))
            points = points[keep]; counts.append(count)
            gt = np.asarray(f["world_from_lidar"]).copy(); gt[:3,3] -= [-20.,-20.,0.]
            truth = points @ gt[:3,:3].T + gt[:3,3]
            masks = {b["name"]: surface_mask(truth,b["center"],b["width"],b["height"]) for b in objects}
            floor = (np.abs(truth[:,2])<=.001) & (np.abs(truth[:,:2])<spec["floor_extent"]).all(axis=1)
            for mode, sequence in poses.items():
                p=sequence[i]; estimated=points@p[:3,:3].T+p[:3,3]
                difference=estimated-truth
                for name, mask in masks.items():
                    if mask.any(): values[mode][name].append(difference[mask])
                if floor.any(): ground[mode].append(difference[floor,2])
        result = dict(completed=True, acceptance=False, natural_rock_validation="NOT_EVALUATED",
            definition="same GT-selected physical returns on known top/side surfaces; no ICP fit or candidate-selected visibility",
            object_selection="height >=0.30 m and width >=0.30 m and >=20 reference returns; small objects separately reported",
            objects={mode: {b["name"]: dict(displacement_stats(values[mode][b["name"]]),
                large_geometry=b["height"]>=.3 and b["width"]>=.3) for b in objects} for mode in poses},
            ground={m: dict(returns=sum(len(v) for v in ground[m]),
                absolute_z_p95_m=float(np.percentile(abs(np.concatenate(ground[m])),95))) for m in poses},
            chunks=len(frames), range_counts=counts,
            input_sha256={str(args.pair/"shadow_pair_audit.json"):sha(args.pair/"shadow_pair_audit.json"),
                **{str(args.capture/n):sha(args.capture/n) for n in ("calibration_spec.json","manifest.json","native_timing_audit.json")}})
        with (out/"surface_displacement.json").open("x") as stream: json.dump(result,stream,indent=2,allow_nan=False)
        print(json.dumps({k:v for k,v in result.items() if k not in ("range_counts","input_sha256")},indent=2))
    except Exception as exc:
        with (out/"failure.json").open("x") as stream: json.dump(dict(error=str(exc)),stream)
        raise


if __name__ == "__main__": main()
