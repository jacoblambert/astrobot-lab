#!/usr/bin/env python3
"""Trace final pose-sensitive map conflicts to exact native point identities.

This is diagnostic selection, NOT terrain GT, recall, a new acceptance mask, or
mapper input. Every input chunk is verified; original indices and transforms are
retained so apparent ground/obstacle disagreement cannot be fitted away.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from raw_return_filter import valid_range_mask
from revision_evidence import validated_transform
from shadow_pose_replay import load_pose_sensitivity, sha


def conflict_cells(reference, candidate):
    if reference.shape != candidate.shape or reference.ndim != 3 or reference.shape[2] != 6:
        raise ValueError("incompatible layered maps")
    if not all(np.isin(a[:, :, 0], [-1, 0, 20, 100]).all() for a in (reference, candidate)):
        raise ValueError("invalid map costs")
    return np.argwhere((reference[:, :, 0] == 100) & np.isin(candidate[:, :, 0], [0, 20]))


def selected_returns(points, truth, estimate, cells_yx, origin, resolution):
    """Correspondence by raw index, never nearest surface or fitted alignment."""
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("invalid selected points")
    transforms = [validated_transform(t) for t in (truth, estimate)]
    maps = [points @ t[:3, :3].T + t[:3, 3] for t in transforms]
    indices = [np.floor((p[:, :2]-origin)/resolution).astype(int) for p in maps]
    rows = []
    for y, x in cells_yx:
        masks = [np.all(index == [x, y], axis=1) for index in indices]
        for i in np.flatnonzero(masks[0] | masks[1]):
            rows.append(dict(cell_yx=[int(y), int(x)], point_index=int(i),
                gt_in_cell=bool(masks[0][i]), slam_in_cell=bool(masks[1][i]),
                gt_xyz=maps[0][i].tolist(), slam_xyz=maps[1][i].tolist(),
                slam_cell_yx=indices[1][i][::-1].tolist(),
                delta_xyz=(maps[1][i]-maps[0][i]).tolist()))
    return rows


def run(capture, pair, replay, output):
    for folder in (capture, pair, replay):
        check_run(folder)
        if (folder / "failure.json").exists():
            raise ValueError("failed input is ineligible")
    report_path = replay / "natural_terrain_sensitivity.json"
    report = json.loads(report_path.read_text())
    if not report.get("completed") or not report.get("frames"):
        raise ValueError("incomplete terrain replay")
    for path, expected in report["input_sha256"].items():
        if sha(Path(path)) != expected:
            raise ValueError("terrain input changed")
    maps = {}
    for mode in ("gt_isolated", "first_published", "final_revised"):
        path = replay / f"{mode}.npz"
        if sha(path) != report["output_sha256"][path.name]:
            raise ValueError("terrain output changed")
        with np.load(path) as data:
            maps[mode] = data["layers"]
    native = json.loads((capture / "native_timing_audit.json").read_text())["frames"]
    final_stamp = float(native[-1]["observed_stamp"])
    if (report["native_chunks_integrated"] != len(native) or
            report["frames"][-1]["stamp"] != final_stamp):
        raise ValueError("terrain native timeline differs")
    pair_report = json.loads((pair / "shadow_pair_audit.json").read_text())
    poses = {mode: load_pose_sensitivity(pair, capture, native, mode)[0]
             for mode in ("first_published", "final_revised")}
    # Pair input identity was checked above; ensure it is the scored pair too.
    if any(report["pose_sensitivity"][mode]["audit_sha256"] != sha(pair / "shadow_pair_audit.json")
           for mode in poses):
        raise ValueError("different scored pose pair")
    cells = {mode: conflict_cells(maps["gt_isolated"], maps[mode]) for mode in poses}
    grid = report["geometry"]
    sensor = json.loads((capture / "manifest.json").read_text())["sensor"]["sensor_attributes"]
    minimum, maximum = float(sensor["minRange"]), float(sensor["maxRange"])
    counts = {mode: 0 for mode in poses}
    with (output / "point_correspondences.jsonl").open("x") as stream:
        for index, frame in enumerate(native):
            path = capture / "native_chunks" / frame["file"]
            if (report["input_sha256"].get(str(path)) != frame["sha256"] or sha(path) != frame["sha256"]):
                raise ValueError("native cloud identity changed")
            with np.load(path) as data:
                points = data["points"]
                if (float(data["observed_stamp"]) != frame["observed_stamp"] or
                        not np.array_equal(data["world_from_lidar"], frame["world_from_lidar"])):
                    raise ValueError("native stamp/pose changed")
            keep, _ = valid_range_mask(points, minimum, maximum)
            original = np.flatnonzero(keep)
            truth = np.array(frame["world_from_lidar"])
            truth[:3, 3] -= pair_report["fixed_home"]
            for mode in poses:
                rows = selected_returns(points[keep], truth, poses[mode][index], cells[mode],
                                        grid["origin"], grid["resolution"])
                if not rows:
                    continue
                for row in rows:
                    row["point_index"] = int(original[row["point_index"]])
                counts[mode] += len(rows)
                json.dump(dict(mode=mode, chunk=index, file=frame["file"], sha256=frame["sha256"],
                    stamp=frame["observed_stamp"], map_from_lidar_gt=truth.tolist(),
                    map_from_lidar_slam=poses[mode][index].tolist(), rows=rows), stream, allow_nan=False)
                stream.write("\n")
    result = dict(completed=True, acceptance=False, terrain_truth=False, gt_input_to_mapper=False,
        selection="All final GT-pose-occupied to SLAM-pose-free cells, both modes; full raw timeline",
        limitation="Pose-sensitive mapper disagreement, not independently labeled terrain or retained-bucket membership",
        final_stamp=final_stamp, geometry=grid, chunks_verified=len(native),
        cells_yx={k: v.tolist() for k, v in cells.items()}, point_correspondences=counts,
        input_sha256={str(report_path): sha(report_path), str(pair / "shadow_pair_audit.json"): sha(pair / "shadow_pair_audit.json")},
        output_sha256=sha(output / "point_correspondences.jsonl"), source_sha256=sha(Path(__file__)))
    with (output / "pose_conflicts.json").open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("capture", "pair", "replay"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        print(json.dumps(run(args.capture, args.pair, args.replay, output), indent=2))
    except Exception as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc)), stream)
        raise


if __name__ == "__main__":
    main()
