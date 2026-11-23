#!/usr/bin/env python3
"""Natural native-cloud pose sensitivity, NOT terrain recall or causal acceptance.

Same returns, grid, timestamps, lifetime and mapper in all three branches. GT is
used only by the explicitly labelled isolation branch. No GT terrain labels enter
any mapper. Disagreement with the GT-pose mapper is not disagreement with terrain
truth: that mapper can also be wrong.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from layered_terrain import TerrainMapper
from raw_return_filter import valid_range_mask
from shadow_pose_replay import load_pose_sensitivity, sha


def map_counts(layers):
    cost = np.asarray(layers)[:, :, 0]
    if not np.isfinite(cost).all() or not np.isin(cost, [-1., 0., 20., 100.]).all():
        raise ValueError("unexpected terrain cost classes")
    return dict(cells=int(cost.size), unknown=int(np.count_nonzero(cost == -1)),
                free=int(np.count_nonzero((cost == 0) | (cost == 20))),
                cost_zero=int(np.count_nonzero(cost == 0)), cost_twenty=int(np.count_nonzero(cost == 20)),
                occupied=int(np.count_nonzero(cost == 100)))


def disagreements(reference, candidate):
    if np.shape(reference) != np.shape(candidate):
        raise ValueError("different terrain grids")
    map_counts(reference); map_counts(candidate)
    a, b = reference[:, :, 0], candidate[:, :, 0]
    names = {-1: "unknown", 0: "free_zero", 20: "free_twenty", 100: "occupied"}
    return {f"gt_pose_{names[x]}__slam_pose_{names[y]}": int(np.count_nonzero((a == x) & (b == y)))
            for x in names for y in names}


def lost_support_reasons(reference, candidate):
    """Explain class losses relative to GT-pose mapping, not terrain truth."""
    if np.shape(reference) != np.shape(candidate):
        raise ValueError("different terrain grids")
    map_counts(reference); map_counts(candidate)
    names = ("NO_RETURNS", "MEASURED_GROUND", "INFERRED_GROUND", "OBSTACLE", "NO_GROUND_MODEL",
             "DISCONNECTED_SURFACE", "LAYER_CAPACITY", "UNCERTAIN_RAISED", "NEGATIVE_EDGE",
             "INSUFFICIENT_AREA", "CLEARANCE_OCCLUDED", "POSITIVE_EDGE", "UNSUPPORTED_EXTENT", "UNRESOLVED_NORMAL")
    reasons = candidate[:, :, 1]
    if not np.isin(reasons, np.arange(len(names))).all():
        raise ValueError("unknown terrain reason code")
    lost = np.isin(reference[:, :, 0], [0, 20]) & (candidate[:, :, 0] == -1)
    return {name: int(np.count_nonzero(lost & (reasons == index))) for index, name in enumerate(names)}


def edge_attribution(reference, candidate, cells):
    """Partition lost support by the actual propagated negative-edge source bits."""
    if cells.shape != (*candidate.shape[:2], 4):
        raise ValueError("diagnostic cell layout mismatch")
    lost = np.isin(reference[:, :, 0], [0, 20]) & (candidate[:, :, 1] == 8)
    if (cells[:, :, :2] > 15).any():
        raise ValueError("unknown diagnostic edge flags")
    bits = cells[:, :, 1] & 11  # Raised-object bit alone is not a boundary cause.
    result = {name: int(np.count_nonzero(lost & (bits == flag))) for flag, name in
              enumerate(("unattributed", "below_plane_only", "normal_only", "both"))}
    if (bits & 8).any():
        result.update({name: int(np.count_nonzero(lost & (bits == flag))) for flag, name in
                       ((8, "native_only"), (9, "below_plane_and_native"),
                        (10, "normal_and_native"), (11, "below_plane_normal_and_native"))})
    return result


def verify_projection(reference, record):
    for key in ("stamp", "through_native_chunk", "layers_sha256"):
        if record[key] != reference[key]:
            raise ValueError("diagnostic projection differs from frozen reference: " + key)


def observation_attribution(reference, candidate, cells, observations):
    """Evidence availability only: no recovered-free or terrain truth claim."""
    if (observations.ndim != 3 or observations.shape[:2] != candidate.shape[:2]
            or observations.shape[2] not in (13, 22) or not np.isfinite(observations).all()):
        raise ValueError("invalid observation diagnostics")
    def expand(mask):
        padded = np.pad(mask, 1)
        h, w = mask.shape
        return np.logical_or.reduce([padded[y:y+h, x:x+w] for y in range(3) for x in range(3)])
    sources = observations[:, :, 0]
    qualified = observations[:, :, 8]
    lost = np.isin(reference[:, :, 0], [0, 20]) & (candidate[:, :, 1] == 8)
    eligible = expand(qualified > 0) & ~expand(sources > qualified) & ((cells[:, :, 1] & 9) == 0)
    totals = observations.sum(axis=(0, 1))
    result = dict(pooled_wall_layers=int(totals[0]),
                source_observations=dict(zip(("rank_deficient", "nonplanar", "ground_normal", "steep_normal"),
                                             map(int, totals[1:5]))),
                scatter_trace=dict(total=float(totals[5]), within=float(totals[6]), between=float(totals[7])),
                qualified_ground_only_layers=int(totals[8]),
                lost_cells_with_only_qualified_sources=int(np.count_nonzero(lost & eligible)),
                limitation="Between-observation scatter includes sampling geometry, not just pose error; normal evidence is not whole-cell clearance.")
    if observations.shape[2] == 22:
        qualified = observations[:, :, 17]
        eligible = expand(qualified > 0) & ~expand(sources > qualified) & ((cells[:, :, 1] & 9) == 0)
        result["spatial_context"] = dict(
            source_observations=dict(zip(("rank_deficient", "nonplanar", "ground_normal", "steep_normal"),
                                         map(int, totals[13:17]))),
            qualified_ground_only_layers=int(totals[17]),
            lost_cells_with_only_qualified_sources=int(np.count_nonzero(lost & eligible)),
            diagnostic_only=True, stencil_cells=9, center_return_residual_bound_m=.005)
    return result


def run(capture, pair, library, output, *, diagnostics=False, reference_run=None):
    library_hash = sha(library)
    evaluator_files = [Path(__file__).with_name(name) for name in
                       (Path(__file__).name, "layered_terrain.py", "raw_return_filter.py", "shadow_pose_replay.py")]
    evaluator_hashes = {path.name: sha(path) for path in evaluator_files}
    reference = None
    if diagnostics:
        if reference_run is None:
            raise ValueError("diagnostics require a frozen uninstrumented reference run")
        check_run(reference_run)
        reference = json.loads((reference_run / "natural_terrain_sensitivity.json").read_text())
        if not reference.get("completed") or reference.get("diagnostics") or (reference_run / "failure.json").exists():
            raise ValueError("ineligible diagnostic reference")
    elif reference_run is not None:
        raise ValueError("reference run requires explicit diagnostics")
    for path in (capture, pair):
        check_run(path)
        if (path / "failure.json").exists():
            raise ValueError("failed input is ineligible")
    complete = json.loads((capture / "natural_holdout.json").read_text())
    if not complete.get("complete") or complete.get("stationary_only") is not False:
        raise ValueError("completed moving natural capture required")
    native = json.loads((capture / "native_timing_audit.json").read_text())["frames"]
    if not native or len(native) != complete["chunks"]:
        raise ValueError("incomplete native capture")
    # This checks every audited input hash and all measurement-time associations.
    poses, provenance = {}, {}
    for mode in ("first_published", "final_revised"):
        poses[mode], provenance[mode] = load_pose_sensitivity(pair, capture, native, mode)
    audit = json.loads((pair / "shadow_pair_audit.json").read_text())
    if audit.get("experiment") != "natural_motion_gt_start":
        raise ValueError("not a qualified natural motion pair")
    home = np.asarray(audit["fixed_home"])
    poses["gt_isolated"] = np.array([f["world_from_lidar"] for f in native])
    poses["gt_isolated"][:, :3, 3] -= home
    # Fixed, axis-aligned 20 m window from the recorded pre-motion seed, not
    # estimator bounds or a best-fit alignment; shared by every branch.
    base = np.asarray(complete["request"]["world_from_base"])
    origin = np.floor((base[:2, 3]-home[:2])/0.1)*0.1-10.
    geometry = dict(width=200, height=200, resolution=.1, origin=origin.tolist(), lifetime=30., interpolate=True)
    sensor = json.loads((capture / "manifest.json").read_text())["sensor"]["sensor_attributes"]
    # USD sensor metadata is serialized as strings in the physical manifest.
    minimum, maximum = float(sensor["minRange"]), float(sensor["maxRange"])
    valid_range_mask(np.empty((0, 3)), minimum, maximum)
    files = sorted((capture / "native_chunks").glob("chunk_*.npz"))
    if len(files) != len(native):
        raise ValueError("native file count differs")
    mappers = {}
    input_hashes = {str(pair / "shadow_pair_audit.json"): sha(pair / "shadow_pair_audit.json")}
    frames, previous, projected, counts = [], -1., -1., []
    try:
        for name in ("gt_isolated", "first_published", "final_revised"):
            mappers[name] = TerrainMapper(library, **geometry)
            if hasattr(mappers[name].lib, "terrain_diagnostics") != diagnostics:
                raise ValueError("library instrumentation differs from explicit diagnostics mode")
        for index, (path, frame) in enumerate(zip(files, native)):
            if path.name != frame["file"] or sha(path) != frame["sha256"]:
                raise ValueError("native cloud changed")
            input_hashes[str(path)] = frame["sha256"]
            with np.load(path) as data:
                points = data["points"]
                stamp = float(data["observed_stamp"])
                if (stamp != frame["observed_stamp"] or stamp <= previous
                        or not np.array_equal(data["world_from_lidar"], frame["world_from_lidar"])):
                    raise ValueError("native pose/stamp changed")
                keep, selected = valid_range_mask(points, minimum, maximum)
                points = points[keep]
            previous = stamp
            counts.append(dict(index=index, **selected))
            project = projected < 0 or stamp-projected >= .1-1e-6 or index == len(native)-1
            maps = {}
            for name, mapper in mappers.items():
                transform = poses[name][index]
                mapper.integrate(points @ transform[:3, :3].T + transform[:3, 3], stamp,
                                 transform[:3, 3], np.arctan2(transform[1, 0], transform[0, 0]))
                if project:
                    maps[name] = mapper.project(stamp)
            if project:
                projected = stamp
                record = dict(stamp=stamp, through_native_chunk=index,
                              counts={name: map_counts(value) for name, value in maps.items()},
                              disagreement={name: disagreements(maps["gt_isolated"], maps[name])
                                            for name in ("first_published", "final_revised")},
                              lost_support_reasons={name: lost_support_reasons(maps["gt_isolated"], maps[name])
                                                    for name in ("first_published", "final_revised")},
                              layers_sha256={name: hashlib.sha256(value.tobytes()).hexdigest()
                                             for name, value in maps.items()})
                frames.append(record)
                if diagnostics:
                    if len(frames) > len(reference["frames"]):
                        raise ValueError("extra diagnostic projection")
                    verify_projection(reference["frames"][len(frames)-1], record)
                    record["edge_attribution"] = {}
                    for name, mapper in mappers.items():
                        cells, corners = mapper.diagnostics()
                        record["edge_attribution"][name] = edge_attribution(maps["gt_isolated"], maps[name], cells)
                        if hasattr(mapper.lib, "terrain_observations"):
                            observations = mapper.observations()
                            record.setdefault("observation_attribution", {})[name] = observation_attribution(
                                maps["gt_isolated"], maps[name], cells, observations)
                            if index == len(native)-1:
                                with (output / f"observations_{name}.npz").open("xb") as stream:
                                    np.savez_compressed(stream, observations=observations)
                        if index == len(native)-1:
                            with (output / f"diagnostics_{name}.npz").open("xb") as stream:
                                np.savez_compressed(stream, cells=cells, corners=corners)
                if index == len(native)-1:
                    for name, value in maps.items():
                        with (output / f"{name}.npz").open("xb") as stream:
                            np.savez_compressed(stream, layers=value)
            if (index+1) % 300 == 0:
                print(f"Integrated {index+1}/{len(native)} native chunks in all branches", flush=True)
    finally:
        for mapper in mappers.values():
            mapper.close()
    if sha(library) != library_hash:
        raise ValueError("mapper library changed during replay")
    if {path.name: sha(path) for path in evaluator_files} != evaluator_hashes:
        raise ValueError("evaluator source changed during replay")
    if diagnostics and (reference["geometry"] != geometry or reference["input_sha256"] != input_hashes
                        or reference["return_counts"] != counts or len(reference["frames"]) != len(frames)):
        raise ValueError("diagnostic inputs/grid/schedule differ from reference")
    result = dict(completed=True, acceptance=False, natural_terrain_recall_evaluated=False,
                  causal_map_evaluation=False, compute_acceptance=False, gt_labels_in_mapper=False,
                  diagnostics=diagnostics,
                  diagnostic_reference_sha256=sha(reference_run / "natural_terrain_sensitivity.json") if diagnostics else None,
                  limitation="GT-pose mapper is an isolation reference, not terrain ground truth; class counts are not recall.",
                  pose_sensitivity=provenance, geometry=geometry, projection_hz=10.,
                  native_chunks_integrated=len(native), frames=frames, return_counts=counts,
                  input_sha256=input_hashes, library_sha256=library_hash,
                  evaluator_sha256=evaluator_hashes,
                  output_sha256={path.name: sha(path) for path in sorted(output.glob("*.npz"))})
    with (output / "natural_terrain_sensitivity.json").open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("pair", type=Path)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    parser.add_argument("--diagnostics", action="store_true")
    parser.add_argument("--reference-run", type=Path)
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        result = run(args.capture, args.pair, args.library, output,
                     diagnostics=args.diagnostics, reference_run=args.reference_run)
        print(json.dumps(result["frames"][-1], indent=2))
    except Exception as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc)), stream)
        raise


if __name__ == "__main__":
    main()
