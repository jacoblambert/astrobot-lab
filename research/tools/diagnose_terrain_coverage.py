#!/usr/bin/env python3
"""Attribute fixed-reference flat coverage without changing maps or acceptance.

GT remains evaluator-only. Counts use fine reference centers and retain unknown
and out-of-map cells in the original denominator. This is development diagnosis,
not permission to remove inconvenient reference regions or relax mapper guards.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from plot_terrain_calibration import evaluation_reference
from score_terrain_calibration import reference_labels
from terrain_metrics import score_terrain


REASONS = ("no_returns", "measured_ground", "inferred_ground", "obstacle", "no_ground_model",
           "disconnected_surface", "layer_capacity", "uncertain_raised", "negative_edge",
           "insufficient_area", "clearance_occluded", "positive_edge", "unsupported_extent", "unresolved_normal")


def rejection_attribution(layers, cells, *, map_origin, map_resolution, labels, gt_origin, gt_resolution):
    """Partition final flat-cell rejections by recorded causes, never infer support.

    Flags are unions over corners/source cells, so a group can have several causes.
    Every group still uses the original global fine-flat denominator.
    """
    if cells.shape != (*layers.shape[:2], 4) or cells.dtype != np.uint32:
        raise ValueError("invalid diagnostic cell layout")
    if ((cells[:, :, 0] > 15).any() or (cells[:, :, 1] > 15).any() or
            (cells[:, :, 2] & np.uint32(~115 & 0xffffffff)).any() or (cells[:, :, 3] > 15).any()):
        raise ValueError("unknown diagnostic flags")
    y, x = np.indices(labels.shape)
    mx = np.floor((gt_origin[0] + (x + .5) * gt_resolution - map_origin[0]) / map_resolution).astype(int)
    my = np.floor((gt_origin[1] + (y + .5) * gt_resolution - map_origin[1]) / map_resolution).astype(int)
    inside = (mx >= 0) & (my >= 0) & (mx < layers.shape[1]) & (my < layers.shape[0])
    flat = labels == 0
    denominator = int(flat.sum())
    if not denominator:
        raise ValueError("empty flat reference")
    reason = np.full(labels.shape, -1)
    fields = np.zeros((*labels.shape, 4), dtype=np.uint32)
    reason[inside] = layers[my[inside], mx[inside], 1]
    fields[inside] = cells[my[inside], mx[inside]]
    result = dict(flat_cells=denominator, causes_may_overlap_within_group=True)
    for name, code, flags in (("extent", 12, fields[:, :, 2] | ((fields[:, :, 3] != 0).astype(np.uint32) << 7)),
                              ("negative_edge", 8, fields[:, :, 1])):
        selected = flat & (reason == code)
        cause_bits = 114 | 128 if name == "extent" else 3 | 8
        if (selected & ((flags & cause_bits) == 0)).any():
            raise ValueError("rejected cell lacks a diagnostic cause")
        groups = []
        for bits in np.unique(flags[selected]):
            count = int(np.count_nonzero(selected & (flags == bits)))
            groups.append(dict(flags=int(bits), fine_cells=count, fraction_of_all_flat=count / denominator))
        result[name] = dict(fine_cells=int(selected.sum()), groups=groups)
    result["extent_flag_names"] = {1: "needed_corner", 2: "no_anchor", 16: "few_directions",
                                   32: "angular_gap", 64: "span", 128: "plane_disagreement"}
    result["edge_flag_names"] = {1: "below_plane", 2: "wall_normal", 4: "raised_obstacle_source_present",
                                 8: "native_boundary"}
    return result


def flat_attribution(layers, *, map_origin, map_resolution, labels, object_ids, gt_origin, gt_resolution):
    layers = np.asarray(layers)
    if layers.ndim != 3 or layers.shape[2] != 6 or not np.isin(layers[:, :, 1], range(len(REASONS))).all():
        raise ValueError("invalid layered map/reason codes")
    metrics = score_terrain(layers[:, :, 0], map_origin=map_origin, map_resolution=map_resolution,
                            labels=labels, object_ids=object_ids, gt_origin=gt_origin, gt_resolution=gt_resolution)
    y, x = np.indices(labels.shape)
    mx = np.floor((gt_origin[0] + (x + .5) * gt_resolution - map_origin[0]) / map_resolution).astype(int)
    my = np.floor((gt_origin[1] + (y + .5) * gt_resolution - map_origin[1]) / map_resolution).astype(int)
    inside = (mx >= 0) & (my >= 0) & (mx < layers.shape[1]) & (my < layers.shape[0])
    costs = np.full(labels.shape, -1)
    reasons = np.full(labels.shape, -1)
    costs[inside] = layers[my[inside], mx[inside], 0]
    reasons[inside] = layers[my[inside], mx[inside], 1]
    flat = labels == 0
    denominator = int(flat.sum())
    if not denominator:
        raise ValueError("empty flat reference")
    def groups_for(mask):
        groups = []
        for cost in (-1, 0, 20, 100):
            for reason in range(-1, len(REASONS)):
                count = int(np.count_nonzero(mask & (costs == cost) & (reasons == reason)))
                if count:
                    groups.append(dict(cost=cost, reason="outside_map" if reason < 0 else REASONS[reason],
                                       fine_cells=count, fraction_of_all_flat=count / denominator))
        assert sum(group["fine_cells"] for group in groups) == int(mask.sum())
        return groups

    # Fixed platform axes, not current robot heading or candidate-derived regions.
    # Retain the GLOBAL denominator even inside each diagnostic quadrant. These
    # partitions explain direction-dependent coverage; none is an acceptance gate.
    positive_x = gt_origin[0] + (x + .5) * gt_resolution >= 0.
    positive_y = gt_origin[1] + (y + .5) * gt_resolution >= 0.
    quadrants = {}
    for x_sign, x_mask in (("negative_x", ~positive_x), ("positive_x", positive_x)):
        for y_sign, y_mask in (("negative_y", ~positive_y), ("positive_y", positive_y)):
            mask = flat & x_mask & y_mask
            quadrants[f"{x_sign}_{y_sign}"] = dict(flat_cells=int(mask.sum()), groups=groups_for(mask))
    assert sum(region["flat_cells"] for region in quadrants.values()) == denominator
    return dict(flat_cells=denominator, groups=groups_for(flat), quadrants=quadrants,
                quadrant_frame="fixed calibration platform XY; zero belongs to positive half-plane"), metrics


def diagnose(capture, evaluation):
    if (evaluation / "failure.json").exists():
        raise ValueError("failed evaluation cannot establish coverage")
    reference = evaluation_reference(capture, evaluation)
    manifest = json.loads((evaluation / "manifest.json").read_text())
    summary = json.loads((evaluation / "summary.json").read_text())
    if not summary.get("frames"):
        raise ValueError("incomplete evaluation")
    grid = manifest.get("terrain_geometry", dict(origin=[-10., -10.], resolution=.1, width=200, height=200))
    spec = json.loads((reference / "calibration_spec.json").read_text())
    with np.load(reference / "collision_tile.npz") as tile:
        labels, ids, _ = reference_labels(spec, tile, json.loads((reference / "summary.json").read_text()))
        gt_origin, gt_resolution = tile["origin"] - spec["center"], float(tile["resolution"])
    result = dict(acceptance=False, evaluation=str(evaluation), reference=str(reference), terrain_geometry=grid,
                  gt_terrain_passed_to_mapper=False, development_diagnosis_only=True, variants={})
    for name in manifest["variants"]:
        if Path(name).name != name:
            raise ValueError("invalid variant name")
        with np.load(evaluation / f"{name}.npz") as saved:
            layers = saved["layers"]
        if layers.shape != (grid["height"], grid["width"], 6):
            raise ValueError("layer shape differs from scored geometry")
        recorded = summary["frames"][-1][name]
        if hashlib.sha256(layers[:, :, :2].tobytes()).hexdigest() != recorded["cost_reason_sha256"]:
            raise ValueError("final cost/reason map changed after scoring")
        attribution, metrics = flat_attribution(layers, map_origin=grid["origin"], map_resolution=grid["resolution"],
                                               labels=labels, object_ids=ids, gt_origin=gt_origin,
                                               gt_resolution=gt_resolution)
        if any(recorded.get(key) != value for key, value in metrics.items()):
            raise ValueError("diagnostic reference/metrics differ from recorded score")
        result["variants"][name] = attribution
        if manifest.get("diagnostic_collection"):
            path = evaluation / f"diagnostics_{name}.npz"
            if hashlib.sha256(path.read_bytes()).hexdigest() != manifest["diagnostic_sha256"].get(path.name):
                raise ValueError("diagnostic array changed after replay")
            with np.load(path) as diagnostic:
                attribution["rejection_details"] = rejection_attribution(
                    layers, diagnostic["cells"], map_origin=grid["origin"], map_resolution=grid["resolution"],
                    labels=labels, gt_origin=gt_origin, gt_resolution=gt_resolution)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("evaluation", type=Path)
    args = parser.parse_args()
    result = diagnose(args.capture, args.evaluation)
    with (args.evaluation / "coverage_diagnostics.json").open("x") as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
