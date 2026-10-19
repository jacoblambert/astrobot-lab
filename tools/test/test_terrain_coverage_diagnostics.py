import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnose_terrain_coverage import flat_attribution, rejection_attribution


@pytest.mark.parametrize("shift", [0., .025, .05, .075])
def test_coverage_reasons_retain_fine_reference_and_outside_map_denominator(shift):
    layers = np.zeros((1, 3, 6), dtype=np.float32)
    layers[0, :, 0] = [0, -1, 100]
    layers[0, :, 1] = [1, 12, 3]
    labels = np.zeros((5, 25), dtype=np.int8)
    result, metrics = flat_attribution(layers, map_origin=[shift, 0.], map_resolution=.1,
                                       labels=labels, object_ids=np.zeros_like(labels),
                                       # Keep centers off exact coarse boundaries so this
                                       # fixture has five fine columns per coarse cell.
                                       gt_origin=[-.103, 0.], gt_resolution=.02)
    groups = {group["reason"]: group for group in result["groups"]}
    assert result["flat_cells"] == metrics["flat_cells"] == 125
    assert sum(group["fine_cells"] for group in result["groups"]) == 125
    assert groups["unsupported_extent"]["fine_cells"] == 25
    assert groups["outside_map"]["fine_cells"] == 50
    assert groups["measured_ground"]["fraction_of_all_flat"] == metrics["flat_free_fraction"]
    assert groups["obstacle"]["fraction_of_all_flat"] == metrics["flat_false_occupied_fraction"]


def test_unknown_reason_codes_cannot_silently_disappear_from_diagnostics():
    layers = np.zeros((1, 1, 6))
    layers[0, 0, 1] = 99
    with pytest.raises(ValueError, match="reason"):
        flat_attribution(layers, map_origin=[0., 0.], map_resolution=.1, labels=np.zeros((1, 1)),
                         object_ids=np.zeros((1, 1)), gt_origin=[0., 0.], gt_resolution=.1)


@pytest.mark.parametrize("shift", [0., .025, .05, .075])
def test_fixed_quadrants_partition_original_denominator_including_unknown_and_outside(shift):
    layers = np.zeros((2, 2, 6), dtype=np.float32)
    layers[:, :, 0] = [[0, -1], [100, 20]]
    layers[:, :, 1] = [[1, 12], [3, 2]]
    labels = np.zeros((20, 20), dtype=np.int8)
    labels[0, 0] = -1  # Only the original reference can exclude a fine cell.
    result, metrics = flat_attribution(
        layers, map_origin=[-.1 + shift, -.1 + shift], map_resolution=.1,
        labels=labels, object_ids=np.zeros_like(labels), gt_origin=[-.2, -.2], gt_resolution=.02)
    regions = result["quadrants"]
    assert [region["flat_cells"] for region in regions.values()] == [99, 100, 100, 100]
    aggregate = {}
    for region in regions.values():
        assert sum(group["fine_cells"] for group in region["groups"]) == region["flat_cells"]
        for group in region["groups"]:
            assert group["fraction_of_all_flat"] == group["fine_cells"] / 399
            key = (group["cost"], group["reason"])
            aggregate[key] = aggregate.get(key, 0) + group["fine_cells"]
    assert aggregate == {(g["cost"], g["reason"]): g["fine_cells"] for g in result["groups"]}
    assert metrics["flat_cells"] == result["flat_cells"] == 399


def test_axis_centers_are_assigned_once_and_empty_quadrants_retained():
    layers = np.zeros((1, 1, 6), dtype=np.float32)
    layers[0, 0, :2] = [0, 1]
    result, _ = flat_attribution(layers, map_origin=[-.05, -.05], map_resolution=.1,
                                labels=np.zeros((1, 1)), object_ids=np.zeros((1, 1)),
                                gt_origin=[-.05, -.05], gt_resolution=.1)
    assert result["quadrants"]["positive_x_positive_y"]["flat_cells"] == 1
    assert sum(region["flat_cells"] for region in result["quadrants"].values()) == 1
    assert sum(not region["groups"] for region in result["quadrants"].values()) == 3


def test_rejection_causes_partition_fine_cells_without_dropping_outside_reference():
    layers = np.zeros((1, 3, 6), dtype=np.float32)
    layers[0, :, :2] = [[-1, 12], [-1, 12], [-1, 8]]
    cells = np.zeros((1, 3, 4), dtype=np.uint32)
    cells[0, 0, 2] = 1 | 16 | 32
    cells[0, 1, 3] = 4  # Plane disagreement on one corner, no failed certificate.
    cells[0, 2, 1] = 2 | 4  # Wall-normal-only source on a known raised obstacle.
    kwargs = dict(map_origin=[0., 0.], map_resolution=.1, labels=np.zeros((1, 4)),
                  gt_origin=[0., 0.], gt_resolution=.1)
    result = rejection_attribution(layers, cells, **kwargs)
    assert result["flat_cells"] == 4
    assert result["extent"]["groups"] == [dict(flags=49, fine_cells=1, fraction_of_all_flat=.25),
                                           dict(flags=128, fine_cells=1, fraction_of_all_flat=.25)]
    assert result["negative_edge"]["groups"] == [dict(flags=6, fine_cells=1, fraction_of_all_flat=.25)]
    for missing_cause in (0, 1):
        cells[0, 0, 2] = missing_cause
        with pytest.raises(ValueError, match="lacks"):
            rejection_attribution(layers, cells, **kwargs)
    cells[0, 0, 2] = 256
    with pytest.raises(ValueError, match="unknown"):
        rejection_attribution(layers, cells, **kwargs)


def test_native_boundary_is_a_distinct_cause_not_a_raised_obstacle_certificate():
    layers = np.zeros((1, 1, 6), dtype=np.float32)
    layers[0, 0, :2] = [-1, 8]
    cells = np.zeros((1, 1, 4), dtype=np.uint32)
    cells[0, 0, :2] = 8
    kwargs = dict(map_origin=[0., 0.], map_resolution=.1, labels=np.zeros((1, 1)),
                  gt_origin=[0., 0.], gt_resolution=.1)
    result = rejection_attribution(layers, cells, **kwargs)
    assert result["edge_flag_names"][8] == "native_boundary"
    assert result["negative_edge"]["groups"] == [dict(flags=8, fine_cells=1, fraction_of_all_flat=1.)]
    cells[0, 0, 1] = 4
    with pytest.raises(ValueError, match="lacks"):
        rejection_attribution(layers, cells, **kwargs)
    for field in (0, 1):
        cells[0, 0, :2] = 8
        cells[0, 0, field] = 16
        with pytest.raises(ValueError, match="unknown"):
            rejection_attribution(layers, cells, **kwargs)
