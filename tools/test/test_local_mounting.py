from pathlib import Path
import json
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.mounting_study import (
    PHASES,
    azimuth_coverage,
    beam_directions,
    first_intersections,
    hazard_boxes,
    region_masks,
    sampled_cone_visibility,
    sampling_metrics,
    sensor_mount,
    valid_returns,
    world_rays,
)


WHEELS = [[x, y] for x in (-0.256, 0.256) for y in (-0.2854, 0.2854)]


@pytest.mark.parametrize("columns,expected", [(1999, True), (666, False), (1, False)])
def test_azimuth_coverage_serializes_native_float_and_counts_missing_sectors(columns, expected):
    report = azimuth_coverage(np.arange(columns, dtype=np.float32) * np.deg2rad(0.18), np.float32(0.18))
    assert json.loads(json.dumps(report))["complete_azimuth"] is expected


@pytest.mark.parametrize("angles", [[], [np.nan], [[0]]])
def test_invalid_azimuth_coverage_rejected(angles):
    with pytest.raises(ValueError):
        azimuth_coverage(angles, 0.18)


def test_positive_native_zenith_is_down_and_positive_pitch_points_front_down():
    np.testing.assert_allclose(beam_directions([0], [np.pi / 4]), [[2**-0.5, 0, -(2**-0.5)]])
    origin, rays = world_rays(sensor_mount(15), 0.145, [0], np.deg2rad([20]))
    depth, labels = first_intersections(origin, rays)
    point = origin + depth[:, None] * rays
    assert point[0, 0] == pytest.approx(0.4 + 0.395 / np.tan(np.deg2rad(35)))
    assert point[0, 2] == pytest.approx(0)
    assert labels.tolist() == [0]


def test_offcenter_yaw_rotates_sensor_origin_as_well_as_beams():
    origin, rays = world_rays(sensor_mount(0), 0.145, [0], [0], np.pi / 2)
    np.testing.assert_allclose(origin, [0, 0.4, 0.395], atol=1e-15)
    np.testing.assert_allclose(rays, [[0, 1, 0]], atol=1e-15)


def test_no_fabricated_ground_for_horizon_or_upward_beams():
    depth, labels = first_intersections([0, 0, 1], beam_directions([0], [-0.1, 0]))
    assert np.isinf(depth).all()
    assert labels.tolist() == [-1, -1]


def test_near_occluder_is_not_removed_before_range_filtering():
    depth, labels = first_intersections([0, 0, 1], [[1, 0, 0]], [([0.1, -1, 0], [0.2, 1, 2])])
    assert depth.tolist() == [0.1] and labels.tolist() == [1]
    assert not valid_returns(depth, 0.5, 100).any()


def test_parallel_ray_box_and_inside_origin():
    rays = [[1, 0, 0], [0, 0, -1]]
    depths, labels = first_intersections([0, 0, 1], rays, [([-1, -1, 0.5], [1, 1, 1.5])])
    assert depths.tolist() == [0, 0] and labels.tolist() == [1, 1]
    depths, labels = first_intersections([0, 0, 1], [[1, 0, 0]], [([1, 1, 0], [2, 2, 2])])
    assert np.isinf(depths[0]) and labels[0] == -1


def test_box_occludes_floor_and_nearest_box_wins_independent_of_order():
    boxes = [([-0.1, -0.1, 0], [0.1, 0.1, 0.08]), ([-0.1, -0.1, 0], [0.1, 0.1, 0.16])]
    depth, labels = first_intersections([0, 0, 1], [[0, 0, -1]], boxes)
    assert depth[0] == pytest.approx(0.84) and labels[0] == 2


@pytest.mark.parametrize("phase", PHASES)
def test_empty_and_repeated_scans_preserve_all_region_denominators(phase):
    empty = sampling_metrics(np.empty((0, 3)), WHEELS, phase)
    assert all(m["cells"] > 0 and m["hit_cells"] == 0 and m["fraction"] == 0 for m in empty.values())
    points = np.array([[1, 0.2854, 0], [2, 0, 0]])
    assert sampling_metrics(points, WHEELS, phase) == sampling_metrics(np.tile(points, (25, 1)), WHEELS, phase)
    masks = region_masks(WHEELS, phase)
    sectors = [masks[k] for k in ("front", "rear", "left", "right")]
    assert np.max(np.sum(sectors, axis=0)) == 1


def test_current_mount_full_yaw_does_not_put_wheel_centers_in_cone():
    visible = sampled_cone_visibility(
        sensor_mount(5), 0.1449231063, WHEELS, np.deg2rad([-20.15, 20.15]), np.linspace(0, 2 * np.pi, 721), 0.5, 100
    )
    assert visible == [False] * 4


@pytest.mark.parametrize("pitch,expected", [(30, False), (-30, True)])
def test_tilt_direction_matters_for_targets_inside_the_sensor_orbit(pitch, expected):
    visible = sampled_cone_visibility(
        sensor_mount(pitch), 0.1449231063, WHEELS, np.deg2rad([-20.15, 20.15]), np.linspace(0, 2 * np.pi, 721), 0.5, 100
    )
    assert visible == [expected] * 4


def test_declared_hazards_are_eight_cm_high_not_eight_cm_grid_voxels():
    boxes = np.asarray(hazard_boxes())
    np.testing.assert_allclose(boxes[:, 1] - boxes[:, 0], np.tile([0.1, 0.1, 0.08], (8, 1)))


@pytest.mark.parametrize("minimum,maximum", [(1, 0.5), (-1, 2), (0, np.inf)])
def test_invalid_ranges_rejected(minimum, maximum):
    with pytest.raises(ValueError):
        valid_returns([1], minimum, maximum)
