from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.ray_design import (
    hazard_bank,
    layouts,
    level_geometry,
    native_pattern_indices,
    ray_mount,
    validate_elevations,
    selected_setup,
    validate_selected_setup,
    terrain_view_transform,
)
from local_terrain.mounting_study import first_intersections, sensor_mount, valid_returns, world_rays


def test_base_yaw_rotates_offcenter_origin_without_changing_mount():
    mount = sensor_mount(0)
    mount[2, 3] += 0.145
    result = terrain_view_transform(mount, [0.2, 0.1], 90)
    np.testing.assert_allclose(result[:3, 3], [0.2, 0.5, 0.395], atol=1e-14)
    np.testing.assert_allclose(result[:3, :3], [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-14)
    np.testing.assert_allclose(terrain_view_transform(mount, [0, 0], 360), mount, atol=1e-14)


def selected_geometry():
    setup, elevations = selected_setup()
    geometry = level_geometry(dict(baseline_mount=sensor_mount(5).tolist()))
    geometry.update(
        full_columns=1999,
        sensor_attributes=dict(
            horizontalResolution=setup["horizontal_resolution_deg"], minRange=0.5, maxRange=100, rotationRate=10
        ),
    )
    return geometry, elevations


def test_selected_setup_exactly_matches_previously_measured_mid_dense():
    geometry, elevations = selected_geometry()
    assert validate_selected_setup(geometry, elevations)["mount_rpy_deg"] == [0, 0, 0]
    np.testing.assert_array_equal(elevations, layouts(np.linspace(-0.3, 0.3, 32))["mid_dense_10_to_-60"])


@pytest.mark.parametrize("fault", ["rings", "pitch", "offset", "columns", "range", "rate"])
def test_selected_setup_rejects_accidental_drift(fault):
    geometry, elevations = selected_geometry()
    if fault == "rings":
        elevations[8] += 0.01
    elif fault == "pitch":
        geometry["experiment_mount"] = sensor_mount(5).tolist()
    elif fault == "offset":
        geometry["experiment_mount"][0][3] += 0.001
    elif fault == "columns":
        geometry["full_columns"] = 2000
    elif fault == "range":
        geometry["sensor_attributes"]["minRange"] = 0.4
    else:
        geometry["sensor_attributes"]["rotationRate"] = 20
    with pytest.raises(ValueError):
        validate_selected_setup(geometry, elevations)


def test_level_experiment_does_not_inherit_or_rewrite_recorded_pitch():
    recorded = dict(baseline_mount=sensor_mount(5).tolist(), wheel_xy=[[0.256, 0.2854]], base_height_m=0.145)
    geometry = level_geometry(recorded)
    np.testing.assert_array_equal(ray_mount(geometry, require_level=True), sensor_mount(0))
    np.testing.assert_array_equal(recorded["baseline_mount"], sensor_mount(5))
    assert geometry["baseline_mount"] == recorded["baseline_mount"]
    geometry["wheel_xy"][0][0] = 99
    assert recorded["wheel_xy"][0][0] == 0.256


@pytest.mark.parametrize("fault", ["missing", "pitch", "roll", "translation"])
def test_level_contract_rejects_silent_mount_regression(fault):
    geometry = level_geometry(dict(baseline_mount=sensor_mount(5).tolist()))
    if fault == "missing":
        del geometry["experiment_mount"]
    elif fault == "pitch":
        geometry["experiment_mount"] = sensor_mount(5).tolist()
    elif fault == "roll":
        geometry["experiment_mount"] = [[1, 0, 0, 0.4], [0, 0, -1, 0], [0, 1, 0, 0.25], [0, 0, 0, 1]]
    else:
        geometry["experiment_mount"][0][3] += 0.01
    with pytest.raises(ValueError):
        ray_mount(geometry, require_level=True)


def test_level_mount_preserves_elevation_for_every_azimuth_not_just_forward():
    mount = ray_mount(level_geometry(dict(baseline_mount=sensor_mount(5).tolist())), require_level=True)
    elevations = np.array([10, 0, -45, -60])
    _, directions = world_rays(mount, 0.145, np.linspace(-np.pi, np.pi, 17), -np.deg2rad(elevations))
    measured = np.rad2deg(np.arctan2(directions[:, 2], np.linalg.norm(directions[:, :2], axis=1)))
    np.testing.assert_allclose(measured, np.tile(elevations, 17), atol=1e-12)


def test_every_layout_keeps_32_unique_rays_and_declared_bounds():
    baseline = np.deg2rad(np.linspace(-20.15, 20.15, 32))
    designs = layouts(baseline)
    assert len(designs) == 11
    np.testing.assert_allclose(designs["baseline"], -np.rad2deg(baseline))
    for name, values in designs.items():
        assert len(values) == len(set(values)) == 32
        if "10_to_-60" in name:
            assert min(values) == -60 and max(values) == 10
    assert np.ptp(designs["uniform_0_to_-40.3"]) == pytest.approx(40.3)
    assert not np.allclose(np.diff(designs["mid_dense_10_to_-60"]), np.diff(designs["mid_dense_10_to_-60"])[0])


@pytest.mark.parametrize("bad", [[], [0] * 32, list(range(31)), [np.nan] + list(range(31)), list(range(59, 91))])
def test_invalid_designs_fail_closed(bad):
    with pytest.raises(ValueError):
        validate_elevations(bad)


def test_steeper_is_not_always_more_useful_due_to_min_range():
    origin, directions = world_rays(sensor_mount(0), 0.145, [0], np.deg2rad([45, 60, 75]))
    depth, _ = first_intersections(origin, directions)
    assert valid_returns(depth, 0.5, 100).tolist() == [True, False, False]


def test_128_independent_hazards_keep_height_size_and_all_bearings():
    bank = hazard_bank()
    assert len(bank) == 128
    assert len({case["radius_m"] for case in bank}) == 8
    assert len({case["bearing_deg"] for case in bank}) == 16
    for case in bank:
        np.testing.assert_allclose(np.diff(case["box"], axis=0)[0], [0.1, 0.1, 0.08])


def test_rotating_flat_hits_matches_recasting_offcenter_sensor():
    azimuth, zenith = np.linspace(-np.pi, np.pi, 40, endpoint=False), np.deg2rad([10, 30, 45])
    origin, rays = world_rays(sensor_mount(5), 0.145, azimuth, zenith)
    depth, _ = first_intersections(origin, rays)
    valid = valid_returns(depth, 0.5, 100)
    ground = origin + depth[valid, None] * rays[valid]
    yaw = 0.7
    c, s = np.cos(yaw), np.sin(yaw)
    rotated = ground @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    new_origin, new_rays = world_rays(sensor_mount(5), 0.145, azimuth, zenith, yaw)
    new_depth, _ = first_intersections(new_origin, new_rays)
    np.testing.assert_allclose(new_origin + new_depth[valid, None] * new_rays[valid], rotated, atol=1e-12)


def test_shifted_bank_preserves_count_and_size_but_changes_all_placements():
    first, second = hazard_bank(), hazard_bank(shifted=True)
    assert len(first) == len(second) == 128
    for a, b in zip(first, second):
        assert b["radius_m"] == pytest.approx(a["radius_m"] + 0.1)
        assert b["bearing_deg"] == a["bearing_deg"] + 11.25
        np.testing.assert_allclose(np.diff(b["box"], axis=0)[0], [0.1, 0.1, 0.08])


def test_native_pattern_id_covers_every_supplied_ray_and_survives_float32():
    az = -np.pi + np.arange(1999) * np.deg2rad(0.18000000715255737)
    elevation = np.linspace(10, -45, 32)
    aa, zz = np.meshgrid(az, -np.deg2rad(elevation), indexing="ij")
    ids = native_pattern_indices(aa.ravel().astype(np.float32), zz.ravel().astype(np.float32), az, elevation)
    np.testing.assert_array_equal(ids, np.arange(1999 * 32))


def test_stale_native_pattern_is_rejected_not_filtered():
    with pytest.raises(ValueError, match="stale"):
        native_pattern_indices([0, 0], [0, np.deg2rad(-20)], [-1, 0, 1], np.linspace(10, -45, 32))
