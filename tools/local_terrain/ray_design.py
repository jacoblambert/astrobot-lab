"""Fixed-budget single-sensor ray-layout screen, not a deployed sensor driver."""

from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from revision_evidence import validated_transform

from local_terrain.mounting_study import PHASES, first_intersections, sampling_metrics, valid_returns, world_rays

TERRAIN_VIEWS_XY = ((0, 0), (0.1, 0), (0.2, 0), (0.2, 0.1), (0.1, 0.1), (0, 0.1))
TERRAIN_YAWS_DEG = tuple(range(0, 360, 15))
SELECTED_SETUP = Path(__file__).with_name("selected_lidar.json")


def terrain_view_transform(initial_world_from_lidar, xy, yaw_deg):
    """Yaw the base AND its off-center sensor; mount remains level relative to base."""
    initial = validated_transform(initial_world_from_lidar)
    if np.asarray(xy).shape != (2,) or not np.isfinite([*xy, yaw_deg]).all():
        raise ValueError("finite XY and yaw required")
    c, s = np.cos(np.deg2rad(yaw_deg)), np.sin(np.deg2rad(yaw_deg))
    base = np.eye(4)
    base[:2, :2] = [[c, -s], [s, c]]
    base[:2, 3] = xy
    return base @ initial


def selected_setup(path=None):
    """The user-selected development geometry; never silently inherit old tilt."""
    setup = json.loads((SELECTED_SETUP if path is None else Path(path)).read_text())
    if setup["schema"] != "selected_single_lidar_v1" or setup["mount_rpy_deg"] != [0, 0, 0]:
        raise ValueError("selected setup requires explicit level mount")
    elevations = np.concatenate([np.linspace(start, end, count) for start, end, count in setup["elevation_bands_deg"]])
    validate_elevations(elevations)
    return setup, elevations


def validate_selected_setup(geometry, elevations):
    setup, selected = selected_setup()
    mount = ray_mount(geometry, require_level=True)
    attrs = geometry["sensor_attributes"]
    if (
        not np.array_equal(np.asarray(elevations), selected)
        or not np.allclose(mount[:3, 3], setup["base_from_lidar_translation_m"], atol=1e-12, rtol=0)
        or geometry["full_columns"] != setup["horizontal_columns"]
        or float(attrs["horizontalResolution"]) != setup["horizontal_resolution_deg"]
        or float(attrs["minRange"]) != setup["min_range_m"]
        or float(attrs["maxRange"]) != setup["max_range_m"]
        or float(attrs["rotationRate"]) != setup["rotation_hz"]
    ):
        raise ValueError("capture geometry disagrees with frozen selected setup")
    return setup


def level_geometry(recorded):
    """New experimental geometry; do not rewrite the recorded calibration.

    Keep translation and all evaluation inputs, but explicitly level the sensor.
    With identity mounting rotation, sensor-local elevations are base elevations.
    """
    geometry = deepcopy(recorded)
    mount = validated_transform(recorded["baseline_mount"]).copy()
    mount[:3, :3] = np.eye(3)
    geometry["experiment_mount"] = mount.tolist()
    geometry["experiment_mount_pitch_deg"] = 0.0
    geometry["elevation_reference"] = "level sensor axes equal base axes; positive up"
    return geometry


def ray_mount(geometry, *, require_level=False):
    if require_level and "experiment_mount" not in geometry:
        raise ValueError("explicit level experimental mount required; cannot inherit recorded pitch")
    mount = validated_transform(geometry.get("experiment_mount", geometry["baseline_mount"]))
    if require_level and not np.allclose(mount[:3, :3], np.eye(3), atol=1e-12, rtol=0):
        raise ValueError("ray redesign requires zero pitch/roll/yaw mounting")
    if "experiment_mount" in geometry:
        recorded = validated_transform(geometry["baseline_mount"])
        if not np.array_equal(mount[:3, 3], recorded[:3, 3]):
            raise ValueError("leveling must preserve the recorded sensor translation")
    return mount


def layouts(baseline_zenith):
    baseline = -np.rad2deg(np.asarray(baseline_zenith, float))
    if baseline.shape != (32,) or not np.isfinite(baseline).all() or len(np.unique(baseline)) != 32:
        raise ValueError("exactly 32 distinct native elevations required")
    result = {"baseline": baseline.tolist()}
    for top, bottom in ((0, -40.3), (0, -45), (10, -45), (0, -60), (10, -60), (0, -75), (10, -75)):
        result[f"uniform_{top:g}_to_{bottom:g}"] = np.linspace(top, bottom, 32).tolist()
    result["mid_dense_10_to_-60"] = np.r_[
        np.linspace(10, 0, 5), np.linspace(-2, -30, 20), np.linspace(-35, -60, 7)
    ].tolist()
    result["steep_dense_10_to_-60"] = np.r_[
        np.linspace(10, -5, 4), np.linspace(-7, -30, 12), np.linspace(-32, -60, 16)
    ].tolist()
    result["far_dense_10_to_-60"] = np.r_[
        [10, 5, 0], np.linspace(-2, -20, 18), np.linspace(-22, -37, 6), np.linspace(-41, -60, 5)
    ].tolist()
    for value in result.values():
        validate_elevations(value)
    return result


def validate_elevations(value):
    value = np.asarray(value, float)
    if (
        value.shape != (32,)
        or not np.isfinite(value).all()
        or len(np.unique(value)) != 32
        or np.any(np.abs(value) >= 90)
    ):
        raise ValueError("32 distinct finite elevations strictly between -90 and 90 degrees required")
    return value


def hazard_bank(*, shifted=False):
    cases = []
    for radius in (0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0):
        for degrees in np.arange(0, 360, 22.5):
            if shifted:
                effective_radius, degrees = radius + 0.1, degrees + 11.25
            else:
                effective_radius = radius
            angle = np.deg2rad(degrees)
            x, y = effective_radius * np.cos(angle), effective_radius * np.sin(angle)
            cases.append(
                dict(
                    radius_m=effective_radius,
                    bearing_deg=float(degrees),
                    box=([x - 0.05, y - 0.05, 0], [x + 0.05, y + 0.05, 0.08]),
                )
            )
    return cases


def native_pattern_indices(azimuth, zenith, expected_azimuth, elevations):
    """Associate native angle identities to the supplied table, never fit poses.

    Reject foreign/stale tables instead of dropping mismatched samples. Angles
    must preserve the supplied [-pi, pi) convention; no blind frame correction.
    """
    a, z, expected = [np.asarray(v, float) for v in (azimuth, zenith, expected_azimuth)]
    elevation = validate_elevations(elevations)
    if a.ndim != 1 or not len(a) or a.shape != z.shape or not np.isfinite([a, z]).all():
        raise ValueError("finite matching nonempty native angle arrays required")
    if expected.ndim != 1 or not len(expected) or not np.isfinite(expected).all() or np.any(np.diff(expected) <= 0):
        raise ValueError("strictly increasing finite azimuth table required")
    right = np.clip(np.searchsorted(expected, a), 0, len(expected) - 1)
    left = np.maximum(right - 1, 0)
    columns = np.where(np.abs(a - expected[left]) < np.abs(a - expected[right]), left, right)
    difference = np.abs(z[:, None] + np.deg2rad(elevation))
    rings = difference.argmin(axis=1)
    if np.max(np.abs(a - expected[columns])) > 1e-6 or difference[np.arange(len(z)), rings].max() > 1e-6:
        raise ValueError("native angles disagree with supplied pattern; possible stale sensor queue")
    return columns * 32 + rings


def evaluate(contract, elevations):
    elevation = validate_elevations(elevations)
    zenith = -np.deg2rad(elevation)  # native convention is positive down
    attrs = contract["sensor_attributes"]
    azimuth = -np.pi + np.arange(contract["full_columns"]) * np.deg2rad(float(attrs["horizontalResolution"]))
    minimum, maximum = float(attrs["minRange"]), float(attrs["maxRange"])
    origin, direction = world_rays(ray_mount(contract), contract["base_height_m"], azimuth, zenith)
    depth, _ = first_intersections(origin, direction)
    good = valid_returns(depth, minimum, maximum)
    ground = origin + depth[good, None] * direction[good]
    stationary = {str(phase): sampling_metrics(ground, contract["wheel_xy"], phase) for phase in PHASES}
    all_points = []
    # Flat-ground world yaw is a rigid rotation of the stationary hit set.
    # This is equivalent to casting the same off-center sensor at each pose.
    for angle in np.deg2rad(np.arange(0, 361, 15)):
        c, s = np.cos(angle), np.sin(angle)
        all_points.append(ground @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]))
    union = np.concatenate(all_points)
    virtual_yaw = {str(phase): sampling_metrics(union, contract["wheel_xy"], phase) for phase in PHASES}
    box_depth, labels = first_intersections(origin, direction, contract["hazard_boxes"])
    box_good = valid_returns(box_depth, minimum, maximum)
    banks = {}
    for shifted in (False, True):
        bank = []
        for case in hazard_bank(shifted=shifted):
            single_depth, single_label = first_intersections(origin, direction, [case["box"]])
            hits = (single_label == 1) & valid_returns(single_depth, minimum, maximum)
            bank.append({**case, "ray_hits": int(hits.sum())})
        banks["shifted_hazard_bank" if shifted else "hazard_bank"] = bank
    return dict(
        elevations_deg=elevation.tolist(),
        rays_per_revolution=int(len(direction)),
        above_horizon_rings=int((elevation > 0).sum()),
        near_horizon_rings=int((np.abs(elevation) <= 10).sum()),
        plane_rays_below_minimum=int((np.isfinite(depth) & ~good & (depth <= minimum + 1e-5)).sum()),
        plane_rays_usable=int(good.sum()),
        plane_rays_within_5m=int((good & (depth <= 5)).sum()),
        stationary=stationary,
        virtual_yaw360=virtual_yaw,
        hazard_ray_counts=[int((box_good & (labels == i)).sum()) for i in range(1, 9)],
        **banks,
    )
