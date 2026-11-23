"""Turning prior cannot manufacture an exit corridor or defeat known hazards."""

import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from shapely import Polygon, Point

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.contact_runtime import startup_footprint
from local_terrain.contact_advisory import ContactAdvisory
from natural_motion import motion_command, route, wrap
from test_contact_advisory import fixture
from assess_startup_observation import exit_candidates, floor_bands
from render_startup_observation import render
from selected_lidar_profile import acquisition_limits
from diagnose_startup_visibility import screen
from local_terrain.ray_design import selected_setup


def turn(degrees):
    value = np.eye(4)
    value[:3, :3] = Rotation.from_euler("z", degrees, degrees=True).as_matrix()
    return value


def test_turn_area_uses_physical_extent_not_inflated_guard_or_skid_allowance():
    polygon = Polygon(startup_footprint(True))
    radius = np.hypot(0.5037, 0.3454)
    assert radius == pytest.approx(0.6107494167004992, abs=1e-12)
    assert np.max(np.linalg.norm(startup_footprint(True), axis=1)) <= radius + 1e-15
    assert polygon.contains(Point(0.6, 0))
    assert not polygon.covers(Point(0.62, 0))
    assert not polygon.covers(Point(0.7, 0))
    with pytest.raises(ValueError):
        startup_footprint(1)


def test_full_turn_supported_but_unknown_exit_not_and_map_not_painted():
    original, layers, record, timeline, base = fixture()
    advisory = ContactAdvisory(original.patches, startup_footprint(True), original.body)
    before = layers.copy()
    advisory.update(layers, record, timeline, base, 200_000_000, grounded=True, deployment_confirmed=True)
    for degrees in range(0, 360, 15):
        # Continuous small arcs, not one 360-degree identity matrix.
        assert (
            advisory.query([np.eye(4), turn(degrees), turn(degrees + 5)], query_ns=200_000_000)["state"] == "available"
        )
    exit_pose = np.eye(4)
    exit_pose[0, 3] = 0.5
    assert advisory.query([np.eye(4), exit_pose], query_ns=200_000_000)["state"] == "unknown"
    assert len(advisory.memory.anchors) == 1
    np.testing.assert_array_equal(layers, before)
    layers[29:32, 29:32, 0] = 100
    advisory.update(layers, record, timeline, base, 200_000_000, grounded=True)
    assert advisory.query([np.eye(4), turn(5)], query_ns=200_000_000)["state"] == "blocked"


def test_full_observation_turn_is_eight_positive_arcs_not_identity_shortcut():
    spec = route("turn360")
    headings = [t[1] for t in spec["targets"][1:9]]
    assert len(headings) == 8 and headings[-1] == 0
    assert sum(wrap(b - a) for a, b in zip([0] + headings[:-1], headings)) == pytest.approx(2 * np.pi)
    previous = 0.0
    for index, target in enumerate(headings, 1):
        linear, angular, arrived = motion_command(np.eye(4), turn(np.degrees(previous)), "turn360", index)
        assert not arrived and angular > 0 and abs(linear) <= 0.03
        assert motion_command(np.eye(4), turn(np.degrees(target)), "turn360", index)[2]
        previous = target


def test_observation_micro_correction_is_bounded_and_keeps_original_origin_guard():
    current = np.eye(4)
    current[0, 3] = 0.1
    linear, angular, _ = motion_command(np.eye(4), current, "turn360", 1)
    assert linear == 0.03 and angular == 0.1
    current[0, 3] = 0.251
    with pytest.raises(ValueError, match="25 cm"):
        motion_command(np.eye(4), current, "turn360", 1)


def test_observation_has_bounded_preposition_and_return_to_original_origin():
    assert route("turn360")["targets"][0] == (0.15, 0.0)
    assert route("turn360")["targets"][-1] == (0.0, 0.0)
    assert motion_command(np.eye(4), np.eye(4), "turn360", 0) == (0.03, 0.0, False)
    assert motion_command(np.eye(4), np.eye(4), "turn360", 9)[2]
    assert acquisition_limits("turn360") == dict(max_duration_s=420, max_native_frames=26000)
    assert acquisition_limits("turn15") == dict(max_duration_s=200, max_native_frames=20000)


def test_reverse_peek_is_only_fifteen_centimetres_with_fixed_origin_and_dwell():
    spec = route("peek_reverse")
    assert spec["targets"] == [(-0.15, 0.0), (0.0, 0.0)] and spec["settle_s"] == 3
    assert motion_command(np.eye(4), np.eye(4), "peek_reverse", 0) == (-0.03, 0.0, False)
    p = np.eye(4)
    p[0, 3] = -0.15
    assert motion_command(np.eye(4), p, "peek_reverse", 0)[2]
    assert motion_command(np.eye(4), p, "peek_reverse", 1) == (0.03, 0.0, False)
    for x in (-0.171, 0.021):
        p[0, 3] = x
        with pytest.raises(ValueError, match="envelope"):
            motion_command(np.eye(4), p, "peek_reverse", 0)


def test_twenty_cm_peek_has_precise_arrival_not_a_silent_old_route_change():
    spec = route("peek_reverse20")
    assert spec["targets"] == [(-0.2, 0.0), (0.0, 0.0)]
    p = np.eye(4)
    p[0, 3] = -0.181
    assert not motion_command(np.eye(4), p, "peek_reverse20", 0)[2]
    p[0, 3] = -0.196
    assert motion_command(np.eye(4), p, "peek_reverse20", 0)[2]
    p[0, 3] = -0.221
    with pytest.raises(ValueError, match="envelope"):
        motion_command(np.eye(4), p, "peek_reverse20", 0)
    assert "translation_arrival_tolerance_m" not in route("peek_reverse")


def test_ideal_visibility_keeps_near_chassis_occlusion_before_range_filter():
    setup, elevations = selected_setup(Path(__file__).parents[1] / "local_terrain/selected_lidar_optical.json")
    parked = screen(setup, elevations)
    back = screen(setup, elevations, base_x=-0.2)
    assert parked["rays"] == back["rays"] == 32 * 1999
    assert parked["chassis_first_hits"] == back["chassis_first_hits"] > 10000
    assert parked["ground_returns_inside_startup_circle"] == 0
    assert back["ground_returns_inside_startup_circle"] > 0
    assert parked["first_front_wheel_strip_ground_x_m"] - back["first_front_wheel_strip_ground_x_m"] == pytest.approx(
        0.2
    )


def test_centering_pauses_turn_before_skid_guard_without_reanchoring():
    p = turn(160)
    p[:2, 3] = [-0.14, 0.205]
    v, w, arrived = motion_command(np.eye(4), p, "turn360", 4)
    assert v == -0.03 and w == 0 and not arrived
    # Correct back inside the centre envelope, then resume the same yaw target.
    p[:2, 3] *= 0.6
    assert motion_command(np.eye(4), p, "turn360", 4)[1] > 0
    p[:2, 3] = [-0.15, 0.21]
    with pytest.raises(ValueError, match="25 cm"):
        motion_command(np.eye(4), p, "turn360", 4)


def test_observation_controller_on_failed_run_identified_slip_model():
    # Failure-informed approximate simulator model, NOT physical qualification.
    # Tests the controller logic before spending another Isaac capture.
    p = np.eye(4)
    p[0, 3] = 0.15
    angle = 0.0
    maximum = 0.0
    for leg in range(1, 9):
        arrived = False
        for _ in range(2400):
            v, w, arrived = motion_command(np.eye(4), p, "turn360", leg)
            if arrived:
                break
            omega = 0.51 * w
            velocity = np.array([0.158 * v - 0.085 * omega, -0.080 * v + 0.162 * omega])
            p[:2, 3] += p[:2, :2] @ velocity / 60
            angle += omega / 60
            p[:3, :3] = turn(np.degrees(angle))[:3, :3]
            maximum = max(maximum, np.linalg.norm(p[:2, 3]))
        assert arrived
    assert maximum < 0.25


def test_exit_candidates_cannot_count_stationary_or_turn_as_progress():
    motions = exit_candidates()
    assert len(motions) == 20
    for poses in motions.values():
        np.testing.assert_array_equal(poses[0], np.eye(4))
        assert 0.1 <= abs(poses[-1][0, 3]) <= 1
        np.testing.assert_array_equal(poses[-1][:3, :3], np.eye(3))


def test_floor_bands_keep_fixed_unknown_and_outside_denominators():
    layers = np.zeros((40, 40, 6))
    layers[:, :, 0] = -1
    points = np.array([[0.1, 0, 0], [0.7, 0, 0], [1.1, 0, 0]])
    record = dict(local_from_map=np.eye(4), origin=[-2, -2], resolution=0.1)
    before = floor_bands(layers, record, points, np.zeros(3), np.zeros(2))
    layers[:, :, 0] = 0
    after = floor_bands(layers, record, points, np.zeros(3), np.zeros(2))
    record["origin"] = [10, 10]
    outside = floor_bands(layers, record, points, np.zeros(3), np.zeros(2))
    for name in before:
        assert before[name]["samples"] == after[name]["samples"] == outside[name]["samples"] == 1
        assert before[name]["unknown"] == after[name]["free"] == outside[name]["outside"] == 1


def test_grid_report_labels_assumptions_and_does_not_hide_unavailable_exit():
    report = dict(
        examples=[
            dict(
                local_from_base=np.eye(4).tolist(),
                resolution=0.1,
                origin=[0, 0],
                classes=[[-1, 0, 100]],
                floor={"outside_startup_to_1m": dict(free=1, samples=3)},
                label="Final <map>",
                elapsed_s=10,
            )
        ],
        final_decisions={"forward_10cm": dict(supported_exit=False)},
        decisions_with_supported_exit=0,
        decisions=11,
        maps=101,
        physical_motion=dict(net_yaw_degrees=359, max_origin_displacement_m=0.2),
    )
    result = render(report)
    assert "0 / 11" in result and "<b>none</b>" in result
    assert "GT-controlled acquisition" in result and "33.3%" in result
    assert "Final &lt;map&gt;" in result and "#cbd5e1" in result and "#c84a44" in result
