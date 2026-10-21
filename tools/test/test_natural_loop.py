"""Host contracts for the separately declared 16 m GT-controlled acquisition."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from natural_motion import route, route_hash, motion_command, survey_points, leg_deadline, HALF


def pose(x=0., y=0., yaw=0.):
    p = np.eye(4)
    p[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    p[:2, 3] = [x, y]
    return p


def test_loop_has_four_forward_legs_four_turns_and_same_endpoint_heading():
    spec = route("loop16")
    assert spec["route_length_m"] == 16.
    xy = np.array([[0., 0.], *[t[:2] for t in spec["targets"]]])
    assert np.linalg.norm(np.diff(xy, axis=0), axis=1).sum() == 16.
    assert [t[3] for t in spec["targets"]] == ["translate", "rotate"] * 4
    assert spec["targets"][-1] == (0., 0., 0., "rotate")
    assert route_hash("loop16") != route_hash("straight_slow")
    assert spec["speed"] == .1 and spec["max_wall_s"] == 1800
    assert [leg_deadline("loop16", i) for i in range(8)] == [85., 75.] * 4
    assert leg_deadline("straight_slow", 0) == 65.


@pytest.mark.parametrize("origin_yaw", [0., .7, -2.])
def test_all_segments_work_in_initial_planar_frame_and_close_loop(origin_yaw):
    origin = pose(20., 18., origin_yaw)
    previous = np.zeros(2)
    for i, (x, y, heading, action) in enumerate(route("loop16")["targets"]):
        at = origin @ pose(x, y, heading)
        assert motion_command(origin, at, "loop16", i)[2]
        if action == "translate":
            start = origin @ pose(*previous, heading)
            linear, angular, arrived = motion_command(origin, start, "loop16", i)
            assert linear == .1 and abs(angular) < 1e-12 and not arrived
        else:
            start = origin @ pose(x, y, heading + math.pi/2)
            linear, angular, arrived = motion_command(origin, start, "loop16", i)
            assert linear == .03 and angular == -.1 and not arrived
        previous = np.array([x, y])


@pytest.mark.parametrize("index,current", [
    (0, pose(0, .081)), (0, pose(-.101, 0)), (0, pose(2.101, 0)),
    (2, pose(2.081, -3, -math.pi/2)), (2, pose(2, -6.101, -math.pi/2)),
    (4, pose(1, -6.081, -math.pi)), (6, pose(-.081, -3, math.pi/2)),
    (1, pose(2.251, 0)), (3, pose(2, -6.251)), (7, pose(.251, 0)),
])
def test_every_leg_and_corner_rejects_envelope_escape(index, current):
    with pytest.raises(ValueError, match="envelope"):
        motion_command(np.eye(4), current, "loop16", index)


@pytest.mark.parametrize("yaw", [0., .3, -1.2])
def test_survey_covers_full_rotation_footprint_with_entire_skid_envelope(yaw):
    origin = pose(20, 18, yaw)
    points, _ = survey_points(origin, "loop16")
    lo, hi = points.min(0), points.max(0)
    for x, y, _, action in route("loop16")["targets"]:
        if action != "rotate": continue
        center = (origin @ pose(x, y))[:2, 3]
        for angle in np.linspace(-math.pi, math.pi, 31):
            r = Rotation.from_euler("z", angle).as_matrix()[:2, :2]
            for drift in np.linspace(-math.pi, math.pi, 31):
                xy = center + .25*np.array([math.cos(drift), math.sin(drift)])
                for dx in (-HALF[0], HALF[0]):
                    for dy in (-HALF[1], HALF[1]):
                        corner = xy + r @ [dx, dy]
                        assert np.all(corner >= lo) and np.all(corner <= hi)


def test_ideal_controller_completes_without_unbounded_or_reverse_legs():
    origin, current = np.eye(4), np.eye(4)
    yaw = 0.
    for i in range(8):
        anchor = current.copy() if i % 2 == 0 else None
        for _ in range(3000):
            linear, angular, arrived = motion_command(origin, current, "loop16", i, segment_start=anchor)
            if arrived: break
            assert abs(linear) <= .1 and abs(angular) <= .1
            current[:2, 3] += linear * .05 * np.array([math.cos(yaw), math.sin(yaw)])
            yaw += angular * .05
            current[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
        else:
            pytest.fail("ideal loop failed to converge")
        target = route("loop16")["targets"][i][:2]
        # Translations arrive within 2 cm; turns retain their declared 25 cm
        # radial bound, including the last turn. Never shift that bound.
        assert np.linalg.norm(current[:2, 3]-target) < (.02 if i % 2 == 0 else .25)
    assert np.linalg.norm(current[:2, 3]) < .25
    assert abs(math.atan2(current[1, 0], current[0, 0])) < .012


def test_recorded_first_corner_skid_is_legal_turn_but_not_legal_next_leg():
    # Seed-43 loop16 attempt, Sep 21: preserve this integration failure rather
    # than widening the straight-leg envelope to force route completion.
    current = pose(1.79952012, .13581530, math.radians(-89.4646352064))
    assert motion_command(np.eye(4), current, "loop16", 1)[2]
    with pytest.raises(ValueError, match="translation exceeded"):
        motion_command(np.eye(4), current, "loop16", 2)


def test_recorded_second_corner_escape_still_stops_and_earlier_feedback_opposes_drift():
    # Sep 27 selected-sensor attempt: retain its 25.0058 cm failure unchanged.
    with pytest.raises(ValueError, match="rotation skid"):
        motion_command(np.eye(4), pose(1.77117422, -6.10083543, -2.59067372), "loop16", 3)
    current = pose(1.78, -6.10, -2.59)
    linear, angular, arrived = motion_command(np.eye(4), current, "loop16", 3)
    assert linear == -.03 and angular == -.1 and not arrived
    correction = linear * current[:2, 0]
    assert correction @ (np.array([2., -6.]) - current[:2, 3]) > 0


def test_corner_feedback_reduces_drift_without_loosening_envelope():
    current = pose(2., -6., -math.pi/2)
    angle = -math.pi/2
    maximum = 0.
    for _ in range(2000):
        linear, angular, arrived = motion_command(np.eye(4), current, "loop16", 3)
        if arrived:
            break
        assert abs(linear) <= .03 and abs(angular) <= .1
        # Deterministic contact-disturbance fixture, not a vehicle dynamics claim.
        current[:2, 3] += .05 * (linear * np.array([math.cos(angle), math.sin(angle)]) + [-.01, -.004])
        angle += angular * .05
        current[:3, :3] = Rotation.from_euler("z", angle).as_matrix()
        maximum = max(maximum, np.linalg.norm(current[:2, 3] - [2., -6.]))
    else:
        pytest.fail("bounded-feedback synthetic turn did not converge")
    assert maximum < .15
    assert route("loop16")["rotation_drift_limit_m"] == .25


def test_recorded_turn_must_match_declared_feedback_not_just_general_speed_limit():
    from natural_motion import validate_loop_turn_feedback
    current = pose(1.78, -6.1, -2.59)
    validate_loop_turn_feedback(np.eye(4), current, 3, -.03, -.1)
    for linear, angular in [(-.031, -.1), (0., -.1), (-.03, .1)]:
        with pytest.raises(ValueError, match="feedback policy"):
            validate_loop_turn_feedback(np.eye(4), current, 3, linear, angular)


@pytest.mark.parametrize("offset,drift", [(.02, -.004), (.06, -.008), (.20, .011), (.25, .015)])
def test_anticipation_reserves_room_for_lateral_turn_slip(offset, drift):
    # Simplified body-twist model bracketed by the two retained failed runs.
    # Lateral velocity is omega*offset; offset is an effective contact ICR,
    # not a measured static TF. This is controller regression, not Isaac truth.
    current = pose(2., 0.)
    angle, maximum = 0., 0.
    for _ in range(3000):
        linear, angular, arrived = motion_command(np.eye(4), current, "loop16", 1)
        if arrived:
            break
        omega = .5*angular
        direction = np.array([math.cos(angle), math.sin(angle)])
        lateral = np.array([-direction[1], direction[0]])
        current[:2, 3] += .025*((linear+drift)*direction + omega*offset*lateral)
        angle += .025*omega
        current[:3, :3] = Rotation.from_euler("z", angle).as_matrix()
        maximum = max(maximum, np.linalg.norm(current[:2, 3]-[2., 0.]))
    else:
        pytest.fail("anticipatory turn did not finish inside the existing deadline")
    assert maximum < .22
    assert route("loop16")["turn_max_linear_speed"] == .03
    assert route("loop16")["rotation_drift_limit_m"] == .25


def test_recorded_corner_connects_to_unchanged_target_with_frozen_actual_start():
    anchor = pose(1.79952012, .13581530, math.radians(-89.4646352064))
    current = anchor.copy()
    yaw = math.atan2(current[1, 0], current[0, 0])
    for _ in range(4000):
        linear, angular, arrived = motion_command(np.eye(4), current, "loop16", 2, segment_start=anchor)
        if arrived: break
        current[:2, 3] += linear*.025*np.array([math.cos(yaw), math.sin(yaw)])
        yaw += angular*.025
        current[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    else: pytest.fail("recorded corner connector did not converge")
    assert np.linalg.norm(current[:2, 3]-[2., -6.]) < .02


def test_connector_keeps_eight_cm_cross_track_and_original_clearance_envelope():
    anchor = pose(1.8, .13, -math.pi/2)
    target = np.array([2., -6.]); direction = target-anchor[:2, 3]; direction /= np.linalg.norm(direction)
    midpoint = (target+anchor[:2, 3])/2
    bad = midpoint + .081*np.array([-direction[1], direction[0]])
    with pytest.raises(ValueError, match="translation exceeded"):
        motion_command(np.eye(4), pose(*bad, math.atan2(direction[1], direction[0])), "loop16", 2, segment_start=anchor)
    with pytest.raises(ValueError, match="corner envelope"):
        motion_command(np.eye(4), anchor, "loop16", 2, segment_start=pose(1.7, 0))
    with pytest.raises(ValueError, match="surveyed center"):
        motion_command(np.eye(4), pose(-.251, -3, math.pi/2), "loop16", 6, segment_start=pose(-.24, -6))


def test_connection_log_rejects_reanchoring_and_fabricated_start():
    from natural_motion import validate_loop_connection_log
    import copy
    anchor = pose().tolist()
    rows = [dict(index=0, segment_start=anchor, world_from_base=anchor, arrived_segment=None),
            dict(index=0, segment_start=anchor, world_from_base=pose(.1).tolist(), arrived_segment=None)]
    assert 0 in validate_loop_connection_log(rows)
    bad = copy.deepcopy(rows); bad[1]["segment_start"] = pose(.1).tolist()
    with pytest.raises(ValueError, match="changed"):
        validate_loop_connection_log(bad)
    bad = copy.deepcopy(rows); bad[0]["segment_start"] = pose(.1).tolist()
    with pytest.raises(ValueError, match="actual first"):
        validate_loop_connection_log(bad)


# Waypoint loops with a corridor survey (transit50, approach_field).
from natural_motion import (WAYPOINTS, polyline_distance, route_vertices, survey_mask,  # noqa: E402
                            validate_ground_tile, validate_loop_connection_log, ROTATION_RADIUS, GRID,
                            MAX_WAYPOINT_TURN)


@pytest.mark.parametrize("pattern", list(WAYPOINTS))
def test_waypoint_loop_alternates_rotate_translate_and_closes_at_home(pattern):
    spec = route(pattern)
    actions = [t[3] for t in spec["targets"]]
    assert actions[0] == actions[-1] == "rotate"
    allowed = {("rotate", "translate"), ("translate", "rotate"), ("translate", "reverse"), ("reverse", "rotate")}
    assert set(zip(actions, actions[1:])) <= allowed
    assert spec["targets"][-1] == (0., 0., 0., "rotate") and spec["targets"][-2][:2] == (0., 0.)
    vertices = route_vertices(spec)
    assert np.allclose(vertices[0], 0) and np.allclose(vertices[-1], 0)
    assert np.isclose(np.linalg.norm(np.diff(vertices, axis=0), axis=1).sum(), spec["route_length_m"], atol=1e-3)
    for i, (x, y, heading, action) in enumerate(spec["targets"]):
        px, py = spec["targets"][i - 1][:2]
        if action == "translate":  # leg heading points from the previous vertex to this one
            assert np.isclose(heading, math.atan2(y - py, x - px))
        if action == "reverse":  # driven backwards: the base faces away from its travel
            assert np.isclose(abs(math.remainder(heading - math.atan2(y - py, x - px), math.tau)), math.pi)
            assert heading == spec["targets"][i - 1][2]
    for i, (_, _, heading, action) in enumerate(spec["targets"]):
        if action == "rotate":  # in-place turns past MAX_WAYPOINT_TURN leave the skid bound
            previous = spec["targets"][i - 1][2] if i else 0.
            assert abs(math.remainder(heading - previous, math.tau)) <= MAX_WAYPOINT_TURN
    assert route_hash(pattern) != route_hash("loop16")


def test_waypoint_loop_lengths_and_speeds():
    assert route("transit50")["route_length_m"] > 49 and route("transit50")["speed"] == .18
    assert np.isclose(route("approach_field")["route_length_m"], 19.0)
    assert route("approach_field")["survey_exempt_prefixes"] == ["/DemoApproachRocks/"]
    assert "survey_exempt_prefixes" not in route("transit50")
    assert "survey_region" not in route("loop16")  # legacy loop keeps the bounding-box survey


@pytest.mark.parametrize("pattern", list(WAYPOINTS))
@pytest.mark.parametrize("origin_yaw", [0., .7])
def test_corridor_mask_covers_every_turn_and_leg_footprint(pattern, origin_yaw):
    origin = pose(20., 18., origin_yaw)
    points, shape = survey_points(origin, pattern)
    mask = survey_mask(origin, pattern, points)
    assert mask.shape == (len(points),) and 0 < mask.sum() < len(points)
    covered = points[mask]
    spec = route(pattern)
    rotation = origin[:2, :2]
    for x, y, heading, action in spec["targets"]:
        center = origin[:2, 3] + rotation @ [x, y]
        for angle in np.linspace(-math.pi, math.pi, 13):
            r = Rotation.from_euler("z", angle).as_matrix()[:2, :2]
            for drift in np.linspace(-math.pi, math.pi, 9):
                xy = center + .25 * np.array([math.cos(drift), math.sin(drift)])
                for dx in (-HALF[0], HALF[0]):
                    for dy in (-HALF[1], HALF[1]):
                        corner = xy + r @ [dx, dy]
                        assert np.min(np.linalg.norm(covered - corner, axis=1)) <= GRID * math.sqrt(2) / 2 + 1e-9
    assert survey_mask(origin, "loop16", points) is None


def test_corridor_validation_matches_full_tile_and_ignores_unsurveyed_cells():
    xy = np.array([(x, y) for x in np.arange(25) * .05 for y in np.arange(25) * .05])
    slope = np.array([.2, -.1])
    z = xy @ slope
    n = np.array([-slope[0], -slope[1], 1.])
    normals = np.tile(n / np.linalg.norm(n), (len(xy), 1))
    full = validate_ground_tile(xy, z, (25, 25), normals)
    masked = validate_ground_tile(xy, z, (25, 25), normals, np.ones(len(xy), bool))
    for key in ("maximum_patch_incline", "maximum_detrended_roughness_m", "maximum_relative_discontinuity_m"):
        assert np.isclose(full[key], masked[key], atol=1e-9)
    mask = np.linalg.norm(xy - .6, axis=1) < .4
    z_out = z.copy()
    z_out[~mask] = np.nan  # never surveyed: not evaluated, not an error
    normals_out = normals.copy()
    normals_out[~mask] = np.nan
    assert validate_ground_tile(xy, z_out, (25, 25), normals_out, mask)["samples"] == mask.sum()
    step = z_out.copy()
    step[np.argmin(np.linalg.norm(xy - .6, axis=1))] += .03
    # Corridor floor limits are recorded, not gated (sim acquisition convenience).
    assert "relative_discontinuity" in validate_ground_tile(xy, step, (25, 25), normals_out, mask)["floor_limit_violations"]
    with pytest.raises(ValueError, match="invalid floor tile"):
        bad = z_out.copy()
        bad[np.flatnonzero(mask)[0]] = np.nan
        validate_ground_tile(xy, bad, (25, 25), normals_out, mask)


def test_corridor_route_envelope_is_the_polyline_not_its_bounding_box():
    spec = route("transit50")
    vertices = route_vertices(spec)
    # A corner of the vertices' bounding box: admitted by the box rule, far from every leg.
    interior = np.array([vertices[:, 0].min(), vertices[:, 1].max()])
    assert polyline_distance(vertices, interior)[0] > 2
    leg = next(i for i, t in enumerate(spec["targets"]) if t[3] == "translate")
    x, y, heading, _ = spec["targets"][leg]
    with pytest.raises(ValueError, match="envelope"):
        motion_command(np.eye(4), pose(*interior, heading), "transit50", leg)
    mid = pose(x / 2, y / 2, heading)
    linear, _, arrived = motion_command(np.eye(4), mid, "transit50", leg)
    assert linear == .18 and not arrived


def test_reverse_leg_drives_backwards_home_on_the_outbound_heading():
    spec = route("approach_field")
    leg = next(i for i, t in enumerate(spec["targets"]) if t[3] == "reverse")
    (px, py, outbound, _), (x, y, heading, _) = spec["targets"][leg - 1], spec["targets"][leg]
    assert (x, y) == (0., 0.) and heading == outbound
    start = pose(px, py, outbound)
    linear, angular, arrived = motion_command(np.eye(4), pose(px / 2, py / 2, outbound), "approach_field", leg,
                                              segment_start=start)
    assert linear == -spec["speed"] and abs(angular) < 1e-9 and not arrived
    # Lateral offset to the left of the leg: the rear must swing back toward the endpoint.
    left = np.array([-math.sin(outbound), math.cos(outbound)])
    _, angular, _ = motion_command(np.eye(4), pose(*(np.array([px, py]) / 2 + .05 * left), outbound),
                                   "approach_field", leg, segment_start=start)
    assert angular > 0
    with pytest.raises(ValueError, match="heading"):
        motion_command(np.eye(4), pose(px / 2, py / 2, outbound + math.pi), "approach_field", leg, segment_start=start)


@pytest.mark.parametrize("lateral", [-.035, .035])
def test_waypoint_leg_holds_its_heading_near_the_endpoint_and_arrives_on_distance(lateral):
    """Capture 569ba2de: 3 cm lateral, 1.5 cm to go; steering at the endpoint bearing swung the
    heading past the 0.15 rad bound. v3 holds the leg heading for the last 0.30 m."""
    spec = route("approach_field")
    leg = next(i for i, t in enumerate(spec["targets"]) if t[3] == "reverse")
    px, py, heading, _ = spec["targets"][leg - 1]
    start = pose(px, py, heading)
    side = np.array([-math.sin(heading), math.cos(heading)])
    along = -np.array([math.cos(heading), math.sin(heading)])  # travel direction of the reverse leg
    # Unicycle reversing home from 0.5 m out with a lateral offset: no heading excursion.
    current, worst = pose(*(.5 * -along + lateral * side), heading), 0.
    for _ in range(2000):
        linear, angular, arrived = motion_command(np.eye(4), current, "approach_field", leg, segment_start=start)
        if arrived:
            break
        psi = math.atan2(current[1, 0], current[0, 0])
        current = pose(*(current[:2, 3] + .1 * linear * np.array([math.cos(psi), math.sin(psi)])), psi + .1 * angular)
        worst = max(worst, abs(math.remainder(psi - heading, math.tau)))
    assert arrived and worst < .10  # bound: translation_heading_limit_rad 0.15
    assert abs(float(current[:2, 3] @ along)) < .02 and abs(float(current[:2, 3] @ side)) > .01
    assert route("loop16")["translation_policy"] == "frozen_post_turn_connection_v2"  # unchanged


def test_waypoint_connection_log_uses_declared_actions():
    spec = route("approach_field")
    first = np.eye(4).tolist()
    rows = [dict(index=0, segment_start=None, world_from_base=first),
            dict(index=1, segment_start=first, world_from_base=first)]
    assert 1 in validate_loop_connection_log(rows, "approach_field")
    assert spec["targets"][1][3] == "translate"
    with pytest.raises(ValueError, match="turn cannot"):
        validate_loop_connection_log([dict(index=0, segment_start=first, world_from_base=first)], "approach_field")


@pytest.mark.parametrize("pattern", list(WAYPOINTS))
def test_corridor_boxes_cover_the_corridor_with_small_overshoot(pattern):
    from natural_motion import corridor_boxes

    origin = pose(20., 18., .4)
    boxes = corridor_boxes(origin, pattern)
    points, _ = survey_points(origin, pattern)
    spec = route(pattern)
    vertices = origin[:2, 3] + route_vertices(spec) @ origin[:2, :2].T
    dist = polyline_distance(vertices, points)
    inside = np.zeros(len(points), bool)
    corners = []
    for center, half, angle in boxes:
        c, s_ = math.cos(angle), math.sin(angle)
        d = points - center
        u, v = d @ [c, s_], d @ [-s_, c]
        inside |= (abs(u) <= half[0] + 1e-9) & (abs(v) <= half[1] + 1e-9)
        corners += [center + a * half[0] * np.array([c, s_]) + b * half[1] * np.array([-s_, c])
                    for a in (-1, 1) for b in (-1, 1)]
    assert inside[dist <= ROTATION_RADIUS].all()
    assert polyline_distance(vertices, np.array(corners)).max() <= ROTATION_RADIUS + .06


def test_demo_clearance_rejects_a_rock_inside_the_guarded_envelope():
    from natural_motion import validate_demo_clearance

    origin = pose(0., 0., 0.)
    far = route_vertices(route("approach_field"))[1]
    mid = far / 2
    normal = np.array([-far[1], far[0]]) / np.linalg.norm(far)
    ok = dict(path="/DemoApproachRocks/rock_0", points=[[*(mid + 1.0 * normal), 0.]])
    assert validate_demo_clearance(origin, "approach_field", [ok])["objects"][0]["route_distance_m"] > .9
    bad = dict(path="/DemoApproachRocks/rock_1", points=[[*(mid + .8 * normal), 0.]])
    with pytest.raises(ValueError, match="guarded route envelope"):
        validate_demo_clearance(origin, "approach_field", [bad])


def test_every_waypoint_loop_declares_acquisition_limits():
    from selected_lidar_profile import EXTENDED_PATTERNS, WAYPOINT_LIMITS, acquisition_limits

    assert set(WAYPOINT_LIMITS) == set(WAYPOINTS) and set(WAYPOINTS) <= set(EXTENDED_PATTERNS)
    for pattern in WAYPOINTS:
        spec = route(pattern)
        turns = sum(abs(math.remainder(b[2] - a[2], 2 * math.pi)) for a, b in zip(spec["targets"], spec["targets"][1:]))
        nominal = spec["route_length_m"] / spec["speed"] + turns / spec["angular_speed"] + len(spec["targets"]) * 12
        assert acquisition_limits(pattern)["max_duration_s"] > 1.3 * nominal


@pytest.mark.parametrize("pattern", list(WAYPOINTS))
@pytest.mark.parametrize("slip", [0., .3])
def test_waypoint_loop_completes_in_closed_loop_within_envelopes_and_deadlines(pattern, slip):
    """Unicycle plant with lossy turning (slip) and lateral skid; the GT command law only."""
    spec = route(pattern)
    origin = pose(20., 18., -.0015)
    current, index, dt, t, leg_t, settled = origin.copy(), 0, .1, 0., 0., None
    segment = current.copy() if spec["targets"][0][3] != "rotate" else None
    while index < len(spec["targets"]):
        linear, angular, arrived = motion_command(origin, current, pattern, index, segment_start=segment)
        assert abs(linear) <= spec["speed"] + 1e-12 and abs(angular) <= spec["angular_speed"] + 1e-12
        if arrived:
            settled = t if settled is None else settled
            if t - settled >= spec["settle_s"]:
                index, leg_t, settled = index + 1, 0., None
                segment = (current.copy() if index < len(spec["targets"]) and spec["targets"][index][3] != "rotate"
                           else None)
                continue
            linear = angular = 0.
        else:
            settled = None
        heading = math.atan2(current[1, 0], current[0, 0])
        skid = -.02 * abs(angular) if slip else 0.
        body = np.array([linear, skid])
        c, s_ = math.cos(heading), math.sin(heading)
        step = pose(*(current[:2, 3] + np.array([[c, -s_], [s_, c]]) @ body * dt), heading + angular * (1 - slip) * dt)
        current = step
        t, leg_t = t + dt, leg_t + dt
        assert leg_t <= leg_deadline(pattern, index), f"leg {index} deadline"
    # The corner law drives toward a reference ahead of the turn (anticipatory skid
    # compensation, as loop16), so an ideal plant ends within the turn envelope, not on it.
    assert np.linalg.norm(current[:2, 3] - origin[:2, 3]) < spec["rotation_drift_limit_m"]



def test_corridor_floor_is_recorded_not_gated():
    xy = np.array([(x, y) for x in np.arange(25)*.05 for y in np.arange(25)*.05])
    z = .04*np.sin(2*np.pi*xy[:, 0]/.7) + .5*xy[:, 1]  # 4 cm waves on a 0.5 incline
    slope = .04*2*np.pi/.7*np.cos(2*np.pi*xy[:, 0]/.7)
    normals = np.column_stack((-slope, -.5*np.ones(len(xy)), np.ones(len(xy))))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    result = validate_ground_tile(xy, z, (25, 25), normals, np.ones(len(xy), bool))
    assert result["floor_gate"] == "recorded_only_v1"
    assert {"absolute_incline", "patch_incline", "detrended_roughness"} <= set(result["floor_limit_violations"])
    with pytest.raises(ValueError, match="incline"):  # the legacy box survey still gates
        validate_ground_tile(xy, z, (25, 25), normals)
