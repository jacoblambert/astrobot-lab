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
