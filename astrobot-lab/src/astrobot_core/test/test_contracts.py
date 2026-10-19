import math
from astrobot_core.integrity import PoseSample, PoseValidator, rotate_vector
from astrobot_core.evaluation import FixedCoverage, mission_verdict


def pose(t, x=0, q=(0, 0, 0, 1)):
    return PoseSample(t, (x, 0, 0), q)


def test_origin_is_valid_but_teleport_is_not():
    validator = PoseValidator()
    assert validator.validate(pose(1), 1) == ""
    assert validator.validate(pose(1.1, .01), 1.1) == ""
    assert validator.validate(pose(1.2, 20), 1.2) == "kinematic_position"
    assert validator.validate(pose(1.3, .02), 1.3) == ""


def test_bad_measurements_do_not_advance_validator():
    validator = PoseValidator()
    assert validator.validate(pose(1), 1) == ""
    for sample, now, reason in [(pose(1), 1, "non_increasing_stamp"),
                                (pose(2, math.nan), 2, "non_finite"),
                                (pose(2, q=(0, 0, 0, 0)), 2, "quaternion_norm"),
                                (pose(2), 3, "stale"), (pose(4), 2, "future_stamp")]:
        assert validator.validate(sample, now) == reason
        assert validator.previous.stamp == 1
    validator.reset()
    assert validator.validate(pose(.1, 30), .1) == ""


def test_pitch_rotates_vertical_lever_arm():
    q = (0, math.sin(math.pi / 4), 0, math.cos(math.pi / 4))
    rotated = rotate_vector(q, (0, 0, 1))
    assert abs(rotated[0] - 1) < 1e-12
    assert abs(rotated[2]) < 1e-12


def test_fixed_coverage_does_not_shrink_to_reachable_island():
    grid = FixedCoverage(0, 0, 8, 2)
    for x, y in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
        grid.observe(x, y)
    assert len(grid.cells) == 52
    assert grid.fraction == 4 / 52
    grid.observe(math.nan, 0)
    grid.observe(100, 100)
    assert grid.fraction == 4 / 52


def test_verdict_failure_reasons_always_fail():
    result = mission_verdict("explore_radius", {"state": "finished", "return_reason": "slam_diverged"}, 0, .8, .8)
    assert not result["pass"] and "slam_diverged" in result["reasons"]
    failed_return = mission_verdict("explore_radius", {"state": "failed", "returning_home": True}, 5, .8, .8)
    assert "false_success" not in failed_return["reasons"]
    invalid = mission_verdict("explore_radius", {"state": "finished"}, 0, .8, .8, reference_valid=False)
    assert not invalid["pass"] and invalid["validity"] == "infrastructure_invalid"
