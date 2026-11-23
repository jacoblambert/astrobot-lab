from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assess_navigation_bottleneck import execution_accounting, support_area
from local_terrain.physx_reference import BodyBox


def row(stamp, linear=0.0, angular=0.0, state="checked_proposal", requested_linear=0.0):
    return dict(
        stamp=stamp,
        linear=linear,
        angular=angular,
        execution_state=state,
        world_from_base=np.eye(4).tolist(),
        proposal_wall_age_s=0.6,
        proposal=dict(linear=requested_linear, angular=0.0, query_ns=0),
    )


def test_time_weighting_stale_translation_and_terminal_tick_are_separate():
    rows = [
        row(0, linear=0.02),
        row(1, state="waiting_for_fresh_proposal", requested_linear=0.02),
        row(3, angular=-0.04),
        row(4),
    ]
    result = execution_accounting(rows, [1, 1])
    assert result["execution_ticks"] == 4 and result["interval_count"] == 3
    assert result["seconds"] == dict(forward=1, stale_wait=2, turn=1, hold=0)
    assert sum(result["seconds"].values()) == result["logged_duration_s"] == 4
    assert result["stale_requested_seconds"] == dict(forward=2)
    assert result["opposite_gt_goal_turn_s"] == 1
    assert not result["last_tick_duration_extrapolated"]


def test_gt_bearing_check_does_not_call_every_turn_wrong():
    assert execution_accounting([row(0, angular=0.04), row(1)], [1, 1])["opposite_gt_goal_turn_s"] == 0
    assert execution_accounting([row(0, angular=-0.04), row(1)], [1, 0])["opposite_gt_goal_turn_s"] == 0


@pytest.mark.parametrize(
    "rows",
    [
        [row(1), row(1)],
        [row(2), row(1)],
        [row(0), row(float("nan"))],
        [row(0, linear=0.02, state="waiting_for_fresh_proposal"), row(1)],
    ],
)
def test_invalid_execution_is_not_a_zero_motion_success(rows):
    with pytest.raises(ValueError):
        execution_accounting(rows, [1, 0])


def test_support_fraction_counts_area_not_number_of_touched_cells():
    layers = np.zeros((10, 10, 6))
    layers[:, :, 0] = -1
    layers[:, 4, 0] = 0
    first = np.eye(4)
    first[0, 3] = -0.05
    first[1, 3] = 0.05
    last = first.copy()
    last[0, 3] = 0.05
    patch = BodyBox((0, 0, 0), (0.02, 0.02, 0.001), "wheel")
    result = support_area(layers, dict(origin=[-0.5, -0.5], resolution=0.1), [first, last], [patch])
    assert result["raw_free_fraction"] == pytest.approx(0.5)
    assert result["area_m2"]["not_raw_free"] > 0
