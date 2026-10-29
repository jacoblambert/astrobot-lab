from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest

pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from phase4_exploration_eval import Phase4ExplorationEval
from geometry_msgs.msg import PoseStamped


def evaluator():
    node = Phase4ExplorationEval.__new__(Phase4ExplorationEval)
    node.started_at = time.monotonic()
    node.args = NS(duration=5, wall_timeout=60, record_dir="")
    node.started_sim = node.last_sim = node.clock_fault = None
    node.latest_status = {}
    node.sim_duration = 0
    node.sim_now = 10.0
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(node.sim_now*1e9)))
    return node


def test_budget_uses_simulation_time_including_pauses():
    node = evaluator()
    assert not node.done()
    node.started_at -= 30  # Wall time advances while the simulation is paused.
    assert not node.done()
    node.sim_now = 15
    assert node.done() and node.sim_duration == pytest.approx(5)
    assert node.clock_fault is None


def test_clock_reversal_invalidates_run():
    node = evaluator()
    assert not node.done()
    node.sim_now = 9
    assert node.done() and node.clock_fault == "clock_reversed"


def test_wrong_reference_frame_does_not_advance_validator():
    node = evaluator()
    node.gt_validator = Mock()
    node.reference_faults = []
    msg = PoseStamped()
    msg.header.frame_id = "map"
    node._gt_pose_cb(msg)
    node.gt_validator.validate.assert_not_called()
    assert node.reference_faults[0]["reason"] == "wrong_frame"


def test_home_from_gt_start_anchors_coverage_at_first_valid_pose():
    from astrobot_core.evaluation import FixedCoverage
    node = evaluator()
    node.args = NS(record_dir="", home_x=20.0, home_y=18.0, coverage_radius=8.0, coverage_cell_size=2.0,
                   spin_window_sec=10.0, mission_mode="explore_radius", home_tol=1.0, coverage_gate=0.7,
                   sample_gate=0.75)
    node.gt_validator = Mock(validate=Mock(return_value=None))
    node.reference_faults, node.gt_history, node.active_goal_record = [], [], None
    node.latest_gt_pose = node.last_gt_received_at = None
    node._maybe_capture_spin = lambda now: None
    node.nominal_coverage = FixedCoverage(20.0, 18.0, 8.0, 2.0)
    node.home, node.physical_coverage = None, None
    node.best_sample, node.localization_events = 0.0, []
    for x, y in ((19.95, 17.97), (21.94, 17.97)):
        msg = PoseStamped()
        msg.header.frame_id = "gt_map"
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.w = 1.0
        node._gt_pose_cb(msg)
    assert node.home == (19.95, 17.97)
    # Both poses share one cell of the start-anchored grid; the nominal grid, 5 cm away, has its
    # x=20.0 boundary between them.
    assert len(node.physical_coverage.visited) == 1
    assert len(node.nominal_coverage.visited) == 2
    result = node.verdict()
    assert result["metrics"]["home_xy"] == [19.95, 17.97]
    assert result["metrics"]["nominal_home_coverage_fraction"] == pytest.approx(2 / 52)


def test_gt_before_the_first_clock_is_skipped_not_a_future_stamp_fault():
    node = evaluator()
    node.sim_now = 0.0
    node.gt_validator = Mock()
    node.reference_faults = []
    msg = PoseStamped()
    msg.header.frame_id = "gt_map"
    msg.header.stamp.sec = 24
    node._gt_pose_cb(msg)
    node.gt_validator.validate.assert_not_called()
    assert node.reference_faults == []
