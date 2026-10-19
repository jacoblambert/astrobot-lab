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
