from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from exploration_manager.exploration_manager_node import ExplorationManager, Pose2D
from astrobot_core.mission import ActionGeneration


@pytest.fixture
def manager():
    node = ExplorationManager.__new__(ExplorationManager)
    node._generation = ActionGeneration()
    node.require_localization = False
    node.home_pose = Pose2D(0, 0)
    node.latest_pose = Pose2D(1.5, 0)
    node.home_reached_radius_m = .9
    node.return_waypoints = []
    node.returning_home = False
    node.finished = node.mission_failed = False
    node.success_count = node.failure_count = 0
    node.active_goal_cancel_reason = ""
    node.active_goal_cancel_stall_pose = None
    node.mission_mode = "explore_radius"
    node._now = lambda: 10.0
    node._publish_event = Mock()
    node._publish_status = Mock()
    node.get_logger = lambda: NS(info=Mock(), warn=Mock(), error=Mock())
    node._add_goal_blacklist = Mock()
    return node


def test_return_uses_global_home_goal(manager):
    manager._send_goal = Mock()
    manager._send_return_home("coverage_goal_met")
    manager._send_goal.assert_called_once_with(manager.home_pose, "return_home", None)
    assert manager.return_waypoints == []


def test_projected_goal_success_is_not_home_success(manager):
    manager._goal_result(NS(result=lambda: NS(status=4)), 0, "return_home", manager.latest_pose)
    assert manager.mission_failed and manager.return_reason == "return_failed"


def test_true_home_success(manager):
    manager.latest_pose = Pose2D(.5, 0)
    manager._goal_result(NS(result=lambda: NS(status=4)), 0, "return_home", manager.home_pose)
    assert manager.finished and not manager.mission_failed


def test_rejected_return_terminates(manager):
    manager.returning_home = True
    manager._goal_response(NS(result=lambda: NS(accepted=False)), 0, "return_home", manager.home_pose)
    assert manager.mission_failed and not manager.active_goal


def test_stale_accepted_action_is_cancelled(manager):
    callback = Mock()
    guarded = manager._guard(manager._generation.value, callback)
    manager._generation.advance()
    handle = NS(accepted=True, cancel_goal_async=Mock())
    guarded(NS(result=lambda: handle))
    callback.assert_not_called()
    handle.cancel_goal_async.assert_called_once()


def test_backup_is_not_cancelled_for_increasing_goal_distance(manager):
    manager.cancel_requested = False
    manager.active_goal_handle = NS(cancel_goal_async=Mock())
    manager.active_goal_started_at = 0
    manager.goal_max_runtime_sec = 240
    manager.active_goal_last_progress_at = -1000
    manager.active_goal_recoveries = 1
    manager._check_goal_progress()
    manager.active_goal_handle.cancel_goal_async.assert_not_called()


def test_budget_preempts_active_goal_before_sending_home(manager):
    manager.started_at = 0
    manager.max_duration_sec = 5
    manager.returning_home = False
    manager.active_goal = True
    manager.pending_goal_check = False
    manager.cancel_requested = False
    manager.active_goal_handle = NS(cancel_goal_async=Mock())
    manager._send_return_home = Mock()
    manager._cycle()
    manager.active_goal_handle.cancel_goal_async.assert_called_once()
    manager._send_return_home.assert_not_called()
    assert manager.active_goal_cancel_reason == "mission_return"
    manager.active_goal = False
    manager._cycle()
    manager._send_return_home.assert_called_once_with("budget_exhausted")


def test_repeated_terminal_health_does_not_repeat_failure_event(manager):
    manager._mark_mission_failed("localization_fault")
    manager._mark_mission_failed("localization_fault")
    assert manager._publish_event.call_count == 1
