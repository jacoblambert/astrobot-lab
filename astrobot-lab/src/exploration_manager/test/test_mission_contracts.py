import math
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
    node.home_approach_radius_m = 0.0
    node.return_behavior_tree = ""
    node.mission_visit_margin_m = 0.0
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


def reveal_ready(manager, distance=1.5):
    import numpy as np

    manager.reveal_distance_m, manager.reveal_max_attempts, manager.reveal_attempts = distance, 4, 0
    manager.latest_reachable_cells = 0
    manager.lethal_cost_min = 80
    manager.grid = np.full((100, 100), -1, dtype=np.int16)
    manager._world_to_cell = lambda x, y: (int(y * 10) + 50, int(x * 10) + 50)
    manager._send_goal = Mock()
    manager.latest_pose = Pose2D(0., 0., 0.)
    return manager


def test_reveal_goes_ahead_then_turns_through_the_blind_ring(manager):
    import math

    reveal_ready(manager)
    for k, offset in enumerate((0., math.pi / 2, -math.pi / 2, math.pi)):
        assert manager._send_reveal_goal()
        pose = manager._send_goal.call_args[0][0]
        assert manager._send_goal.call_args[0][1] == "reveal"
        assert math.isclose(pose.x, 1.5 * math.cos(offset), abs_tol=1e-9) and math.isclose(pose.y, 1.5 * math.sin(offset), abs_tol=1e-9)
    assert not manager._send_reveal_goal()  # bounded: then the mission may end with no_candidates


def test_reveal_skips_lethal_targets_and_also_runs_beside_a_reachable_pocket(manager):
    reveal_ready(manager)
    manager.grid[50, 65] = 100  # lethal straight ahead
    assert manager._send_reveal_goal()
    pose = manager._send_goal.call_args[0][0]
    assert abs(pose.x) < 1e-9 and pose.y > 1.4 and manager.reveal_attempts == 2
    manager.latest_reachable_cells = 10  # a seeded pocket without candidates still reveals
    assert manager._send_reveal_goal() and manager.reveal_attempts == 3
    assert not reveal_ready(manager, distance=0.)._send_reveal_goal()  # legacy default: off


def test_home_approach_moves_the_return_goal_off_a_rock_at_home(manager):
    import numpy as np
    from types import SimpleNamespace as NS

    manager.home_pose = Pose2D(5.0, 5.0, 0.3)
    manager.home_approach_radius_m, manager.home_approach_clearance_m = 0.5, 0.5
    manager.lethal_cost_min, manager.free_cost_max = 80, 35
    manager.grid = np.zeros((100, 100), dtype=np.int16)
    manager.latest_map = NS(info=NS(resolution=0.1, width=100, height=100,
                                    origin=NS(position=NS(x=0.0, y=0.0))))
    manager._publish_event = Mock()
    assert manager._home_goal_pose() is manager.home_pose  # clear home: unchanged
    manager.grid[47:53, 51:55] = 100  # rock 0.1-0.5 m east of home
    pose = manager._home_goal_pose()
    assert 0.3 < math.hypot(pose.x - 5.0, pose.y - 5.0) <= 0.5 and pose.x < 5.0 and pose.yaw == 0.3
    manager.home_approach_radius_m = 0.0
    assert manager._home_goal_pose() is manager.home_pose  # legacy default: off


@pytest.mark.parametrize("kind", ["explore", "return_home"])
def test_only_return_home_goals_carry_the_return_behavior_tree(manager, kind):
    sent = []
    manager.return_behavior_tree = "/bt/precise.xml"
    manager.action_client = NS(wait_for_server=lambda timeout_sec: True,
                               send_goal_async=lambda goal, feedback_callback: sent.append(goal) or Mock())
    manager._distance_from_robot = lambda pose: 1.0
    manager._timeout_for_distance = lambda distance: 60.0
    manager._remember_goal = Mock()
    manager._publish_selected_goal = Mock()
    manager._guard = lambda generation, callback: callback
    manager.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: __import__("builtin_interfaces.msg").msg.Time()))
    manager.frame_id, manager.goal_count = "map", 0
    manager._send_nav_goal(Pose2D(1.0, 2.0), kind, None)
    assert sent[0].behavior_tree == ("/bt/precise.xml" if kind == "return_home" else "")


def test_visit_margin_ignores_poses_near_a_cell_edge(manager):
    manager.home_pose, manager.home_radius_m, manager.mission_cell_size_m = Pose2D(0, 0), 8.0, 2.0
    manager.visited_mission_cells = set()
    manager._mark_mission_cell_visited(Pose2D(1.95, 1.0))  # 5 cm from the x=2 edge
    assert len(manager.visited_mission_cells) == 1  # legacy: any touch counts
    manager.visited_mission_cells, manager.mission_visit_margin_m = set(), 0.15
    manager._mark_mission_cell_visited(Pose2D(1.95, 1.0))
    assert not manager.visited_mission_cells
    manager._mark_mission_cell_visited(Pose2D(1.0, 1.0))
    assert manager.visited_mission_cells == {(4, 4)}
