import copy
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.step_trial import StepTrial, validate_proposal
from natural_motion import motion_command, route


def advisory(stamp, *, x=0.0, state="available"):
    pose = np.eye(4)
    pose[0, 3] = x
    queries = []

    def query(path, *, query_ns):
        assert query_ns == stamp
        queries.append(path)
        return dict(state=state, reason="supported" if state == "available" else "new_ground")

    return SimpleNamespace(
        context=SimpleNamespace(
            query_ns=stamp,
            local_from_base=pose,
            session="s",
            epoch=1,
            revision=1,
            pose_ns=stamp - 100_000_000,
            evidence_ns=stamp - 200_000_000,
        ),
        record=dict(local_from_map=np.eye(4)),
        query=query,
        queries=queries,
    )


def test_uses_only_estimated_pose_and_checks_more_than_commanded_step():
    planner = StepTrial("capture")
    a = advisory(1_000_000_000)
    proposal = planner.update(a, a.context.query_ns)
    assert proposal["linear"] == 0.02 and proposal["angular"] == 0
    assert proposal["goal_local_xy"] == [0.3, 0]
    assert np.isclose(a.queries[0][-1][0, 3], 0.07)
    assert validate_proposal(proposal, "capture", ("s", 1), 1_010_000_000) == (0.02, 0)


def test_stop_observe_then_recheck_new_step_not_open_loop_chain():
    planner = StepTrial("capture")
    planner.update(advisory(1_000_000_000), 1_000_000_000)
    stop = planner.update(advisory(2_000_000_000, x=0.047), 2_000_000_000)
    assert stop["linear"] == 0 and stop["completed_steps"] == 1
    wait = advisory(2_100_000_000, x=0.049)
    assert planner.update(wait, wait.context.query_ns)["linear"] == 0
    assert not wait.queries
    new = advisory(2_600_000_000, x=0.049, state="unknown")
    refusal = planner.update(new, new.context.query_ns)
    assert refusal["linear"] == 0 and new.queries
    end = planner.update(advisory(4_600_000_000, x=0.049, state="unknown"), 4_600_000_000)
    assert end["terminal"] == "terrain_refused" and end["linear"] == end["angular"] == 0


def test_backward_estimator_shift_keeps_next_checked_increment_bounded():
    planner = StepTrial("capture")
    planner.update(advisory(1_000_000_000), 1_000_000_000)
    a = advisory(1_100_000_000, x=-0.014)
    proposal = planner.update(a, a.context.query_ns)
    assert np.isclose(proposal["relative_poses"][-1][0][3], 0.07)
    assert validate_proposal(proposal, "capture", ("s", 1), a.context.query_ns) == (0.02, 0)


def test_old_step_off_axis_stops_and_replans_without_claiming_progress():
    planner = StepTrial("capture")
    planner.update(advisory(1_000_000_000), 1_000_000_000)
    # Heading is close to the final goal but not the very nearby step target.
    a = advisory(1_100_000_000)
    angle = 0.1
    a.context.local_from_base[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    a.context.local_from_base[1, 3] = -0.03
    value = planner.update(a, a.context.query_ns)
    assert value["linear"] == value["angular"] == 0
    assert value["decision"]["reason"] == "stopped_to_replan_step"
    assert value["completed_steps"] == 0 and value["step_replans"] == 1
    assert value["goal_local_xy"] == [0.3, 0]
    assert planner.target is None and not a.queries


def test_goal_completion_is_explicit_estimator_only_and_stops():
    planner = StepTrial("capture")
    planner.update(advisory(1_000_000_000), 1_000_000_000)
    end = planner.update(advisory(2_000_000_000, x=0.29), 2_000_000_000)
    assert end["terminal"] == "estimated_goal_reached" and end["linear"] == 0


@pytest.mark.parametrize("change", ["frame", "session", "clock"])
def test_changed_context_cannot_authorize_motion(change):
    planner = StepTrial("capture")
    planner.update(advisory(1_000_000_000), 1_000_000_000)
    a = advisory(2_000_000_000)
    if change == "frame":
        a.record["local_from_map"][0, 3] = 1
    elif change == "session":
        a.context.session = "other"
    else:
        a = advisory(1_000_000_000)
    with pytest.raises(ValueError):
        planner.update(a, a.context.query_ns)


@pytest.mark.parametrize(
    "mutation",
    [
        dict(valid=False),
        dict(capture="other"),
        dict(epoch=2),
        dict(sequence=0),
        dict(query_ns=1_100_000_000),
        dict(query_ns=700_000_000),
        dict(linear=0.1),
        dict(angular=0.04),
        dict(terminal="terrain_refused"),
        dict(decision=dict(state="unknown")),
        dict(relative_poses=[np.eye(4).tolist()]),
        dict(pose_ns=0),
        dict(evidence_ns=0),
    ],
)
def test_transport_rejects_stale_wrong_identity_unchecked_or_refused_motion(mutation):
    planner = StepTrial("capture")
    proposal = planner.update(advisory(1_000_000_000), 1_000_000_000)
    proposal.update(copy.deepcopy(mutation))
    with pytest.raises(ValueError):
        validate_proposal(proposal, "capture", ("s", 1), 1_000_000_000)


def test_trial_cannot_fall_back_to_gt_waypoint_controller():
    assert route("terrain_step")["survey_targets_only"]
    assert not route("terrain_step")["controller_uses_gt"]
    with pytest.raises(ValueError, match="GT waypoint"):
        motion_command(np.eye(4), np.eye(4), "terrain_step", 0)


def driver_class(monkeypatch):
    import importlib
    from types import ModuleType

    package = ModuleType("step_driver_fixture")
    package.__path__ = [str(Path(__file__).resolve().parents[2] / "OmniLRS/src/environments_wrappers/ros2")]
    monkeypatch.setitem(sys.modules, "step_driver_fixture", package)
    return importlib.import_module("step_driver_fixture.terrain_step_drive").TerrainStepDrive


def fake_driver(monkeypatch, tmp_path):
    import io
    import time
    import threading

    cls = driver_class(monkeypatch)
    driver = cls.__new__(cls)
    driver.validate = validate_proposal
    driver.displacement_bound = 0.5
    driver.lock = threading.RLock()
    proposal = StepTrial("capture").update(advisory(4_000_000_000), 4_000_000_000)
    driver.owner = SimpleNamespace(run=tmp_path / "capture", stamp=4.0, lease=SimpleNamespace(identity=("s", 1)))
    driver.origin = np.eye(4)
    driver.spec = route("terrain_step")
    driver.start = 0.0
    driver.failure = driver.finished = driver.mode = driver.mode_start = driver.progress = None
    driver.command_stamp, driver.command_active, driver.command_elapsed = None, False, 0.0
    driver.turn_guard_id = driver.turn_guard_pose = driver.turn_guard_anchor = None
    driver.proposal = proposal
    driver.proposal_sequence = proposal["sequence"]
    driver.proposal_wall = time.monotonic()
    driver.guard = lambda current: None
    driver.commands = []
    driver.publish = lambda linear=0.0, angular=0.0: driver.commands.append((linear, angular))
    driver.log = io.StringIO()
    return driver


def test_gt_pose_does_not_select_command(monkeypatch, tmp_path):
    for x in (0.0, 0.1):
        driver = fake_driver(monkeypatch, tmp_path)
        gt = np.eye(4)
        gt[0, 3] = x
        assert driver.step(4.0, gt) is False
        assert driver.commands == [(0.02, 0.0)]


def test_missing_proposals_stop_and_gt_can_only_veto(monkeypatch, tmp_path):
    driver = fake_driver(monkeypatch, tmp_path)
    driver.proposal_wall -= 1.0
    assert not driver.step(4.0, np.eye(4))
    assert driver.commands == [(0.0, 0.0)]
    assert '"execution_state": "waiting_for_fresh_proposal"' in driver.log.getvalue()
    import time

    driver.proposal_wall = time.monotonic()
    assert not driver.step(4.0, np.eye(4))
    assert driver.commands[-1] == (0.02, 0.0)
    driver = fake_driver(monkeypatch, tmp_path)
    outside = np.eye(4)
    outside[0, 3] = 0.51
    with pytest.raises(ValueError, match="emergency"):
        driver.step(4.0, outside)
    assert not driver.commands  # Owner's fault handler/watchdog sends zero.


@pytest.mark.parametrize(
    "names,accepted",
    [
        (["terrain_shadow_mapper"], True),
        (["terrain_local_v2"], False),
        ([], False),
        (["terrain_shadow_mapper", "other"], False),
    ],
)
def test_driver_checks_actual_launch_name_and_sole_publisher(monkeypatch, tmp_path, names, accepted):
    import json

    driver = fake_driver(monkeypatch, tmp_path)
    driver.proposal_sequence = 0
    driver.reference = None
    driver.last_revision = -1
    driver.owner.node = SimpleNamespace(
        get_publishers_info_by_topic=lambda _: [SimpleNamespace(node_name=n) for n in names]
    )
    driver.receive_proposal(SimpleNamespace(data=json.dumps(driver.proposal)))
    assert (driver.failure is None) == accepted
    if not accepted:
        assert driver.commands == [(0.0, 0.0)]


def test_publisher_contract_matches_declared_launch_remap():
    import ast

    path = Path(__file__).resolve().parents[2] / "astrobot-lab/src/astrobot_launch/launch/terrain_shadow.launch.py"
    nodes = [
        node
        for node in ast.walk(ast.parse(path.read_text()))
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "Node"
    ]
    names = [
        kw.value.value
        for node in nodes
        for kw in node.keywords
        if kw.arg == "name" and isinstance(kw.value, ast.Constant)
    ]
    assert "terrain_shadow_mapper" in names


def test_proposal_subscription_uses_independent_receiver_executor():
    import ast

    path = Path(__file__).resolve().parents[2] / "OmniLRS/src/environments_wrappers/ros2/terrain_step_drive.py"
    subscriptions = [
        node
        for node in ast.walk(ast.parse(path.read_text()))
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_subscription"
    ]
    assert len(subscriptions) == 1
    assert ast.unparse(subscriptions[0].func.value) == "owner.lease_node"
