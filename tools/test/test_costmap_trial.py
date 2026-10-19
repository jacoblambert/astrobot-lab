import copy
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.costmap_trial import CostmapTrial, grid_route, validate_proposal, visible
from local_terrain.physx_reference import BodyBox
from natural_motion import motion_command, route


def advisory(stamp, x=0.0, y=0.0):
    pose = np.eye(4)
    pose[:2, 3] = [x, y]
    layers = np.zeros((60, 60, 6))
    layers[:, :, 0] = -1
    layers[30, 40, 0] = 0
    return SimpleNamespace(
        context=SimpleNamespace(
            query_ns=stamp,
            pose_ns=stamp,
            evidence_ns=stamp,
            local_from_base=pose,
            session="s",
            epoch=1,
            revision=1,
            poses=lambda relative: [pose @ p for p in relative],
        ),
        record=dict(origin=[-3, -3], resolution=0.1, local_from_map=np.eye(4)),
        layers=layers,
        patches=[BodyBox((x, y, 0), (0.04, 0.06, 0.001), "wheel") for x in (-0.256, 0.256) for y in (-0.2854, 0.2854)],
        body=BodyBox((0, 0, 0), (0.85, 0.7, 0.001), "guard"),
    )


def started():
    controller = CostmapTrial("c")
    first = controller.update(advisory(1_000_000_000), 1_000_000_000)
    assert not first["linear"] and first["goal_local_xy"] is None
    return controller


def test_validator_import_does_not_require_mapper_geometry(monkeypatch):
    import builtins
    import importlib.util

    original = builtins.__import__

    def guarded(name, *args, **kwargs):
        if name.startswith("shapely") or name == "local_terrain.contact_query":
            raise ImportError("mapper dependency unavailable in simulator")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded)
    path = Path(__file__).resolve().parents[1] / "local_terrain/costmap_trial.py"
    spec = importlib.util.spec_from_file_location("validator_without_mapper", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert callable(module.validate_proposal)


def test_continuous_motion_through_unknown_and_long_lookahead():
    controller = started()
    for i, x in enumerate([0, 0.047, 0.1, 0.2, 0.4, 0.6]):
        stamp = 4_000_000_000 + i * 100_000_000
        value = controller.update(advisory(stamp, x), stamp)
        assert value["linear"] == 0.05 and not value["angular"]
        assert value["decision"]["unknown_wheel_cells"] > 0
        assert np.linalg.norm(np.asarray(value["lookahead_xy"]) - [x, 0]) == pytest.approx(0.4)
        assert validate_proposal(value, "c", ("s", 1), stamp) == (0.05, 0)
    assert value["goal_local_xy"] == pytest.approx([1.05, 0.05])


@pytest.mark.parametrize("reason,occupied", [(0, True), (7, False)])
def test_new_hazard_stops_existing_motion(reason, occupied):
    controller = started()
    controller.update(advisory(4_000_000_000), 4_000_000_000)
    a = advisory(4_100_000_000)
    a.layers[30, 34, :2] = [100 if occupied else -1, reason]
    value = controller.update(a, a.context.query_ns)
    assert not value["linear"] and not value["angular"]
    assert value["decision"]["state"] != "available"


def test_position_noise_does_not_create_tiny_target_stop_cycles():
    controller = started()
    for i, (x, y) in enumerate([(0, 0), (0.05, -0.03), (0.01, 0.03), (0.08, -0.02), (0.07, 0.02)]):
        stamp = 4_000_000_000 + i * 100_000_000
        value = controller.update(advisory(stamp, x, y), stamp)
        assert value["linear"] == 0.05


def test_goal_stays_fixed_and_stops_on_arrival():
    controller = started()
    first = controller.update(advisory(4_000_000_000), 4_000_000_000)
    last = controller.update(advisory(5_000_000_000, 1.01, 0.02), 5_000_000_000)
    assert first["goal_local_xy"] == last["goal_local_xy"]
    assert last["terminal"] == "estimated_goal_reached" and not last["linear"]


@pytest.mark.parametrize("change", ["frame", "session", "clock"])
def test_frame_and_clock_change_reject(change):
    controller = started()
    a = advisory(2_000_000_000)
    if change == "frame":
        a.record["local_from_map"][0, 3] = 0.1
    elif change == "session":
        a.context.session = "other"
    else:
        a = advisory(1_000_000_000)
    with pytest.raises(ValueError):
        controller.update(a, a.context.query_ns)


@pytest.mark.parametrize("change", ["stale", "future", "arc", "sweep", "terminal", "protocol"])
def test_driver_rejects_invalid_authority(change):
    value = started().update(advisory(4_000_000_000), 4_000_000_000)
    value = copy.deepcopy(value)
    now = 4_000_000_000
    if change == "stale":
        now += 200_000_001
    elif change == "future":
        now -= 1
    elif change == "arc":
        value["angular"] = 0.04
    elif change == "sweep":
        value["relative_poses"][-1][0][3] = 0.05
    elif change == "terminal":
        value["terminal"] = "estimated_goal_reached"
    else:
        value["protocol"] = "terrain_step_trial_v1"
    with pytest.raises(ValueError):
        validate_proposal(value, "c", ("s", 1), now)


def test_route_avoids_obstacle_but_accepts_unknown():
    a = advisory(1)
    a.layers[30, 32, 0] = 100
    path, blocked = grid_route(a.layers, a.record, [-1, 0], [1.5, 0], 0.3)
    assert path and not visible(path[0], path[-1], blocked, a.record)
    assert max(abs(p[1]) for p in path) > 0.3
    for p, q in zip(path, path[1:]):
        assert visible(p, q, blocked, a.record)


def test_visibility_rejects_corner_contact():
    blocked = np.zeros((6, 6), bool)
    blocked[2, 2] = True
    assert not visible([1.5, 2.5], [2.5, 1.5], blocked, dict(origin=[0, 0], resolution=1))


def test_long_goal_near_map_edge_is_not_an_invented_obstacle():
    a = advisory(1)
    path, _ = grid_route(a.layers, a.record, [0, 0], [2.05, 0.05], 1.1)
    assert path


def test_immediate_sweep_cannot_leave_map_even_when_route_allows_unknown():
    controller = CostmapTrial("c")
    a = advisory(1_000_000_000, 2.0)
    controller.update(a, a.context.query_ns)
    # Explicit fixture fixes the goal; the production selector never changes it.
    controller.goal = np.array([2.8, 0])
    a = advisory(4_000_000_000, 2.0)
    value = controller.update(a, a.context.query_ns)
    assert value["decision"]["reason"] == "outside_grid"
    assert not value["linear"] and not value["angular"]


@pytest.mark.parametrize("pattern", ["costmap1", "costmap2"])
def test_new_protocol_cannot_fall_back_to_gt_waypoints(pattern):
    spec = route(pattern)
    assert spec["survey_targets_only"] and not spec["controller_uses_gt"]
    with pytest.raises(ValueError):
        motion_command(np.eye(4), np.eye(4), pattern, 0)
