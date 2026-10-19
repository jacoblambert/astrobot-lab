import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from shapely import Polygon, box, union_all

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.cost_planner import CostPlanner
from local_terrain.navigation_cost import navigation_costs
from local_terrain.physx_reference import BodyBox
from local_terrain.support_sweep import swept_polygons


PATCHES = [BodyBox((x, y, -0.145), (0.04, 0.06, 0.001), "wheel") for x in (-0.256, 0.256) for y in (-0.2854, 0.2854)]
BODY = BodyBox((0, 0, 0), (0.85, 0.70, 0.001), "existing_guard")


def fixture():
    layers = np.zeros((60, 60, 6))
    record = dict(origin=[-3, -3], resolution=0.1)
    pose = np.eye(4)
    pose[0, 3] = -1.4
    return layers, record, pose


def collision(poses, obstacle, patches=(BODY,)):
    sweep = union_all([Polygon(p) for p in swept_polygons(poses, patches)])
    return sweep.intersects(obstacle)


def test_path_around_seen_obstacle_checks_whole_body_not_centerline():
    layers, record, pose = fixture()
    layers[28:32, 29:31, 0] = 100
    costs, _ = navigation_costs(layers)
    result = CostPlanner(costs, record, pose, PATCHES, BODY).plan([1.4, 0])
    assert result["found"], result
    assert not collision(result["poses"], box(-0.1, -0.2, 0.1, 0.2))
    assert collision([pose, np.eye(4)], box(-0.1, -0.2, 0.1, 0.2))


def test_unknown_wall_and_map_boundary_are_not_crossed():
    layers, record, pose = fixture()
    layers[:, 29:31, 0] = -1
    costs, mask = navigation_costs(layers)
    assert not mask.any()
    result = CostPlanner(costs, record, pose, PATCHES, BODY).plan([1.4, 0])
    assert not result["found"]


def test_goal_in_free_map_and_budget_exhaustion_distinct_from_no_path():
    layers, record, pose = fixture()
    costs, _ = navigation_costs(layers)
    result = CostPlanner(costs, record, pose, PATCHES, BODY).plan([1, 0])
    assert result["found"]
    result = CostPlanner(costs, record, pose, PATCHES, BODY, max_expansions=1).plan([1, 0])
    assert not result["found"] and result["reason"] == "search_budget"


def test_isolated_unknown_can_connect_a_narrow_open_corridor():
    layers, record, pose = fixture()
    layers[:22, 30, 0] = layers[38:, 30, 0] = 100
    layers[33, 30, 0] = -1
    costs, mask = navigation_costs(layers)
    assert mask[33, 30]
    result = CostPlanner(costs, record, pose, PATCHES, BODY).plan([1.4, 0])
    assert result["found"], result
    assert collision(result["poses"], box(0, 0.3, 0.1, 0.4), PATCHES)
    # Deliberately hidden hazard: the same sensor evidence can hide a small
    # object/hole. The evaluator MUST count this as exposure, not certify safety.
    assert collision(result["poses"], box(0.02, 0.32, 0.08, 0.38), PATCHES)
    strict, _ = navigation_costs(layers, enabled=False)
    baseline = CostPlanner(strict, record, pose, PATCHES, BODY).plan([1.4, 0])
    assert not baseline["found"], baseline


def test_boundary_only_missing_ground_matches_exact_support_without_cover():
    layers, record, pose = fixture()
    costs, _ = navigation_costs(layers)
    costs[30, 30] = -1
    eligible = costs == -1
    index = SimpleNamespace(resolution=0.1, origin=np.array([-3, -3]), unions={})
    planner = CostPlanner(costs, record, pose, PATCHES, BODY, support_index=index, support_eligible=eligible)
    # Unknown cell [0,.1] x [0,.1] touches the edge of a supported patch.
    geometry = (np.array([[30, 30]]), np.array([[29, 30]]), box(-0.1, 0, 0, 0.1))
    assert planner.check(geometry) == 0
    # Any meaningful area still needs actual support. Do not clear conflicts.
    assert planner.check((*geometry[:2], box(-0.1, 0, 0.001, 0.1))) is None
    planner.support_eligible[30, 30] = False
    assert planner.check(geometry) is None
    planner.costs[30, 30] = 100
    assert planner.check(geometry) is None
