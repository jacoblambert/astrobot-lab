"""Python Nav2 contract vs exact geometry and (when built) the installed-Nav2 probe.

Host tests use shapely as an independent overlap oracle. The probe tests need
NAV2_SHADOW_PROBE built from tools/nav2_shadow in a sourced Jazzy environment.
"""
import math
import os

import numpy as np
import pytest

from local_terrain.nav2_contract import (
    INSCRIBED, LETHAL, NO_INFORMATION, Footprint, Nav2Config, evaluate, inflate, master_costs, static_costs,
)


def test_config_mirrors_installed_nav2_yaml():
    for costmap in ("local_costmap", "global_costmap"):
        config = Nav2Config.from_yaml(costmap=costmap)
        for field in ("half_length", "half_width", "inflation_radius", "cost_scaling_factor"):
            assert getattr(config, field) == pytest.approx(getattr(Nav2Config(), field))
        assert config.lethal_threshold == 100 and config.planner_allow_unknown is False
    assert Nav2Config().inscribed == pytest.approx(.435)
    assert Nav2Config().circumscribed == pytest.approx(math.hypot(.595, .435))


def test_static_layer_value_mapping():
    out = static_costs(np.array([[-1, 0, 20, 70, 99, 100]]))
    assert out.tolist() == [[NO_INFORMATION, 0, 50, 177, 251, LETHAL]]


def test_inflation_gradient_and_unknown_overwrite():
    grid = np.zeros((41, 41), int)
    grid[20, 20] = 100
    grid[20, 23] = -1  # 0.3 m: inside the inscribed radius -> becomes INSCRIBED
    grid[20, 26] = -1  # 0.6 m: outside -> stays NO_INFORMATION
    out = master_costs(grid, .1)
    assert out[20, 20] == LETHAL and out[20, 24] == INSCRIBED  # 0.4 m <= 0.435
    assert out[20, 23] == INSCRIBED and out[20, 26] == NO_INFORMATION
    assert out[20, 25] == int(252 * math.exp(-2 * (.5 - .435)))
    assert out[20, 28] > 0 and out[20, 29] == 0  # Nav2 inflates to ceil(0.75/0.1) = 8 cells


def _oracle(fp, x, y, yaw):
    shapely = pytest.importorskip("shapely")
    from shapely import affinity, box

    c = fp.config
    body = affinity.rotate(box(-c.half_length, -c.half_width, c.half_length, c.half_width), yaw, use_radians=True,
                           origin=(0, 0))
    body = affinity.translate(body, x, y)
    squares = shapely.box(fp.lower[:, 0], fp.lower[:, 1], fp.lower[:, 0] + fp.res, fp.lower[:, 1] + fp.res)
    return shapely.intersects(body, squares)


@pytest.mark.parametrize("seed", range(4))
def test_static_footprint_overlap_equals_exact_geometry(seed):
    rng = np.random.default_rng(seed)
    fp = Footprint((60, 60), (-3. + rng.uniform(0, .1), -3. + rng.uniform(0, .1)), .1)
    for _ in range(25):
        x, y, yaw = rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-np.pi, np.pi)
        mask, outside = fp.overlap(x, y, yaw, x, y, yaw)
        assert not outside
        assert np.array_equal(mask, _oracle(fp, x, y, yaw))


def test_sweep_contains_every_sampled_pose_and_stays_tight():
    fp = Footprint((60, 60), (-3., -3.), .1)
    mask, _ = fp.overlap(0., 0., 0., 0., 0., 1.2)
    union = np.zeros_like(mask)
    for yaw in np.linspace(0, 1.2, 200):
        union |= _oracle(fp, 0., 0., yaw)
    assert not (union & ~mask).any()
    # midpoint Minkowski envelope adds <= STEP/2 of margin, i.e. at most one ring of cells
    assert (mask & ~union).sum() <= .25 * union.sum()


def test_out_of_grid_is_rejected():
    fp = Footprint((60, 60), (-3., -3.), .1)
    assert fp.overlap(2.5, 0., 0., 2.5, 0., 0.)[1]
    assert not fp.overlap(0., 0., 0., 1.5, 0., 0.)[1]


def test_unknown_ahead_is_controller_clear_but_area_and_planner_blocked():
    grid = np.zeros((60, 60), int)
    grid[28:32, 40:44] = -1  # 1.0-1.4 m ahead of a rover at the grid centre facing +x
    fp = Footprint(grid.shape, (-3., -3.), .1)
    out, start = evaluate(master_costs(grid, .1), fp, (0., 0., 0.))
    ahead = next(p for p in out if p["offset"] == 0.)
    # beyond the 0.9 m centre line: only the footprint sweep reaches it
    assert ahead["controller"] and ahead["planner"] and not ahead["area"]
    behind = next(p for p in out if abs(p["offset"]) == np.pi)
    assert behind["area"] and behind["controller"] and behind["planner"]
    assert not start["start_blocked"]
    grid[29:31, 35:37] = -1  # on the centre line, inside the turning circle
    out, _ = evaluate(master_costs(grid, .1), fp, (0., 0., 0.))
    ahead = next(p for p in out if p["offset"] == 0.)
    assert ahead["controller"] and not ahead["planner"] and not ahead["area"]
    behind = next(p for p in out if abs(p["offset"]) == np.pi)
    assert not behind["area"]  # the in-place turn sweeps the circumscribed circle
    grid[28:32, 40:44] = 100
    out, _ = evaluate(master_costs(grid, .1), fp, (0., 0., 0.))
    ahead = next(p for p in out if p["offset"] == 0.)
    assert not (ahead["controller"] or ahead["area"] or ahead["planner"])


@pytest.fixture
def executable():
    value = os.environ.get("NAV2_SHADOW_PROBE")
    if not value:
        pytest.skip("requires rebuilt standalone Nav2 shadow probe")
    return value


@pytest.mark.parametrize("seed", range(3))
def test_inflation_and_swept_area_match_installed_nav2(executable, seed):
    from nav2_shadow_probe import probe

    rng = np.random.default_rng(seed)
    costs = rng.choice([-1, 0, 20, 100], size=(50, 50), p=[.004, .985, .008, .003])
    costs[20:30, 20:30] = 0
    origin = (-2.5 + rng.uniform(0, .1), -2.5 + rng.uniform(0, .1))
    motions = []
    for _ in range(60):
        x, y, yaw = rng.uniform(-.6, .6), rng.uniform(-.6, .6), rng.uniform(-np.pi, np.pi)
        if rng.random() < .5:
            motions.append([x, y, yaw, x, y, yaw + rng.uniform(-np.pi, np.pi)])
        else:
            d = rng.uniform(.1, .9)
            motions.append([x, y, yaw, x + d * math.cos(yaw), y + d * math.sin(yaw), yaw])
    result = probe(executable, costs, [[0., 0., 0.]], origin=origin, motions=np.array(motions))
    ours = master_costs(costs, .1)
    assert np.array_equal(ours, result["inflated"])
    fp = Footprint(costs.shape, origin, .1)
    flat = ours.reshape(-1)
    mine = []
    for m in motions:
        mask, outside = fp.overlap(*m)
        mine.append(not outside and not (flat[mask] >= LETHAL).any())
    nav2 = [r["swept_area_clear"] for r in result["motions"]]
    assert any(nav2) and not all(nav2)
    assert mine == nav2


def test_clear_footprint_frees_only_unknown_under_the_body():
    from local_terrain.nav2_contract import clear_footprint

    grid = np.full((60, 60), -1)
    grid[30, 30] = 100
    fp = Footprint(grid.shape, (0., 0.), .1)
    out = clear_footprint(grid, fp, (3.0, 3.0, 0.))
    assert out[30, 30] == 100  # occupied under the rover stays occupied
    assert out[30, 34] == 0 and out[30, 25] == 0 and out[32, 30] == 0  # within 0.495 x 0.335
    assert out[30, 35] == -1 and out[33, 30] == -1  # padding band is not cleared
    assert (out == 0).sum() < 0.7 * 1.0 / 0.01 + 1


def test_planner_costs_is_policy_then_footprint_clear_at_the_record_pose():
    from local_terrain.nav2_contract import clear_footprint, grid_pose, planner_costs
    from local_terrain.navigation_cost import PUBLISHED_POLICIES, navigation_costs

    rng = np.random.default_rng(3)
    layers = np.zeros((60, 60, 6))
    layers[:, :, 0] = rng.choice([-1, 0, 100], size=(60, 60), p=[.3, .6, .1])
    layers[25:36, 25:36, 0] = -1  # blind patch about the rover
    layers[30, 32, 0] = 100  # occupied under the body stays occupied
    local_from_base = np.eye(4)
    yaw = .4
    local_from_base[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    local_from_base[:2, 3] = [0.05, 0.05]
    record = dict(origin=[-3., -3.], local_from_base=local_from_base.tolist())
    pose = grid_pose(record)
    assert np.allclose(pose, (3.05, 3.05, yaw))
    fp = Footprint((60, 60), (0., 0.), .1, Nav2Config.from_yaml())
    for policy in (None, *PUBLISHED_POLICIES.values()):
        grid, uncertain = planner_costs(layers, record, policy, fp)
        expected, mask = navigation_costs(layers, enabled=policy is not None, policy=policy or PUBLISHED_POLICIES["isolated"])
        assert np.array_equal(grid, clear_footprint(expected, fp, pose))
        cleared = (expected == -1) & (grid == 0)
        assert cleared.any() and not uncertain[cleared].any()
        assert np.array_equal(uncertain[~cleared], mask[~cleared])
        assert grid[30, 32] == 100
