import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.navigation_cost import GapPolicy, navigation_costs


def terrain(size=21):
    return np.zeros((size, size, 6))


def test_single_gap_is_penalized_without_rewriting_labels_or_inferring_probability():
    layers = terrain()
    layers[10, 10, 0] = -1
    before = layers.copy()
    costs, mask = navigation_costs(layers)
    assert costs[10, 10] == 70 and mask.sum() == 1
    assert costs[10, 9] == 0
    np.testing.assert_array_equal(layers, before)
    strict, strict_mask = navigation_costs(layers, enabled=False)
    assert strict[10, 10] == -1 and not strict_mask.any()


@pytest.mark.parametrize("reason", [1, 2, 3, 5, 6, 7, 8, 10, 11])
def test_explicit_terrain_conflicts_never_become_soft_unknown(reason):
    layers = terrain()
    layers[10, 10, :2] = [-1, reason]
    assert navigation_costs(layers)[0][10, 10] == -1


@pytest.mark.parametrize("other", [(10, 11), (11, 11), (9, 9)])
def test_connected_or_diagonal_unknown_cells_do_not_bootstrap(other):
    layers = terrain()
    layers[10, 10, 0] = layers[other[0], other[1], 0] = -1
    assert not navigation_costs(layers)[1].any()


@pytest.mark.parametrize(
    "change", ["inferred", "nonfinite", "step", "slope", "obstacle", "edge", "border", "large_cell"]
)
def test_surrounding_measured_compatible_ground_required(change):
    layers = terrain()
    layers[10, 10, 0] = -1
    kwargs = {}
    if change == "inferred":
        layers[10, 11, 0] = 20
    elif change == "nonfinite":
        layers[10, 11, 2] = np.nan
    elif change == "step":
        layers[10, 11, 2] = 0.08
    elif change == "slope":
        layers[:, :, 2] = np.arange(21)[None, :] * 0.05
    elif change == "obstacle":
        layers[10, 12, 0] = 100
    elif change == "edge":
        layers[10, 12, :2] = [-1, 11]
    elif change == "border":
        layers[10, 10, 0] = 0
        layers[0, 10, 0] = -1
    else:
        kwargs["resolution"] = 0.2
    assert not navigation_costs(layers, **kwargs)[1].any()


def test_relative_slope_not_absolute_height_controls_consistency():
    layers = terrain()
    layers[:, :, 2] = 400 + np.arange(21)[None, :] * 0.01
    layers[10, 10, 0] = -1
    assert navigation_costs(layers)[1][10, 10]


def test_occupied_stays_lethal_and_invalid_policy_fails():
    layers = terrain()
    layers[10, 10, 0] = 100
    assert navigation_costs(layers)[0][10, 10] == 100
    with pytest.raises(ValueError):
        GapPolicy(cost=100)


@pytest.mark.parametrize("connected", [False, True])
def test_no_candidate_fast_path_skips_fitting_without_changing_costs(monkeypatch, connected):
    layers = terrain()
    layers[2, 2, 0] = 100
    if connected:
        layers[10:12, 10:12, 0] = -1
    expected, _ = navigation_costs(layers, enabled=False)

    def unexpected_fit(*args):
        raise AssertionError("no eligible cell can require a plane fit")

    monkeypatch.setattr(np.linalg, "pinv", unexpected_fit)
    actual, mask = navigation_costs(layers)
    np.testing.assert_array_equal(actual, expected)
    assert not mask.any()


BOUNDED = GapPolicy(mode="bounded", hazard_clearance_m=0.30)


def test_bounded_frees_connected_gap_edges_but_not_a_large_gap_interior():
    layers = terrain(41)
    layers[5:36, 5:36, 0] = -1  # 3.1 m square gap of missing returns
    costs, mask = navigation_costs(layers, policy=BOUNDED)
    assert mask[5, 5:36].all() and mask[9, 20]  # 0.4 m from measured ground
    assert not mask[20, 20] and costs[20, 20] == -1  # 1.5 m inside: stays unknown
    isolated, _ = navigation_costs(layers)
    assert not _.any() and (isolated[5:36, 5:36] == -1).all()


@pytest.mark.parametrize("reason", [3, 5, 6, 7, 8, 10, 11])
def test_bounded_never_softens_explicit_conflicts_or_their_shadow(reason):
    layers = terrain()
    layers[8:13, 8:13, 0] = -1
    layers[10, 10, 1] = reason
    costs, mask = navigation_costs(layers, policy=BOUNDED)
    assert costs[10, 10] == -1 and not mask[10, 10]
    # every softened cell keeps the policy clearance from the conflict
    rows, cols = np.nonzero(mask)
    assert (np.hypot(rows - 10, cols - 10) * .1 > .30).all()


def test_bounded_unknown_next_to_an_obstacle_stays_unknown():
    layers = terrain()
    layers[:, 12:, 0] = -1  # occluded half-plane behind a rock
    layers[10, 11, 0] = 100
    costs, mask = navigation_costs(layers, policy=BOUNDED)
    assert not mask[10, 12:14].any() and costs[10, 12] == -1
    assert mask[2, 12] and costs[2, 12] >= BOUNDED.cost


def test_bounded_requires_measured_not_inferred_support():
    layers = terrain()
    layers[:, :, 0] = 20
    layers[10, 10, 0] = -1
    _, mask = navigation_costs(layers, policy=BOUNDED)
    assert not mask.any()


def test_invalid_mode_and_gap_fail():
    with pytest.raises(ValueError):
        GapPolicy(mode="optimistic")
    with pytest.raises(ValueError):
        GapPolicy(mode="bounded", max_support_gap_m=0)


def test_occupied_shadow_is_published_lethal_only_when_requested():
    layers = terrain()
    layers[:, 12:, 0] = -1
    layers[10, 11, 0] = 100
    policy = GapPolicy(mode="bounded", hazard_clearance_m=0.30, occupied_shadow_lethal_m=0.20)
    costs, mask = navigation_costs(layers, policy=policy)
    assert costs[10, 12] == 100 and costs[10, 13] == 100 and costs[10, 14] < 100
    assert not mask[10, 12:14].any()
    plain, _ = navigation_costs(layers, policy=BOUNDED)
    assert plain[10, 12] == -1
    with pytest.raises(ValueError):
        GapPolicy(occupied_shadow_lethal_m=-1)
