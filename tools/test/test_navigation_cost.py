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
