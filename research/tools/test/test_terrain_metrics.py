import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from terrain_metrics import score_terrain


def score(cost, labels, ids=None, **kwargs):
    return score_terrain(cost, labels=labels, object_ids=np.zeros_like(labels) if ids is None else ids,
                         map_origin=(0,0), map_resolution=.1, gt_origin=(0,0), gt_resolution=.02, **kwargs)


def test_unknown_not_removed_from_denominator():
    result = score(np.array([[0,-1]]), np.zeros((5,10), dtype=int))
    assert result["flat_free_fraction"] == .5
    assert result["flat_unknown_fraction"] == .5


def test_zero_return_object_remains_miss_and_no_spatial_tolerance():
    labels = np.zeros((5,15), dtype=int)
    ids = np.zeros_like(labels)
    labels[:,5:10] = 1
    ids[:,5:10] = 1
    result = score(np.array([[100,-1,100]]), labels, ids)
    assert result["object_count"] == 1
    assert result["object_recall"] == 0
    assert result["flat_false_occupied_fraction"] == 1


def test_outside_map_is_not_excluded():
    result = score(np.array([[0]]), np.zeros((5,10), dtype=int))
    assert result["observable_outside_prediction"] == 25
    assert result["flat_free_fraction"] == .5


def test_inferred_dropoff_is_false_free():
    result = score(np.array([[20]]), np.full((5,5), 2))
    assert result["dropoff_free_cells"] == 25


def test_invalid_labels_rejected():
    with pytest.raises(ValueError):
        score(np.array([[0]]), np.zeros((5,5)), np.ones((5,5)))
