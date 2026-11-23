import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("shapely")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.contact_index import IncrementalSupportIndex
from local_terrain.contact_support import SupportIndex


def square(x):
    return np.array([[x, 0.0], [x + 0.08, 0.0], [x + 0.08, 0.12], [x, 0.12]])


def compare(index, polygons, origin, resolution):
    reference = SupportIndex(polygons, origin, resolution, backend="geos")
    index.update(polygons, origin, resolution)
    assert index.unions.keys() == reference.unions.keys()
    for ij, expected in reference.unions.items():
        assert index.unions[ij].symmetric_difference(expected).area < 1e-12


def test_incremental_index_corrections_eviction_and_non_ratcheting_grid_phase():
    index = IncrementalSupportIndex()
    polygons = [(square(i * 0.01), "track") for i in range(30)]
    compare(index, polygons, [-3, -3], 0.1)
    compare(index, polygons, [-3, -3], 0.1)
    assert index.rebuilt_cells == 0
    compare(index, polygons, [-2.9, -3.1], 0.1)
    assert index.rebuilt_cells == 0
    polygons = polygons[2:] + [(square(1.0), "corrected")]
    compare(index, polygons, [-2.9, -3.1], 0.1)
    assert index.rebuilt_cells > 0
    compare(index, polygons, [-2.875, -3.075], 0.1)
    compare(index, polygons, [-2.875, -3.075], 0.05)
    compare(index, [], [-2.875, -3.075], 0.05)
    assert not index.geometry and not index.unions


def test_duplicate_polygons_do_not_disappear_until_last_instance_removed():
    index = IncrementalSupportIndex()
    p = (square(0), "contact")
    compare(index, [p, p], [0, 0], 0.1)
    compare(index, [p], [0, 0], 0.1)
    assert index.rebuilt_cells == 0
    compare(index, [], [0, 0], 0.1)


def test_failed_update_preserves_complete_previous_index():
    index = IncrementalSupportIndex(max_entries=20, max_polygons=2)
    index.update([(square(0), "contact")], [0, 0], 0.1)
    old = index.unions.copy()
    with pytest.raises(ValueError, match="budget"):
        index.update([(square(i), "contact") for i in range(3)], [0, 0], 0.1)
    assert index.unions == old
    with pytest.raises(ValueError, match="budget"):
        index.update([(square(0), "contact")], [0, 0], 0.001)
    assert index.unions == old
