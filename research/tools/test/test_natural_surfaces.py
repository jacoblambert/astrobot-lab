"""Host-only fixed-correspondence placement contracts, not simulator validation."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnose_natural_surfaces import point_errors, summarize


def hit(index=0, path="/rock"):
    return dict(point_index=index, residual_m=0., first_hit=dict(collision=path,
                position=[10., 20., 1.], normal=[0., 0., 1.]))


def test_known_origin_and_height_error_cannot_be_fitted_away():
    transform = np.eye(4)
    result = point_errors([[0., 0., 1.]], [hit()], transform, [10., 20., 0.])
    assert result[0]["displacement_m"] == 0.
    transform[2, 3] = .05
    result = point_errors([[0., 0., 1.]], [hit()], transform, [10., 20., 0.])
    assert result[0]["displacement_m"] == pytest.approx(.05)
    assert result[0]["normal_error_m"] == pytest.approx(.05)


@pytest.mark.parametrize("fault", ["duplicate", "missing", "residual", "normal", "index"])
def test_invalid_first_hit_is_never_replaced_by_other_geometry(fault):
    records = [hit()]
    if fault == "duplicate": records *= 2
    if fault == "missing": records[0]["first_hit"] = None
    if fault == "residual": records[0]["residual_m"] = .00101
    if fault == "normal": records[0]["first_hit"]["normal"] = [0., 0., 0.]
    if fault == "index": records[0]["point_index"] = 1
    with pytest.raises(ValueError): point_errors([[0., 0., 1.]], records, np.eye(4), [10., 20., 0.])


def test_exact_collision_root_and_descendants_count_but_prefix_neighbor_does_not():
    obj = dict(path="/rock", minimum=[0, 0, 0], maximum=[1, 1, 1])
    rows = [dict(collision="/rock", displacement_m=.04, normal_error_m=.03)] * 10
    rows += [dict(collision="/rock/mesh", displacement_m=.04, normal_error_m=.03)] * 10
    rows += [dict(collision="/rock2", displacement_m=2., normal_error_m=2.)] * 100
    result = summarize(rows, [obj])
    assert result["objects"][0]["metrics"]["samples"] == 20
    assert result["eligible_observed_large"] == 1
    assert summarize(rows[1:], [obj])["eligible_observed_large"] == 0
