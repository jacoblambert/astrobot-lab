import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from raw_return_filter import valid_range_mask


def test_clamp_boundary_and_invalid_returns_are_accounted():
    points = np.array([[.5,0,0],[.5000001,0,0],[.5001,0,0],[1.,0,0],[100,0,0],[np.nan,0,0]])
    keep, counts = valid_range_mask(points,.5,100.)
    np.testing.assert_equal(keep,[False,False,True,True,False,False])
    assert counts["near_limit"] == 2
    assert counts["far_limit"] == counts["nonfinite"] == 1
    assert sum(counts[n] for n in ("near_limit","far_limit","nonfinite","retained")) == len(points)


def test_no_height_or_terrain_filtering():
    keep,_ = valid_range_mask(np.array([[0,0,1],[0,0,-1],[1,0,0]]),.5,100.)
    assert keep.all()


@pytest.mark.parametrize("limits", [(np.nan,100.),(1.,.5),(0.,100.)])
def test_invalid_configuration(limits):
    with pytest.raises(ValueError):
        valid_range_mask(np.zeros((1,3)),*limits)
