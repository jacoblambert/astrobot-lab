import numpy as np
import pytest

from local_terrain.global_stitch import GlobalStitch


def test_latest_observation_wins_and_unknown_never_erases():
    stitch = GlobalStitch((0., 0.), (2., 2.), first_origin=(0.05, 0.05), resolution=0.1)
    assert np.allclose(stitch.origin, (-0.05, -0.05)) and stitch.grid.shape == (21, 21)
    first = np.zeros((5, 5), dtype=np.int8)
    first[2, 2] = 100
    stitch.update(first, (0.05, 0.05))
    assert stitch.grid[3, 3] == 100 and (stitch.grid[1:6, 1:6] != -1).all()
    later = np.full((5, 5), -1, dtype=np.int8)
    later[0, 0] = 70  # one cell along the lattice: mosaic (2, 2); its unknown (1, 1) lands on (3, 3)
    stitch.update(later, (0.15, 0.15))
    assert stitch.grid[2, 2] == 70 and stitch.grid[3, 3] == 100  # unknown kept the earlier lethal


def test_partial_and_disjoint_windows_are_clipped():
    stitch = GlobalStitch((0., 0.), (1., 1.), first_origin=(0., 0.), resolution=0.1)
    stitch.update(np.zeros((6, 6), dtype=np.int8), (0.7, 0.7))
    assert (stitch.grid[7:, 7:] == 0).all() and stitch.clipped == 1
    stitch.update(np.zeros((6, 6), dtype=np.int8), (5., 5.))
    assert stitch.clipped == 2 and stitch.windows == 1


def test_off_lattice_window_is_refused():
    stitch = GlobalStitch((0., 0.), (1., 1.), first_origin=(0., 0.), resolution=0.1)
    with pytest.raises(ValueError):
        stitch.update(np.zeros((2, 2), dtype=np.int8), (0.03, 0.))
