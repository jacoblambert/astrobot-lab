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


def test_policy_cells_fill_unknown_but_never_replace_an_observation():
    stitch = GlobalStitch((0., 0.), (1., 1.), first_origin=(0., 0.), resolution=0.1)
    approach = np.zeros((4, 4), dtype=np.int8)
    stitch.update(approach, (0., 0.), observed=np.ones((4, 4), bool))  # ground seen on approach
    blind = np.full((6, 6), 70, dtype=np.int8)  # the stationary rover's blind ring, gap policy only
    blind[5, 5] = -1
    stitch.update(blind, (0., 0.), observed=np.zeros((6, 6), bool))
    assert (stitch.grid[:4, :4] == 0).all()  # observation kept
    assert stitch.grid[4, 4] == 70 and stitch.grid[5, 5] == -1  # policy fills only unknown cells
    later = np.full((6, 6), 100, dtype=np.int8)
    stitch.update(later, (0., 0.), observed=np.ones((6, 6), bool))
    assert (stitch.grid[:6, :6] == 100).all()  # a newer observation still wins


def test_window_reads_back_memory_and_pads_outside_with_unknown():
    stitch = GlobalStitch((0, 0), (2, 2), (0, 0), 0.1)
    first = np.zeros((5, 5), dtype=np.int8)
    first[1, 1] = 100  # a rock observed once
    stitch.update(first, (0, 0), np.ones((5, 5), bool))
    later = np.full((5, 5), 70, dtype=np.int8)  # the rock now out of view: policy cost only
    stitch.update(later, (0, 0), np.zeros((5, 5), bool))
    assert stitch.window((0, 0), (5, 5))[1, 1] == 100
    stitch.grid[:] = 0
    edge = stitch.window((1.8, 1.8), (5, 5))  # 2 x 2 cells inside the mosaic, the rest outside
    assert (edge[:2, :2] != -1).all() and (edge[2:, :] == -1).all() and (edge[:, 2:] == -1).all()
