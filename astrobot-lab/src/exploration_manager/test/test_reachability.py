import numpy as np
from exploration_manager.reachability import clearance_components, component_at, component_near


def test_wall_cannot_be_crossed_or_used_as_a_snapped_start():
    free = np.ones((40, 40), dtype=bool)
    free[:, 20] = False
    labels = clearance_components(free, .1, .3)
    assert not component_at(labels, (20, 20)).any()
    assert component_at(labels, (20, 10))[20, 10]
    assert not component_at(labels, (20, 10))[20, 30]


def test_unknown_and_boundaries_do_not_prove_footprint_support():
    free = np.ones((40, 40), dtype=bool)
    free[20, 20] = False
    labels = clearance_components(free, .1, .75)
    assert not component_at(labels, (0, 0)).any()
    assert not component_at(labels, (20, 25)).any()
    assert not component_at(labels, None).any()


def test_diagonal_corner_does_not_connect_regions():
    labels = clearance_components(np.eye(3, dtype=bool), .1, 0)
    assert component_at(labels, (0, 0)).sum() == 1


def test_corridor_width_respects_configured_clearance():
    free = np.zeros((50, 50), dtype=bool)
    free[:, 17:33] = True
    assert component_at(clearance_components(free, .1, .75), (25, 25)).any()
    assert not component_at(clearance_components(free, .1, .9), (25, 25)).any()


def test_start_beside_an_unseen_patch_seeds_from_nearby_clear_cells_without_crossing_walls():
    free = np.ones((60, 60), dtype=bool)
    free[:, 40] = False  # wall
    free[28:31, 22:25] = False  # unseen patch 0.3-0.5 m from the rover at (30, 30)
    labels = clearance_components(free, .1, .75)
    assert not component_at(labels, (30, 30)).any()
    near = component_near(labels, free, (30, 30), 10)
    assert near[30, 10] and not near[:, 41:].any()
    assert not component_near(labels, free, (30, 30), 0).any()  # disabled: unchanged behaviour
    free[29:32, 29:32] = True
    blocked = free.copy()
    blocked[30, 30] = False  # start on an obstacle/unknown cell: nothing
    assert not component_near(clearance_components(blocked, .1, .75), blocked, (30, 30), 10).any()
    walled = free.copy()
    walled[:, 39] = False
    walled[:, 41:] = True  # the far side is clear, but only through the wall
    labels = clearance_components(walled, .1, .75)
    assert not component_near(labels, walled, (30, 37), 10)[:, 41:].any()
