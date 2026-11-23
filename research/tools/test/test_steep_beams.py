import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnose_steep_beams import forward_mount_screen, variants
from diagnose_startup_visibility import screen
from local_terrain.ray_design import selected_setup


def test_comparison_preserves_budget_upper_beams_and_baseline():
    _, elevations = selected_setup(Path(__file__).resolve().parents[1] / "local_terrain/selected_lidar_optical.json")
    candidates = variants(elevations)
    np.testing.assert_array_equal(candidates["-60"], elevations)
    for beams in candidates.values():
        assert len(beams) == len(np.unique(beams)) == 32
        np.testing.assert_array_equal(beams[:25], elevations[:25])


def test_rejected_chassis_returns_do_not_become_visible_ground():
    setup, elevations = selected_setup(
        Path(__file__).resolve().parents[1] / "local_terrain/selected_lidar_optical.json"
    )
    baseline = screen(setup, elevations)
    for beams in variants(elevations).values():
        result = screen(setup, beams)
        assert result["rays"] == baseline["rays"] == 63968
        assert result["chassis_first_hits"] >= baseline["chassis_first_hits"]
        assert result["minimum_ground_radius_from_start_m"] >= baseline["minimum_ground_radius_from_start_m"] - 1e-12
        assert result["ground_returns_inside_startup_circle"] == 0


def test_forward_mount_is_separate_and_does_not_mutate_selected_setup():
    import copy

    setup, elevations = selected_setup(
        Path(__file__).resolve().parents[1] / "local_terrain/selected_lidar_optical.json"
    )
    before = copy.deepcopy(setup)
    result = forward_mount_screen(setup, elevations)
    assert setup == before
    assert result["-80"]["rays"] == 63968
    assert result["-80"]["ground_returns_inside_startup_circle"] > 0
    assert result["-80"]["base_from_lidar_translation_m"] == [0.52, 0.0, 0.45]
