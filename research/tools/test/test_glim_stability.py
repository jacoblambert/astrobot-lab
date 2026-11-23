from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assess_glim_stability import assess


def fixture():
    times = np.arange(101) * 0.05
    truth = np.tile(np.eye(4), (len(times), 1, 1))
    truth[:, 0, 3] = times
    truth[:, :3, :3] = Rotation.from_euler("z", times).as_matrix()
    return times, truth.copy(), truth


def test_true_motion_is_not_mistaken_for_error_jump():
    report = assess(*fixture())
    assert report["catastrophic_screen_pass"]
    assert report["maximum_residual_step_m"] == 0
    assert sum(q["statistics"]["samples"] for q in report["equal_count_time_quarters"]) == 101


def test_sustained_divergence_is_detected_without_large_steps():
    times, estimate, truth = fixture()
    estimate[:, 0, 3] += times * 0.3
    report = assess(times, estimate, truth)
    assert report["failures"]["sustained_position"]
    assert not report["failures"]["position_error_jump"]
    assert not report["catastrophic_screen_pass"]


def test_brief_jump_cannot_hide_in_percentiles_or_endpoint():
    times, estimate, truth = fixture()
    estimate[50, 0, 3] += 0.6
    report = assess(times, estimate, truth)
    assert report["horizontal_p50_p95_p99_max_m"][1] == 0
    assert report["failures"]["position_error_jump"] and not report["catastrophic_screen_pass"]


def test_orientation_failure_is_not_hidden_by_position_accuracy():
    times, estimate, truth = fixture()
    estimate[40:, :3, :3] = Rotation.from_euler("x", 10, degrees=True).as_matrix() @ estimate[40:, :3, :3]
    report = assess(times, estimate, truth)
    assert report["failures"]["sustained_rotation"] and report["failures"]["rotation_error_jump"]


def test_missing_time_interval_is_not_scored_as_continuous():
    times, estimate, truth = fixture()
    times[50:] += 1
    with pytest.raises(ValueError, match="gapped"):
        assess(times, estimate, truth)


def test_native_interpolation_cannot_dilute_one_scan_jump():
    times = np.arange(121) / 60
    truth = np.tile(np.eye(4), (len(times), 1, 1))
    estimate = truth.copy()
    estimate[:, 0, 3] = np.maximum(0, 0.6 - 6 * np.abs(times - 1.0))
    report = assess(times, estimate, truth)
    assert report["maximum_residual_step_m"] < 0.11
    assert report["maximum_scan_interval_residual_step_m"] > 0.59
    assert report["failures"]["position_error_jump"]
