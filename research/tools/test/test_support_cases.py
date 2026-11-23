import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.support_cases import loaded_cases, summarize_case


def test_cases_are_fixed_finite_and_include_control_rocks_drops_and_partial_support():
    cases = loaded_cases()
    assert len(cases) == len({c["name"] for c in cases}) == 6
    for case in cases:
        for low, high in case["boxes"]:
            assert np.isfinite([low, high]).all() and (np.asarray(high) > low).all()
    assert cases[-1]["boxes"][-1][1][2] == 0.08
    assert cases[-2]["boxes"][0][1][2] == -0.30
    assert cases[1]["boxes"][1][1][1] - cases[1]["boxes"][1][0][1] == pytest.approx(0.06)


def test_summary_keeps_unloaded_wheels_and_requires_every_tick():
    row = dict(
        positions=np.zeros((5, 3)).tolist(),
        orientations_wxyz=[[1, 0, 0, 0]] * 5,
        contact_forces_n=np.zeros((5, 3)).tolist(),
        velocities=np.zeros((5, 6)).tolist(),
    )
    report = summarize_case([row] * 480)
    assert report["wheel_frames_without_positive_up_force"] == [60] * 4
    assert report["settled_max_tilt_deg"] == 0 and not report["acceptance"]
    with pytest.raises(ValueError, match="480"):
        summarize_case([row] * 479)
