from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.self_filter import chassis_returns
from selected_lidar_profile import profile_path, selected_setup


def test_chassis_removal_is_calibrated_and_does_not_invent_rays():
    setup, _ = selected_setup(profile_path("selected_optical_v2"))
    # Actual rear roof hit, free ground outside the body, ground below the body.
    points = np.array([[-0.8, 0, -0.19127], [1, 0, -0.595], [-0.8, 0, -0.595]])
    np.testing.assert_array_equal(chassis_returns(points, setup), [True, False, False])
    legacy, _ = selected_setup()
    assert not chassis_returns(points, legacy).any()
