from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.observability import minimum_forward_history


def mount(pitch=0):
    value = np.eye(4)
    value[:3, :3] = Rotation.from_euler("y", pitch).as_matrix()
    value[:3, 3] = [0.4, 0, 0.25]
    return value


@pytest.mark.parametrize("pitch", [0.0, 0.08726646259971647, -0.04])
@pytest.mark.parametrize("wheel_x", [-0.256, 0.256])
def test_centerline_visibility_matches_exact_cone_geometry(pitch, wheel_x):
    height = 0.145
    angle = np.deg2rad(20.15)
    result = minimum_forward_history(mount(pitch), height, [wheel_x, 0], [-angle, angle])
    expected = 0.4 + (height + 0.25) / np.tan(angle + pitch) - wheel_x
    assert result == pytest.approx(expected, abs=1e-10)


def test_lateral_ground_and_already_visible_regions():
    angle = np.deg2rad(20)
    radius = 0.395 / np.tan(angle)
    actual = minimum_forward_history(mount(), 0.145, [0.256, 0.2854], [-angle, angle])
    assert actual == pytest.approx(0.4 + np.sqrt(radius**2 - 0.2854**2) - 0.256)
    assert minimum_forward_history(mount(), 0.145, [2, 0], [-angle, angle]) == 0


def test_upward_only_beams_and_range_limits_cannot_supply_ground():
    assert minimum_forward_history(mount(), 0.145, [0, 0], [-0.3, -0.1]) is None
    assert minimum_forward_history(mount(), 0.145, [0, 0], [-0.3, 0.3], maximum=0.6) is None


@pytest.mark.parametrize("angles", [[], [np.nan], [np.pi / 2]])
def test_invalid_native_angles_rejected(angles):
    with pytest.raises(ValueError):
        minimum_forward_history(mount(), 0.145, [0, 0], angles)
