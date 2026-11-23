import copy
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from replay_native_terrain import validate_native_source, grid_geometry, validate_diagnostic_mode


@pytest.mark.parametrize("builds,requested,valid", [
    ([False, False], False, True), ([True, True], True, True),
    ([False], True, False), ([True], False, False),
    ([False, True], True, False), ([False, True], False, False), ([], False, False)])
def test_diagnostics_are_explicit_and_cannot_masquerade_as_compute_acceptance(builds, requested, valid):
    if valid:
        validate_diagnostic_mode(builds, requested)
    else:
        with pytest.raises(ValueError, match="diagnostic"):
            validate_diagnostic_mode(builds, requested)


def test_declared_grid_phases_never_change_resolution_extent_or_reference():
    for phase in (0.,.025,.05,.075):
        assert grid_geometry(phase)==dict(origin=[-10.+phase,-10.+phase],resolution=.1,width=200,height=200)
    for phase in (-.025,.1,float("nan"),float("inf")):
        with pytest.raises(ValueError,match="declared"):
            grid_geometry(phase)


def fixture():
    phase = dict(residuals_m=[.00001] * 10001, missing=0, nonhorizontal_surface_samples=101)
    frames = []
    for i in range(3):
        pose = np.eye(4)
        pose[0, 3] = .2 * i
        frames.append(dict(observed_stamp=i * .05, full_columns=2000, columns=1000, configured_rotation_rate=10.,
                           world_time=i * .05, timeline_time=i * .05, world_physics_dt=.025, world_rendering_dt=.05,
                           current_pose=copy.deepcopy(phase), previous_pose=copy.deepcopy(phase),
                           world_from_lidar=pose.tolist()))
    return dict(frames=frames, duplicate_native_reads=0)


def test_native_gate_does_not_unlock_ros_full_scan_deskew():
    report = validate_native_source(fixture())
    assert report["cadence_passed"] and not report["eligible_for_moving_replay"]


@pytest.mark.parametrize("failure", ["double_rate", "pose_error", "missing_ray", "stationary", "duplicate", "dropped_tick"])
def test_native_motion_gate_fails_closed(failure):
    audit = fixture()
    if failure == "double_rate":
        for frame in audit["frames"]:
            frame["observed_stamp"] *= .5
            frame["world_time"] *= .5
    elif failure == "pose_error":
        audit["frames"][1]["current_pose"]["residuals_m"][0] = .002
    elif failure == "missing_ray":
        audit["frames"][1]["current_pose"]["missing"] = 1
    elif failure == "stationary":
        for frame in audit["frames"]:
            frame["world_from_lidar"] = np.eye(4).tolist()
    elif failure == "duplicate":
        audit["duplicate_native_reads"] = 1
    else:
        for frame in audit["frames"]:
            frame["world_rendering_dt"] = .025
    with pytest.raises(ValueError):
        validate_native_source(audit)
