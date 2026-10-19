"""Host NumPy/SciPy/pytest; first-cloud seed, frame and stationarity contracts."""
from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from glim_gt_start import initial_state, verify_first_output


def fixture():
    t = np.arange(61, dtype=np.int64)*33_333_333 + 10_000_000_000
    p = np.tile(np.eye(4), (len(t), 1, 1))
    p[:, :3, :3] = Rotation.from_euler("xyz", [.1, -.08, 1.2]).as_matrix()
    p[:, :3, 3] = [-20, -19, .15]
    imu = np.eye(4)
    imu[:3, :3] = Rotation.from_euler("z", np.pi).as_matrix()
    imu[:3, 3] = [.264, .017, .262]
    lidar = np.eye(4)
    lidar[:3, :3] = Rotation.from_euler("y", .087).as_matrix()
    lidar[:3, 3] = [.4, 0, .25]
    return t, p, 11_500_000_000, [-20, -20, 0], imu, lidar


def test_seed_composes_nonzero_spawn_and_sensor_mounts():
    args = fixture()
    s = initial_state(*args)
    map_base = args[1][0].copy()
    map_base[:3, 3] -= args[3]
    np.testing.assert_allclose(s["map_from_imu"], map_base@args[4])
    np.testing.assert_allclose(s["map_from_lidar"], map_base@args[5])
    assert not s["ongoing_direct_gt"] and not s["trajectory_alignment"]


@pytest.mark.parametrize("fault", ["moving", "turning", "gap", "duplicate", "extrapolate", "short"])
def test_seed_fails_closed(fault):
    a = list(fixture())
    if fault == "moving": a[1][30, 0, 3] += .1
    if fault == "turning": a[1][30, :3, :3] = np.eye(3)
    if fault == "gap": a[0] = np.delete(a[0], [30, 31, 32]); a[1] = np.delete(a[1], [30, 31, 32], axis=0)
    if fault == "duplicate": a[0][30] = a[0][29]
    if fault == "extrapolate": a[2] = int(a[0][-1]+1)
    if fault == "short": a[2] = int(a[0][10])
    with pytest.raises(ValueError): initial_state(*a)


def test_first_output_has_no_posthoc_alignment_and_cannot_use_later_frame():
    seed = initial_state(*fixture())
    frame = dict(stamp_ns=seed["stamp_ns"], map_from_imu=seed["map_from_imu"])
    assert verify_first_output([dict(frames=[frame])], seed)["passed"]
    shifted = np.array(seed["map_from_imu"])
    shifted[0, 3] += .02
    assert not verify_first_output([dict(frames=[dict(frame, map_from_imu=shifted)])], seed)["passed"]
    with pytest.raises(ValueError):
        verify_first_output([dict(frames=[dict(frame, stamp_ns=frame["stamp_ns"]+1)])], seed)
