"""Host NumPy/SciPy/pytest. Linked evidence and fixed-anchor pose diagnostics."""
from copy import deepcopy
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from audit_shadow_pair import compare_pose_sequence, validate_links
from shadow_terrain import FramePose, PoseTimeline


def test_fixed_home_does_not_fit_away_slam_error_and_retains_latency():
    timeline = PoseTimeline("s", 1)
    poses = [np.eye(4), np.eye(4)]
    poses[0][0, 3], poses[1][0, 3] = .2, .3
    timeline.apply(session="s", epoch=1, revision=1, parent=0, arrival_ns=200_000_000,
                   frames=[FramePose(i, i*100_000_000, p, np.eye(4)) for i, p in enumerate(poses)])
    truth = np.eye(4)
    truth[:3, 3] = [-19.95, -20, 0]
    rows, stats = compare_pose_sequence(timeline, [dict(observed_stamp=.05, world_from_lidar=truth)], [-20, -20, 0])
    assert stats["causal"]["position_m"]["maximum"] == pytest.approx(.2)
    assert stats["causal"]["horizontal_position_p95_m"] == pytest.approx(.2)
    np.testing.assert_allclose(stats["causal"]["position_xyz_signed_median_m"], [.2, 0, 0], atol=1e-12)
    assert rows[0]["causal"]["availability_delay_s"] == .15
    for p in poses:
        p[0, 3] -= .2
    timeline.apply(session="s", epoch=1, revision=2, parent=1, arrival_ns=300_000_000,
                   frames=[FramePose(i, i*100_000_000, p, np.eye(4)) for i, p in enumerate(poses)])
    _, stats = compare_pose_sequence(timeline, [dict(observed_stamp=.05, world_from_lidar=truth)], [-20, -20, 0])
    assert stats["causal"]["position_m"]["maximum"] == pytest.approx(.2)
    assert stats["revised"]["position_m"]["maximum"] < 1e-12
    with pytest.raises(ValueError, match="bracket"):
        compare_pose_sequence(timeline, [dict(observed_stamp=.15, world_from_lidar=truth)], [-20, -20, 0])


def linked():
    cap, rec = "revision-" + "a"*32, "revision-" + "b"*32
    request = dict(capture=cap, pattern="wide_sweep15_return", pitch_deg=5., home_xyz=[-20., -20., 0.])
    grant = dict(schema=1, scope="guarded_terrain_capture", ready=True, capture=cap, recorder=rec,
                 session="s", epoch=1, stamp=10., sequence=1, config_sha256="c"*64)
    sim = dict(request=request, first_grant=grant)
    recorder = dict(request=dict(request, stamp=9.), recorder=rec, identity=["s", 1],
                    config_sha256="c"*64, home_xyz=[-20., -20., 0.], live_geometry_errors=dict(imu=1e-6))
    drive = dict(shadow_first_grant=deepcopy(grant), failure=None,
                 commands=[dict(stamp=10.1, linear=.15, angular=0.)])
    return cap, rec, sim, recorder, drive


def test_exact_bidirectional_capture_link():
    args = linked()
    assert validate_links(*args) == args[2]["first_grant"]


@pytest.mark.parametrize("change", ["epoch", "config", "early_motion", "failed_drive", "home", "output_home", "nan_tf", "geometry"])
def test_failed_or_mixed_links_never_qualify(change):
    cap, rec, sim, recorder, drive = linked()
    if change == "epoch":
        recorder["identity"][1] = 2
    elif change == "config":
        recorder["config_sha256"] = "d"*64
    elif change == "early_motion":
        drive["commands"][0]["stamp"] = 9.9
    elif change == "failed_drive":
        drive["failure"] = "watchdog"
    elif change == "home":
        recorder["request"]["home_xyz"] = [20., 18., 0.]
    elif change == "output_home":
        recorder["home_xyz"] = [-20.1, -20., 0.]
    elif change == "nan_tf":
        recorder["live_geometry_errors"]["imu"] = float("nan")
    else:
        recorder["request"]["base_from_imu"] = np.eye(4).tolist()
    with pytest.raises(ValueError):
        validate_links(cap, rec, sim, recorder, drive)
