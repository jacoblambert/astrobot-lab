"""Host NumPy/SciPy contracts for fixed-frame live/failed-prefix diagnostics."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from measure_live_glim_accuracy import base_reference, statistics, timeline_from_events, validate_gt_frame, acquisition_seconds
from imu_consistency import interpolate_poses
from measure_live_glim_accuracy import published_gt_at_native_ticks
from measure_live_glim_accuracy import validate_projection_tail


def pose(x=0., yaw=0.):
    value = np.eye(4)
    value[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    value[0, 3] = x
    return value


def revision(number, x):
    return dict(kind=1, session="test", epoch=1, revision=number, parent=number-1, snapshot=False,
                frames=[dict(frame_id=0, stamp_ns=0, map_from_imu=pose(x).tolist(),
                             imu_from_lidar=pose(.4).tolist())])


def events(tmp_path, values):
    path = tmp_path / "events.jsonl"
    path.write_text("".join(json.dumps(dict(kind=k, payload=v, receipt_wall_ns=i)) + "\n"
                            for i, (k, v) in enumerate(values)))
    return path


def test_fixed_home_and_rotating_lever_arm_not_fitted_alignment():
    base, mount = pose(20., np.pi/2), pose(.4, .1)
    reference = base_reference(base @ mount, mount, [-20., -20., 0.])
    expected = base.copy(); expected[:3, 3] += [20., 20., 0.]
    np.testing.assert_allclose(reference, expected, atol=1e-12)
    error = reference.copy(); error[1, 3] += .15
    report = statistics([error], [reference])
    assert report["horizontal_m"]["absolute_p95"] == pytest.approx(.15)
    assert not report["diagnostic_pose_targets_pass"]


def test_first_publication_not_replaced_and_next_clock_bounds_receipt(tmp_path):
    path = events(tmp_path, [("clock", 0), ("revision", revision(1, .01)), ("clock", 100_000_000),
                             ("revision", revision(2, .02)), ("clock", 200_000_000),
                             ("fault", "simulation wall heartbeat lost")])
    timeline, last, pending, failure = timeline_from_events(path)
    assert last == 200_000_000 and pending == 0 and failure
    with pytest.raises(ValueError, match="extrapolation"):
        timeline.associate(0, 99_999_999)
    assert timeline.associate(0, last).map_from_lidar[0, 3] == pytest.approx(.41)
    assert timeline.associate(0, last, mode="revised").map_from_lidar[0, 3] == pytest.approx(.42)


@pytest.mark.parametrize("tail", [
    [("clock", 0)], [("clock", 400_000_000)],
    [("fault", "heartbeat"), ("clock", 200_000_000)],
    [("revision", dict(revision(2, 0), kind=2)), ("clock", 200_000_000)],
])
def test_resets_gaps_fault_tail_and_global_revisions_are_rejected(tmp_path, tail):
    path = events(tmp_path, [("clock", 0), ("revision", revision(1, 0)), ("clock", 100_000_000), *tail])
    with pytest.raises(ValueError):
        timeline_from_events(path)


def test_unbounded_tail_is_counted_not_backdated(tmp_path):
    path = events(tmp_path, [("clock", 0), ("revision", revision(1, 0)), ("clock", 100_000_000),
                             ("revision", revision(2, 5)), ("fault", "heartbeat")])
    timeline, last, pending, _ = timeline_from_events(path)
    assert pending == 1
    assert timeline.associate(0, last, mode="revised").map_from_lidar[0, 3] == pytest.approx(.4)


@pytest.mark.parametrize("home", [[0, 0], [float("nan"), 0, 0]])
def test_invalid_frame_declaration_rejected(home):
    with pytest.raises(ValueError):
        base_reference(pose(), pose(), home)


def test_gt_world_label_is_not_glim_map_or_base_frame():
    validate_gt_frame("gt_map")
    for frame in ("map", "base_link", "odom", ""):
        with pytest.raises(ValueError):
            validate_gt_frame(frame)


def test_native_float_clock_uses_ros_nanoseconds_without_tail_extrapolation():
    end = 96.266671687
    queries = acquisition_seconds([dict(observed_stamp=96.26667168736458)])
    assert queries[0] == end
    interpolate_poses([end-.033333333, end], [pose(), pose()], queries)
    with pytest.raises(ValueError, match="extrapolation"):
        interpolate_poses([end-.033333333, end], [pose(), pose()], queries + 1e-9)


def test_native_snapshot_matches_round_truncate_without_extrapolating():
    end = 79.700004156
    frames = [dict(observed_stamp=79.70000415667892, world_from_lidar=pose(.4).tolist())]
    poses, report = published_gt_at_native_ticks(frames, [end-.033333333, end], [pose(), pose()], pose(.4))
    np.testing.assert_array_equal(poses, [pose()])
    assert report["pairs"] == 1
    with pytest.raises(ValueError, match="exact published GT"):
        published_gt_at_native_ticks(frames, [end-.033333333, end-.000001], [pose(), pose()], pose(.4))


def test_interrupted_callback_tail_is_explicit_and_cannot_qualify_completion():
    collection = dict(experiment_completed=False, status=dict(integrated=1725))
    assert validate_projection_tail(collection, 1722, 1722) == 3
    with pytest.raises(ValueError, match="projected input"):
        validate_projection_tail(collection, 1721, 1722)
    with pytest.raises(ValueError, match="counts"):
        validate_projection_tail(dict(collection, experiment_completed=True), 1722, 1722)
    with pytest.raises(ValueError, match="counts"):
        validate_projection_tail(collection, 1726, 1726)
