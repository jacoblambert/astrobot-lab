from pathlib import Path
import sys
import json

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from measure_exported_glim import evaluate_poses
from shadow_terrain import FramePose, PoseTimeline
import measure_exported_glim as diagnostic


@pytest.mark.parametrize("fault", ["failure_marker", "incomplete", "failed_export", "changed_source"])
def test_no_completed_route_diagnostic_from_failed_or_changed_recordings(tmp_path, monkeypatch, fault):
    capture, recorder, trajectory, output = [tmp_path / n for n in ("capture", "recorder", "trajectory", "output")]
    for path in (capture, recorder, trajectory, output):
        path.mkdir()
    monkeypatch.setattr(diagnostic, "check_run", lambda path: None)
    (recorder / "collection.json").write_text(json.dumps(dict(collection_completed=fault != "incomplete")))
    (capture / "natural_holdout.json").write_text(json.dumps(dict(complete=True)))
    (recorder / "input").write_text("changed")
    (trajectory / "shadow_trajectory.json").write_text(
        json.dumps(dict(passed=fault != "failed_export", source=str(recorder), input_sha256={"input": "incorrect"}))
    )
    if fault == "failure_marker":
        (capture / "failure.json").write_text("{}")
    with pytest.raises(ValueError):
        diagnostic.measure(capture, recorder, trajectory, output)
    assert list(output.iterdir()) == []


def fixture():
    mount = np.eye(4)
    mount[:3, 3] = [0.4, 0, 0.25]
    imu = np.eye(4)
    imu[:3, 3] = [-0.07, 0.03, 0.1]
    home = [-20.0, -20.0, 0.0]
    times = np.arange(41, dtype=np.int64) * 50_000_000
    truth = np.tile(np.eye(4), (len(times), 1, 1))
    truth[:, 0, 3] = 20 + times / 1e9 * 0.05
    truth[:, 1, 3] = 18
    timeline = PoseTimeline("session", 1)
    frames = []
    for i, (t, pose) in enumerate(zip(times, truth)):
        estimate = pose.copy()
        estimate[:3, 3] -= home
        estimate[0, 3] += 0.03  # Must remain a measured error, never fitted away.
        timeline.apply(
            session="session",
            epoch=1,
            revision=i + 1,
            parent=i,
            arrival_ns=int(t) + 100_000_000,
            frames=[FramePose(i, int(t), estimate @ imu, np.linalg.inv(imu) @ mount)],
        )
        frames.append(dict(observed_stamp=t / 1e9, world_from_lidar=(pose @ mount).tolist()))
    request = dict(base_from_lidar=mount.tolist(), base_from_imu=imu.tolist(), home_xyz=home)
    return timeline, frames, request, times, truth


def test_fixed_frame_errors_retain_all_ticks_and_real_offset():
    result, arrays, rows, gt = evaluate_poses(*fixture())
    assert len(rows) == gt["pairs"] == 41
    assert arrays["gt_base"][0, 0, 3] == 40
    for mode in ("first_published", "final_revised"):
        assert result[mode]["overall"]["horizontal_m"]["absolute_p95"] == pytest.approx(0.03)
        assert result[mode]["relative_1s"]["horizontal_m"]["absolute_max"] < 1e-12
        assert "targets_pass" not in result[mode]["overall"]


def test_missing_gt_tick_is_not_interpolated_or_dropped():
    timeline, frames, request, times, truth = fixture()
    with pytest.raises(ValueError, match="exact published GT"):
        evaluate_poses(timeline, frames, request, np.delete(times, 10), np.delete(truth, 10, axis=0))


def test_wrong_imu_mount_rejected():
    args = list(fixture())
    args[2]["base_from_imu"][0][3] += 0.1
    with pytest.raises(ValueError, match="extrinsics"):
        evaluate_poses(*args)


def test_missing_final_glim_bracket_is_not_trimmed():
    timeline, frames, request, times, truth = fixture()
    pose = truth[-1].copy()
    frames.append(dict(observed_stamp=2.1, world_from_lidar=(pose @ request["base_from_lidar"]).tolist()))
    with pytest.raises(ValueError, match="extrapolation forbidden"):
        evaluate_poses(timeline, frames, request, np.r_[times, 2_100_000_000], np.concatenate([truth, pose[None]]))
