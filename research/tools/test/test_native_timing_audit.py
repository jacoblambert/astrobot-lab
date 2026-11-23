import copy
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from audit_native_terrain import summarize_frames


def frames(dt):
    phase = dict(residuals_m=[.00001], missing=0, nonhorizontal_surface_samples=1)
    return [dict(observed_stamp=i * dt, full_columns=2000, columns=1000, configured_rotation_rate=10.,
                 world_time=i * dt, timeline_time=i * .05, world_physics_dt=dt, world_rendering_dt=.05,
                 current_pose=copy.deepcopy(phase), previous_pose=copy.deepcopy(phase)) for i in range(3)]


def test_doubled_sensor_density_is_not_a_cadence_pass():
    report = summarize_frames(frames(.025))
    assert report["observed_scan_rate_hz"] == 20 and not report["cadence_passed"]
    assert report["timeline_time_delta_ratio"] == 2
    assert not report["eligible_for_moving_replay"]
    assert json.loads(json.dumps(report))["cadence_passed"] is False
    assert summarize_frames(frames(.05))["cadence_passed"]


def test_phase_audit_does_not_ignore_worst_residual():
    rows = frames(.05)
    rows[-1]["current_pose"]["residuals_m"] = [.02]
    assert summarize_frames(rows)["current_pose"]["maximum_m"] == .02
    rows[-1]["observed_stamp"] = 0.
    with pytest.raises(ValueError, match="nonmonotonic"):
        summarize_frames(rows)
