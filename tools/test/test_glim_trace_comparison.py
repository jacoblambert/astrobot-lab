import copy
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from compare_glim_traces import compare
from diagnose_glim_accuracy import trace_stages


def records():
    return [dict(stage="processed_input", stamp_ns=1, points=20, xyz_fnv1a=4, neighbors_fnv1a=5,
                 times_fnv1a=6, imu_count=3, last_imu_stamp_ns=2),
            *[dict(stage=s, frame_id=1, map_from_imu=np.eye(4).tolist()) for s in ("imu_prediction", "optimized")],
            dict(stage="pre_smoother", poses=[], factors=[dict(keys=[(ord("x") << 56)+1], gps_position=[0, 0, 0])])]


def test_identical_trace_and_accuracy_parser():
    result = compare(records(), records())
    assert result["xyz_fnv1a"]["different_frames"] == 0
    assert result["gps"]["different_measurements"] == 0
    assert trace_stages(records())[1]["gps_factors"] == 1


@pytest.mark.parametrize("field", ["xyz_fnv1a", "neighbors_fnv1a", "times_fnv1a", "imu_count", "last_imu_stamp_ns"])
def test_each_input_difference_retained(field):
    a, b = records(), records()
    b[0][field] += 1
    assert compare(a, b)[field] == dict(different_frames=1, first_stamp_ns=1)


def test_factor_and_early_pose_difference_not_hidden():
    a, b = records(), copy.deepcopy(records())
    b[1]["map_from_imu"][0][3] = .01
    b[-1]["factors"][0]["gps_position"][0] = .05
    result = compare(a, b)
    assert result["imu_prediction"]["first_frame_id"] == 1
    assert result["gps"]["different_measurements"] == 1


def test_incomplete_duplicate_trace_rejected():
    with pytest.raises(ValueError, match="missing or duplicate"):
        compare(records(), records()+[records()[0]])
    with pytest.raises(ValueError, match="stamps"):
        rows = records()
        rows[0]["stamp_ns"] = 5
        compare(records(), rows)
