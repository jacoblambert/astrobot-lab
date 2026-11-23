import copy
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from export_shadow_trajectory import apply_record
from live_revision_gate import LiveRevisionGate
from shadow_terrain import PoseTimeline


def record(revision=1, count=4):
    return dict(
        session="test",
        epoch=1,
        revision=revision,
        parent=revision - 1,
        arrival_ns=1000 + revision,
        snapshot=False,
        frames=[
            dict(frame_id=i, stamp_ns=i, map_from_imu=np.eye(4).tolist(), imu_from_lidar=np.eye(4).tolist())
            for i in range(count)
        ],
    )


def apply(gate, value):
    return gate.apply_record(value) if isinstance(gate, LiveRevisionGate) else apply_record(gate, value)


def test_large_batch_matches_reference_digest_and_does_not_retain_pose_history():
    reference, live = PoseTimeline("test", 1), LiveRevisionGate("test", 1)
    for revision, count in [(1, 1835), (2, 1835), (3, 1836)]:
        value = record(revision, count)
        value["arrival_ns"] = 10000 + revision
        value["frames"][0]["map_from_imu"][0][3] = revision * 0.1
        for gate in (reference, live):
            assert apply(gate, value)
            assert not apply(gate, value)
        assert reference.events == live.events
        assert reference.stamps == live.stamps
        np.testing.assert_array_equal(reference.extrinsic, live.extrinsic)
    assert len(live.frame_stamps) == 1836
    assert not hasattr(live, "frames") and not hasattr(live, "associate")


@pytest.mark.parametrize(
    "mutation",
    [
        "epoch",
        "gap",
        "backward_arrival",
        "measurement_after_arrival",
        "changed_stamp",
        "duplicate_stamp",
        "duplicate_frame",
        "mount",
        "nonrigid",
        "nan",
        "snapshot_omission",
        "conflicting_duplicate",
        "boolean_id",
    ],
)
def test_same_fail_closed_contract_and_atomic_state_as_reference(mutation):
    reference, live = PoseTimeline("test", 1), LiveRevisionGate("test", 1)
    for gate in (reference, live):
        apply(gate, record())
    bad = record(2)
    if mutation == "epoch":
        bad["epoch"] = 2
    elif mutation == "gap":
        bad["revision"] = 3
    elif mutation == "backward_arrival":
        bad["arrival_ns"] = 1000
    elif mutation == "measurement_after_arrival":
        bad["frames"][0]["stamp_ns"] = 2000
    elif mutation == "changed_stamp":
        bad["frames"][0]["stamp_ns"] = 99
    elif mutation == "duplicate_stamp":
        bad["frames"][0]["stamp_ns"] = 1
    elif mutation == "duplicate_frame":
        bad["frames"].append(copy.deepcopy(bad["frames"][0]))
    elif mutation == "mount":
        bad["frames"][0]["imu_from_lidar"][0][3] = 0.01
    elif mutation == "nonrigid":
        bad["frames"][0]["map_from_imu"][0][0] = 1.01
    elif mutation == "nan":
        bad["frames"][0]["map_from_imu"][0][3] = float("nan")
    elif mutation == "snapshot_omission":
        bad["snapshot"] = True
        bad["frames"].pop()
    elif mutation == "conflicting_duplicate":
        bad["revision"] = 1
        bad["parent"] = 0
        bad["frames"][0]["map_from_imu"][0][3] = 0.1
    elif mutation == "boolean_id":
        bad["frames"][0]["frame_id"] = True
    for gate in (reference, live):
        before = (gate.revision, gate.last_arrival_ns, dict(gate.stamps), dict(gate.events))
        with pytest.raises(ValueError):
            apply(gate, bad)
        assert before == (gate.revision, gate.last_arrival_ns, gate.stamps, gate.events)


def test_snapshot_recovery_and_empty_delta_match_reference():
    reference, live = PoseTimeline("test", 1), LiveRevisionGate("test", 1)
    values = [record(), dict(record(4), snapshot=True), record(5, count=0)]
    for value in values:
        for gate in (reference, live):
            assert apply(gate, value)
    assert reference.events == live.events and reference.stamps == live.stamps
