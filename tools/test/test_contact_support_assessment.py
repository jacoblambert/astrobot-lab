"""Replay timing must match the mapper, not the bag recorder's callback timing."""

import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assess_contact_support import apply_mapper_revision, extended_motions, mapper_revisions
from assess_contact_support import retraced_motion
from assess_contact_support import compact_pose_history
from local_terrain.contact_support import SupportAnchor
from local_terrain.bookkeeping import BookkeepingLedger
from local_terrain.pose_lookup import PublicationPoseLookup
from shadow_terrain import PoseTimeline


def revision(number, x=0.0, kind=1):
    pose = np.eye(4)
    pose[0, 3] = x
    return dict(
        session="s",
        epoch=1,
        revision=number,
        parent=number - 1,
        snapshot=False,
        kind=kind,
        arrival_ns=number * 10,
        frames=[
            dict(
                frame_id=0,
                stamp_ns=0,
                map_from_imu=pose.tolist(),
                imu_from_lidar=np.eye(4).tolist(),
                deskew_recomputed=False,
            )
        ],
    )


def test_mapper_uses_next_received_clock_not_previous_clock(tmp_path):
    path = tmp_path / "events.jsonl"
    events = [
        dict(kind="clock", payload=5),
        dict(kind="revision", payload=revision(1)),
        dict(kind="chunk", payload={}),
        dict(kind="clock", payload=10),
        dict(kind="revision", payload=revision(2)),
        dict(kind="clock", payload=20),
    ]
    path.write_text("\n".join(map(json.dumps, events)))
    assert [r["arrival_ns"] for r in mapper_revisions(path)] == [10, 20]
    path.write_text(json.dumps(dict(kind="revision", payload=revision(1))))
    with pytest.raises(ValueError, match="unbounded"):
        list(mapper_revisions(path))


def test_corrected_history_is_receipt_causal_and_bookkeeping_not_a_new_pose():
    timeline, ledger = PoseTimeline("s", 1), BookkeepingLedger(100)
    apply_mapper_revision(timeline, ledger, revision(1))
    apply_mapper_revision(timeline, ledger, revision(2, kind=2))
    assert timeline.revision == 2 and ledger.accepted == 1
    assert PublicationPoseLookup(timeline, 20).associate(0).available_ns == 10
    apply_mapper_revision(timeline, ledger, revision(3, x=1))
    assert PublicationPoseLookup(timeline, 20).associate(0).map_from_lidar[0, 3] == 0
    assert PublicationPoseLookup(timeline, 30).associate(0).map_from_lidar[0, 3] == 1
    with pytest.raises(ValueError, match="canonical"):
        apply_mapper_revision(timeline, ledger, revision(4, x=2, kind=3))


def test_extended_motions_keep_original_candidates_and_include_reverse():
    from assess_navigation_gaps import candidate_motions

    start = np.eye(4)
    result = extended_motions(start)
    for name, poses in candidate_motions(start).items():
        np.testing.assert_array_equal(result[name], poses)
    for distance in (0.05, 0.1, 0.25, 0.5):
        assert result[f"reverse_{distance:.2f}"][-1][0, 3] == -distance
    for degrees in (-5, 5, -15, 15, -30, 30):
        assert f"turn_{degrees:+d}deg" in result


def test_retrace_retains_actual_skid_path_and_requires_meaningful_net_motion():
    poses = [np.eye(4) for _ in range(3)]
    poses[1][0, 3] = 0.03
    poses[1][1, 3] = 0.01
    poses[2][0, 3] = 0.06
    anchors = [SupportAnchor(i * 100_000_000, (), "traversed") for i in range(3)]
    resolve = lambda stamp: (poses[stamp // 100_000_000], 0.0)
    result = retraced_motion(poses[2], 200_000_000, anchors, resolve)
    np.testing.assert_array_equal(result, poses[::-1])
    assert retraced_motion(poses[2], 200_000_000, anchors, lambda _: None) is None
    assert retraced_motion(poses[2], 200_000_000, anchors[:1], resolve) is not None
    assert retraced_motion(poses[2], 400_000_000, anchors[:1], resolve) is None
    stationary = lambda _: (poses[2], 0.0)
    assert retraced_motion(poses[2], 200_000_000, anchors, stationary) is None


def test_compaction_keeps_same_latest_causal_historical_pose_and_bounded_versions():
    original, compacted = PoseTimeline("s", 1), PoseTimeline("s", 1)
    ledgers = [BookkeepingLedger(100), BookkeepingLedger(100)]
    for number in range(1, 21):
        for timeline, ledger in zip((original, compacted), ledgers):
            apply_mapper_revision(timeline, ledger, revision(number, x=number * 0.01))
        compact_pose_history(compacted, 100)
        before = PublicationPoseLookup(original, number * 10).associate(0)
        after = PublicationPoseLookup(compacted, number * 10).associate(0)
        np.testing.assert_array_equal(before.map_from_lidar, after.map_from_lidar)
        assert before.available_ns == after.available_ns
        assert len(compacted.frames[0]) == 1
    assert len(original.frames[0]) == 20
    with pytest.raises(ValueError, match="budget"):
        compact_pose_history(compacted, 0)
