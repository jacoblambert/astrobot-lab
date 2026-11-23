import copy
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.core import LocalConfig
from test_local_terrain import feed, make, revision, transform


def batch(number, frames, kind=2):
    event = revision(number, frames)
    event["kind"] = kind
    for row in event["frames"]:
        row["deskew_recomputed"] = False
    return event


def test_large_bookkeeping_preserves_pruned_poses_geometry_and_chain():
    core = make(LocalConfig(window_s=2, max_frames=30, allow_bookkeeping=True))
    for i in range(80):
        feed(core, i)
    old_frames = dict(core.timeline.frames)
    before = core.last_result.copy()
    last_record = copy.deepcopy(core.last_record)
    event = batch(81, [(i, i * 10**8, transform(0.01 * i + 1e-14)) for i in range(80)])
    core.revision(event, 79 * 10**8)
    assert core.timeline.frames == old_frames  # No availability/pose changes or resurrection.
    assert len(core.timeline.frames) < 30 and len(core.bookkeeping.frames) == 80
    assert core.timeline.revision == 81 and core.bookkeeping.accepted == 1
    np.testing.assert_array_equal(core.last_result, before)
    assert core.last_record == last_record
    core.revision(event, 79 * 10**8)  # Exact retransmission is idempotent.
    assert core.bookkeeping.accepted == 1
    feed(core, 80, number=82)
    assert core.timeline.revision == 82 and core.status()["valid"]


@pytest.mark.parametrize(
    "fault",
    [
        "pose",
        "old_pose",
        "unknown",
        "stamp",
        "duplicate",
        "extrinsic",
        "nan",
        "shape",
        "deskew",
        "missing_deskew",
        "epoch",
        "parent",
        "snapshot",
    ],
)
def test_changed_or_unverifiable_batches_fail_atomically(fault):
    core = make(LocalConfig(window_s=2, allow_bookkeeping=True))
    for i in range(40):
        feed(core, i)
    event = batch(41, [(i, i * 10**8, transform(0.01 * i)) for i in range(40)], kind=3)
    row = event["frames"][0 if fault == "old_pose" else -1]
    if fault in ("pose", "old_pose"):
        row["map_from_imu"][0][3] += 0.001
    elif fault == "unknown":
        row["frame_id"] = 1000
    elif fault == "stamp":
        row["stamp_ns"] -= 1
    elif fault == "duplicate":
        event["frames"].append(copy.deepcopy(row))
    elif fault == "extrinsic":
        row["imu_from_lidar"][0][3] += 0.01
    elif fault == "nan":
        row["map_from_imu"][0][0] = float("nan")
    elif fault == "shape":
        row["map_from_imu"] = [[1.0]]
    elif fault == "deskew":
        row["deskew_recomputed"] = True
    elif fault == "missing_deskew":
        row.pop("deskew_recomputed")
    elif fault == "epoch":
        event["epoch"] += 1
    elif fault == "parent":
        event["parent"] -= 1
    elif fault == "snapshot":
        event["snapshot"] = True
    with pytest.raises(ValueError):
        core.revision(event, 39 * 10**8)
    assert core.timeline.revision == 40 and core.failure and core.last_result is None


def test_roundoff_cannot_ratchet_and_duplicate_payload_cannot_change():
    for duplicate in (False, True):
        core = make(LocalConfig(allow_bookkeeping=True))
        feed(core, 0)
        core.revision(batch(2, [(0, 0, transform(6e-11))]), 0)
        bad = batch(2 if duplicate else 3, [(0, 0, transform(1.2e-10))])
        with pytest.raises(ValueError):
            core.revision(bad, 0)
        assert core.timeline.revision == 2
        np.testing.assert_array_equal(core.bookkeeping.frames[0].map_from_imu, transform())


def test_ledger_budget_and_immutable_historical_ids_fail_closed():
    core = make(LocalConfig(allow_bookkeeping=True, max_bookkeeping_frames=3))
    for i in range(3):
        feed(core, i)
    with pytest.raises(ValueError, match="ledger budget"):
        feed(core, 3)
    assert core.timeline.revision == 3


def test_bookkeeping_and_odom_cannot_reuse_same_revision():
    core = make(LocalConfig(allow_bookkeeping=True))
    feed(core, 0)
    core.revision(batch(2, [(0, 0, transform())]), 0)
    with pytest.raises(ValueError, match="conflicts"):
        core.revision(revision(2, [(0, 0, transform())]), 0)


def test_enabled_and_disabled_odom_only_outputs_remain_exact():
    a, b = [make(LocalConfig(allow_bookkeeping=enabled)) for enabled in (False, True)]
    for i in range(70):
        feed(a, i)
        feed(b, i)
        if a.last_record:
            x, y = copy.deepcopy(a.last_record), copy.deepcopy(b.last_record)
            x.pop("compute_ms")
            y.pop("compute_ms")
            assert x == y
            np.testing.assert_array_equal(a.last_result, b.last_result)


@pytest.mark.parametrize("fault", ["id_stamp", "stamp_owner"])
def test_pruning_cannot_forget_immutable_historical_identity(fault):
    core = make(LocalConfig(window_s=2, allow_bookkeeping=True))
    for i in range(40):
        feed(core, i)
    assert 0 not in core.timeline.frames
    rows = [(0, 1, transform())] if fault == "id_stamp" else [(100, 0, transform())]
    with pytest.raises(ValueError, match="historical"):
        core.revision(revision(41, rows), 39 * 10**8)


def test_candidate_replay_opt_in_does_not_admit_unrelated_failed_sources(tmp_path):
    import json
    from replay_live_local_terrain import replay

    source = tmp_path / "source"
    source.mkdir()
    (source / "manifest.json").write_text(json.dumps(dict(policy="live_local_v2")))
    for failure in ("simulation wall heartbeat lost", "only contiguous ODOM deltas are supported"):
        (source / "collection.json").write_text(
            json.dumps(dict(experiment_completed=False, status=dict(failure=failure)))
        )
        with pytest.raises(ValueError, match="completed healthy"):
            replay(source, tmp_path)
        if failure != "only contiguous ODOM deltas are supported":
            with pytest.raises(ValueError, match="completed healthy"):
                replay(source, tmp_path, bookkeeping_candidate=True)
