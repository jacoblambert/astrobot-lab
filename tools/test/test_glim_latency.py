"""Host NumPy/SciPy contracts: fixed-deadline queries must not use future revisions."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnose_glim_latency import deadline_associations, deadline_coverage, DEADLINE_NS
from shadow_terrain import FramePose, PoseTimeline


def timeline():
    result = PoseTimeline("fixture", 1)
    for revision, arrival, x in ((1, 100_000_000, 1.), (2, 200_000_000, 2.), (3, 400_000_000, 99.)):
        pose = np.eye(4)
        pose[0, 3] = x
        result.apply(session="fixture", epoch=1, revision=revision, parent=revision-1,
                     arrival_ns=arrival, frames=[FramePose(0, 100_000_000, pose, np.eye(4))])
    return result


def test_historical_deadline_does_not_use_final_revision():
    row, = deadline_associations(timeline(), [100_000_000])
    assert row["output_ns"] == 100_000_000 + DEADLINE_NS
    assert row["available_ns"] == 200_000_000
    assert row["revisions"] == [2, 2]
    assert row["map_from_lidar"][0][3] == 2.


@pytest.mark.parametrize("stamps", [[], [100_000_000, 100_000_000], [400_000_000], [0], [100_000_000, 150_000_000]])
def test_incomplete_or_ambiguous_sequence_is_not_trimmed(stamps):
    with pytest.raises(ValueError):
        deadline_associations(timeline(), stamps)


@pytest.mark.parametrize("excess_ns", [0, 10, 50_000_000])
def test_deadline_diagnostics_keep_even_nanosecond_excess_and_all_samples(excess_ns):
    value = PoseTimeline("fixture", 1)
    value.apply(session="fixture", epoch=1, revision=1, parent=0,
                arrival_ns=100_000_000+DEADLINE_NS+excess_ns,
                frames=[FramePose(0, 100_000_000, np.eye(4), np.eye(4))])
    report = deadline_coverage(value, [100_000_000])
    assert report["samples"] == 1 and report["samples_dropped"] == 0
    assert report["strict_deadline_proven"] == (excess_ns == 0)
    assert report["tolerance_added_ns"] == 0
    if excess_ns:
        assert report["misses"][0]["upper_bound_excess_ns"] == excess_ns
        with pytest.raises(ValueError, match="bracket"):
            deadline_associations(value, [100_000_000])


def test_deadline_diagnostics_do_not_hide_missing_tail_or_missing_bracket():
    report = deadline_coverage(timeline(), [0, 100_000_000, 400_000_000])
    assert report["samples"] == 3 and report["missed_samples"] == 2
    assert [r["index"] for r in report["misses"]] == [0, 2]
    assert "eventual_failure" in report["misses"][0]


def test_recording_clock_tail_can_continue_after_last_revision_without_future_pose():
    value = PoseTimeline("fixture", 1)
    value.apply(session="fixture", epoch=1, revision=1, parent=0, arrival_ns=100_000_000,
                frames=[FramePose(0, 100_000_000, np.eye(4), np.eye(4))])
    assert not deadline_coverage(value, [100_000_000])["strict_deadline_proven"]
    assert deadline_coverage(value, [100_000_000], 400_000_000)["strict_deadline_proven"]
    assert len(deadline_associations(value, [100_000_000], 400_000_000)) == 1
    with pytest.raises(ValueError, match="extrapolation"):
        deadline_associations(value, [150_000_000], 400_000_000)
    with pytest.raises(ValueError, match="precedes"):
        deadline_associations(value, [100_000_000], 90_000_000)


def test_export_retains_audited_clock_tail_separate_from_last_revision(tmp_path, monkeypatch):
    import json
    import export_shadow_trajectory as exporter
    source, output = tmp_path / "source", tmp_path / "output"
    (source / "bag").mkdir(parents=True); output.mkdir()
    (source / "collection.json").write_text(json.dumps(dict(collection_completed=True, failure=None)))
    (source / "bag/metadata.yaml").write_text("fixture")
    (source / "bag/bag_0.mcap").write_bytes(b"mock serialized input")
    record = dict(session="fixture", epoch=1, revision=1, parent=0, snapshot=False,
                  frames=[dict(frame_id=i, stamp_ns=stamp, map_from_imu=np.eye(4).tolist(),
                               imu_from_lidar=np.eye(4).tolist()) for i, stamp in enumerate([0, 50_000_000])])
    events = [(1, "clock", 0), (2, "revision", record), (3, "clock", 100_000_000),
              (4, "clock", 200_000_000), (5, "clock", 300_000_000)]
    monkeypatch.setattr(exporter, "check_run", lambda *args: None)
    monkeypatch.setattr(exporter, "bag_events", lambda *args: iter(events))
    result = exporter.export(source, output)
    assert result["recording_end_clock_ns"] == 300_000_000
    assert json.loads((output / "trajectory.jsonl").read_text())["arrival_ns"] == 100_000_000
