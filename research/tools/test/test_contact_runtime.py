import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.contact_runtime import ContactRuntime, short_candidates, verify_contact_seal


@pytest.mark.parametrize("trial", [False, True])
@pytest.mark.parametrize("current_turn", [False, True])
@pytest.mark.parametrize("hazard_only", [False, True])
def test_runtime_uses_mapper_clock_fences_and_publishes_invalid_after_fault(
    tmp_path, monkeypatch, trial, current_turn, hazard_only
):
    class String:
        def __init__(self, *, data):
            self.data = data

    monkeypatch.setitem(sys.modules, "std_msgs", SimpleNamespace(msg=SimpleNamespace(String=String)))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(String=String))
    messages = []
    node = SimpleNamespace(
        run=tmp_path,
        hashes={},
        request=dict(pattern="costmap1" if hazard_only else "terrain_step", capture="capture") if trial else {},
        core=SimpleNamespace(
            config=SimpleNamespace(max_bookkeeping_frames=8192),
            base_from_imu=np.eye(4),
            last_clock_ns=0,
            completed=True,
            failure=None,
        ),
        create_publisher=lambda *args: SimpleNamespace(publish=lambda m: messages.append(json.loads(m.data))),
        get_logger=lambda: SimpleNamespace(error=lambda text: None),
    )
    runtime = ContactRuntime(
        node,
        deployment_confirmed=True,
        startup_turn_area=trial,
        current_turn_area=current_turn,
        hazard_only=hazard_only,
    )
    if hazard_only:

        def forbidden(*args, **kwargs):
            raise AssertionError("unknown-permitted commands must not build support history")

        monkeypatch.setattr(runtime.advisory.memory, "placed", forbidden)
        monkeypatch.setattr(runtime.advisory.memory, "observe", forbidden)
        monkeypatch.setattr(runtime.advisory.index, "update", forbidden)
    frames = [
        dict(
            frame_id=i,
            stamp_ns=i * 100_000_000,
            map_from_imu=np.eye(4).tolist(),
            imu_from_lidar=np.eye(4).tolist(),
            deskew_recomputed=False,
        )
        for i in (0, 1)
    ]
    revision = dict(session="s", epoch=1, revision=1, parent=0, snapshot=False, kind=1, frames=frames)
    record = dict(
        session="s",
        epoch=1,
        revision=1,
        evidence_ns=0,
        publication_ns=200_000_000,
        origin=[-3, -3],
        resolution=0.1,
        local_from_map=np.eye(4).tolist(),
    )
    layers = np.zeros((60, 60, 6))
    layers[:, :, 0] = -1
    runtime.receive(revision)
    assert runtime.timeline is None
    runtime.clock(200_000_000, layers, record)
    assert runtime.timeline.frames[0][0][0] == 200_000_000
    advisory_message = [m for m in messages if "motions" in m][-1]
    assert advisory_message["valid"] and advisory_message["motions"]["stationary"]["state"] == "available"
    assert advisory_message.get("current_turn_area_assumed", False) == (current_turn and not hazard_only)
    assert json.loads((tmp_path / "contact_manifest.json").read_text())["current_turn_area"] == current_turn
    assert len(advisory_message["motions"]) == 5 and not advisory_message["navigation_authority"]
    if trial:
        assert messages[-1]["protocol"] == ("costmap_unknown_trial_v1" if hazard_only else "terrain_step_trial_v1")
        assert messages[-1]["linear"] == (0.0 if hazard_only else 0.02)
        assert messages[-1]["decision"]["state"] == "available"
    runtime.clock(800_000_000, layers, record)
    assert runtime.failure == "stale terrain snapshot"
    assert not messages[-1]["valid"] and not messages[-1]["motions"]
    runtime.finish()
    collection = json.loads((tmp_path / "contact_collection.json").read_text())
    assert not collection["completed"] and collection["frames"] == 1
    assert collection["output_sha256"] == hashlib.sha256((tmp_path / "contact_advisory.jsonl").read_bytes()).hexdigest()
    if trial:
        proposals = [json.loads(s) for s in (tmp_path / "step_trial.jsonl").read_text().splitlines()]
        assert proposals[-1]["valid"] is False and proposals[-1]["linear"] == 0
        assert not json.loads((tmp_path / "step_trial_collection.json").read_text())["completed"]


def test_short_query_candidates_have_no_command_authority_or_large_step():
    motions = short_candidates()
    assert len(motions) == 5
    for poses in motions.values():
        np.testing.assert_array_equal(poses[0], np.eye(4))
        assert np.linalg.norm(poses[-1][:3, 3]) <= 0.05


def test_refusal_snapshot_records_actual_uncovered_geometry_and_terminal(tmp_path):
    from local_terrain.physx_reference import BodyBox
    from shapely.geometry import shape

    runtime = ContactRuntime.__new__(ContactRuntime)
    runtime.node = SimpleNamespace(run=tmp_path)
    runtime.trial_snapshots = set()
    runtime.trial_timings = []
    proposal = dict(decision=dict(state="unknown"), terminal=None, relative_poses=[np.eye(4).tolist()])
    runtime.trial = SimpleNamespace(update=lambda *args: dict(proposal))
    runtime.emit_trial = lambda value: None
    patch = BodyBox((0, 0, 0), (0.04, 0.06, 0.001), "test")
    runtime.advisory = SimpleNamespace(
        context=SimpleNamespace(poses=lambda p: p, local_from_base=np.eye(4)),
        patches=[patch],
        body=patch,
        index=SimpleNamespace(unions={}),
    )
    record = dict(origin=[-1, -1], resolution=0.1, local_from_map=np.eye(4).tolist())
    layers = np.zeros((20, 20, 6))
    layers[:, :, 0] = -1
    runtime.record_trial(1, layers, record)
    first = json.loads((tmp_path / "first_refusal.json").read_text())
    assert sum(f["area_m2"] for f in first["fragments"]) == pytest.approx(0.08 * 0.12)
    for fragment in first["fragments"]:
        assert shape(fragment["geometry"]).area == pytest.approx(fragment["area_m2"])
    np.testing.assert_array_equal(np.load(tmp_path / "first_refusal.npz")["layers"], layers)
    proposal["terminal"] = "terrain_refused"
    runtime.record_trial(2, layers, record)
    assert (tmp_path / "terminal_refusal.json").exists()
    assert runtime.trial_snapshots == {"first_refusal", "terminal_refusal"}


def test_contact_seal_rejects_missing_or_failed_advisory(tmp_path):
    run = tmp_path / "revision-abc"
    run.mkdir()
    status = dict(archive=str(run))
    (run / "contact_advisory.jsonl").write_text("{}\n")
    report = dict(
        completed=True, failure=None, frames=10, query_batches=1, output_sha256=hashlib.sha256(b"{}\n").hexdigest()
    )
    (run / "contact_collection.json").write_text(json.dumps(report))
    assert verify_contact_seal(tmp_path, status)["completed"]
    report["completed"] = False
    (run / "contact_collection.json").write_text(json.dumps(report))
    with pytest.raises(ValueError, match="complete"):
        verify_contact_seal(tmp_path, status)
