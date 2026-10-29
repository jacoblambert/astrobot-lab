"""Local v2 contracts: host NumPy/SciPy/pytest; real core via TERRAIN_LIBRARY."""

import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from local_terrain.core import LocalConfig, LocalReconstructor
from local_terrain.evaluation import query_grid, independent_score, relative_error, stopping_distance
from diagnose_local_motion import window_errors
from diagnose_local_reconstruction import evaluate as evaluate_coherent
from replay_local_terrain import sha
from native_chunk_transport import acquisition_stamp_ns
from terrain_shadow_runtime import ReceiptScheduler


def transform(x=0, yaw=0):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    result[0, 3] = x
    return result


def revision(number, frames):
    return dict(
        kind=1,
        session="test",
        epoch=0,
        revision=number,
        parent=number - 1,
        snapshot=False,
        frames=[
            dict(frame_id=i, stamp_ns=stamp, map_from_imu=pose.tolist(), imu_from_lidar=transform(0.4).tolist())
            for i, stamp, pose in frames
        ],
    )


class FakeMapper:
    instances = []
    interrupt = False

    def __init__(self, origin, lifetime):
        self.origin, self.lifetime = origin, lifetime
        self.points, self.sensors, self.stamps = [], [], []
        self.closed = False
        self.instances.append(self)

    def integrate(self, points, stamp, sensor, yaw):
        self.points.append(points)
        self.sensors.append(sensor)
        self.stamps.append(stamp)

    def project(self, stamp):
        if self.interrupt:
            raise KeyboardInterrupt("test interruption")
        result = np.zeros((60, 60, 6), dtype=np.float32)
        result[:, :, 2] = np.mean(np.concatenate(self.points)[:, 0]) if self.points else 0
        return result

    def close(self):
        self.closed = True


def make(config=None, factory=FakeMapper):
    return LocalReconstructor(factory, np.eye(4), config)


@pytest.mark.parametrize("legacy", [False, True])
def test_real_one_nanosecond_future_tick_uses_clock_quantization_without_relaxing_deadline(legacy):
    acquired = 158.05000824294984
    clock_ns = 158050008242  # Actual next received /clock in the failed live run.
    core = make()
    runtime = ReceiptScheduler(core)
    stamp = round(acquired * 1e9) if legacy else acquisition_stamp_ns(acquired)
    runtime.receive(
        "chunk",
        dict(capture="test", sequence=0, stamp_ns=stamp, points=np.array([[1.0, 0.0, 0.0]], dtype=np.float32)),
        40,
    )
    if legacy:
        with pytest.raises(ValueError, match="native acquisition deadline exceeded"):
            runtime.clock(clock_ns)
        assert core.failure
    else:
        runtime.clock(clock_ns)
        assert not core.failure and core.received == 1 and core.last_input_ns == clock_ns


def feed(core, i, *, number=None, pose=None):
    stamp = i * 100_000_000
    core.revision(
        revision(i + 1 if number is None else number, [(i, stamp, transform(0.01 * i) if pose is None else pose)]),
        stamp,
    )
    core.chunk(
        dict(
            capture="test",
            sequence=i,
            stamp_ns=stamp,
            points=np.array([[1.0, 0.0, 0.0], [1.0, 0.1, 0.0]], dtype=np.float32),
        ),
        stamp,
    )
    return core.advance(stamp)


def test_reconstruction_is_deterministic_and_bounded():
    cores = [make(LocalConfig(window_s=2)) for _ in range(2)]
    hashes = []
    for core in cores:
        results = []
        for i in range(60):
            if feed(core, i) is not None:
                results.append(core.last_record["layers_sha256"])
        hashes.append(results)
        assert len(core.observations) <= 23
        assert len(core.timeline.frames) <= 24
        assert core.last_record["age_s"] == 0.2
        assert core.last_record["observations"] == 21
    assert hashes[0] == hashes[1]


def test_gauge_change_cancels_but_relative_revision_changes_geometry():
    core = make()
    for i in range(4):
        feed(core, i)
    before = core.local_from_map.copy()
    anchor_pose = core.timeline.frames[core.anchor][-1][2].map_from_imu
    gauge = transform(7, 0.3)
    frames = [(i, i * 100_000_000, gauge @ transform(0.01 * i)) for i in range(4)]
    core.revision(revision(5, frames), 300_000_000)
    np.testing.assert_allclose(core.local_from_map @ gauge, before, atol=1e-12)
    np.testing.assert_allclose(
        core.local_from_map @ core.timeline.frames[core.anchor][-1][2].map_from_imu, before @ anchor_pose, atol=1e-12
    )
    # A relative correction away from the shared anchor must NOT be removed.
    core.revision(revision(6, [(0, 0, gauge @ transform(0.12))]), 300_000_000)
    core.advance(400_000_000)
    assert FakeMapper.instances[-1].points[0][0, 0] == pytest.approx(1.52)
    assert core.last_record["revision"] == 6


def test_lever_arm_is_rotated_after_imu_interpolation():
    core = make()
    core.revision(revision(1, [(0, 0, transform()), (1, 100_000_000, transform(yaw=np.pi))]), 100_000_000)
    core.chunk(dict(capture="s", sequence=0, stamp_ns=50_000_000, points=np.array([[1.0, 0, 0]])), 100_000_000)
    core.advance(100_000_000)
    # Direct association tests the production helper used by reconstruction.
    pose = core.timeline.associate(50_000_000, 100_000_000, mode="revised").map_from_lidar
    np.testing.assert_allclose(pose[:3, 3], [0, 0.4, 0], atol=1e-12)


def test_frozen_observation_ablation_only_changes_old_placements():
    cores = [make(LocalConfig(observation_policy=p)) for p in ("coherent", "frozen_observation")]
    for core in cores:
        for i in range(4):
            feed(core, i)
        # Change an old observation without changing the shared anchor/current rover.
        core.revision(revision(5, [(0, 0, transform(0.12))]), 300_000_000)
        core.advance(400_000_000)
    a, b = [core.last_record for core in cores]
    np.testing.assert_array_equal(a["local_from_base"], b["local_from_base"])
    np.testing.assert_array_equal(a["origin"], b["origin"])
    assert a["observations"] == b["observations"] == 3
    assert a["layers_sha256"] != b["layers_sha256"]
    assert a["associations"][0]["local_from_lidar"][0][3] == pytest.approx(0.52)
    assert b["associations"][0]["local_from_lidar"][0][3] == pytest.approx(0.4)
    assert b["associations"][0]["output_ns"] == 200_000_000
    assert b["policy"] == "local_frozen_observation_v1"


def test_frozen_observation_survives_common_gauge_without_map_frame_reuse():
    core = make(LocalConfig(observation_policy="frozen_observation"))
    for i in range(4):
        feed(core, i)
    frozen = core.frozen_placements[0][0].copy()
    gauge = transform(7, 0.3)
    core.revision(revision(5, [(i, i * 100_000_000, gauge @ transform(0.01 * i)) for i in range(4)]), 300_000_000)
    core.advance(400_000_000)
    np.testing.assert_array_equal(core.last_record["associations"][0]["local_from_lidar"], frozen)
    np.testing.assert_allclose(core.last_record["local_from_base"], transform(0.02), atol=1e-12)
    assert core.last_record["associations"][2]["local_from_lidar"][0][3] == pytest.approx(0.42)


@pytest.mark.parametrize("terminal", ["close", "fault"])
def test_frozen_placement_lifetime_and_terminal_invalidation(terminal):
    core = make(LocalConfig(window_s=2, observation_policy="frozen_observation"))
    for i in range(50):
        feed(core, i)
    assert len(core.frozen_placements) == 21
    assert min(core.frozen_placements) == 27
    core.close() if terminal == "close" else core.fault("test")
    assert not core.frozen_placements and not core.status()["valid"]


@pytest.mark.parametrize("cells", [60, 200])
def test_grid_ablation_keeps_resolution_and_pose(cells):
    class SizedMapper(FakeMapper):
        def project(self, stamp):
            return np.zeros((cells, cells, 6), dtype=np.float32)

    core = make(LocalConfig(cells=cells), factory=SizedMapper)
    for i in range(4):
        feed(core, i)
    assert core.last_result.shape == (cells, cells, 6)
    assert core.last_record["resolution"] == 0.1
    np.testing.assert_allclose(core.last_record["local_from_base"], transform(0.01))
    assert core.last_record["origin"] == [-cells * 0.05, -cells * 0.05]


@pytest.mark.parametrize("kwargs", [dict(cells=100), dict(cells=60.0), dict(observation_policy="final_revised")])
def test_undeclared_ablation_controls_rejected(kwargs):
    with pytest.raises(ValueError):
        LocalConfig(**kwargs)


@pytest.mark.parametrize("fault", ["kind", "epoch", "parent", "extrinsic", "future", "missing_anchor"])
def test_invalid_revision_or_future_publication_invalidates(fault):
    core = make()
    for i in range(3):
        feed(core, i)
    event = revision(4, [(3, 300_000_000, transform())])
    if fault == "kind":
        event["kind"] = 3
    if fault == "epoch":
        event["epoch"] = 2
    if fault == "parent":
        event["parent"] = 1
    if fault == "extrinsic":
        event["frames"][0]["imu_from_lidar"][0][3] = 0.8
    if fault == "missing_anchor":
        core.timeline.frames.pop(core.anchor)
    with pytest.raises(ValueError):
        core.revision(event, 300_000_000)
        if fault == "future":
            core.advance(250_000_000)
    assert not core.status()["valid"] and core.failure


def test_partial_batch_cannot_change_published_map_or_anchor():
    core = make()
    for i in range(3):
        feed(core, i)
    pose_before = core.timeline.frames[0][-1][2].map_from_imu.copy()
    event = revision(4, [(0, 0, transform(9)), (1, 100_000_000, transform())])
    event["frames"][1]["imu_from_lidar"][0][3] = 1.0
    with pytest.raises(ValueError):
        core.revision(event, 300_000_000)
    np.testing.assert_array_equal(core.timeline.frames[0][-1][2].map_from_imu, pose_before)
    assert core.last_result is None


@pytest.mark.parametrize("mode", ["bytes", "chunks", "frames", "sequence", "stale", "reset"])
def test_resource_and_timing_faults_fail_closed(mode):
    config = LocalConfig(
        max_bytes=20 if mode == "bytes" else 10000,
        max_chunks=1 if mode == "chunks" else 50,
        max_frames=1 if mode == "frames" else 50,
    )
    core = make(config)
    with pytest.raises(ValueError):
        feed(core, 0)
        if mode == "sequence":
            feed(core, 2)
        elif mode == "stale":
            core.advance(300_000_000)
        elif mode == "reset":
            core.advance(0)
            core.advance(-1)
        else:
            feed(core, 1)
    assert not core.status()["valid"]


def test_interrupt_during_rebuild_closes_mapper_and_invalidates(monkeypatch):
    core = make()
    for i in range(3):
        feed(core, i)
    assert core.status()["valid"]
    monkeypatch.setattr(FakeMapper, "interrupt", True)
    with pytest.raises(KeyboardInterrupt):
        feed(core, 3)
    assert FakeMapper.instances[-1].closed
    assert not core.status()["valid"] and core.last_result is None


def test_completion_and_close_are_explicit():
    core = make()
    for i in range(4):
        feed(core, i)
    core.complete(dict(capture="test", sequence=3, stamp_ns=300_000_000), 300_000_000)
    core.advance(400_000_000)
    core.advance(500_000_000)
    assert core.completed
    core.close()
    assert not core.status()["valid"] and core.status()["closed"]
    with pytest.raises(ValueError):
        core.advance(600_000_000)


def test_rolling_negative_grid_coordinates():
    core = make()
    for i in range(5):
        feed(core, i, pose=transform(-0.15 * i))
    np.testing.assert_allclose(core.last_record["origin"], [-3.3, -3])


def test_real_library_reconstructs_supported_flat_ground():
    path = os.getenv("TERRAIN_LIBRARY")
    if not path:
        pytest.skip("TERRAIN_LIBRARY required")
    from layered_terrain import TerrainMapper

    factory = lambda origin, lifetime: TerrainMapper(path, width=60, height=60, origin=origin, lifetime=lifetime)
    core = make(factory=factory)
    x, y = np.meshgrid(np.arange(-2.5, 2.51, 0.05), np.arange(-2.5, 2.51, 0.05))
    points = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, -0.5)]).astype(np.float32)
    for i in range(5):
        stamp = i * 100_000_000
        core.revision(revision(i + 1, [(i, stamp, transform())]), stamp)
        core.chunk(dict(capture="real", sequence=i, stamp_ns=stamp, points=points), stamp)
        core.advance(stamp)
    assert np.count_nonzero(core.last_result[:, :, 0] == 0) > 1000
    assert not np.any(core.last_result[:, :, 0] == 100)


def test_legacy_implementations_preserved_and_shell_syntax():
    directory = ROOT / "tools/legacy/terrain_sweeps"
    for name, expected in json.loads((directory / "original_sha256.json").read_text()).items():
        assert hashlib.sha256((directory / name).read_bytes()).hexdigest() == expected
        for path in (directory / name, ROOT / "tools" / name):
            subprocess.run(["bash", "-n", str(path)], check=True)


def test_legacy_forwarding_is_inspected_without_running_sweeps(tmp_path):
    # PATH intercepts exec bash. The real historical implementations never run.
    stub = tmp_path / "bash"
    stub.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\nexit 37\n')
    stub.chmod(0o755)
    for path in (ROOT / "tools/legacy/terrain_sweeps").glob("*.sh"):
        wrapper = ROOT / "tools" / path.name
        result = subprocess.run(
            ["/bin/bash", str(wrapper), "a b", "--flag"],
            env=dict(os.environ, PATH=str(tmp_path) + os.pathsep + os.environ["PATH"]),
            capture_output=True,
            text=True,
            cwd=tmp_path,
        )
        assert result.returncode == 37
        assert result.stdout.splitlines() == [str(path), "a b", "--flag"]


def test_missing_reference_and_outside_cells_stay_in_denominator():
    layers = np.zeros((60, 60, 6))
    costs, outside = query_grid(layers, [-3, -3], np.eye(4), [[0, 0, 0], [10, 0, 0], [1, 1, 0]])
    np.testing.assert_array_equal(costs, [0, -1, 0])
    score = independent_score(costs, [100, 0, -1])
    assert score == dict(
        queries=3, reference_missing=1, known_free=1, known_hazard=1, false_safe=1, unnecessary_blocked=1
    )
    assert outside.tolist() == [False, True, False]


def test_relative_motion_is_gauge_invariant_not_endpoint_fit():
    gauge = transform(50, 1.0)
    error = relative_error(gauge @ transform(), gauge @ transform(1), transform(), transform(1))
    assert error["translation_m"] < 1e-12
    assert relative_error(transform(), transform(1.1), transform(), transform(1))["translation_m"] == pytest.approx(0.1)


def test_fixed_windows_do_not_bridge_missing_ticks():
    times = np.arange(0, 7, 0.1)
    poses = np.array([transform(t) for t in times])
    indices = np.delete(np.arange(len(times)), 20)
    result = window_errors(times, poses, poses[indices], indices, 2)
    assert result["missing_pairs"] == 2
    assert result["translation_m"]["maximum"] == 0


def test_stopping_distance_needs_real_deceleration():
    assert stopping_distance(0.1, 0.3, 0.2, 0.1) == pytest.approx(0.155)
    with pytest.raises(ValueError):
        stopping_distance(0.1, 0.3, 0, 0.1)


@pytest.mark.parametrize("kind", ["positive", "hole"])
@pytest.mark.parametrize("phase", [0.0, 0.025, 0.05, 0.075])
@pytest.mark.parametrize("rotation", [0.0, 0.17, 0.61])
@pytest.mark.parametrize("height", [0.04, 0.10])
@pytest.mark.parametrize("backend", ["reference", "prepared"])
def test_local_rebuild_preserves_partial_face_and_hole_veto(kind, phase, rotation, height, backend):
    path = os.getenv("TERRAIN_LIBRARY")
    if not path:
        pytest.skip("TERRAIN_LIBRARY required")
    from layered_terrain import TerrainMapper
    from test_terrain_normal_confidence import face_fixture

    points, _, _ = face_fixture(kind, phase, rotation, 0.008, height)
    # Shift the physical scene, not the candidate-dependent query denominator.
    points[:, :2] -= phase
    c, s = np.cos(rotation), np.sin(rotation)
    rotation_matrix = np.array([[c, s], [-s, c]])
    if kind == "positive":
        probes = np.c_[np.full(30, 0.32), np.linspace(0.31, 0.89, 30)]
    else:
        x, y = np.meshgrid(np.arange(0.31, 0.9, 0.02), np.arange(0.31, 0.9, 0.02))
        probes = np.c_[x.ravel(), y.ravel()]
    queries = np.c_[probes @ rotation_matrix - phase, np.zeros(len(probes))]
    core = make(
        LocalConfig(backend=backend),
        factory=lambda origin, lifetime: TerrainMapper(path, width=60, height=60, origin=origin, lifetime=lifetime),
    )
    for i in range(5):
        stamp = i * 100_000_000
        core.revision(revision(i + 1, [(i, stamp, transform())]), stamp)
        core.chunk(
            dict(capture="geometry", sequence=i, stamp_ns=stamp, points=(points - [0.4, 0, 0]).astype(np.float32)),
            stamp,
        )
        output = core.advance(stamp)
        if output is not None:
            costs, _ = query_grid(output, core.last_record["origin"], core.last_record["local_from_base"], queries)
            assert not np.isin(costs, [0, 20]).any()


@pytest.mark.parametrize("policy", ["coherent", "frozen_observation"])
def test_prepared_backend_matches_reference_across_revisions_and_expiry(policy):
    cores = [make(LocalConfig(window_s=2, observation_policy=policy, backend=b)) for b in ("reference", "prepared")]
    for i in range(50):
        for core in cores:
            feed(core, i, pose=transform(0.01 * i, 0.01 * i))
        if i >= 2:
            a, b = [core.last_record for core in cores]
            for key in ("layers_sha256", "associations", "local_from_base", "local_from_map", "origin"):
                assert a[key] == b[key]
    assert cores[1].return_cache.bytes <= 64 * 1024**2
    assert len(cores[1].return_cache.entries) <= 21
    cores[1].fault("invalidate cached geometry")
    assert not cores[1].return_cache.entries and not cores[1].status()["valid"]


@pytest.mark.parametrize("fault", [None, "future", "changed_arrays", "missing_tick"])
def test_coherent_evaluator_same_pairs_and_causal_failures(tmp_path, fault):
    local, accuracy, capture = [tmp_path / name for name in ("local", "accuracy", "capture")]
    for directory in (local, accuracy, capture):
        directory.mkdir()
    link = capture / "shadow_link.json"
    link.write_text(json.dumps(dict(request=dict(base_from_lidar=np.eye(4).tolist()))))
    arrays = accuracy / "accuracy_arrays.npz"
    np.savez(
        arrays,
        times=np.array([0.0, 1.0 if fault != "missing_tick" else 1.00001]),
        gt_base=np.array([transform(), transform(1)]),
        first_published_indices=np.array([0, 1]),
        first_published=np.array([transform(), transform(1.1)]),
    )
    (accuracy / "accuracy_report.json").write_text(
        json.dumps(
            dict(
                diagnostic_completed=True,
                output_sha256={"accuracy_arrays.npz": sha(arrays)},
                input_sha256={str(link): sha(link)},
            )
        )
    )
    poses = [
        dict(
            acquisition_ns=i * 10**9,
            available_ns=2 * 10**9 if fault == "future" else 1_100_000_000,
            output_ns=1_200_000_000,
            map_from_lidar=transform(i).tolist(),
        )
        for i in range(2)
    ]
    projections = local / "projections.jsonl"
    projections.write_text(
        json.dumps(dict(policy="local_latest_available_odom_v1", associations=poses, publication_ns=1_200_000_000))
        + "\n"
    )
    (local / "report.json").write_text(
        json.dumps(
            dict(
                diagnostic_completed=True,
                output_sha256=sha(projections),
                input_sha256={str(link): sha(link)},
                source_failed=True,
                bounded_prefix=False,
                window_s=5,
            )
        )
    )
    if fault == "changed_arrays":
        arrays.write_bytes(arrays.read_bytes() + b"changed")
    if fault:
        with pytest.raises(ValueError):
            evaluate_coherent(local, accuracy, capture)
    else:
        result = evaluate_coherent(local, accuracy, capture)
        assert result["source_failed"] and not result["acceptance"]
        assert result["results"]["coherent_available"]["translation_m"]["p95"] == 0
        assert result["results"]["first_published_same_pairs"]["translation_m"]["p95"] == pytest.approx(0.1)


def test_profile_does_not_change_any_output_channel():
    plain, profile = make(), make(LocalConfig(profile=True))
    for i in range(5):
        a, b = feed(plain, i), feed(profile, i)
        if a is not None:
            np.testing.assert_array_equal(a, b)
    assert set(profile.last_record["stage_ms"]) == {"association", "transform", "integrate", "project"}


def test_canonical_document_paths_and_legacy_index_exist():
    assert "docs/implementation/agent-runbook.md" in (ROOT / "agent-runbook.md").read_text()
    assert (ROOT / "docs/implementation/agent-runbook.md").is_file()
    assert (ROOT / "tools/legacy/README.md").is_file()


def test_resume_after_joins_a_running_stream_once():
    core = make()
    core.resume_after("sim", 41)
    points = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    core.revision(revision(1, [(0, 0, transform())]), 0)
    core.chunk(dict(capture="sim", sequence=42, stamp_ns=0, points=points), 0)
    assert core.received == 1 and core.sequence == 42
    with pytest.raises(ValueError, match="already joined"):
        core.resume_after("sim", 50)
    fresh = make()
    fresh.resume_after("sim", 41)
    with pytest.raises(ValueError, match="discontinuity"):
        fresh.chunk(dict(capture="sim", sequence=44, stamp_ns=0, points=points), 0)


@pytest.mark.parametrize("tolerant", [False, True])
def test_nav_stream_skips_dropped_and_late_chunks_and_pauses_when_stale(tolerant):
    core = make(LocalConfig(tolerate_stream_gaps=True) if tolerant else None)
    points = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    core.revision(revision(1, [(0, 0, transform())]), 0)
    core.chunk(dict(capture="sim", sequence=0, stamp_ns=0, points=points), 0)
    if not tolerant:
        with pytest.raises(ValueError, match="discontinuity"):
            core.chunk(dict(capture="sim", sequence=3, stamp_ns=10, points=points), 10)
        return
    core.chunk(dict(capture="sim", sequence=3, stamp_ns=10, points=points), 10)  # two dropped
    late = 20 + 200_000_001  # received after its acquisition deadline: dropped, not a fault
    core.chunk(dict(capture="sim", sequence=4, stamp_ns=20, points=points), late)
    status = core.status()
    assert core.received == 2 and status["skipped_sequences"] == 2 and status["late_chunks"] == 1
    assert core.advance(late + 300_000_000) is None and core.failure is None  # stale input: no output
    # Duplicates and stamp regressions still fault: they are not a lossy stream.
    with pytest.raises(ValueError, match="discontinuity"):
        core.chunk(dict(capture="sim", sequence=4, stamp_ns=30, points=points), late + 300_000_000)
    regressed = make(LocalConfig(tolerate_stream_gaps=True))
    regressed.chunk(dict(capture="sim", sequence=0, stamp_ns=50, points=points), 50)
    with pytest.raises(ValueError, match="discontinuity"):
        regressed.chunk(dict(capture="sim", sequence=1, stamp_ns=40, points=points), 50)


def test_revision_received_ahead_of_the_clock_waits_for_a_covering_clock():
    core = make()
    runtime = ReceiptScheduler(core)
    points = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    runtime.receive("revision", revision(1, [(0, 100_000_000, transform())]), 10)
    runtime.receive("chunk", dict(capture="test", sequence=0, stamp_ns=50_000_000, points=points), 20)
    runtime.clock(60_000_000)  # a lagging clock: before the revision's measurement
    assert not core.failure and core.timeline is None and core.received == 0
    assert len(runtime.pending) == 2 and runtime.bytes == 30
    runtime.clock(100_000_000)
    assert not core.failure and core.timeline is not None and core.received == 1
    assert core.last_arrival_ns == 100_000_000 and not runtime.pending and runtime.bytes == 0
