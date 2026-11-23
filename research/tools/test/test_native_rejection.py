import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from generic_lidar_adapter import NativeWindow
from native_rejection import guarded_push, recast_rejected
from diagnose_native_rejection import inspect_arrays
from test_generic_lidar_adapter import tick


def test_rejected_bytes_preserved_and_stream_still_fails(tmp_path):
    adapter = NativeWindow()
    arrays = list(tick(adapter))
    arrays[0][0, 0] += 0.1
    pose = dict(world_from_base=np.eye(4), base_from_lidar=np.eye(4))
    with pytest.raises(ValueError, match="association mismatch"):
        guarded_push(adapter, 0, 1.0, arrays, tmp_path, pose)
    assert adapter.failed and not adapter.pending
    report = json.loads((tmp_path / "rejected_native_tick.json").read_text())
    assert not report["acceptance"] and not report["published"]
    with np.load(tmp_path / report["file"]) as data:
        np.testing.assert_array_equal(data["points"], arrays[0])
        assert data["stamp"] == 1.0 and data["producer_sequence"] == 0
    with pytest.raises(ValueError, match="invalidated"):
        adapter.push(0, 1.0, *tick(adapter))


def test_success_unchanged_and_no_rejection_artifact(tmp_path):
    a, b = NativeWindow(), NativeWindow()
    for i in range(6):
        arrays = tick(a)
        result = guarded_push(a, i, 1 + i / 60, arrays, tmp_path, {})
        reference = b.push(i, 1 + i / 60, *arrays)
        assert result == reference
    assert not list(tmp_path.iterdir())


def test_rejection_diagnostic_uses_unchanged_gate_and_keeps_misses_separate():
    arrays = list(tick(NativeWindow()))
    arrays[0][0, 0] += 0.01
    arrays[0][1, 0] += 20.0
    arrays[2][1] = 0
    report = inspect_arrays(*arrays)
    assert report["association_bad_hits"] == 1
    assert report["worst_hits"][0]["index"] == 0
    assert report["worst_hits"][0]["residual_m"] == pytest.approx(0.01)
    assert not report["acceptance"]


def test_recast_keeps_native_arrays_and_exposes_physx_internal_inconsistency():
    arrays = list(tick(NativeWindow()))
    arrays[0][0, 0] += 0.01
    original = [a.copy() for a in arrays]
    pose = dict(world_from_base=np.eye(4), base_from_lidar=np.eye(4))
    pose["world_from_base"][:3, 3] = [20, 12, 0.5]
    calls = []

    def cast(origin, direction, distance):
        calls.append((origin, direction, distance))
        position = np.asarray(origin) + np.asarray(direction) * 2.0012
        return [(2.0, position, (0, 0, 1), "/World/Terrain")]

    report = recast_rejected(arrays, pose, cast)
    assert len(calls) == 2 and report["association_bad_hits"] == 1
    assert not report["acceptance"]
    for ray in report["inspected"][0]["rays"].values():
        assert ray["first_hits"][0]["position_distance_residual_m"] == pytest.approx(0.0012)
        np.testing.assert_array_equal(ray["origin"], [20, 12, 0.5])
    for before, after in zip(original, arrays):
        np.testing.assert_array_equal(before, after)


def test_recast_bounded_and_missing_hits_retained():
    arrays = list(tick(NativeWindow()))
    arrays[0][:25, 0] += 0.01
    pose = dict(world_from_base=np.eye(4), base_from_lidar=np.eye(4))
    report = recast_rejected(arrays, pose, lambda *args: [])
    assert report["association_bad_hits"] == 25
    assert len(report["inspected"]) == 20
    assert all(ray["hit_count"] == 0 for row in report["inspected"] for ray in row["rays"].values())
