"""Sample-only diagnostics retain unknown/outside exposure and full frame math."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from pathlib import Path
import sys
import json

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from score_live_floor_samples import sample_floor
import score_live_floor_samples as diagnostic
from natural_motion import survey_points
from replay_local_terrain import sha


def test_classes_and_outside_are_all_counted():
    layers = np.zeros((2, 2, 6))
    layers[:, :, 0] = [[0, 20], [100, -1]]
    points = np.array([[0.05, 0.05, 0], [0.15, 0.05, 0], [0.05, 0.15, 0], [0.15, 0.15, 0], [-0.01, 0, 0]])
    record = dict(origin=[0, 0], local_from_map=np.eye(4), resolution=0.1, evidence_ns=1, publication_ns=2)
    result = sample_floor(layers, record, points, [0, 0, 0])
    assert result == dict(queries=5, free=2, occupied=1, unknown=2, outside=1, evidence_ns=1, publication_ns=2)


def test_world_home_and_full_se3_not_base_or_xy_only():
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", [17, 23, 61], degrees=True).as_matrix()
    transform[:3, 3] = [3, -2, 4]
    home = np.array([-20, -20, 0])
    local_points = np.array([[0.05, 0.05, 2], [0.15, 0.05, -1]])
    world_points = (local_points - transform[:3, 3]) @ transform[:3, :3] + home
    layers = np.zeros((1, 2, 6))
    layers[0, :, 0] = [0, 100]
    record = dict(origin=[0, 0], local_from_map=transform, resolution=0.1, evidence_ns=3, publication_ns=4)
    result = sample_floor(layers, record, world_points, home)
    assert result["free"] == result["occupied"] == 1
    assert result["outside"] == result["unknown"] == 0


def test_invalid_declared_home_rejected():
    with pytest.raises(ValueError, match="home"):
        sample_floor(np.zeros((1, 1, 6)), {}, [[0, 0, 0]], [float("nan"), 0, 0])


@pytest.fixture
def source_fixture(tmp_path, monkeypatch):
    source, capture, output = [tmp_path / name for name in ("mapper", "capture", "output")]
    for folder in (source, capture, output):
        folder.mkdir()
    request = dict(capture=capture.name, pattern="straight_slow", home_xyz=[0, 0, 0])
    origin = np.eye(4)
    xy, _ = survey_points(origin, request["pattern"])
    tile = capture / "clearance_tile.npz"
    np.savez_compressed(tile, xy=xy, z=np.zeros(len(xy)), normal=np.tile([0, 0, 1], (len(xy), 1)))
    documents = {
        source / "manifest.json": dict(request=dict(request, stamp=1), config=dict(evidence_lifetime_s=60)),
        capture / "shadow_link.json": dict(request=request),
        capture / "natural_holdout.json": dict(request=request, complete=True),
        capture
        / "clearance.json": dict(
            passed=True, origin=origin.tolist(), pattern=request["pattern"], tile_sha256=sha(tile)
        ),
    }
    for path, document in documents.items():
        path.write_text(json.dumps(document))

    def fake_replay(source, archive, *, projection_observer):
        layers = np.zeros((60, 60, 6))
        record = dict(
            origin=[-3, -3],
            local_from_map=np.eye(4).tolist(),
            resolution=0.1,
            evidence_ns=0,
            publication_ns=200_000_000,
        )
        layers[:, :, 0] = -1
        projection_observer(layers, record, published_valid=True)
        layers[:, :, 0] = 0
        record.update(evidence_ns=61_000_000_000, publication_ns=61_200_000_000)
        projection_observer(layers, record, published_valid=True)
        return dict(passed=True)

    monkeypatch.setattr(diagnostic, "replay", fake_replay)
    return source, capture, tmp_path, output


def test_completed_score_includes_startup_and_expiry_period(source_fixture):
    result = diagnostic.evaluate(*source_fixture)
    assert result["overall"]["queries"] == 2 * result["unique_floor_samples"]
    assert result["first_projection"]["unknown"] == result["unique_floor_samples"]
    assert result["projections_after_evidence_lifetime"] == 1
    # The far edge of the frozen survey also lies outside this fixed grid.
    assert result["last_projection"]["outside"] > 0
    expected_free = result["unique_floor_samples"] - result["last_projection"]["outside"]
    assert result["sampled_floor_supported_fraction"] == expected_free / (2 * result["unique_floor_samples"])
    for name in (
        "acceptance",
        "navigation_authority",
        "continuous_terrain_truth_evaluated",
        "obstacle_recall_evaluated",
        "false_safe_evaluated",
    ):
        assert result[name] is False


def test_completion_does_not_score_stale_supported_map(source_fixture, monkeypatch):
    def completed_replay(source, archive, *, projection_observer):
        layers = np.zeros((60, 60, 6))
        record = dict(
            origin=[-3, -3], local_from_map=np.eye(4).tolist(), resolution=0.1, evidence_ns=1, publication_ns=2
        )
        projection_observer(layers, record, published_valid=False)
        return dict(passed=True)

    monkeypatch.setattr(diagnostic, "replay", completed_replay)
    result = diagnostic.evaluate(*source_fixture)
    assert result["overall"]["free"] == 0
    assert result["overall"]["unknown"] == result["overall"]["queries"]
    assert result["last_projection"]["published_valid"] is False
    assert result["last_valid_projection"] is None


@pytest.mark.parametrize("fault", ["failure", "incomplete", "identity", "hash", "denominator"])
def test_bad_reference_rejected_before_reconstruction(source_fixture, monkeypatch, fault):
    source, capture, archive, output = source_fixture

    def forbidden_replay(*args, **kwargs):
        pytest.fail("invalid reference reached reconstruction")

    monkeypatch.setattr(diagnostic, "replay", forbidden_replay)
    if fault == "failure":
        (capture / "failure.json").write_text("{}")
    elif fault == "incomplete":
        path = capture / "natural_holdout.json"
        value = json.loads(path.read_text())
        value["complete"] = False
        path.write_text(json.dumps(value))
    elif fault == "identity":
        path = source / "manifest.json"
        value = json.loads(path.read_text())
        value["request"]["home_xyz"] = [1, 0, 0]
        path.write_text(json.dumps(value))
    elif fault == "hash":
        (capture / "clearance_tile.npz").write_bytes(b"changed")
    else:
        tile = capture / "clearance_tile.npz"
        with np.load(tile) as data:
            xy, z, normal = data["xy"].copy(), data["z"].copy(), data["normal"].copy()
        np.savez_compressed(tile, xy=xy[1:], z=z[1:], normal=normal[1:])
        path = capture / "clearance.json"
        value = json.loads(path.read_text())
        value["tile_sha256"] = sha(tile)
        path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match="source|denominator"):
        diagnostic.evaluate(source, capture, archive, output)
