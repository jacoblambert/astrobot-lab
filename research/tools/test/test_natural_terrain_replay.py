"""Host NumPy contracts; no ROS, Isaac or archive mount. No terrain acceptance."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import replay_natural_terrain as replay


def test_disagreement_partitions_every_cell_including_unknown():
    a, b = np.zeros((4, 4, 6)), np.zeros((4, 4, 6))
    a[:, :, 0] = np.array([-1, 0, 20, 100])[:, None]
    b[:, :, 0] = [-1, 0, 20, 100]
    assert list(replay.disagreements(a, b).values()) == [1]*16
    assert replay.map_counts(a) == dict(cells=16, unknown=4, free=8, cost_zero=4, cost_twenty=4, occupied=4)
    with pytest.raises(ValueError, match="different terrain grids"):
        replay.disagreements(a, b[:2])
    b[0, 0, 0] = 50
    with pytest.raises(ValueError, match="cost classes"):
        replay.map_counts(b)


def test_reason_attribution_never_counts_gt_pose_unknown_as_known_free():
    reference, candidate = np.zeros((2, 3, 6)), np.zeros((2, 3, 6))
    reference[0, :, 0] = [0, 20, -1]
    candidate[0, :, 0] = -1
    candidate[0, :, 1] = [8, 5, 8]
    reasons = replay.lost_support_reasons(reference, candidate)
    assert sum(reasons.values()) == 2
    assert reasons["NEGATIVE_EDGE"] == 1 and reasons["DISCONNECTED_SURFACE"] == 1
    candidate[1, 0, 1] = 14
    with pytest.raises(ValueError, match="reason code"):
        replay.lost_support_reasons(reference, candidate)


def test_edge_sources_partition_lost_support_and_reference_is_exact():
    reference, candidate = np.zeros((1, 5, 6)), np.zeros((1, 5, 6))
    candidate[:, :, 0], candidate[:, :, 1] = -1, 8
    reference[0, 4, 0] = -1
    cells = np.zeros((1, 5, 4), dtype=np.uint32)
    cells[0, :, 1] = [0, 1, 2, 7, 2]  # raised-obstacle bit does not erase both sources
    assert list(replay.edge_attribution(reference, candidate, cells).values()) == [1]*4
    row = dict(stamp=1., through_native_chunk=3, layers_sha256={"gt": "same"})
    replay.verify_projection(row, dict(row))
    with pytest.raises(ValueError, match="projection differs"):
        replay.verify_projection(row, dict(row, layers_sha256={"gt": "changed"}))


def test_native_boundary_attribution_cannot_be_misreported_as_normal_recovery():
    reference, candidate = np.zeros((1, 8, 6)), np.zeros((1, 8, 6))
    candidate[:, :, 0], candidate[:, :, 1] = -1, 8
    cells = np.zeros((1, 8, 4), dtype=np.uint32)
    cells[0, :, 1] = [0, 1, 2, 7, 8, 9, 10, 15]
    groups = replay.edge_attribution(reference, candidate, cells)
    assert list(groups.values()) == [1]*8
    assert groups["native_only"] == 1
    observations = np.zeros((1, 8, 22))
    observations[:, :, [0, 8, 17]] = 1
    result = replay.observation_attribution(reference, candidate, cells, observations)
    assert result["lost_cells_with_only_qualified_sources"] == 2
    assert result["spatial_context"]["lost_cells_with_only_qualified_sources"] == 2
    cells[0, 0, 1] = 16
    with pytest.raises(ValueError, match="unknown diagnostic"):
        replay.edge_attribution(reference, candidate, cells)


@pytest.mark.parametrize("fault", [None, "failed", "cloud", "stamp", "library", "evaluator"])
def test_replay_same_raw_points_times_and_grid_without_gt_labels(tmp_path, monkeypatch, fault):
    capture, pair, output = [tmp_path / name for name in ("capture", "pair", "output")]
    for path in (capture, pair, output): path.mkdir()
    chunks = capture / "native_chunks"; chunks.mkdir()
    library = tmp_path / "library.so"; library.write_bytes(b"mock library")
    transform = np.eye(4); transform[:3, 3] = [20., 18., .4]
    frames = []
    for index, stamp in enumerate([1., 1.+1/30, 1.+2/30, 1.1]):
        path = chunks / f"chunk_{index:06}.npz"
        np.savez(path, points=np.array([[1., 0., 0.], [.1, 0., 0.]]),
                 observed_stamp=stamp, world_from_lidar=transform)
        frames.append(dict(file=path.name, sha256=replay.sha(path), observed_stamp=stamp,
                           world_from_lidar=transform.tolist()))
    request = dict(world_from_base=transform.tolist())
    (capture / "natural_holdout.json").write_text(json.dumps(dict(complete=True, stationary_only=False, chunks=4, request=request)))
    (capture / "native_timing_audit.json").write_text(json.dumps(dict(frames=frames)))
    (capture / "manifest.json").write_text(json.dumps(dict(sensor=dict(sensor_attributes=dict(minRange="0.5", maxRange="100.0")))))
    (pair / "shadow_pair_audit.json").write_text(json.dumps(dict(experiment="natural_motion_gt_start", fixed_home=[-20., -20., 0.])))
    if fault == "failed": (capture / "failure.json").write_text("{}")
    if fault == "cloud": (chunks / frames[0]["file"]).write_bytes(b"changed")
    if fault == "stamp":
        frames[0]["observed_stamp"] = .9
        (capture / "native_timing_audit.json").write_text(json.dumps(dict(frames=frames)))
    monkeypatch.setattr(replay, "check_run", lambda path: None)
    expected = np.repeat(transform[None], 4, axis=0); expected[:, :2, 3] += 20.
    monkeypatch.setattr(replay, "load_pose_sensitivity", lambda *args: (expected.copy(), dict(test_only=True)))
    made = []
    original_sha = replay.sha
    if fault == "evaluator":
        monkeypatch.setattr(replay, "sha", lambda path:
            "changed" if path.name == "layered_terrain.py" and any(m.closed for m in made) else original_sha(path))
    class Mapper:
        def __init__(self, lib, **geometry):
            self.lib, self.geometry, self.inputs, self.projections = SimpleNamespace(), geometry, [], []
            self.closed = False; made.append(self)
        def integrate(self, points, stamp, sensor, yaw):
            self.inputs.append((points.copy(), stamp, sensor.copy(), yaw))
        def project(self, stamp):
            self.projections.append(stamp)
            return np.zeros((200, 200, 6), dtype=np.float32)
        def close(self):
            self.closed = True
            if fault == "library": library.write_bytes(b"mutated library")
    monkeypatch.setattr(replay, "TerrainMapper", Mapper)
    if fault:
        with pytest.raises(ValueError): replay.run(capture, pair, library, output)
        assert not (output / "natural_terrain_sensitivity.json").exists()
    else:
        result = replay.run(capture, pair, library, output)
        assert not result["acceptance"] and not result["natural_terrain_recall_evaluated"]
        assert not result["causal_map_evaluation"] and not result["compute_acceptance"]
        assert result["native_chunks_integrated"] == 4 and len(result["frames"]) == 2
        assert len(made) == 3 and all(m.closed for m in made)
        for mapper in made:
            assert mapper.geometry == made[0].geometry
            assert len(mapper.inputs) == 4 and len(mapper.projections) == 2
            for index, (points, stamp, sensor, yaw) in enumerate(mapper.inputs):
                np.testing.assert_allclose(points, [[41., 38., .4]])
                assert stamp == frames[index]["observed_stamp"]
                np.testing.assert_allclose(sensor, [40., 38., .4])
    assert all(m.closed for m in made)
