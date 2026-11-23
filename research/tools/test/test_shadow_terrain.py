"""Host NumPy/SciPy/pytest; native oracle additionally needs TERRAIN_LIBRARY."""
import os
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from shadow_terrain import FramePose, PoseTimeline, clock_bound_events, perturb_native, pose_error, reconstruct_native


def pose(x=0., yaw=0.):
    value = np.eye(4)
    value[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    value[0, 3] = x
    return value


def frame(i, stamp, x=0., yaw=0., extrinsic=None):
    return FramePose(i, stamp, pose(x, yaw), np.eye(4) if extrinsic is None else extrinsic)


def apply(timeline, revision, frames, arrival=300_000_000, **kwargs):
    return timeline.apply(session="s", epoch=1, revision=revision, parent=revision-1,
                          arrival_ns=arrival, frames=frames, **kwargs)


def test_receipt_cutoff_first_publication_and_corrections():
    timeline = PoseTimeline("s", 1)
    apply(timeline, 1, [frame(0, 0)], arrival=10)
    with pytest.raises(ValueError, match="bracket"):
        timeline.associate(50_000_000, 100_000_000)
    apply(timeline, 2, [frame(1, 100_000_000, 1)], arrival=120_000_000)
    initial = timeline.associate(50_000_000, 120_000_000)
    assert initial.map_from_lidar[0, 3] == .5
    assert initial.available_ns == 120_000_000
    apply(timeline, 3, [frame(0, 0, 2), frame(1, 100_000_000, 3)])
    # Loading the full bag must not leak later estimates into earlier outputs.
    with pytest.raises(ValueError, match="bracket"):
        timeline.associate(50_000_000, 100_000_000)
    for mode in ["causal", "revised"]:
        historical = timeline.associate(50_000_000, 120_000_000, mode=mode)
        np.testing.assert_array_equal(historical.map_from_lidar, initial.map_from_lidar)
    assert timeline.associate(50_000_000, 300_000_000).map_from_lidar[0, 3] == .5
    revised = timeline.associate(50_000_000, 300_000_000, mode="revised")
    assert revised.map_from_lidar[0, 3] == 2.5
    assert revised.revisions == (3, 3)
    assert revised.record()["available_ns"] == 300_000_000


def test_interpolate_imu_before_rotating_lidar_lever_arm():
    timeline = PoseTimeline("s", 1)
    lever = pose(1.)
    apply(timeline, 1, [frame(0, 0, extrinsic=lever), frame(1, 100_000_000, yaw=np.pi, extrinsic=lever)])
    result = timeline.associate(50_000_000, 300_000_000)
    np.testing.assert_allclose(result.map_from_lidar[:3, 3], [0, 1, 0], atol=1e-12)
    assert np.linalg.norm(result.map_from_lidar[:3, 3]) == pytest.approx(1.)


@pytest.mark.parametrize("query", [-1, .5, float("nan"), True])
def test_bad_timestamps_rejected(query):
    with pytest.raises(ValueError):
        PoseTimeline("s", 1).associate(query, 1)


def test_gap_exact_endpoint_and_extrapolation():
    timeline = PoseTimeline("s", 1)
    apply(timeline, 1, [frame(0, 0), frame(1, 300_000_000)])
    assert timeline.associate(0, 300_000_000).frame_ids == (0, 0)
    with pytest.raises(ValueError, match="gap"):
        timeline.associate(100_000_000, 300_000_000)
    with pytest.raises(ValueError, match="bracket"):
        timeline.associate(400_000_000, 400_000_000)


def test_atomic_rejection_snapshot_and_identity():
    timeline = PoseTimeline("s", 1)
    apply(timeline, 1, [frame(0, 0)])
    assert not apply(timeline, 1, [frame(0, 0)], arrival=400_000_000)
    assert timeline.last_arrival_ns == 300_000_000
    for frames in [[frame(0, 1)], [frame(1, 0)], [frame(1, 1), frame(1, 2)],
                   [frame(1, 1, extrinsic=pose(1))], [frame(1, 400_000_000)]]:
        with pytest.raises(ValueError):
            apply(timeline, 2, frames)
        assert timeline.revision == 1 and len(timeline.frames) == 1
    with pytest.raises(ValueError, match="conflicting"):
        apply(timeline, 1, [frame(0, 0, 1)])
    with pytest.raises(ValueError, match="gap"):
        apply(timeline, 3, [frame(1, 1)])
    with pytest.raises(ValueError, match="omitted"):
        apply(timeline, 3, [frame(1, 1)], snapshot=True)
    apply(timeline, 3, [frame(0, 0), frame(1, 1)], snapshot=True)
    with pytest.raises(ValueError, match="epoch"):
        timeline.apply(session="s", epoch=2, revision=4, parent=3, arrival_ns=400_000_000, frames=[])
    reset = PoseTimeline("s", 2)
    with pytest.raises(ValueError, match="bracket"):
        reset.associate(0, 400_000_000)


def test_owned_immutable_se3_and_error():
    matrix = pose(2.)
    entry = FramePose(0, 0, matrix, pose())
    matrix[0, 3] = 10
    assert entry.map_from_imu[0, 3] == 2
    with pytest.raises(ValueError):
        entry.map_from_imu[0, 3] = 4
    invalid = np.eye(4)
    invalid[2, 2] = -1
    with pytest.raises(ValueError, match="SE"):
        frame(0, 0, extrinsic=invalid)
    assert pose_error(pose(2, .1), pose()) == pytest.approx(dict(position_m=2., rotation_rad=.1))


@pytest.mark.parametrize("profile", ["baseline", "range_1cm", "range_3cm", "dropout_10pct"])
@pytest.mark.parametrize("seed", [42, 411, 7])
def test_noise_deterministic_shared_input_and_radial_only(profile, seed):
    points = np.tile([3., 4., 0.], (10000, 1))
    original = points.copy()
    actual, indices = perturb_native(points, profile=profile, seed=seed, chunk_id=17)
    # Reordering experiments/chunks cannot advance shared random state.
    perturb_native(points, profile=profile, seed=seed, chunk_id=18)
    other, other_indices = perturb_native(points, profile=profile, seed=seed, chunk_id=17)
    np.testing.assert_array_equal(actual, other)
    np.testing.assert_array_equal(indices, other_indices)
    np.testing.assert_array_equal(points, original)
    np.testing.assert_allclose(np.cross(actual, points[indices]), 0, atol=1e-12)
    if profile.startswith("range"):
        sigma = .01 if profile == "range_1cm" else .03
        error = np.linalg.norm(actual, axis=1) - 5
        assert abs(error.mean()) < sigma * .04
        assert .96 * sigma < error.std() < 1.04 * sigma
    elif profile == "dropout_10pct":
        assert 8800 < len(indices) < 9200
    else:
        np.testing.assert_array_equal(actual, points)


def test_noise_invalid_returns_and_parameters():
    points = np.array([[0., 0., 0.], [np.nan, 1., 0.], [1., 0., 0.]])
    actual, indices = perturb_native(points, profile="baseline", seed=42, chunk_id=0)
    np.testing.assert_array_equal(indices, [2])
    np.testing.assert_array_equal(actual, [[1, 0, 0]])
    with pytest.raises(ValueError, match="undeclared"):
        perturb_native(points, profile="tuned", seed=42, chunk_id=0)
    with pytest.raises(ValueError):
        perturb_native(points, profile="baseline", seed=-1, chunk_id=0)


def test_cpp_reconstruction_nonuniform_revision_and_expiry():
    library = os.environ.get("TERRAIN_LIBRARY")
    if not library:
        pytest.skip("requires frozen C++ terrain library")
    xx, yy = np.meshgrid(np.arange(-.9, .9, .015), np.arange(-.9, .9, .015))
    points = np.c_[xx.ravel(), yy.ravel(), np.full(xx.size, -.5)]
    timeline = PoseTimeline("s", 1)
    apply(timeline, 1, [frame(0, 100_000_000), frame(1, 200_000_000)])
    chunks = [(0, 100_000_000, points), (1, 200_000_000, points)]
    initial = {i: timeline.associate(t, 300_000_000) for i, t, _ in chunks}
    apply(timeline, 2, [frame(0, 100_000_000, .2), frame(1, 200_000_000, -.2)], arrival=400_000_000)
    revised = {i: timeline.associate(t, 400_000_000, mode="revised") for i, t, _ in chunks}
    fresh = PoseTimeline("s", 1)
    apply(fresh, 1, [frame(0, 100_000_000, .2), frame(1, 200_000_000, -.2)], arrival=400_000_000)
    expected = {i: fresh.associate(t, 400_000_000, mode="revised") for i, t, _ in chunks}
    kwargs = dict(library=library, width=40, height=40, origin=(-2., -2.),
                  output_ns=400_000_000, projection_stamps_ns=[100_000_000, 200_000_000])
    before = reconstruct_native(chunks, initial, **kwargs)
    after = reconstruct_native(chunks, revised, **kwargs)
    np.testing.assert_array_equal(after, reconstruct_native(chunks, expected, **kwargs))
    assert not np.array_equal(before, after, equal_nan=True)
    kwargs["output_ns"] = 31_000_000_000
    expired = reconstruct_native(chunks, revised, **kwargs)
    assert (expired[:, :, 0] == -1).all()


def test_arrival_bound_uses_next_clock_not_previous_or_wall_stamp():
    events = [(10**18, "clock", 100), (10**18 + 1, "revision", {"revision": 1}),
              (10**18 + 2, "clock", 200)]
    assert list(clock_bound_events(events)) == [({"revision": 1}, 200, 10**18 + 1)]


@pytest.mark.parametrize("events,reason", [
    ([(1, "revision", {})], "prefix"),
    ([(1, "clock", 0), (2, "revision", {})], "tail"),
    ([(1, "clock", 10), (2, "clock", 0)], "reset"),
    ([(2, "clock", 0), (1, "clock", 0)], "order"),
    ([(1, "clock", 0), (2, "clock", 300_000_000)], "gap"),
])
def test_invalid_receipt_stream_fails_closed(events, reason):
    with pytest.raises(ValueError, match=reason):
        list(clock_bound_events(events))


def test_pending_receipt_budget():
    with pytest.raises(ValueError, match="budget"):
        list(clock_bound_events([(1, "clock", 0), (2, "revision", {}), (3, "revision", {})], max_pending=1))


def test_cpp_time_window_crop_would_resurrect_expired_bucket():
    library = os.environ.get("TERRAIN_LIBRARY")
    if not library:
        pytest.skip("requires frozen C++ terrain library")
    xx, yy = np.meshgrid(np.arange(-.9, .9, .015), np.arange(-.9, .9, .015))
    points = np.c_[xx.ravel(), yy.ravel(), np.full(xx.size, -.5)]
    timeline = PoseTimeline("s", 1)
    apply(timeline, 1, [frame(0, 1_000_000_000), frame(1, 1_500_000_000)], arrival=2_000_000_000)
    chunks = [(0, 1_000_000_000, points), (1, 1_500_000_000, points)]
    associations = {i: timeline.associate(t, 2_000_000_000) for i, t, _ in chunks}
    kwargs = dict(library=library, output_ns=31_200_000_000, width=40, height=40, origin=(-2., -2.))
    full = reconstruct_native(chunks, associations, **kwargs)
    cropped = reconstruct_native(chunks[1:], associations, **kwargs)
    assert (full[:, :, 0] == -1).all()
    assert np.isin(cropped[:, :, 0], [0, 20]).any(), "fixture must expose artificial evidence rejuvenation"
