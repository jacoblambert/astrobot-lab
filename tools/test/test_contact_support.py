import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.contact_support import (
    ContactMemory,
    SupportIndex,
    area,
    convex,
    erode,
    patch_polygons,
    subtract,
    supported_motion,
)
from local_terrain.physx_reference import BodyBox


def rectangle(x0, y0, x1, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=float)


def setup():
    patches = [BodyBox((0, 0, -0.1), (0.04, 0.06, 0.001), "assumed_wheel")]
    body = BodyBox((0, 0, 0), (0.3, 0.25, 0.001), "body")
    layers = np.zeros((60, 60, 6))
    layers[:, :, 0] = -1
    record = dict(origin=np.array([-3.0, -3.0]), resolution=0.1)
    memory = ContactMemory(patches, rectangle(-0.3, -0.25, 0.3, 0.25))
    memory.start("s", 1, 0, grounded=True)
    return patches, body, layers, record, memory


def test_polygon_union_does_not_fill_holes_or_corner_cells():
    poly = rectangle(0, 0, 1, 1)
    pieces = subtract(poly, rectangle(0.2, 0.2, 0.8, 0.8))
    assert sum(map(area, pieces)) == pytest.approx(0.64)
    index = SupportIndex([(rectangle(0, 0, 0.04, 0.04), "track")], [0, 0], 0.1)
    assert index.uncovered(rectangle(0, 0, 0.1, 0.1), (0, 0)) == pytest.approx(0.0084)
    index = SupportIndex([(rectangle(0, 0, 0.04, 0.1), "a"), (rectangle(0.06, 0, 0.1, 0.1), "b")], [0, 0], 0.1)
    assert index.uncovered(rectangle(0, 0, 0.1, 0.1), (0, 0)) == pytest.approx(0.002)


@pytest.mark.parametrize("phase", [0.0, 0.025, 0.05, 0.075])
def test_startup_support_and_new_ground_separated(phase):
    patches, body, layers, record, memory = setup()
    record["origin"] += phase
    placed, _ = memory.placed("s", 1, 0, lambda _: (np.eye(4), 0.0))
    index = SupportIndex(placed, **record)
    original = layers.copy()
    assert supported_motion(layers, record, [np.eye(4)], patches, body, index)["state"] == "available"
    end = np.eye(4)
    end[0, 3] = 0.5
    assert supported_motion(layers, record, [np.eye(4), end], patches, body, index)["state"] == "unknown"
    np.testing.assert_array_equal(layers, original)


@pytest.mark.parametrize("reason", [3, 5, 6, 7, 8, 10, 11, 13])
def test_hazard_conflicts_never_overridden(reason):
    patches, body, layers, record, memory = setup()
    layers[29:31, 29:31, 1] = reason
    placed, _ = memory.placed("s", 1, 0, lambda _: (np.eye(4), 0.0))
    index = SupportIndex(placed, **record)
    assert supported_motion(layers, record, [np.eye(4)], patches, body, index)["state"] == "unknown"
    layers[30, 32, 0] = 100
    assert supported_motion(layers, record, [np.eye(4)], patches, body, index)["state"] == "blocked"


def test_track_retained_after_60_seconds_and_repositioned_not_duplicated():
    patches, _, _, record, memory = setup()
    memory.observe("s", 1, 10, grounded=True)
    pose = np.eye(4)
    pose[0, 3] = 2.0
    placed, missing = memory.placed("s", 1, 120_000_000_000, lambda stamp: (pose, 0.0) if stamp == 10 else None)
    assert missing == 1 and len(placed) == 1
    index = SupportIndex(placed, **record)
    assert index.uncovered(rectangle(1.99, -0.01, 2.01, 0.01), (50, 30)) == 0
    assert (30, 30) not in index.cells  # No stale free trail at the previous pose.
    shifted, _ = memory.placed("s", 1, 120_000_000_000, lambda _: (np.eye(4), 0.0))
    assert (50, 30) not in SupportIndex(shifted, **record).cells


def test_hanging_wheel_is_not_written_as_ground_even_on_a_level_body():
    patches = [
        BodyBox((x, y, -0.145), (0.04, 0.06, 0.001), "wheel") for x in (-0.256, 0.256) for y in (-0.2854, 0.2854)
    ]
    memory = ContactMemory(patches, rectangle(-0.504, -0.346, 0.504, 0.346))
    memory.start("s", 1, 0, grounded=True)
    mask = [True, True, True, False]
    memory.observe("s", 1, 1, grounded=True, supported_wheels=mask)
    pose = np.eye(4)
    pose[0, 3] = 2.0
    placed, missing = memory.placed("s", 1, 1, lambda stamp: (pose, 0.0) if stamp == 1 else None)
    assert missing == 1 and len(placed) == 3
    index = SupportIndex(placed, [-3, -3], 0.1)
    hanging = patch_polygons(pose, [patches[3]])[0]
    from local_terrain.support_sweep import touched_cells

    cells, _ = touched_cells([hanging], [-3, -3], 0.1)
    assert sum(index.uncovered(hanging, ij) for ij in cells) == pytest.approx(area(hanging))
    # Historical corrections must not restore omitted wheels through the cache.
    placed_again, _ = memory.placed("s", 1, 1, lambda stamp: (np.eye(4), 0.0) if stamp == 1 else None)
    assert len(placed_again) == 3


def test_invalid_or_empty_wheel_masks_do_not_create_evidence():
    _, _, _, _, memory = setup()
    for mask in ([1], [True, False], [float("nan")]):
        with pytest.raises(ValueError, match="boolean"):
            memory.observe("s", 1, 1, grounded=True, supported_wheels=mask)
        assert memory.last_stamp == 0
    with pytest.raises(ValueError, match="conflicts"):
        memory.observe("s", 1, 1, grounded=False, supported_wheels=[True])
    memory.observe("s", 1, 1, grounded=True, supported_wheels=[False])
    assert len(memory.anchors) == 1


def test_no_connection_across_missing_contacts_no_command_inference():
    _, _, _, record, memory = setup()
    memory.observe("s", 1, 100, grounded=False)
    memory.observe("s", 1, 200, grounded=True)
    pose = np.eye(4)
    pose[0, 3] = 2.0
    placed, _ = memory.placed("s", 1, 200, lambda stamp: (np.eye(4) if stamp == 0 else pose, 0.0))
    index = SupportIndex(placed, **record)
    assert (40, 30) not in index.cells
    assert len(memory.anchors) == 2
    with pytest.raises(ValueError, match="reseeded"):
        memory.start("s", 1, 300, grounded=True)


def test_uncertainty_erodes_support_and_reset_withholds_history():
    _, _, _, _, memory = setup()
    assert area(erode(rectangle(-0.1, -0.1, 0.1, 0.1), 0.02)) == pytest.approx(0.16**2)
    assert area(erode(rectangle(-0.1, -0.1, 0.1, 0.1), 0.11)) == 0
    with pytest.raises(ValueError):
        memory.placed("s", 2, 0, lambda _: (np.eye(4), 0.0))
    memory.reset()
    assert not memory.anchors
    with pytest.raises(ValueError):
        memory.placed("s", 1, 0, lambda _: (np.eye(4), 0.0))


def test_pitch_roll_and_contact_height_are_not_discarded():
    patches, _, _, _, memory = setup()
    memory.observe("s", 1, 1, grounded=True)
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("xyz", [0.12, -0.18, 0.4]).as_matrix()
    placed, _ = memory.placed("s", 1, 1, lambda stamp: (pose, 0.0) if stamp == 1 else None)
    np.testing.assert_allclose(placed[0][0], convex(patch_polygons(pose, patches)[0]))


def test_capacity_future_and_nonfinite_guards():
    patches, _, _, _, _ = setup()
    memory = ContactMemory(patches, rectangle(-0.3, -0.3, 0.3, 0.3), capacity=2)
    memory.start("s", 1, 0, grounded=True)
    memory.observe("s", 1, 1, grounded=True)
    memory.observe("s", 1, 2, grounded=True)
    assert memory.evicted == 1 and len(memory.anchors) == 2
    with pytest.raises(ValueError, match="future"):
        memory.placed("s", 1, 1, lambda _: (np.eye(4), 0.0))
    with pytest.raises(ValueError):
        memory.placed("s", 1, 2, lambda _: (np.eye(4), float("nan")))


def test_geometry_cache_never_reuses_missing_corrected_or_reset_support():
    _, _, _, _, memory = setup()
    first, _ = memory.placed("s", 1, 0, lambda _: (np.eye(4), 0.0))
    second, _ = memory.placed("s", 1, 0, lambda _: (np.eye(4), 0.0))
    assert first[0][0] is second[0][0]
    assert not first[0][0].flags.writeable
    assert memory.placed("s", 1, 0, lambda _: None) == ([], 1)
    moved = np.eye(4)
    moved[0, 3] = 0.2
    corrected, _ = memory.placed("s", 1, 0, lambda _: (moved, 0.0))
    np.testing.assert_allclose(corrected[0][0], first[0][0] + [0.2, 0.0])
    eroded, _ = memory.placed("s", 1, 0, lambda _: (moved, 0.02))
    assert area(eroded[0][0]) < area(corrected[0][0])
    memory.reset()
    assert not memory.geometry_cache


def test_startup_includes_projected_contact_edges_on_tilted_body():
    patches, body, layers, record, memory = setup()
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("xyz", [0.12, -0.18, 0.4]).as_matrix()
    placed, _ = memory.placed("s", 1, 0, lambda _: (pose, 0.0))
    index = SupportIndex(placed, **record)
    assert supported_motion(layers, record, [pose], patches, body, index)["state"] == "available"


def test_known_body_obstacle_has_priority_over_outside_grid():
    patches, body, layers, record, _ = setup()
    pose = np.eye(4)
    pose[0, 3] = -2.9
    layers[30, 0, 0] = 100
    index = SupportIndex([], **record)
    assert supported_motion(layers, record, [pose], patches, body, index)["state"] == "blocked"


def test_geos_union_matches_independent_clipping_including_holes():
    pytest.importorskip("shapely")
    rng = np.random.default_rng(17)
    for trial in range(30):
        covers = []
        for _ in range(8):
            p = rectangle(-0.04, -0.02, 0.04, 0.02)
            angle = rng.uniform(-np.pi, np.pi)
            rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
            covers.append((p @ rotation + rng.uniform(0, 0.1, 2), "contact"))
        reference = SupportIndex(covers, [0, 0], 0.1)
        fast = SupportIndex(covers, [0, 0], 0.1, backend="geos")
        query = rectangle(-0.01, -0.01, 0.11, 0.11)
        assert fast.uncovered(query, (0, 0)) == pytest.approx(reference.uncovered(query, (0, 0)), abs=1e-12)
    ring = [
        (rectangle(0, 0, 0.1, 0.02), "a"),
        (rectangle(0, 0.08, 0.1, 0.1), "b"),
        (rectangle(0, 0, 0.02, 0.1), "c"),
        (rectangle(0.08, 0, 0.1, 0.1), "d"),
    ]
    index = SupportIndex(ring, [0, 0], 0.1, backend="geos")
    assert index.uncovered(rectangle(0, 0, 0.1, 0.1), (0, 0)) == pytest.approx(0.06**2)


def test_small_straight_motion_inside_tilted_deployment_footprint():
    patches = [
        BodyBox((x, y, -0.145), (0.04, 0.06, 0.001), "wheel") for x in (-0.256, 0.256) for y in (-0.2854, 0.2854)
    ]
    body = BodyBox((0, 0, 0), (0.5037, 0.3454, 0.001), "body")
    layers = np.zeros((60, 60, 6))
    layers[:, :, 0] = -1
    memory = ContactMemory(patches, rectangle(-0.5037, -0.3454, 0.5037, 0.3454))
    memory.start("s", 1, 0, grounded=True)
    start = np.eye(4)
    start[:3, :3] = Rotation.from_euler("xyz", [0.057, 0.005, -0.001]).as_matrix()
    end = start.copy()
    end[:3, 3] += start[:3, 0] * 0.05
    placed, _ = memory.placed("s", 1, 0, lambda _: (start, 0.0))
    record = dict(origin=[-3.0, -3.0], resolution=0.1)
    index = SupportIndex(placed, **record)
    assert supported_motion(layers, record, [start, end], patches, body, index)["state"] == "available"


@pytest.mark.parametrize("phase", [0.0, 0.025, 0.05, 0.075])
def test_prepared_queries_match_reference_with_conflicts_and_partial_support(phase):
    pytest.importorskip("shapely")
    from local_terrain.contact_query import PreparedMotion
    from assess_contact_support import extended_motions

    patches, body, layers, record, memory = setup()
    record["origin"] += phase
    placed, _ = memory.placed("s", 1, 0, lambda _: (np.eye(4), 0.0))
    index = SupportIndex(placed, **record, backend="geos")
    for reason in (0, 8, 9, 11, 12):
        layers[29:31, 29:31, 1] = reason
        layers[33:35, 30:33, 0] = 0
        for poses in extended_motions(np.eye(4)).values():
            expected = supported_motion(layers, record, poses, patches, body, index)
            actual = PreparedMotion(layers, record, poses, patches, body).evaluate(index)
            assert {k: actual[k] for k in expected} == expected
            if actual["state"] == "available":
                assert actual["new_ground_area_m2"] == 0
    layers[30, 30, 0] = 100
    assert PreparedMotion(layers, record, [np.eye(4)], patches, body).evaluate(index)["state"] == "blocked"


@pytest.mark.parametrize("hazard", [None, 8, 11, 7, 100])
def test_core_policy_keeps_full_wheel_hazard_veto(hazard):
    pytest.importorskip("shapely")
    from local_terrain.contact_query import PreparedMotion, evaluate_support_core

    layers = np.zeros((120, 120, 6))
    layers[:, :, 0] = -1
    record = dict(origin=[-0.6, -0.6], resolution=0.01)
    full = [BodyBox((0, 0, 0), (0.04, 0.06, 0.001), "full")]
    core = [BodyBox((0, 0, 0), (0.02, 0.03, 0.001), "core")]
    body = BodyBox((0, 0, 0), (0.3, 0.25, 0.001), "body")
    end = np.eye(4)
    end[0, 3] = 0.05
    poses = [np.eye(4), end]
    index = SupportIndex([(rectangle(-0.03, -0.04, 0.10, 0.04), "known_support")], **record, backend="geos")
    if hazard == 100:
        layers[65, 65, 0] = 100
    elif hazard is not None:
        layers[65, 65, 1] = hazard
    original = layers.copy()
    full_query = PreparedMotion(layers, record, poses, full, body)
    core_query = PreparedMotion(layers, record, poses, core, body)
    result = evaluate_support_core(full_query, core_query, index)
    assert result["state"] == ("available" if hazard is None else "blocked" if hazard == 100 else "unknown")
    if hazard is None:
        assert full_query.evaluate(index)["state"] == "unknown"
    np.testing.assert_array_equal(layers, original)


def test_refinement_keeps_path_endpoints_and_encloses_actual_rotating_patches():
    pytest.importorskip("shapely")
    from shapely import Polygon, union_all
    from scipy.spatial.transform import Slerp
    from local_terrain.contact_query import refine_trajectory
    from local_terrain.support_sweep import swept_polygons

    start = np.eye(4)
    end = np.eye(4)
    end[:3, :3] = Rotation.from_euler("xyz", [0.02, -0.03, 0.09]).as_matrix()
    end[:3, 3] = [0.04, 0.01, 0.003]
    refined = refine_trajectory([start, end])
    np.testing.assert_allclose(refined[0], start)
    np.testing.assert_allclose(refined[-1], end, atol=1e-15)
    patches = [BodyBox((0.256, 0.285, -0.145), (0.02, 0.03, 0.001), "core")]
    cover = union_all([Polygon(p) for p in swept_polygons(refined, patches)])
    rotations = Slerp([0, 1], Rotation.from_matrix(np.array([start[:3, :3], end[:3, :3]])))
    for t in np.linspace(0, 1, 301):
        pose = np.eye(4)
        pose[:3, :3] = rotations(t).as_matrix()
        pose[:3, 3] = end[:3, 3] * t
        for p in patch_polygons(pose, patches):
            assert Polygon(p).difference(cover).area < 1e-12
    with pytest.raises(ValueError, match="budget"):
        refine_trajectory([start, end], max_segments=1)
