"""Analytic visibility checks must distinguish occlusion, missing rays and support."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assess_nearfield_hazards import (
    box_interval,
    ellipsoid_hit,
    make_sensor,
    placements,
    render,
    scene_depth,
    summarize,
    view_counts,
)
from local_terrain.mounting_study import first_intersections
from assess_recorded_hazard_views import matched_rows, recorded_views, size_sweep


def case(kind="box", x=0.65, y=0, width=0.10, height=0.08):
    return dict(kind=kind, x=x, y=y, width=width, height=height)


def test_box_parallel_miss_inside_and_forward_intersections():
    rays = np.array([[1, 0, 0], [0, 1, 0], [-1, 0, 0]], float)
    near, far = box_interval(np.array([0, 0, 0]), rays, [1, -1, -1], [2, 1, 1])
    np.testing.assert_equal(near, [1, np.inf, np.inf])
    assert far[0] == 2
    near, _ = box_interval(np.array([1.5, 0, 0]), rays, [1, -1, -1], [2, 1, 1])
    np.testing.assert_equal(near, [0, 0, 0])


def test_rounded_object_is_not_filled_bounding_box():
    rays = np.array([[1, 0, 0]], float)
    center = np.array([2, 0, 0])
    assert ellipsoid_hit(np.array([0, 0, 0]), rays, center, [1, 1, 1])[0] == pytest.approx(1)
    assert ellipsoid_hit(np.array([0, 0.9, 0.9]), rays, center, [1, 1, 1])[0] == np.inf
    with pytest.raises(ValueError):
        ellipsoid_hit(np.zeros(3), rays, center, [0, 1, 1])


def test_hole_keeps_far_ground_when_ray_passes_over_opening():
    origin = np.array([0, 0, 1.0])
    rays = np.array([[1, 0, -1]], float) / np.sqrt(2)
    depth, labels = first_intersections(origin, rays)
    after, hazard = scene_depth(origin, rays, depth, labels, case("pit", x=0.5, width=0.3, height=0.4))
    np.testing.assert_equal(after, depth)
    assert not hazard[0]  # Hole is invisible; a far-ground hit is not ground support here.


def test_hole_returns_from_bottom_or_wall_not_infinite_missing():
    origin = np.array([0, 0, 1.0])
    rays = np.array([[1, 0, -1]], float) / np.sqrt(2)
    depth, labels = first_intersections(origin, rays)
    bottom, _ = scene_depth(origin, rays, depth, labels, case("pit", x=1, width=0.6, height=0.2))
    wall, _ = scene_depth(origin, rays, depth, labels, case("pit", x=1, width=0.2, height=0.4))
    assert bottom[0] == pytest.approx(1.2 * np.sqrt(2))
    assert wall[0] == pytest.approx(1.1 * np.sqrt(2))


def test_chassis_first_hit_not_replaced_by_rock_behind_it():
    origin = np.array([0, 0, 1.0])
    rays = np.array([[1, 0, -1]], float) / np.sqrt(2)
    depth, labels = first_intersections(origin, rays, [([0.2, -0.1, 0.7], [0.3, 0.1, 0.8])])
    after, hazard = scene_depth(origin, rays, depth, labels, case(x=0.6, width=0.2, height=0.8))
    np.testing.assert_equal(after, depth)
    assert not hazard[0]


@pytest.fixture(scope="module")
def sensor():
    return make_sensor()


@pytest.mark.parametrize("kind", ["box", "rounded", "pit"])
@pytest.mark.parametrize("yaw,reverse", [(0, 0), (75, 0.2), (180, 0)])
def test_angular_crop_matches_all_63968_rays(sensor, kind, yaw, reverse):
    hazard = case(kind, x=0.675, y=0.18, width=0.3, height=0.2)
    assert view_counts(sensor, hazard, yaw, reverse) == view_counts(sensor, hazard, yaw, reverse, crop=False)


def test_tall_near_rock_can_occlude_but_fail_minimum_range_filter(sensor):
    # Never treat rejecting a too-close hit as seeing through it to flat ground.
    counts = view_counts(sensor, case(x=0.56, height=0.5), 0)
    assert counts["missing_expected_returns"] > 0


def test_fixed_banks_include_prior_overlap_cases_without_counting_them_as_external_hazards():
    cases = placements()
    assert len(cases) == 1728 and len({r["id"] for r in cases}) == 1728
    assert {r["bank"] for r in cases} == {"regular", "shifted"}
    assert any(not r["outside_startup"] for r in cases)
    assert any(r["outside_startup"] for r in cases)
    for r in cases:
        r["protocols"] = {"parked": dict(max_changed_returns=0, max_missing_expected_returns=1)}
    report = summarize(cases)
    assert sum(r["placements"] for r in report) == sum(r["outside_startup"] for r in cases)
    assert all(r["no_observable_change"] == 0 for r in report)
    assert all(r["visible_placements_by_min_changed_returns"]["1"] == 0 for r in report)
    assert "not probabilities" in render(report)


def test_size_comparison_keeps_identical_positions_for_every_size():
    rows = matched_rows(placements())
    for bank in ("regular", "shifted"):
        for kind in ("box", "rounded", "pit"):
            group = [r for r in rows if r["bank"] == bank and r["kind"] == kind]
            sets = [
                {(r["radius"], r["bearing"]) for r in group if (r["height"], r["width"]) == size}
                for size in {(r["height"], r["width"]) for r in group}
            ]
            assert sets and all(s == sets[0] for s in sets)
            assert all(r["outside_startup"] for r in group)


def test_size_sweep_keeps_baseline_and_does_not_inherit_old_visibility():
    source = placements()
    rows = size_sweep(source)
    assert len(rows) == 1536
    assert all(r["kind"] == "rounded" and not r["protocols"] and not r["views"] for r in rows)
    assert {r["height"] for r in rows} == {0.08, 0.10, 0.12, 0.16}
    assert {r["width"] for r in rows} == {0.10, 0.12, 0.15, 0.20}
    assert len([r for r in rows if r["height"] == 0.08 and r["width"] == 0.10]) == 96
    assert "protocols" not in source[0]


def test_recorded_view_selection_preserves_xy_and_yaw_but_declares_planar_model(tmp_path):
    import json
    from scipy.spatial.transform import Rotation

    (tmp_path / "natural_holdout.json").write_text(json.dumps(dict(complete=True)))
    origin = np.eye(4)
    origin[:3, 3] = [20, 18, 0.123]
    (tmp_path / "clearance.json").write_text(json.dumps(dict(origin=origin.tolist())))
    rows = []
    for angle in range(361):
        pose = origin.copy()
        pose[:3, :3] = Rotation.from_euler("z", angle, degrees=True).as_matrix()
        pose[0, 3] += 0.15
        rows.append(dict(stamp=angle, world_from_base=pose.tolist()))
    (tmp_path / "motion.jsonl").write_text("\n".join(json.dumps(r) for r in rows))
    views = recorded_views(tmp_path)
    assert len(views) == 25
    assert views[-1]["yaw"] == pytest.approx(360)
    assert all(v["xy"] == pytest.approx([0.15, 0]) for v in views)
