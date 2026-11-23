import sys
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assess_navigation_gaps import candidate_motions, cell_counts, floor_scores, motion_score, summarize, route_goal
from local_terrain.physx_reference import BodyBox
from explain_navigation_gaps import explain
from assess_grounded_start import remaining_unknown
from render_navigation_gaps import render
from replay_local_terrain import sha


def fixture():
    classes = np.zeros((60, 60), dtype=np.int8)
    record = dict(origin=[-3, -3], resolution=0.1, local_from_base=np.eye(4), local_from_map=np.eye(4))
    patch = BodyBox((0, 0.35, 0), (0.04, 0.04, 0.001), "assumed_test_patch")
    body = BodyBox((0, 0, 0), (0.5, 0.5, 0.001), "test_body")
    return classes, record, [patch], body


def test_short_route_goals_use_distance_heading_not_loop_xy():
    origin = np.eye(4)
    origin[:2, 3] = [12, -8]
    origin[:3, :3] = Rotation.from_euler("z", np.pi / 2).as_matrix()
    phase, goal = route_goal(origin, "straight_slow", 0)
    assert phase == "translate"
    np.testing.assert_allclose(goal, [12, -6])
    np.testing.assert_allclose(route_goal(origin, "straight_slow", 1)[1], [12, -8])
    phase, goal = route_goal(origin, "combined10", 0)
    assert phase == "rotate"
    np.testing.assert_allclose(goal, [12 - 2 * np.sin(np.pi / 12), -8 + 2 * np.cos(np.pi / 12)])
    assert route_goal(origin, "combined10", 1)[0] == "translate"
    np.testing.assert_allclose(route_goal(origin, "combined10", 2)[1], [12, -8])
    assert route_goal(origin, "combined10", 4)[0] == "hold"
    assert route_goal(origin, "turn15", 1)[0] == "rotate"
    with pytest.raises(ValueError):
        route_goal(origin, "combined10", 5)


def test_loop_route_goals_unchanged_including_turn_and_postroll():
    from natural_motion import route, yaw
    origin = np.eye(4)
    origin[:2, 3] = [12, -8]
    origin[:3, :3] = Rotation.from_euler("z", 0.3).as_matrix()
    h = yaw(origin)
    planar = np.array([[np.cos(h), -np.sin(h)], [np.sin(h), np.cos(h)]])
    targets = route("loop16")["targets"]
    for leg in range(9):
        next_leg = leg if leg % 2 == 0 else leg + 1
        expected = targets[next_leg][:2] if next_leg < 8 else [2, 0]
        phase, goal = route_goal(origin, "loop16", leg)
        np.testing.assert_array_equal(goal, origin[:2, 3] + planar @ expected)
        assert phase == (targets[leg][3] if leg < 8 else "hold")


def test_gap_under_body_is_distinct_from_missing_wheel_support():
    classes, record, patches, body = fixture()
    poses = candidate_motions(np.eye(4))["move_0.50_+0deg"]
    classes[30, 32] = -1
    result = motion_score(classes, record, poses, patches, body, np.array([2, 0]))
    assert result["state"] == "available" and result["toward_goal"]
    assert not result["whole_footprint_available"]
    classes[33, 32] = -1
    result = motion_score(classes, record, poses, patches, body, np.array([2, 0]))
    assert result["state"] == "unknown" and result["no_known_obstacle"]


def test_obstacle_between_wheels_still_blocks_body_and_start_not_omitted():
    classes, record, patches, body = fixture()
    poses = candidate_motions(np.eye(4))["move_0.50_+0deg"]
    classes[30, 26] = 100
    result = motion_score(classes, record, poses, patches, body, np.array([2, 0]))
    assert result["state"] == "blocked" and not result["no_known_obstacle"]


def test_unknown_all_and_outside_never_become_available():
    classes, record, patches, body = fixture()
    poses = candidate_motions(np.eye(4))["move_0.50_+0deg"]
    classes[:] = -1
    assert motion_score(classes, record, poses, patches, body, [2, 0])["state"] == "unknown"
    classes[:] = 0
    record["origin"] = [0, 0]
    result = motion_score(classes, record, poses, patches, body, [2, 0])
    assert result["state"] == "unknown" and result["body_projection"]["outside"] > 0


def test_motion_uses_full_base_transform_and_turn_is_not_progress():
    start = np.eye(4)
    start[:3, :3] = Rotation.from_euler("z", np.pi / 2).as_matrix()
    start[:3, 3] = [1, 2, 0.3]
    motions = candidate_motions(start)
    np.testing.assert_allclose(motions["move_0.50_+0deg"][-1][:3, 3], [1, 2.5, 0.3])
    np.testing.assert_allclose(motions["turn_+30deg"][-1][:3, 3], start[:3, 3])
    assert len(motions) == 9


def test_clear_move_away_from_goal_does_not_count_as_progress():
    classes, record, patches, body = fixture()
    poses = candidate_motions(np.eye(4))["move_0.50_+0deg"]
    result = motion_score(classes, record, poses, patches, body, [-2, 0])
    assert result["state"] == "available" and not result["toward_goal"]
    assert result["progress_m"] == pytest.approx(-0.5)


def test_reference_floor_transform_no_fitted_alignment_and_outside_kept():
    classes, record, _, _ = fixture()
    record["local_from_map"][:3, 3] = [-40, -38, 0]
    classes[30, 30] = 100
    counts = floor_scores(classes, record, np.array([[20.05, 18.05, 0], [40, 40, 0]]), [-20, -20, 0])
    assert counts["whole_survey"] == dict(samples=2, free=0, occupied=1, unknown=1, outside=1)
    assert counts["front_0_1m"]["occupied"] == 1
    with pytest.raises(ValueError):
        cell_counts(np.array([[13]]), [[0, 0]])


def test_summary_keeps_startup_turns_and_failed_stretches():
    def row(t, state):
        score = dict(state=state, toward_goal=False, whole_footprint_available=False, no_known_obstacle=True)
        return dict(elapsed_s=t, phase="rotate", motions={"turn_+30deg": score, "stationary": score})
    result = summarize([row(0, "unknown"), row(1, "unknown"), row(2, "available"), row(20, "unknown")])
    assert result["all"]["samples"] == 4
    assert result["all"]["progress_available"] == 0
    assert result["all"]["turn_available"] == 1
    assert result["after_15s"]["no_action"] == 1
    assert result["no_action_sample_stretches"] == [
        dict(start_s=0, end_s=1, samples=2), dict(start_s=20, end_s=20, samples=1)]


def test_unknown_reason_is_preserved_and_occupied_not_counted_as_missing_hits():
    classes, record, patches, _ = fixture()
    layers = np.zeros((*classes.shape, 6))
    layers[33, 30, :2] = [-1, 9]
    result = explain(layers, record, patches)
    assert result["unknown_reasons"]["INSUFFICIENT_AREA"] == 1
    assert result["unknown_reasons"]["NO_RETURNS"] == 0
    layers[33, 30, :2] = [100, 3]
    result = explain(layers, record, patches)
    assert result["occupied"] == 1 and result["unknown"] == 0
    layers[33, 30, :2] = [-1, 99]
    with pytest.raises(ValueError):
        explain(layers, record, patches)


def test_grounded_start_comparison_does_not_fill_new_ground_or_outside():
    classes = np.array([[-1, -1, 100]], dtype=np.int8)
    counts = remaining_unknown(classes, [[0, 0], [1, 0], [2, 0], [3, 0]], [[0, 0]])
    assert counts == dict(samples=3, free=0, unknown=2, occupied=1, outside=1)
    assert remaining_unknown(classes, [[0, 0]], [[0, 0]])["samples"] == 0


def test_html_preserves_scope_denominators_and_actual_grid(tmp_path):
    source, output = tmp_path / "source", tmp_path / "output"
    source.mkdir()
    output.mkdir()
    classes, record, patches, _ = fixture()
    grid = source / "grid.npz"
    np.savez_compressed(grid, classes=classes, **record)
    summary = dict(samples=10, progress_available=5, progress_or_turn=6, whole_footprint_progress=1)
    report = dict(diagnostic_completed=True, summary={k: summary for k in
                  ("all", "first_15s", "after_15s", "translation", "rotation")}, floor_totals={},
                  contract=dict(route_pattern="straight_slow",
                                patches=[dict(center=p.center, half_extent=p.half_extent) for p in patches]),
                  grids=dict(recorded_projections=10, missing_projection_stamps=[],
                             examples=[dict(file="grid.npz", sha256=sha(grid), elapsed_s=0)]))
    (source / "report.json").write_text(json.dumps(report))
    render(source, output)
    text = (output / "navigation-gaps.html").read_text()
    assert "50.0%" in text and "60.0%" in text and "10 recorded maps" in text
    assert "not autonomous navigation" in text and "does not" in text and "stroke=\"#1747b0\"" in text
    assert "straight_slow" in text and "do not include reverse" in text
    assert "completed 16 m loop" not in text
    grid.write_bytes(b"changed")
    with pytest.raises(ValueError, match="changed displayed grid"):
        render(source, output)
