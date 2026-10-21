import sys
from collections import Counter
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from demo_rock_profile import HEIGHT, WIDTH, OFFSETS, PROFILE, rounded_mesh, validate_profile
from natural_motion import survey_points
from assess_demo_rocks import object_score, sampled_hits
from render_demo_rocks import diagram
from diagnose_demo_rock_rays import mesh_hits


def test_selected_dimensions_are_ten_by_fifteen_not_recommended_twelve():
    points, faces = rounded_mesh()
    np.testing.assert_allclose(np.ptp(points, axis=0), [0.15, 0.15, 0.10], atol=1e-14)
    assert HEIGHT == 0.10 and WIDTH == 0.15 and points[:, 2].min() == 0
    assert len(points) == 354 and len(faces) == 704


def test_mesh_is_closed_nondegenerate_and_outward():
    points, faces = rounded_mesh()
    edges = Counter(tuple(sorted((int(a), int(b)))) for face in faces for a, b in zip(face, np.roll(face, -1)))
    assert set(edges.values()) == {2}
    p = points[faces]
    normal = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
    assert np.all(np.linalg.norm(normal, axis=1) > 1e-10)
    assert np.all(np.sum(normal * (p.mean(axis=1) - [0, 0, HEIGHT / 2]), axis=1) > 0)


@pytest.mark.parametrize("route", ["turn15", "turn360", "peek_reverse20"])
def test_rocks_do_not_replace_any_original_clearance_survey_points(route):
    points, _ = survey_points(np.eye(4), route)
    for xy in OFFSETS:
        assert np.min(np.linalg.norm(points - xy, axis=1)) > WIDTH / 2 + 0.05


def test_profile_is_opt_in_and_does_not_silently_change_other_routes():
    assert validate_profile("", "loop16") == ""
    assert validate_profile(PROFILE, "turn360") == PROFILE
    with pytest.raises(ValueError):
        validate_profile(PROFILE, "loop16")
    with pytest.raises(ValueError):
        validate_profile("rounded_12x15", "turn360")


def test_object_score_does_not_credit_nearby_occupied_halo():
    points, _ = rounded_mesh()
    obj = dict(path="rock", points=(points + [0.45, 0.45, 0]).tolist())
    layers = np.full((10, 10, 6), -1.0)
    record = dict(local_from_map=np.eye(4).tolist(), origin=[0, 0], resolution=0.1)
    layers[4, 6, 0] = 100
    score = object_score(layers, record, obj, np.zeros(3))
    assert not score["detected"] and score["apex_cost"] == -1
    layers[4, 4, 0] = 100
    assert object_score(layers, record, obj, np.zeros(3))["detected"]
    record["origin"] = [2, 2]
    score = object_score(layers, record, obj, np.zeros(3))
    assert not score["detected"] and score["footprint_cell_counts"]["outside"] > 0


def test_native_object_witnesses_are_hash_checked_and_zero_counts_retained(tmp_path):
    import hashlib
    import json

    (tmp_path / "native_chunks").mkdir()
    hits = tmp_path / "native_chunks/chunk_000000.hits.json"
    hits.write_text(json.dumps([dict(first_hit=dict(collision="rock0")), dict(first_hit=None)]))
    audit = dict(
        frames=[
            dict(
                current_pose=dict(
                    hit_records_file=hits.name, hit_records_sha256=hashlib.sha256(hits.read_bytes()).hexdigest()
                )
            )
        ]
    )
    (tmp_path / "native_timing_audit.json").write_text(json.dumps(audit))
    counts, _ = sampled_hits(tmp_path, [dict(path="rock0"), dict(path="rock1")])
    assert counts == dict(rock0=1, rock1=0)
    hits.write_text("[]")
    with pytest.raises(ValueError, match="changed native"):
        sampled_hits(tmp_path, [dict(path="rock0")])


def test_raw_grid_report_retains_unknown_and_actual_rock_outline():
    points, _ = rounded_mesh()
    example = dict(classes=[[-1, 0], [20, 100]], local_from_base=np.eye(4).tolist())
    record = dict(origin=[-0.1, -0.1], resolution=0.1, local_from_map=np.eye(4).tolist())
    result = diagram(example, record, dict(objects=[dict(points=points.tolist())]), np.zeros(3))
    assert result.count("<rect") == 4 and result.count("<polygon") == 1
    assert "#cbd5e1" in result and 'fill="none"' in result


def test_diagnostic_mesh_queries_preserve_authored_precision_and_misses():
    points, faces = rounded_mesh()
    obj = dict(path="rock", points=points.tolist(), triangles=faces.tolist())
    hits = mesh_hits([obj], [0, 0, 1], [0, 0, -1], 2)
    assert hits and hits[0]["distance"] == pytest.approx(1 - float(np.float32(HEIGHT)), abs=1e-14, rel=0)
    assert hits[0]["collision"] == "rock"
    assert mesh_hits([obj], [0.2, 0, 1], [0, 0, -1], 2) == []


def test_approach_field_rocks_sit_in_danger_range_but_clear_the_guard_and_turns():
    import math
    from demo_rock_profile import APPROACH_PROFILE, approach_layout, validate_profile
    from natural_motion import HALF, ROTATION_RADIUS, route, route_vertices, polyline_distance

    spec = route("approach_field")
    far = route_vertices(spec)[1]
    length = float(np.linalg.norm(far))
    guard = HALF[1] + spec["translation_lateral_limit_m"] + HALF[0] * math.sin(spec["translation_heading_limit_rad"])
    for (along, lateral), height, width in approach_layout():
        edge = abs(lateral) - width / 2  # leg frame: the leg runs along +x from 0 to `length`
        assert guard < edge <= 1.0 - 0.04  # inside the C4 danger range, outside the footprint guard
        # never inside a turn's rotation circle (start and far end)
        assert math.hypot(along, lateral) - width / 2 > ROTATION_RADIUS
        assert math.hypot(length - along, lateral) - width / 2 > ROTATION_RADIUS
        assert np.isclose(polyline_distance(np.array([[0, 0], [length, 0]]), [[along, lateral]])[0], abs(lateral))
    heights = sorted(h for _, h, _ in approach_layout())
    assert heights == [0.12, 0.20, 0.30, 0.40]
    assert validate_profile(APPROACH_PROFILE, "approach_field") == APPROACH_PROFILE
    for name, pattern in ((APPROACH_PROFILE, "loop16"), ("", "approach_field"), (APPROACH_PROFILE, "turn15")):
        with pytest.raises(ValueError, match="declared together"):
            validate_profile(name, pattern)
