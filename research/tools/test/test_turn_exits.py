import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assess_turn_exits import summarize, turn_candidates
from local_terrain.support_sweep import swept_polygons
from local_terrain.physx_reference import BodyBox


def test_fixed_candidates_include_rotation_before_translation():
    candidates = turn_candidates()
    assert len(candidates) == 16
    for poses in candidates.values():
        start, turn, end = poses
        np.testing.assert_array_equal(start, np.eye(4))
        np.testing.assert_array_equal(turn[:3, 3], [0, 0, 0])
        np.testing.assert_array_equal(turn[:3, :3], end[:3, :3])
        assert np.isclose(np.linalg.norm(end[:2, 3]), 0.5) or np.isclose(np.linalg.norm(end[:2, 3]), 1)
        assert np.dot(end[:2, 3], end[:2, 0]) > 0


def test_turn_has_real_continuous_sweep_not_just_translated_endpoint():
    from shapely import Polygon, union_all

    patch = BodyBox((0.3, 0, 0), (0.02, 0.02, 0.001), "test")
    poses = turn_candidates()["turn_+90_drive_0.5m"]
    sweep = union_all([Polygon(p) for p in swept_polygons(poses, [patch])])
    from shapely import Point

    assert sweep.covers(Point(0.3 / np.sqrt(2), 0.3 / np.sqrt(2)))
    assert sweep.covers(Point(0, 0.7))


def test_every_bin_and_failed_candidate_remains_in_denominator():
    rows = [dict(motions=None)]
    for passed in (False, False, True):
        rows.append(dict(motions=dict(straight={"a": dict(exit=False)}, turn_then_drive={"b": dict(exit=passed)})))
    assert summarize(rows) == dict(decision_bins=3, straight_exit_bins=0, turn_exit_bins=1, tested_turn_paths=3)


def test_obstacle_crossed_only_during_turn_blocks_whole_candidate():
    from local_terrain.contact_query import PreparedMotion

    layers = np.zeros((200, 200, 6))
    # Wheel traces the first-quadrant arc at radius 30 cm. This obstacle is
    # nowhere near its final, straight drive along +Y.
    layers[121, 121, 0] = 100
    patch = BodyBox((0.3, 0, 0), (0.02, 0.02, 0.001), "test_wheel")
    body = BodyBox((0, 0, 0), (0.04, 0.04, 0.001), "test_body")
    poses = turn_candidates()["turn_+90_drive_0.5m"]
    record = dict(origin=[-1.0, -1.0], resolution=0.01)
    full = PreparedMotion(layers, record, poses, [patch], body)
    assert full.terminal == dict(state="blocked", reason="wheel_obstacle")
    drive_only = PreparedMotion(layers, record, poses[1:], [patch], body)
    assert drive_only.terminal is None
