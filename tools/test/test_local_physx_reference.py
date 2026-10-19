from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.physx_reference import BodyBox, body_clearance, envelope_segments


def pose(x=0, rotation=None):
    value = np.eye(4)
    value[0, 3] = x
    if rotation is not None:
        value[:3, :3] = Rotation.from_rotvec(rotation).as_matrix()
    return value


BOX = BodyBox((0.2, 0, 0.4), (0.5, 0.2, 0.1), "fixture-box")


class Scene:
    def __init__(self, overlap=(), sweep=(), swallow=False):
        self.overlap, self.sweep, self.calls, self.swallow = overlap, sweep, [], swallow

    def overlap_box(self, half, position, rotation, callback, any_hit):
        self.calls.append(("overlap", half, position, rotation))
        for path in self.overlap:
            callback(SimpleNamespace(collision=path))
        return len(self.overlap)

    def sweep_box_all(self, half, position, rotation, direction, distance, callback, both_sides):
        self.calls.append(("sweep", half, position, rotation, direction, distance, both_sides))
        for path in self.sweep:
            callback(SimpleNamespace(collision=path))
        return bool(self.sweep)


def evaluate(scene, poses=None, token=lambda: ("epoch", 1), **kwargs):
    return body_clearance(
        scene,
        poses if poses is not None else [pose(), pose(1)],
        [BOX],
        scene_token=token,
        robot_root="/Robots/astrobot_0",
        **kwargs
    )


def test_continuous_sweep_catches_hazard_not_at_endpoints():
    scene = Scene(sweep=["/Rocks/midway"])
    result = evaluate(scene)
    assert not result["body_envelope_clear"]
    assert result["possible_collisions"] == ["/Rocks/midway"]
    assert len(scene.calls) == result["query_count"] == 2
    assert scene.calls[1][-2:] == (1.0, True)
    assert not result["wheel_support_evaluated"] and not result["acceptance"]


def test_static_overlap_and_precise_self_filter_keep_terrain_and_other_robots():
    scene = Scene(overlap=["/Robots/astrobot_0/base_link", "/Lunaryard/Terrain", "/Robots/astrobot_01/base_link"])
    result = evaluate(scene, [pose()])
    assert result["possible_collisions"] == ["/Lunaryard/Terrain", "/Robots/astrobot_01/base_link"]
    assert len(scene.calls) == 1


def test_clear_envelope_is_not_a_motion_certificate():
    result = evaluate(Scene())
    assert result["body_envelope_clear"]
    assert not result["navigation_authority"] and not result["wheel_support_evaluated"]


@pytest.mark.parametrize("rotation", [(0, 0, 1.2), (0.3, -0.2, 0.4), (-0.7, 0.8, 0.1)])
def test_rotating_lever_arm_and_all_corners_remain_in_conservative_envelopes(rotation):
    first, last = pose(), pose(1, rotation)
    pieces = list(envelope_segments([first, last], [BOX]))
    rotations = Slerp([0, 1], Rotation.from_matrix(np.array([first[:3, :3], last[:3, :3]])))
    corners = np.array([[x, y, z] for x in (-0.5, 0.5) for y in (-0.2, 0.2) for z in (-0.1, 0.1)]) + BOX.center
    for i, segment in enumerate(pieces):
        reference = Rotation.from_quat(segment["quaternion"])
        for fraction in np.linspace(0, 1, 11):
            t = (i + fraction) / len(pieces)
            points = rotations(t).apply(corners) + [t, 0, 0]
            center = segment["position"] + fraction * segment["displacement"]
            local = reference.inv().apply(points - center)
            assert np.all(np.abs(local) <= segment["half_extent"] + 1e-12)


def test_native_callback_failures_cannot_be_swallowed_into_clearance():
    with pytest.raises(ValueError, match="callback failed"):
        evaluate(Scene(overlap=[None]))


def test_changed_scene_epoch_is_not_a_partial_certificate():
    tokens = iter([("old", 1), ("new", 1)])
    with pytest.raises(ValueError, match="changed"):
        evaluate(Scene(), token=lambda: next(tokens))


def test_budget_rejection_precedes_any_active_query():
    scene = Scene()
    with pytest.raises(ValueError, match="budget"):
        evaluate(scene, [pose(), pose(0, [0, 0, 1])], max_queries=2)
    assert not scene.calls


@pytest.mark.parametrize("center,half", [((0, 0), (1, 1, 1)), ((0, 0, 0), (0, 1, 1)), ((np.nan, 0, 0), (1, 1, 1))])
def test_invalid_envelope_rejected(center, half):
    with pytest.raises(ValueError):
        BodyBox(center, half, "fixture")
