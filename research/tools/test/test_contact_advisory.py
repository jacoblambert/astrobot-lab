import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.contact_advisory import ContactAdvisory
from local_terrain.physx_reference import BodyBox
from test_control_context import setup
from local_terrain.contact_runtime import startup_footprint
from shadow_terrain import FramePose


def fixture(**kwargs):
    timeline, record, base = setup()
    record.update(origin=[-3.0, -3.0], resolution=0.1)
    patches = [
        BodyBox((x, y, -0.145), (0.04, 0.06, 0.001), "wheel") for x in (-0.256, 0.256) for y in (-0.2854, 0.2854)
    ]
    footprint = np.array([[-0.504, -0.346], [0.504, -0.346], [0.504, 0.346], [-0.504, 0.346]])
    advisory = ContactAdvisory(patches, footprint, BodyBox((0, 0, 0), (0.85, 0.70, 0.001), "guard"), **kwargs)
    layers = np.zeros((60, 60, 6))
    layers[:, :, 0] = -1
    return advisory, layers, record, timeline, base


def test_current_unknown_wheels_can_stand_but_do_not_create_historical_free_ground():
    advisory, layers, record, timeline, base = fixture()
    advisory.update(layers, record, timeline, base, 200_000_000, grounded=True, deployment_confirmed=True)
    assert advisory.query([np.eye(4)], query_ns=200_000_000)["state"] == "available"
    assert advisory.status()["lidar_confirmed_wheels"] == [False] * 4
    step = np.eye(4)
    step[0, 3] = 0.05
    # The initial clear deployment prior uses this exact same pose, not a
    # historical pose with an unrelated 2 cm placement uncertainty.
    assert advisory.query([np.eye(4), step], query_ns=200_000_000)["state"] == "available"
    # Same pose may be queried again, but cannot reseed or add duplicate tracks.
    advisory.update(layers, record, timeline, base, 200_000_001, grounded=True)
    assert len(advisory.memory.anchors) == 1
    with pytest.raises(ValueError, match="consumer clock"):
        advisory.query([np.eye(4)], query_ns=200_000_002)


def test_failed_update_invalidates_old_available_result():
    advisory, layers, record, timeline, base = fixture()
    advisory.update(layers, record, timeline, base, 200_000_000, grounded=True, deployment_confirmed=True)
    with pytest.raises(ValueError, match="stale"):
        advisory.update(layers, record, timeline, base, 800_000_000, grounded=True)
    assert not advisory.status()["valid"]
    with pytest.raises(ValueError):
        advisory.query([np.eye(4)], query_ns=200_000_000)


def test_hazards_win_even_with_startup_and_core_and_input_is_immutable():
    advisory, layers, record, timeline, base = fixture(support_core=True)
    layers[29:32, 29:32, 0] = 100
    advisory.update(layers, record, timeline, base, 200_000_000, grounded=True, deployment_confirmed=True)
    layers[:, :, 0] = 0  # Caller mutation cannot rewrite the prepared snapshot.
    result = advisory.query([np.eye(4)], query_ns=200_000_000)
    assert result["state"] == "blocked" and not result["navigation_authority"]


def test_missing_explicit_deployment_cannot_seed_prior():
    advisory, layers, record, timeline, base = fixture()
    with pytest.raises(ValueError, match="deployment"):
        advisory.update(layers, record, timeline, base, 200_000_000, grounded=True)
    assert advisory.memory.identity is None


def test_current_turn_assumption_moves_only_with_received_robot_pose_not_planned_path():
    advisory, layers, record, timeline, base = fixture(current_turn_footprint=startup_footprint(True))
    original = layers.copy()
    advisory.update(layers, record, timeline, base, 200_000_000, grounded=True, deployment_confirmed=True)
    # Move the actual published pose away from startup; no LiDAR-supported wheels.
    pose = timeline.frames[1][0][2].map_from_imu.copy()
    pose[0, 3] += 1.0
    timeline.frames[2] = [(280_000_000, 3, FramePose(2, 250_000_000, pose, timeline.extrinsic))]
    timeline.revision = record["revision"] = 3
    record["publication_ns"] = 300_000_000
    advisory.update(layers, record, timeline, base, 300_000_000, grounded=True)
    short = np.eye(4)
    short[0, 3] = 0.07
    assert advisory.query([np.eye(4), short], query_ns=300_000_000)["state"] == "available"
    long = np.eye(4)
    long[0, 3] = 1.0
    assert advisory.query([np.eye(4), long], query_ns=300_000_000)["state"] == "unknown"
    assert len(advisory.memory.anchors) == 1
    assert advisory.status()["current_turn_area_assumed"]
    np.testing.assert_array_equal(layers, original)


@pytest.mark.parametrize("occupied", [False, True])
def test_current_turn_assumption_never_overrides_detected_hazard(occupied):
    advisory, layers, record, timeline, base = fixture(current_turn_footprint=startup_footprint(True))
    advisory.update(layers, record, timeline, base, 200_000_000, grounded=True, deployment_confirmed=True)
    pose = advisory.context.local_from_base
    wheel = (pose @ np.array([0.256, 0.2854, -0.145, 1]))[:2]
    x, y = np.floor((wheel - record["origin"]) / record["resolution"]).astype(int)
    layers[y, x, :2] = [100, 1] if occupied else [-1, 11]
    advisory.update(layers, record, timeline, base, 200_000_001, grounded=True)
    result = advisory.query([np.eye(4)], query_ns=200_000_001)
    assert result["state"] == ("blocked" if occupied else "unknown")
