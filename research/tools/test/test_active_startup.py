import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from audit_lidar_attachment import vertical_separation
from diagnose_active_startup import HEIGHT, flat_returns, observe_route, pose
from local_terrain.ray_design import selected_setup


def test_probe_returns_to_center_before_each_turn():
    previous = pose()
    route = observe_route()
    assert len(route) == 213
    for target, duration, action in route:
        assert duration > 0
        assert np.linalg.norm(target[:2, 3]) <= 0.15 + 1e-12
        if action == "turn":
            np.testing.assert_array_equal(previous[:2, 3], [0, 0])
            np.testing.assert_array_equal(target[:2, 3], [0, 0])
        else:
            np.testing.assert_allclose(target[:3, :3], previous[:3, :3], atol=1e-14)
            assert np.isclose(np.linalg.norm(target[:2, 3] - previous[:2, 3]), 0.05)
        previous = target


def test_flat_scan_retains_chassis_occlusion_and_software_minimum():
    setup, elevations = selected_setup(
        Path(__file__).resolve().parents[1] / "local_terrain/selected_lidar_optical.json"
    )
    points, sensor = flat_returns(setup, elevations)
    assert len(points) == 38427
    np.testing.assert_allclose(points[:, 2], 0, atol=1e-12)
    assert np.linalg.norm(points[:, :2], axis=1).min() > 0.73
    assert np.linalg.norm(points - sensor, axis=1).min() > 0.5
    assert np.isclose(sensor[2], HEIGHT + 0.45)


def test_attachment_separation_ignores_distant_tall_mesh():
    support = [[0.35, -0.05, 0.26], [0.45, 0.05, 0.41]]
    body = [[-0.4, -0.28, 0], [0.4, 0.28, 0.224]]
    remote = [[-0.3, -0.1, 0], [-0.2, 0.1, 5]]
    assert np.isclose(vertical_separation(support, [body, remote]), 0.036)
    assert vertical_separation(support, [remote]) is None


def test_all_declared_observation_segments_fit_fixed_eroded_startup_support():
    from local_terrain.contact_index import IncrementalSupportIndex
    from local_terrain.contact_query import PreparedMotion
    from local_terrain.contact_runtime import startup_footprint
    from local_terrain.contact_support import erode
    from local_terrain.physx_reference import BodyBox

    layers = np.zeros((60, 60, 6))
    layers[:, :, 0] = -1
    record = dict(origin=[-3.0, -3.0], resolution=0.1)
    index = IncrementalSupportIndex()
    index.update([(erode(startup_footprint(True), 0.02), "fixed_startup")], record["origin"], 0.1)
    patches = [
        BodyBox((x, y, -HEIGHT), (0.04, 0.06, 0.001), "wheel") for x in (-0.256, 0.256) for y in (-0.2854, 0.2854)
    ]
    body = BodyBox((0, 0, 0), (0.85, 0.70, 0.001), "body")
    previous = pose()
    for target, _, _ in observe_route():
        decision = PreparedMotion(layers, record, [previous, target], patches, body).evaluate(index)
        assert decision["state"] == "available"
        previous = target
