"""Actual live publish method + ROS serialization, without launching a robot.

Extract the nested node method to inject a deterministic six-channel fixture;
this tests output wiring/types, not live native/GLIM scheduling or motion.
"""

import ast
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

pytest.importorskip("rclpy")
from rclpy.serialization import serialize_message, deserialize_message
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.costmap import inflate, map_from_grid
from local_terrain.nav2_contract import Footprint, Nav2Config, planner_costs
from local_terrain.navigation_cost import PUBLISHED_POLICIES


@pytest.mark.parametrize("policy", ["", "isolated", "nav2_bounded_shadow"])
def test_live_publish_preserves_original_outputs_and_invalidates_candidates(policy):
    enabled = bool(policy)
    source = Path(__file__).resolve().parents[1] / "live_local_terrain.py"
    methods = [
        n for n in ast.walk(ast.parse(source.read_text())) if isinstance(n, ast.FunctionDef) and n.name == "publish"
    ]
    assert len(methods) == 1
    namespace = dict(
        np=np,
        json=json,
        Rotation=Rotation,
        OccupancyGrid=OccupancyGrid,
        String=String,
        inflate=inflate,
        map_from_grid=map_from_grid,
        planner_costs=planner_costs,
    )
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), "exec"), namespace)
    messages = {name: [] for name in ("class_pub", "map_pub", "planner_cost_pub", "uncertainty_pub", "status_pub")}
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", [0.1, -0.02, 0.3]).as_matrix()
    transform[:3, 3] = [40, 38, 0.1]
    local_from_base = np.eye(4)
    local_from_base[:2, 3] = [-1.5, -1.5]  # rover at grid (1.5, 1.5) m, clear of the isolated cell
    record = dict(origin=[-3, -3], local_from_map=transform, local_from_base=local_from_base.tolist())
    node = SimpleNamespace(
        contact_advisory=None,
        isolated_gap_costs=enabled,
        planner_policy=PUBLISHED_POLICIES.get(policy),
        planner_footprint=Footprint((60, 60), (0.0, 0.0), 0.1, Nav2Config.from_yaml()),
        last_grid_pose=np.eye(4),
        core=SimpleNamespace(last_record=record, watermark_ns=1_234_567_890),
        status=lambda: {},
    )
    for name in messages:
        setattr(node, name, SimpleNamespace(publish=lambda message, name=name: messages[name].append(message)))
    layers = np.zeros((60, 60, 6))
    layers[30, 30, 0] = -1
    layers[12:19, 12:19, 0] = -1  # the rover's own LiDAR blind patch, centred on it
    namespace["publish"](node, layers)
    assert messages["class_pub"][0].data[30 * 60 + 30] == -1
    assert messages["map_pub"][0].data[30 * 60 + 30] == -1
    if enabled:
        cost = messages["planner_cost_pub"][0]
        expected, mask = planner_costs(layers, record, node.planner_policy, node.planner_footprint)
        assert list(cost.data) == expected.ravel().tolist()
        assert list(messages["uncertainty_pub"][0].data) == (mask.astype(np.int8) * 100).ravel().tolist()
        assert 0 < cost.data[30 * 60 + 30] < 100 and messages["uncertainty_pub"][0].data[30 * 60 + 30] == 100
        # The body never starts on unknown: isolated leaves the patch unknown, so the footprint clear
        # frees it (0); bounded already gives it the finite uncertainty cost, which stays.
        assert cost.data[15 * 60 + 15] == (0 if policy == "isolated" else 70)
        assert cost.info == messages["class_pub"][0].info
        assert cost.header.frame_id == "map" and cost.header.stamp.nanosec == 234_567_890
        decoded = deserialize_message(serialize_message(cost), OccupancyGrid)
        assert decoded.header == cost.header and decoded.data == cost.data
        assert decoded.info.origin == cost.info.origin
        assert (decoded.info.width, decoded.info.height) == (60, 60)
        # MapMetaData.resolution is float32 on the wire, Python float64 before.
        assert decoded.info.resolution == float(np.float32(cost.info.resolution))
    else:
        assert not messages["planner_cost_pub"] and not messages["uncertainty_pub"]
    namespace["publish"](node, None)
    for name in ("class_pub", "map_pub", "planner_cost_pub", "uncertainty_pub"):
        if messages[name]:
            assert set(messages[name][-1].data) == {-1}
