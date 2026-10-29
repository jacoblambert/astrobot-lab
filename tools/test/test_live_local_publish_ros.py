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
from local_terrain.global_stitch import GlobalStitch
from local_terrain.nav2_contract import Footprint, Nav2Config, clear_footprint, grid_pose, planner_costs
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
        array=__import__("array").array,
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
        nav=False,
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


def node_methods(*names, **extra):
    """The named nested LocalNode methods, compiled against `extra` as their globals."""
    source = Path(__file__).resolve().parents[1] / "live_local_terrain.py"
    methods = [n for n in ast.walk(ast.parse(source.read_text())) if isinstance(n, ast.FunctionDef) and n.name in names]
    assert sorted(m.name for m in methods) == sorted(names)
    namespace = dict(array=__import__("array").array, np=np, json=json, Rotation=Rotation, OccupancyGrid=OccupancyGrid, String=String, inflate=inflate,
                     map_from_grid=map_from_grid, planner_costs=planner_costs, GlobalStitch=GlobalStitch,
                     clear_footprint=clear_footprint, grid_pose=grid_pose, **extra)
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), "exec"), namespace)
    return namespace


def test_nav_publish_feeds_the_mosaic_and_publishes_it_as_one_grid():
    ns = node_methods("publish", "stitch", "publish_mosaic")
    sent = {name: [] for name in ("class_pub", "map_pub", "planner_cost_pub", "uncertainty_pub", "status_pub", "mosaic_pub",
                                  "explore_pub")}
    local_from_map = np.eye(4)
    local_from_map[:3, 3] = [-40.05, -38.05, -0.1]  # nav grids are axis-aligned (translation only)
    local_from_base = np.eye(4)
    local_from_base[:2, 3] = [-1.5, -1.5]
    record = dict(origin=[-3, -3], local_from_map=local_from_map, local_from_base=local_from_base.tolist())
    node = SimpleNamespace(contact_advisory=None, nav=True, isolated_gap_costs=True, mosaic=None, explore=None,
                           mosaic_half_extent=10.,
                           planner_policy=PUBLISHED_POLICIES["nav2_bounded_shadow"],
                           planner_footprint=Footprint((60, 60), (0.0, 0.0), 0.1, Nav2Config.from_yaml()),
                           last_grid_pose=np.eye(4), core=SimpleNamespace(last_record=record, watermark_ns=5 * 10**9),
                           status=lambda: {})
    for name in sent:
        setattr(node, name, SimpleNamespace(publish=lambda message, name=name: sent[name].append(message)))
    node.stitch = lambda cost, classes: ns["stitch"](node, cost, classes)
    layers = np.zeros((60, 60, 6))
    layers[40:42, 40:42, 0] = 100
    layers[5:8, 5:8, 0] = 20  # inferred ground: free on the exploration map
    layers[50:53, 50:53, 0] = -1  # unobserved: policy cost on the Nav2 mosaic, unknown for exploration
    layers[5:25, 5:25, 0] = -1  # unobserved around the rover (body centred on cell 15, 15)
    layers[15, 15, 0] = 100  # an observed obstacle under the body stays occupied
    ns["publish"](node, layers)
    ns["publish"](node, None)  # unknown output never erases the mosaic
    ns["publish_mosaic"](node)
    mosaic = sent["mosaic_pub"][-1]
    grid = np.asarray(mosaic.data, dtype=np.int8).reshape(mosaic.info.height, mosaic.info.width)
    window = np.asarray(sent["planner_cost_pub"][0].data, dtype=np.int8).reshape(60, 60)
    o = np.rint((np.array([sent["planner_cost_pub"][0].info.origin.position.x, sent["planner_cost_pub"][0].info.origin.position.y])
                 - np.array([mosaic.info.origin.position.x, mosaic.info.origin.position.y])) / 0.1).astype(int)
    assert (grid[o[1]:o[1] + 60, o[0]:o[0] + 60] == window).all() and (grid == 100).sum() == (window == 100).sum() > 0
    assert grid.shape == (200, 200) and mosaic.header.frame_id == "map" and mosaic.info.origin.orientation.w == 1.
    decoded = deserialize_message(serialize_message(mosaic), OccupancyGrid)
    assert decoded.data == mosaic.data
    explore = np.asarray(sent["explore_pub"][-1].data, dtype=np.int8).reshape(200, 200)[o[1]:o[1] + 60, o[0]:o[0] + 60]
    assert set(np.unique(explore)) <= {-1, 0, 100} and (explore[40:42, 40:42] == 100).all()
    assert (explore[50:53, 50:53] == -1).all()
    # unobserved cells under the unpadded body (half extents ~0.5 x 0.34 m) are free; beyond it, unknown
    assert (explore[13:17, 11:19] == 0).sum() == 31 and explore[15, 15] == 100
    assert (explore[5:8, 5:8] == -1).all() and explore[15, 22] == -1 and explore[20, 15] == -1


def test_nav_planner_window_keeps_an_obstacle_the_live_window_lost():
    """Nav2's local costmap reads the published window: a rock seen earlier, now behind the front
    LiDAR or past the evidence lifetime, stays lethal there (seed-411 gate: MPPI reversed into one)."""
    ns = node_methods("publish", "stitch")
    sent = {name: [] for name in ("class_pub", "map_pub", "planner_cost_pub", "uncertainty_pub", "status_pub")}
    local_from_base = np.eye(4)
    local_from_base[:2, 3] = [-1.5, -1.5]
    record = dict(origin=[-3, -3], local_from_map=np.eye(4), local_from_base=local_from_base.tolist())
    node = SimpleNamespace(contact_advisory=None, nav=True, isolated_gap_costs=True, mosaic=None, explore=None,
                           mosaic_half_extent=10., planner_policy=PUBLISHED_POLICIES["nav2_bounded_shadow"],
                           planner_footprint=Footprint((60, 60), (0.0, 0.0), 0.1, Nav2Config.from_yaml()),
                           last_grid_pose=np.eye(4), core=SimpleNamespace(last_record=record, watermark_ns=0),
                           status=lambda: {})
    for name in sent:
        setattr(node, name, SimpleNamespace(publish=lambda message, name=name: sent[name].append(message)))
    node.stitch = lambda cost, classes: ns["stitch"](node, cost, classes)
    seen = np.zeros((60, 60, 6))
    seen[40:42, 40:42, 0] = 100
    lost = np.array(seen)
    lost[35:47, 35:47, 0] = -1  # same pose, the rock no longer observed
    ns["publish"](node, seen)
    ns["publish"](node, lost)
    window = np.asarray(sent["planner_cost_pub"][-1].data, dtype=np.int8).reshape(60, 60)
    assert (window[40:42, 40:42] == 100).all()
    assert (window[36:39, 36:39] >= 0).all()  # observed-free memory, not unknown


def test_reset_map_rebuilds_the_core_and_refuses_a_stale_epoch():
    from astrobot_interfaces.srv import ResetMap

    ns = node_methods("reset_map", stamp_ns=lambda t: t.sec * 10**9 + t.nanosec, ReceiptScheduler=lambda core: ("sched", core))
    closed, events, published = [], [], []
    old = SimpleNamespace(close=lambda: closed.append(True))
    node = SimpleNamespace(core=old, epoch=None, armed_ns=5, mosaic="old", explore="old", make_core=lambda: "fresh",
                           event=lambda kind, payload: events.append((kind, payload)), publish=published.append,
                           get_logger=lambda: SimpleNamespace(error=print))
    request = ResetMap.Request(epoch=2)
    request.minimum_stamp.sec = 7
    response = ns["reset_map"](node, request, ResetMap.Response())
    assert response.success and response.epoch == 2 and closed == [True] and published == [None]
    assert (node.core, node.scheduler, node.epoch, node.armed_ns, node.mosaic, node.explore, node.minimum_ns) == (
        "fresh", ("sched", "fresh"), 2, None, None, None, 7 * 10**9)
    assert events == [("reset", dict(epoch=2, minimum_ns=7 * 10**9))]
    stale = ns["reset_map"](node, ResetMap.Request(epoch=1), ResetMap.Response())
    assert not stale.success and stale.reason == "stale_epoch" and node.epoch == 2


def test_nav_inputs_wait_for_the_reset_epoch_then_join_the_stream():
    received = []
    ns = node_methods("revision", "chunk", revision_record=lambda m: dict(m), decode=lambda m: dict(m),
                      chassis_returns=lambda points, setup: np.zeros(len(points), bool), hashlib=__import__("hashlib"))

    class Core:
        capture, received, joined = None, 0, None

        def resume_after(self, capture, sequence):
            self.capture, self.joined = capture, sequence

    node = SimpleNamespace(nav=True, epoch=None, armed_ns=None, minimum_ns=0, stream_capture=None, core=Core(),
                           contact_advisory=None, setup=None, guarded=lambda f: f(), event=lambda *a: None,
                           raw=SimpleNamespace(tell=lambda: 0, write=len, flush=lambda: None),
                           scheduler=SimpleNamespace(receive=lambda kind, payload, size: received.append((kind, payload))))
    def rev(epoch, stamps, kind=1):
        message = type("M", (dict,), {"kind": kind})(epoch=epoch, frames=[dict(stamp_ns=t) for t in stamps])
        ns["revision"](node, message)

    def nat(seq, stamp, capture="sim-a"):
        message = type("C", (dict,), {"data": b"x"})(capture=capture, sequence=seq, stamp_ns=stamp,
                                                     points=np.zeros((2, 3)), intensities=np.ones(2))
        ns["chunk"](node, message)

    rev(1, [10])
    nat(0, 5)
    assert received == []  # nothing before the supervisor's reset
    node.epoch, node.minimum_ns = 2, 20
    rev(1, [30])  # previous child's late revision
    rev(2, [25], kind=2)  # bookkeeping cannot arm
    nat(1, 21)
    assert received == [] and node.armed_ns is None
    rev(2, [25, 26])
    assert node.armed_ns == 25 and [k for k, _ in received] == ["revision"]
    nat(2, 24)  # before the epoch's first frame: dropped
    nat(3, 25)
    assert node.core.joined == 2 and [k for k, _ in received] == ["revision", "chunk"]
    with pytest.raises(ValueError):
        nat(4, 26, capture="sim-b")


def test_projection_lines_drop_associations_only_without_the_raw_archive():
    ns = node_methods("projection_line")
    record = dict(associations=[[1, 2]] * 3, layers_sha256="x")
    lean = json.loads(ns["projection_line"](SimpleNamespace(archive_raw=False), record))
    assert lean["associations"] is None and lean["association_count"] == 3 and lean["layers_sha256"] == "x"
    assert json.loads(ns["projection_line"](SimpleNamespace(archive_raw=True), record)) == record
    assert record["associations"] == [[1, 2]] * 3  # the core's record is not mutated


@pytest.mark.parametrize("nav", [False, True])
def test_nav_tolerates_a_transient_second_revision_publisher(nav):
    clock = [100.0]
    ns = node_methods("watchdog", check_run=lambda run: None, HEARTBEAT_S=2.0, DUPLICATE_AUTHORITY_GRACE_S=10.0,
                      time=SimpleNamespace(monotonic=lambda: clock[0]))
    faults, counts = [], {"/terrain_shadow/native_chunks": 1, "/astrobot_0/slam_raw/revisions": 2}

    def guarded(action):
        try:
            action()
        except ValueError as exc:
            faults.append(str(exc))

    node = SimpleNamespace(nav=nav, run=None, last_clock_wall=clock[0], duplicate_authority_since=None,
                           core=SimpleNamespace(failure=None, completed=False, capture="sim"),
                           count_publishers=lambda topic: counts[topic], guarded=guarded, status=lambda: {},
                           status_pub=SimpleNamespace(publish=lambda message: None))
    ns["watchdog"](node)
    assert faults == ([] if nav else ["multiple v2 input authorities"])
    counts["/astrobot_0/slam_raw/revisions"] = 1  # the old GLIM left discovery: the grace resets
    ns["watchdog"](node)
    assert node.duplicate_authority_since is None
    counts["/astrobot_0/slam_raw/revisions"] = 2
    ns["watchdog"](node)
    clock[0] = node.last_clock_wall = 111.0
    ns["watchdog"](node)  # a second authority that persists still faults
    assert faults[-1] == "multiple v2 input authorities"
