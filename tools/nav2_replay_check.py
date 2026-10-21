#!/usr/bin/env python3
"""Offline Nav2 planning check on a recorded /terrain_shadow/planner_costmap (Nav2 step N1).

Runs two real planner_server instances (SmacPlanner2D + StaticLayer + InflationLayer
from nav2_phase3_bev.yaml): `shadow` is fed the costmap the live publisher recorded,
`gt` a grid of the GT rock cores on the same lattice. At sampled frames both plan
from the rover to goals on a ring around it. Every shadow path is moved into the GT
frame (gt_base @ inv(slam_base) at the frame stamp: the rover tracks the path with
its own pose estimate) and its body footprint is swept against the GT rock cores.

Evaluator only. Container: needs ROS 2 + nav2, and Shapely (contact deps on PYTHONPATH).
"""
import argparse
import copy
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import shapely
import yaml
from shapely import affinity, box

from archive_guard import ARCHIVE_ROOT, allocate_run
from audit_terrain_robustness import HAZARD_HEIGHTS, LARGE, GTTrajectory, Scene, find_pose_arrays
from local_terrain.global_stitch import GlobalStitch

CONFIG = Path(__file__).resolve().parents[1] / "astrobot-lab/src/astrobot_launch/config/nav2_phase3_bev.yaml"
BODY = (0.495, 0.335)  # half length/width, no padding: physical contact
PADDING = 0.10  # footprint_padding: a goal closer than this to a rock is not a goal
GOAL_TOLERANCE = 0.65  # general_goal_checker xy_goal_tolerance: the controller's job past here
NAMESPACES = dict(shadow=("map", "base_link"), gt=("gt_map", "gt_base_link"))


def yaw_of(T):
    return float(np.arctan2(T[1, 0], T[0, 0]))


def params(config, namespace, frames, topic):
    """planner_server + global_costmap parameters for one replay instance."""
    source = yaml.safe_load(config.read_text())
    planner = copy.deepcopy(source["planner_server"]["ros__parameters"])
    costmap = copy.deepcopy(source["global_costmap"]["global_costmap"]["ros__parameters"])
    planner["use_sim_time"] = costmap["use_sim_time"] = False
    costmap.update(global_frame=frames[0], robot_base_frame=frames[1], update_frequency=20.0, publish_frequency=10.0)
    costmap["static_layer"]["map_topic"] = topic
    return {f"/{namespace}/planner_server": dict(ros__parameters=planner),
            f"/{namespace}/global_costmap/global_costmap": dict(ros__parameters=costmap)}


def gt_grid(scene, origin, shape, resolution, height):
    """100 where a cell centre lies in a GT rock core at least `height` tall, else 0."""
    iy, ix = np.mgrid[:shape[0], :shape[1]]
    x, y = origin[0] + (ix + .5) * resolution, origin[1] + (iy + .5) * resolution
    grid = np.zeros(shape, dtype=np.int8)
    for o in scene.objects:
        if o["height"] >= height:
            grid[shapely.contains_xy(o["core"], x, y)] = 100
    return grid


def transit(xy):
    """The path up to where the goal checker would accept arrival (the start pose at least)."""
    far = np.linalg.norm(xy - xy[-1], axis=1) > GOAL_TOLERANCE
    return xy[:max(1, int(np.argmin(far)) if not far.all() else len(xy))]


def densify(xy, step=0.05):
    out = [xy[0]]
    for a, b in zip(xy[:-1], xy[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out.extend(a + (b - a) * (k / n) for k in range(1, n + 1))
    return np.asarray(out)


def swept(xy, start_yaw, step=0.1):
    """Body rectangles along a path, heading from the path tangent. The rover starts at its
    own yaw; any heading change (start, kinks) is swept as an in-place turn (conservative)."""
    xy = densify(xy)
    d = np.diff(xy, axis=0)
    yaw = np.concatenate([[start_yaw], np.arctan2(d[:, 1], d[:, 0])])
    rect = box(-BODY[0], -BODY[1], BODY[0], BODY[1])
    poses = [(*xy[0], yaw[0])]
    for i in range(1, len(xy)):
        turn = (yaw[i] - poses[-1][2] + np.pi) % (2 * np.pi) - np.pi
        n = int(abs(turn) // step)
        poses.extend((*xy[i - 1], poses[-1][2] + turn * k / (n + 1)) for k in range(1, n + 1))
        poses.append((*xy[i], yaw[i]))
    return [affinity.translate(affinity.rotate(rect, a, origin=(0, 0), use_radians=True), x, y) for x, y, a in poses]


def collisions(scene, footprints, pad=0.):
    """Tallest GT rock core the swept body touches (0 when none) and the body-core clearance."""
    union = shapely.union_all(footprints)
    if pad:
        union = union.buffer(pad)
    tallest, clearance, where = 0.0, np.inf, None
    for o in scene.objects:
        if o["height"] < HAZARD_HEIGHTS[0]:
            continue
        if union.intersects(o["core"]):
            tallest = max(tallest, o["height"])
            first = int(np.argmax(shapely.intersects(footprints, o["core"])))
            where = first / max(1, len(footprints) - 1) if where is None else min(where, first / max(1, len(footprints) - 1))
        if o["height"] >= LARGE:
            clearance = min(clearance, union.distance(o["core"]))
    return tallest, clearance, where


def frames_from_bag(bag, every):
    from nav_msgs.msg import OccupancyGrid
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag), storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    reader.set_filter(StorageFilter(topics=["/terrain_shadow/planner_costmap"]))
    last = -np.inf
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, OccupancyGrid)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        grid = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        if stamp - last < every or (grid == -1).all():
            continue
        q = msg.info.origin.orientation
        if abs(q.w) < 1 - 1e-9:
            raise ValueError("rotated planner_costmap origin; StaticLayer ignores origin yaw")
        last = stamp
        yield stamp, grid, (msg.info.origin.position.x, msg.info.origin.position.y), msg.info.resolution


class Replay:
    """The ROS side: grid/TF publishers, costmap sync and ComputePathToPose clients."""

    def __init__(self):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from nav2_msgs.action import ComputePathToPose
        from nav_msgs.msg import OccupancyGrid
        from tf2_ros import TransformBroadcaster

        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("nav2_replay_check")
        self.tf = TransformBroadcaster(self.node)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grids, self.costmaps, self.clients = {}, {}, {}
        for ns in NAMESPACES:
            self.grids[ns] = self.node.create_publisher(OccupancyGrid, f"/nav2_replay/{ns}_grid", 1)
            self.node.create_subscription(OccupancyGrid, f"/{ns}/global_costmap/costmap",
                                          lambda m, ns=ns: self.costmaps.__setitem__(ns, m), latched)
            self.clients[ns] = ActionClient(self.node, ComputePathToPose, f"/{ns}/compute_path_to_pose")
        self.Goal = ComputePathToPose.Goal
        self.OccupancyGrid = OccupancyGrid
        self.poses = {}

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
            self.broadcast()

    def broadcast(self):
        from geometry_msgs.msg import TransformStamped

        now = self.node.get_clock().now().to_msg()
        for ns, (x, y, yaw) in self.poses.items():
            t = TransformStamped()
            t.header.stamp, (t.header.frame_id, t.child_frame_id) = now, NAMESPACES[ns]
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.rotation.z, t.transform.rotation.w = np.sin(yaw / 2), np.cos(yaw / 2)
            self.tf.sendTransform(t)

    def wait_ready(self, timeout=60.):
        end = time.monotonic() + timeout
        while not all(c.server_is_ready() for c in self.clients.values()):
            if time.monotonic() > end:
                raise TimeoutError("planner_server actions not available")
            self.spin(0.2)

    def load(self, grids, origins, resolution, timeout=5.):
        """Publish one grid per instance; return the master costmaps once both reflect them."""
        sent = self.node.get_clock().now()
        for ns, grid in grids.items():
            msg = self.OccupancyGrid()
            msg.header.frame_id, msg.header.stamp = NAMESPACES[ns][0], sent.to_msg()
            msg.info.width, msg.info.height, msg.info.resolution = grid.shape[1], grid.shape[0], resolution
            msg.info.origin.position.x, msg.info.origin.position.y = origins[ns]
            msg.info.origin.orientation.w = 1.
            msg.data = grid.ravel().tolist()
            self.grids[ns].publish(msg)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin(0.05)
            done = {}
            for ns, grid in grids.items():
                m = self.costmaps.get(ns)
                if m is None or self.rclpy.time.Time.from_msg(m.header.stamp) <= sent:
                    continue
                master = np.asarray(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
                o = m.info.origin.position
                # lethal cores carry through StaticLayer unchanged: the costmap holds this grid
                if (master.shape == grid.shape and np.allclose((o.x, o.y), origins[ns], atol=1e-6)
                        and ((master == 100) == (grid == 100)).all()):
                    done[ns] = master
            if len(done) == len(grids):
                self.spin(0.1)  # one more update cycle so the planner's copy is current
                return done
        raise TimeoutError("costmap did not take the replayed grid")

    def plan(self, ns, start, goal, timeout=10.):
        from geometry_msgs.msg import PoseStamped

        request = self.Goal()
        request.use_start, request.planner_id = True, "GridBased"
        for field, (x, y, yaw) in (("start", start), ("goal", goal)):
            p = PoseStamped()
            p.header.frame_id = NAMESPACES[ns][0]
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation.z, p.pose.orientation.w = np.sin(yaw / 2), np.cos(yaw / 2)
            setattr(request, field, p)
        future = self.clients[ns].send_goal_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        handle = future.result()
        if handle is None or not handle.accepted:
            return None, "rejected"
        result = handle.get_result_async()
        self.rclpy.spin_until_future_complete(self.node, result, timeout_sec=timeout)
        if result.result() is None:
            return None, "timeout"
        value = result.result().result
        path = np.array([[p.pose.position.x, p.pose.position.y] for p in value.path.poses])
        return (path if len(path) else None), int(value.error_code)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def start_servers(output, config):
    env = dict(os.environ)
    procs = []
    for ns, frames in NAMESPACES.items():
        path = output / f"{ns}_params.yaml"
        path.write_text(yaml.safe_dump(params(config, ns, frames, f"/nav2_replay/{ns}_grid")))
        log = (output / f"{ns}_planner.log").open("w")
        procs.append(subprocess.Popen(["ros2", "run", "nav2_planner", "planner_server", "--ros-args",
                                       "-r", f"__ns:=/{ns}", "--params-file", str(path)],
                                      stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True))
        procs.append(subprocess.Popen(["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager", "--ros-args",
                                       "-r", f"__ns:=/{ns}", "-r", "__node:=lifecycle_manager_replay",
                                       "-p", "autostart:=true", "-p", "node_names:=['planner_server']",
                                       "-p", "bond_timeout:=0.0"],
                                      stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True))
    return procs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recorder", required=True, help="recorder run (revision-... prefix)")
    parser.add_argument("--every", type=float, default=2.0, help="sim seconds between sampled frames")
    parser.add_argument("--radius", type=float, default=2.4, help="goal ring radius (m)")
    parser.add_argument("--headings", type=int, default=16)
    parser.add_argument("--gt-height", type=float, default=0.10, help="GT rocks at least this tall block the GT planner")
    parser.add_argument("--limit", type=int, default=None, help="stop after this many frames")
    parser.add_argument("--global-map", action="store_true",
                        help="plan on the mosaic of every window so far, to home and to points back along the route")
    parser.add_argument("--back", type=float, nargs="+", default=[5., 10., 20., 40.],
                        help="--global-map goals: route distances back from the rover (m), plus home")
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive") if Path("/archive").is_dir() else ARCHIVE_ROOT)
    args = parser.parse_args()

    matches = sorted(args.archive_root.glob(args.recorder + "*"))
    if len(matches) != 1:
        raise SystemExit(f"{args.recorder}: {len(matches)} archive matches")
    recorder = matches[0]
    request = json.loads((recorder / "terrain_request.json").read_text())
    capture = args.archive_root / request["capture"]
    scene = Scene(capture, request["home_xyz"])
    with np.load(find_pose_arrays(args.archive_root, request["capture"]), allow_pickle=False) as data:
        gt, slam = GTTrajectory(data["times"], data["gt_base"]), GTTrajectory(data["times"], data["first_published"])

    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    procs = start_servers(output, args.config)
    replay = Replay()
    rows, masters, invalid, began = [], [], 0, time.monotonic()
    stitch, truth_global, last = None, None, -np.inf
    route = slam.p[:, :2, 3]
    along = np.concatenate([[0.], np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))])
    try:
        replay.wait_ready()
        count = 0
        for stamp, grid, origin, resolution in frames_from_bag(recorder / "bag", 0. if args.global_map else args.every):
            if args.limit is not None and count >= args.limit:
                break
            if args.global_map:
                if stitch is None:
                    lo, hi = gt.p[:, :2, 3].min(0) - 8., gt.p[:, :2, 3].max(0) + 8.
                    stitch = GlobalStitch(lo, hi, origin, resolution)
                    truth_global = gt_grid(scene, stitch.origin, stitch.grid.shape, resolution, args.gt_height)
                stitch.update(grid, origin)
                if stamp - last < args.every:
                    continue
            if not gt.t[0] <= stamp <= gt.t[-1]:
                continue
            last = stamp
            count += 1
            S, G = slam(stamp), gt(stamp)
            delta = G @ np.linalg.inv(S)  # SLAM map -> GT map at this stamp
            start = (S[0, 3], S[1, 3], yaw_of(S))
            gstart = (G[0, 3], G[1, 3], yaw_of(G))
            if args.global_map:
                # one GT raster on the mosaic lattice; delta (< 0.1 m here) only moves start/goals
                grid, origin, truth, g_origin = stitch.grid.copy(), tuple(stitch.origin), truth_global, tuple(stitch.origin)
            else:
                # GT grid on the same lattice, origin carried by delta (yaw error < cell over 3 m; recorded)
                g_origin = tuple((delta @ np.array([*origin, 0., 1.]))[:2])
                truth = gt_grid(scene, g_origin, grid.shape, resolution, args.gt_height)
            replay.poses = dict(shadow=start, gt=gstart)
            master = replay.load(dict(shadow=grid, gt=truth), dict(shadow=origin, gt=g_origin), resolution)
            if not args.global_map:
                masters.append((stamp, master["shadow"], np.asarray(origin)))
            lo = np.asarray(origin) + 0.5
            hi = np.asarray(origin) + np.array(grid.shape[::-1]) * resolution - 0.5
            if args.global_map:
                now = along[min(len(along) - 1, np.searchsorted(slam.t, stamp))]
                targets = [route[0]] + [route[np.searchsorted(along, now - d)] for d in args.back if now - d >= 0]
                goals = [(x, y, float(np.arctan2(y - start[1], x - start[0]))) for x, y in targets
                         if np.hypot(x - start[0], y - start[1]) > args.radius]
            else:
                goals = []
                for k in range(args.headings):
                    a = start[2] + 2 * np.pi * k / args.headings
                    goals.append((start[0] + args.radius * np.cos(a), start[1] + args.radius * np.sin(a), a))
            for k, goal in enumerate(goals):
                a = goal[2]
                if not ((lo <= goal[:2]) & (goal[:2] <= hi)).all():
                    continue
                gg = delta @ np.array([goal[0], goal[1], 0., 1.])
                ggoal = (gg[0], gg[1], a + yaw_of(delta))
                if collisions(scene, swept(np.array([ggoal[:2]]), ggoal[2]), pad=PADDING)[0] >= args.gt_height:
                    invalid += 1  # no mission would send it: the body at the goal touches a rock
                    continue
                path, code = replay.plan("shadow", start, goal)
                gpath, gcode = replay.plan("gt", gstart, ggoal)
                row = dict(stamp=stamp, heading=k, goal_distance=float(np.hypot(goal[0] - start[0], goal[1] - start[1])),
                           shadow_code=code, gt_code=gcode,
                           shadow_found=path is not None, gt_found=gpath is not None,
                           slam_error_m=float(np.hypot(*(G[:2, 3] - S[:2, 3]))))
                if path is not None:
                    moved = (delta[:2, :2] @ path.T).T + delta[:2, 3]
                    tallest, clearance, where = collisions(scene, swept(moved, gstart[2]))
                    row["shadow_transit_hit_height"] = collisions(scene, swept(transit(moved), gstart[2]))[0]
                    row.update(shadow_length=float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
                               shadow_hit_height=tallest, shadow_hit_at=where,
                               shadow_large_clearance=float(clearance) if np.isfinite(clearance) else None,
                               shadow_path=np.round(path, 3).tolist())
                if gpath is not None:
                    row["gt_path"] = np.round(gpath, 3).tolist()
                    row["gt_length"] = float(np.linalg.norm(np.diff(gpath, axis=0), axis=1).sum())
                    row["gt_hit_height"], _, row["gt_hit_at"] = collisions(scene, swept(gpath, gstart[2]))
                    row["gt_transit_hit_height"] = collisions(scene, swept(transit(gpath), gstart[2]))[0]
                rows.append(row)
            if count % 20 == 0:
                print(f"frame {count} t={stamp:.1f} plans={len(rows)}", flush=True)
    finally:
        replay.close()
        for p in procs:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in procs:
            try:
                p.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)

    summary = summarize(rows)
    summary.update(mode="global_mosaic" if args.global_map else "local_window",
                   mosaic=dict(shape=list(stitch.grid.shape), windows=stitch.windows, clipped=stitch.clipped,
                               observed_fraction=float((stitch.grid != -1).mean())) if stitch else None,
                   goals_invalid_skipped=invalid, wall_s=round(time.monotonic() - began, 1), recorder=recorder.name, capture=capture.name, every_s=args.every, radius_m=args.radius,
                   gt_height_m=args.gt_height, config=str(args.config),
                   scope="SmacPlanner2D + StaticLayer + InflationLayer on the recorded costmap; no controller")
    with (output / "plans.jsonl").open("x") as stream:
        for row in rows:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
    if stitch is not None:
        with (output / "mosaic.npz").open("xb") as stream:
            np.savez_compressed(stream, grid=stitch.grid, origin=stitch.origin, truth=truth_global)
    with (output / "masters.npz").open("xb") as stream:
        np.savez_compressed(stream, stamps=np.array([m[0] for m in masters]),
                            masters=np.array([m[1] for m in masters]), origins=np.array([m[2] for m in masters]))
    with (output / "summary.json").open("x") as stream:
        json.dump(summary, stream, indent=2, allow_nan=False)
    print(json.dumps(summary, indent=2))


def summarize(rows):
    found = [r for r in rows if r["shadow_found"]]
    feasible = [r for r in rows if r["gt_found"] and r.get("gt_hit_height", 0) < LARGE]
    out = dict(frames=len({r["stamp"] for r in rows}), goals=len(rows), shadow_found=len(found),
               gt_found=sum(r["gt_found"] for r in rows),
               availability=(sum(r["shadow_found"] for r in feasible) / len(feasible)) if feasible else None,
               shadow_found_gt_blocked=sum(r["shadow_found"] and not r["gt_found"] for r in rows))
    for h in HAZARD_HEIGHTS:
        for kind, prefix in (("transit", "_transit"), ("arrival", "")):
            out[f"shadow_{kind}_hits_ge_{h:.2f}"] = sum(r[f"shadow{prefix}_hit_height"] >= h for r in found)
            out[f"gt_{kind}_hits_ge_{h:.2f}"] = sum(r.get(f"gt{prefix}_hit_height", 0) >= h for r in rows if r["gt_found"])
    clear = [r["shadow_large_clearance"] for r in found if r["shadow_large_clearance"] is not None]
    out["shadow_large_clearance_m"] = dict(min=min(clear), p05=float(np.percentile(clear, 5))) if clear else None
    both = [r for r in found if r["gt_found"]]
    out["length_ratio_median"] = float(np.median([r["shadow_length"] / r["gt_length"] for r in both])) if both else None
    codes = {}
    for r in rows:
        if not r["shadow_found"]:
            codes[str(r["shadow_code"])] = codes.get(str(r["shadow_code"]), 0) + 1
    out["shadow_failure_codes"] = codes
    out["slam_error_m_max"] = max((r["slam_error_m"] for r in rows), default=None)
    return out


if __name__ == "__main__":
    sys.exit(main())
