#!/usr/bin/env python3
"""Closed-loop v2 Nav2 trial (N3 T2/T3): scripted NavigateToPose goals, GT-scored in-node.

Run in the dev container beside a running v2 stack (phase3_nav slam_stack:=v2) and the sim nav
stream. Goals are drawn from GT geometry (the sim nav run's natural_scene.json and terrain tile):
3-8 m from the rover, the padded body clear of every >= 0.10 m rock, inside the terrain. Every GT
pose sample is scored for body contact with GT rock cores, and SLAM error is TF map->base_link
against GT. Faults (T3) are injected outside this script, e.g. an RTK outage window.

Evaluator only: GT never reaches the stack.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import shapely
from shapely import affinity, box

from archive_guard import ARCHIVE_ROOT, GIB, allocate_run
from audit_terrain_robustness import HAZARD_HEIGHTS, LARGE, Scene
from nav2_replay_check import BODY, PADDING

GOAL_REACHED_M = 0.8  # Nav2 xy_goal_tolerance 0.65 plus slack for the GT/estimate offset


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def body(x, y, yaw, pad=0.):
    rect = box(-BODY[0] - pad, -BODY[1] - pad, BODY[0] + pad, BODY[1] + pad)
    return affinity.translate(affinity.rotate(rect, yaw, origin=(0, 0), use_radians=True), x, y)


class GoalSampler:
    def __init__(self, scene, rng, *, margin=3.0, clear=0.30, min_height=HAZARD_HEIGHTS[0]):
        self.scene, self.rng, self.clear = scene, rng, clear
        self.lo, self.hi = scene.tile_xy.min(0) + margin, scene.tile_xy.max(0) - margin
        self.rocks = [o["core"] for o in scene.objects if o["height"] >= min_height]
        self.tree = shapely.STRtree(self.rocks)

    def free(self, x, y, yaw):
        footprint = body(x, y, yaw, self.clear)
        return not len(self.tree.query(footprint, predicate="intersects"))

    def __call__(self, x, y, near=3.0, far=8.0, tries=500):
        for _ in range(tries):
            d, a = self.rng.uniform(near, far), self.rng.uniform(-math.pi, math.pi)
            gx, gy = x + d * math.cos(a), y + d * math.sin(a)
            if ((self.lo <= (gx, gy)) & ((gx, gy) <= self.hi)).all() and self.free(gx, gy, a):
                return gx, gy, a
        raise RuntimeError("no free goal found")


class Trial:
    def __init__(self, args, scene, home):
        import rclpy
        from rclpy.action import ActionClient
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose
        from astrobot_interfaces.msg import LocalizationStatus
        from tf2_ros import Buffer, TransformListener

        rclpy.init()
        self.rclpy, self.PoseStamped, self.Goal = rclpy, PoseStamped, NavigateToPose.Goal
        self.node = rclpy.create_node("v2_nav_trial", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.scene, self.home = scene, np.asarray(home[:2], dtype=float)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self.node)
        self.client = ActionClient(self.node, NavigateToPose, "/astrobot_0/navigate_to_pose")
        self.gt, self.slam, self.status = [], [], []
        self.node.create_subscription(PoseStamped, "/gt/base_link_pose", self.gt_pose, 50)
        self.node.create_subscription(LocalizationStatus, "/astrobot_0/localization/status", self.health, 10)
        self.rocks = [(o["core"], o["height"]) for o in scene.objects if o["height"] >= HAZARD_HEIGHTS[0]]
        self.tree = shapely.STRtree([c for c, _ in self.rocks])
        self.contacts = []

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def gt_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x, y, yaw = p.x - self.home[0], p.y - self.home[1], yaw_of(q)  # map = world - home
        self.gt.append((t, x, y, yaw))
        hits = self.tree.query(body(x, y, yaw), predicate="intersects")
        if len(hits):
            tallest = max(self.rocks[i][1] for i in hits)
            self.contacts.append((t, x, y, yaw, tallest))
        try:
            tf = self.tf.lookup_transform("map", "base_link", msg.header.stamp)
            tr = tf.transform.translation
            self.slam.append((t, tr.x, tr.y, yaw_of(tf.transform.rotation)))
        except Exception:
            pass

    def health(self, msg):
        self.status.append((self.now(), int(msg.state), bool(msg.motion_permitted), msg.reason))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_permitted(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin(0.5)
            if self.status and self.status[-1][2] and self.gt:
                return True
        return False

    def go(self, goal, timeout_sim):
        request = self.Goal()
        request.pose.header.frame_id = "map"
        request.pose.header.stamp = self.node.get_clock().now().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = goal[0], goal[1]
        request.pose.pose.orientation.z, request.pose.pose.orientation.w = math.sin(goal[2] / 2), math.cos(goal[2] / 2)
        future = self.client.send_goal_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=30)
        handle = future.result()
        if handle is None or not handle.accepted:
            return "rejected", 0.
        started = self.now()
        result = handle.get_result_async()
        while not result.done():
            self.spin(0.2)
            if self.now() - started > timeout_sim:
                cancel = handle.cancel_goal_async()
                self.rclpy.spin_until_future_complete(self.node, cancel, timeout_sec=10)
                return "timeout", self.now() - started
        status = {4: "succeeded", 5: "canceled", 6: "aborted"}.get(result.result().status, str(result.result().status))
        return status, self.now() - started

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def slam_error(gt, slam):
    if not gt or not slam:
        return None
    g, s = np.asarray(gt), np.asarray(slam)
    gx, gy = np.interp(s[:, 0], g[:, 0], g[:, 1]), np.interp(s[:, 0], g[:, 0], g[:, 2])
    e = np.hypot(s[:, 1] - gx, s[:, 2] - gy)
    return dict(samples=len(e), p50_m=float(np.median(e)), p95_m=float(np.percentile(e, 95)), max_m=float(e.max()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-run", required=True, help="sim nav run (revision-... prefix) with natural_scene.json")
    parser.add_argument("--goals", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--near", type=float, default=3.0)
    parser.add_argument("--far", type=float, default=8.0)
    parser.add_argument("--goal-timeout", type=float, default=240.0, help="sim seconds per goal")
    parser.add_argument("--home", type=float, nargs=3, default=[-20.0, -20.0, 0.0], help="RTK home (map = world - home)")
    parser.add_argument("--archive-root", type=Path, default=Path("/archive") if Path("/archive").is_dir() else ARCHIVE_ROOT)
    args = parser.parse_args()
    matches = sorted(args.archive_root.glob(args.scene_run + "*"))
    if len(matches) != 1:
        raise SystemExit(f"{args.scene_run}: {len(matches)} archive matches")
    scene = Scene(matches[0], args.home)
    sampler = GoalSampler(scene, np.random.default_rng(args.seed))
    output = allocate_run(args.archive_root, limit=GIB)
    print(f"Run: {output}", flush=True)
    trial = Trial(args, scene, args.home)
    goals = []
    try:
        if not trial.client.wait_for_server(timeout_sec=60) or not trial.wait_permitted(300):
            raise RuntimeError("navigation not ready (action server or motion permission)")
        for k in range(args.goals):
            _, x, y, yaw = trial.gt[-1]
            goal = sampler(x, y, args.near, args.far)
            contacts_before = len(trial.contacts)
            status, elapsed = trial.go(goal, args.goal_timeout)
            trial.spin(1.0)
            _, x1, y1, _ = trial.gt[-1]
            reached = math.hypot(x1 - goal[0], y1 - goal[1])
            new = trial.contacts[contacts_before:]
            goals.append(dict(index=k, goal=list(goal), distance_m=math.hypot(goal[0] - x, goal[1] - y), status=status,
                              sim_s=elapsed, gt_final_error_m=reached, success=status == "succeeded" and reached <= GOAL_REACHED_M,
                              contact_samples=len(new), contact_max_height_m=max((c[4] for c in new), default=0.)))
            print(json.dumps(goals[-1]), flush=True)
            if not trial.wait_permitted(300):
                goals[-1]["note"] = "motion not permitted afterwards"
                break
    finally:
        held = [s for s in trial.status if not s[2]]
        summary = dict(
            scene_run=matches[0].name, goals=goals, attempted=len(goals), succeeded=sum(g["success"] for g in goals),
            contact_samples=len(trial.contacts),
            contacts_by_height={f"ge_{h:.2f}": sum(c[4] >= h for c in trial.contacts) for h in HAZARD_HEIGHTS},
            large_contact=any(c[4] >= LARGE for c in trial.contacts),
            slam_error=slam_error(trial.gt, trial.slam),
            motion_held_samples=len(held), held_reasons=sorted({s[3] for s in held}),
            scope="GT-scored closed loop: Nav2 on v2 costmaps, supervisor in the loop; sim at < real time")
        with (output / "trial.json").open("x") as stream:
            json.dump(summary, stream, indent=2, allow_nan=False)
        with (output / "trajectories.npz").open("xb") as stream:
            np.savez_compressed(stream, gt=np.asarray(trial.gt), slam=np.asarray(trial.slam),
                                contacts=np.asarray(trial.contacts).reshape(-1, 5),
                                status=np.asarray([(t, state, permitted) for t, state, permitted, _ in trial.status]).reshape(-1, 3))
        trial.close()
        print(json.dumps({k: v for k, v in summary.items() if k != "goals"}, indent=2))


if __name__ == "__main__":
    main()
