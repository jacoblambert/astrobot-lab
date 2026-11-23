#!/usr/bin/env python3
"""Post-launch preflight for the Phase 4 gate harness. Run INSIDE the dev container
after launching the mission, with the SLAM/nav/exploration stack coming up. Confirms
a run is VALID to score before committing the (long) eval; it does NOT judge mission
success -- that's the GT-scored eval verdict's job (which also drives the retry on
freeze/divergence/false-success). Checks:

  1. GLIM alive   -- slam/pose_corrected is publishing (polls up to ~60s for it).
     No pose -> GLIM is dead / init stalled -> retryable.
  2. Init sane    -- that pose is finite and within a generous bound of the origin
     (SLAM starts at the spawn = identity). This only catches a NaN / runaway-
     divergence init; the rover may already be driving by now (start_delay), so we do
     NOT require it to still be at the origin -- normal early motion is fine.
  3. GT-leak guard -- odom/base_link must be single-parent in the TF tree and gt_map
     must be absent (OMNILRS_GT_TF_ENABLED=false). A gt_map->odom leak both breaks nav
     and contaminates GT scoring. (Lifted from slam_validation_run.sh.)

Exit 0 = OK to proceed; non-zero = retryable failure (reason on stderr).
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from astrobot_interfaces.msg import LocalizationStatus
from tf2_msgs.msg import TFMessage

ODOM_WAIT_S = 60.0   # poll up to this long for GLIM's first odom_corrected
                     # (GLIM startup varies ~40-60s; reception is clock-independent
                     # so we use wall-clock time and do NOT set use_sim_time).
INIT_SANE_M = 30.0   # finite + within this of origin -> not a NaN/runaway init


def main() -> int:
    rclpy.init()
    node = Node("phase4_gate_preflight", parameter_overrides=[Parameter("use_sim_time", value=True)])

    state = {"odom": None, "health": None, "health_wall": 0.0}
    parents: dict[str, set] = {}
    node.create_subscription(LocalizationStatus, "/astrobot_0/localization/status",
        lambda m: state.update(health=m, health_wall=time.monotonic()), 20)

    # Use pose_corrected (geometry_msgs/PoseStamped) -- odom_corrected is
    # nav_msgs/Odometry; a PoseStamped sub on it silently receives nothing.
    node.create_subscription(
        PoseStamped,
        "/astrobot_0/slam/pose_corrected",
        lambda m: state.update(odom=(m.pose.position.x, m.pose.position.y)),
        10,
    )
    node.create_subscription(
        TFMessage,
        "/tf",
        lambda m: [parents.setdefault(t.child_frame_id, set()).add(t.header.frame_id) for t in m.transforms],
        200,
    )

    # Wait (adaptively) for GLIM's first odom, then gather TF for a few seconds.
    t0 = time.time()
    while time.time() - t0 < ODOM_WAIT_S and state["odom"] is None:
        rclpy.spin_once(node, timeout_sec=0.05)
    t1 = time.time()
    while time.time() - t1 < 4.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    odom = state["odom"]
    navchain = {c: sorted(p) for c, p in parents.items() if len(p) > 1}
    gt_in_tree = "gt_map" in {pp for ps in parents.values() for pp in ps}
    health = state["health"]
    health_valid = (health is not None and health.state == LocalizationStatus.HEALTHY
                    and health.motion_permitted and time.monotonic()-state["health_wall"] < .1)
    complete_chain = parents.get("odom") == {"map"} and parents.get("base_link") == {"odom"}
    rclpy.shutdown()

    if odom is None:
        print("PREFLIGHT FAIL: no slam/pose_corrected after "
              f"{ODOM_WAIT_S:.0f}s (GLIM not up / init stalled)", file=sys.stderr)
        return 4
    init_dist = math.hypot(odom[0], odom[1])
    if not math.isfinite(init_dist) or init_dist > INIT_SANE_M:
        print(f"PREFLIGHT FAIL: GLIM init not sane (odom {init_dist:.2f} m from origin "
              f"-- NaN/runaway divergence)", file=sys.stderr)
        return 5
    if navchain or gt_in_tree or not complete_chain:
        print(f"PREFLIGHT FAIL: GT leak -- navchain-multi-parent={navchain} gt_map_in_tree={gt_in_tree} "
              f"(set OMNILRS_GT_TF_ENABLED=false)", file=sys.stderr)
        return 3
    if not health_valid:
        print("PREFLIGHT FAIL: fresh healthy localization permission is required", file=sys.stderr)
        return 6

    print(f"PREFLIGHT OK: GLIM alive, init sane (odom {init_dist:.2f} m from origin); "
          f"odom/base_link single-parent; gt_map absent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
