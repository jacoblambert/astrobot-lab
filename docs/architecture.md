# Architecture

All rover nodes run in the `astrobot_0` namespace (`robot_namespace`), inside the `astrobot-dev`
container; the simulator runs in `astrobot-sim`. Both use ROS domain 177 with Cyclone DDS
(`config/cyclonedds_config.xml`).

```
astrobot-sim (Isaac Sim + OmniLRS, lunar yard)
  /astrobot_0/lidar_selected/points   PointCloud2, 10 Hz LiDAR windows   -> GLIM
  /astrobot_0/lidar_selected/chunks   PointCloud2, 60 Hz native ticks   -> terrain
  /astrobot_0/imu_physics, odom       IMU, wheel odometry                -> GLIM, supervisor, TF
  /gt/base_link_pose, /gt/map, resource/<name>/sample   ground truth (sim / eval only)
  <- /astrobot_0/cmd_vel

localization_supervisor ──spawns──> glim_rosnode (slam_raw)
  - restarts GLIM from a stationary RTK + gravity reference, after a terrain map reset
  - gates /rtk/pose against GLIM's own position (per axis) -> localization/absolute_pose
  - relays slam_raw/* -> slam/* only from a validated epoch
  - localization/status: state, epoch, motion_permitted
synthetic_rtk (sim)       /gt/base_link_pose -> /rtk/pose (noise 0.02 m + 0.001 m per m)
map_to_odom_tf            slam/pose_corrected + odom -> TF map -> odom
terrain                   native chunks + slam_raw/revisions + /clock
                          -> terrain/local_costmap   6 m window, 10 Hz (Nav2 local costmap)
                          -> terrain/global_costmap  60 m mosaic, 1 Hz (Nav2 global costmap)
                          -> terrain/exploration_map observed free/occupied/unknown, 1 Hz
                          <- slam/reset_map (from the supervisor, once per GLIM epoch)
Nav2                      Smac 2D (global) + MPPI (local) -> cmd_vel_raw -> velocity_smoother
                          -> cmd_vel_nav -> command_mux -> cmd_vel
exploration_manager       terrain/exploration_map, slam/pose_corrected, localization/status
                          -> NavigateToPose / ComputePathToPose; exploration/{status,events,...}
ui_backend (optional)     everything above -> web console on :8080
```

## Frames

- `map`: the SLAM map, anchored by the RTK fixes. map = world − home, with home
  (−20, −20) in the simulator world, so the spawn at world (20, 18) is map (40, 38)
  (`astrobot_bringup/config/sim.yaml`).
- `odom` -> `base_link`: wheel odometry from the simulator. `map` -> `odom` comes from
  `map_to_odom_tf` using SLAM's base pose.
- GLIM's `pose_corrected` is the IMU frame: base_link <- IMU is `base_from_imu` in
  `astrobot_bringup/config/robot.yaml` (0.264, 0.017, 0.262 m, yaw π).

## Localization state machine

`STARTING` → (stationary RTK + gravity reference, map reset accepted) → `REINITIALIZING` →
(first valid GLIM pose) → `REBUILDING` → (terrain window ready near the rover, Nav2 costmaps
cleared, 2 s of poses) → `HEALTHY`. Motion is permitted only in `HEALTHY` and `DEGRADED`
(RTK older than 0.25 s). A bad pose, an RTK disagreement, a stale map or pose, or an RTK
outage over 2 s moves to `STOPPING`: navigation is cancelled, GLIM is stopped, and the cycle
restarts (at most 2 consecutive attempts). Unrecoverable conditions are `FAULTED`.

## Terrain

`astrobot_terrain` reconstructs a layered height model from native LiDAR returns placed with
GLIM's latest poses: up to four surface layers per 0.1 m cell, ground fits with support and
slope checks, obstacles 0.08 m above local ground, a 60 s evidence lifetime and a rolling 6 m
window. Cells without support stay unknown; a bounded gap policy gives small plain gaps a
finite cost (70) and marks unknown within 0.2 m of an obstacle lethal. Each projection is
merged into the map-frame mosaic, from which the Nav2 local window is cut, so obstacles out of
view behind the front LiDAR stay lethal. A non-trivial SLAM correction to an old pose cannot
move frozen evidence: the terrain faults and the supervisor rebuilds (new epoch).

## Missions

`astrobot_exploration` owns goal dispatch: a planner precheck of each goal (path cost and an
unknown fraction that tolerates more near the rover), failure memory, short reveal goals when
nothing is reachable, and the return home (to a clear cell beside home if a rock sits there).
A `MissionStrategy` plugin ranks candidates and decides when the mission is complete; see
[extending.md](extending.md).
