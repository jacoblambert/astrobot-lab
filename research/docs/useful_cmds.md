Quick start commands

### Docker compose

```bash
docker compose -f docker-compose.lunaryard.yml up -d
docker compose -f docker-compose.lunaryard.yml logs -f astrobot-sim
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash
source /etc/ros_setup.sh
ros2 topic list
```

### Local GUI

```bash
xhost +local:root
DISPLAY=${DISPLAY:-:0} OMNILRS_HEADLESS=false docker compose -f docker-compose.lunaryard.yml up -d
docker compose -f docker-compose.lunaryard.yml logs -f astrobot-sim
```
Only build when you need to:
``` ASTROBOT_AUTO_BUILD=false ```

### Rocks on/off

```bash
OMNILRS_ROCKS_ENABLED=true docker compose -f docker-compose.lunaryard.yml up -d --force-recreate astrobot-sim
OMNILRS_ROCKS_ENABLED=false docker compose -f docker-compose.lunaryard.yml up -d --force-recreate astrobot-sim
DISPLAY=${DISPLAY:-:0} OMNILRS_HEADLESS=false OMNILRS_ROCKS_ENABLED=true docker compose -f docker-compose.lunaryard.yml up -d --force-recreate astrobot-sim
```

### Foxglove

```bash
docker compose -f docker-compose.lunaryard.yml exec -d astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765'

ssh -L 8765:localhost:8765 <user>@<remote-host>
```

```text
ws://localhost:8765
```

### Phase 5 mission console

Frontend-only mockup:

```bash
cd ui/astrobot-console
npm install
npm run dev
```

Open:

```text
http://localhost:5173
```

Live backend, after launching the autonomy stack:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && ros2 launch astrobot_launch phase5_gui.launch.py'
```

Health/state checks:

```bash
curl http://127.0.0.1:8080/api/health
curl http://127.0.0.1:8080/api/state | jq '.robots[0], .mission'
```

### Phase 1 nav

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && ros2 launch astrobot_launch phase1_nav.launch.py'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && ros2 action send_goal /astrobot_0/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: gt_map}, pose: {position: {x: 22.0, y: 18.0, z: 0.0}, orientation: {w: 1.0}}}}"'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && ros2 topic pub --once /OmniLRS/Terrain/EnableRocks std_msgs/msg/Bool "{data: true}"'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && ros2 topic pub --once /OmniLRS/Robots/Teleport geometry_msgs/msg/PoseStamped "{header: {frame_id: astrobot_0}, pose: {position: {x: 20.0, y: 18.0, z: 0.5}, orientation: {w: 1.0}}}"'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && python3 /workspace/astrobot-lab/tools/phase1_goal_sweep.py --timeout 90 --relative-goal=2.0,0.0 --relative-goal=0.0,2.0 --relative-goal=1.5,1.5'
```

### Phase 1 baseline validation

Rocks-enabled final Phase 1 baseline:

```bash
OMNILRS_ROCKS_ENABLED=true docker compose -f docker-compose.lunaryard.yml up -d --build --force-recreate astrobot-sim astrobot-dev

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && ros2 launch astrobot_launch phase1_nav.launch.py'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && PYTHONUNBUFFERED=1 python3 /workspace/astrobot-lab/tools/phase1_goal_sweep.py --timeout 120 --auto-free 10 --auto-occupied 4 --min-distance 1.0 --max-distance 5.0 --clearance 0.60 --seed 278'
```

Expected result for the 2026-04-26 baseline:

```text
SUMMARY_FREE attempted=10 succeeded=10 min_err=0.116 max_err=0.158 mean_err=0.144
SUMMARY_OCCUPIED attempted=4 blocked=4 error_codes=[208]
```

Broader rocks stress run:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && PYTHONUNBUFFERED=1 python3 /workspace/astrobot-lab/tools/phase1_goal_sweep.py --timeout 120 --auto-free 12 --auto-occupied 4 --min-distance 1.0 --max-distance 7.0 --clearance 0.45 --seed 177'
```

Expected stress result from 2026-04-26:

```text
SUMMARY_FREE attempted=12 succeeded=11 min_err=0.121 max_err=0.170 mean_err=0.148
SUMMARY_OCCUPIED attempted=4 blocked=4 error_codes=[208]
```

### Bags

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash
source /etc/ros_setup.sh
ros2 bag record -o /workspace/astrobot-lab/rosbags/lunaryard_$(date +%Y%m%d_%H%M%S) \
  /astrobot_0/camera_0/image_raw /astrobot_0/camera_0/camera_info \
  /astrobot_0/imu /astrobot_0/odom /astrobot_0/lidar_0/pointcloud/raw /tf
```

### Phase 2 GLIM

Start/recreate the Docker services. `astrobot-dev` auto-builds the ROS overlay
at container startup, so you should not need to run `colcon build` manually after
launching unless you edit code inside an already-running container.

```bash
OMNILRS_ROCKS_ENABLED=true docker compose -f docker-compose.lunaryard.yml up -d --build --force-recreate
```

GLIM source build status:

```bash
docker compose -f docker-compose.lunaryard.yml run --rm --no-deps astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && ros2 pkg prefix glim && ros2 pkg prefix glim_ros && ros2 pkg executables glim_ros'

docker compose -f docker-compose.lunaryard.yml run --rm --no-deps --entrypoint bash astrobot-dev -lc \
  'source /etc/ros_setup.sh && ldd /opt/astrobot/glim_source/lib/glim_ros/glim_rosnode | grep -E "libglim|libglim_ros|not found"'
```

Expected prefixes are `/opt/astrobot/glim_source`. The Docker image builds `glim`
and `glim_ros` from the `Open-Space-Systems` submodules; runtime workspace
auto-build skips those packages by default.

Keep simulator GT state enabled for Nav2 control. Phase 2 runs GLIM in parallel
for evaluation and visualization, not as the Nav2 localization source.

```bash
docker compose -f docker-compose.lunaryard.yml logs -f astrobot-dev
```

Run Phase 1 Nav2 plus Phase 2 GLIM in safe-eval mode:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && ros2 launch astrobot_launch astrobot_master.launch.py nav2_params:=/workspace/astrobot-lab/astrobot-lab/src/astrobot_launch/config/nav2_phase2_slam.yaml'
```

The GLIM input cloud is `/astrobot_0/lidar_0/pointcloud/filtered`, produced from `/astrobot_0/lidar_0/pointcloud/raw` by a
small robot-body crop box. Override the box if robot returns are still visible:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && ros2 launch astrobot_launch astrobot_master.launch.py nav2_params:=/workspace/astrobot-lab/astrobot-lab/src/astrobot_launch/config/nav2_phase2_slam.yaml crop_min_x:=-0.95 crop_max_x:=0.05 crop_min_y:=-0.45 crop_max_y:=0.45 crop_min_z:=-0.30 crop_max_z:=0.25'
```

Visualize these topics in Foxglove:

```text
/tf
/gt/map
/astrobot_0/odom
/gt/base_link_pose
/astrobot_0/lidar_0/pointcloud/raw
/astrobot_0/lidar_0/pointcloud/filtered
/astrobot_0/imu
/astrobot_0/cmd_vel
/astrobot_0/cmd_vel_nav
/astrobot_0/control/status
/astrobot_0/slam/pose
/astrobot_0/slam/pose_corrected
/astrobot_0/slam/odom
/astrobot_0/slam/odom_corrected
/astrobot_0/slam/aligned_points_corrected
/astrobot_0/slam/points_corrected
/astrobot_0/slam/glim_map
/astrobot_0/slam/prob_voxel_map
/astrobot_0/slam/bev_costmap
/astrobot_0/navigate_to_pose/_action/status
```

For Foxglove live SLAM visualization, use fixed frame `map` and display
`/astrobot_0/slam/aligned_points_corrected` first. `/astrobot_0/slam/glim_map` is GLIM's
global-map PointCloud2 output, but it may only update after global-map/keyframe
events. GLIM is configured with `keep_raw_points=true` so the global map is
more useful visually than the heavily preprocessed odometry cloud.

For navigation-map work, prefer the modular map chain over `/astrobot_0/slam/glim_map`:
`/astrobot_0/slam/aligned_points_corrected` + `/astrobot_0/slam/pose_corrected` ->
`/astrobot_0/slam/prob_voxel_map` -> `/astrobot_0/slam/bev_costmap`. `/astrobot_0/slam/prob_voxel_map` is a
probabilistic 3D occupancy map with volumetric ray clearing; high LiDAR beams
clear only the voxels they pass through, not the full BEV column below them.
`/astrobot_0/slam/bev_costmap` is a regular `nav_msgs/OccupancyGrid` derived from the 3D
map. GLIM's `/astrobot_0/slam/glim_map` is useful when it publishes, but it is not the
navigation map source.

Run the probabilistic mapper unit tests and a short offline rosbag smoke test:

```bash
docker compose -f docker-compose.lunaryard.yml run --rm --no-deps -e ASTROBOT_AUTO_BUILD=false astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && colcon build --symlink-install --packages-select slam_mapping astrobot_launch --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && ./build/slam_mapping/test_probabilistic_voxel_map'

docker compose -f docker-compose.lunaryard.yml run --rm --no-deps -e ASTROBOT_AUTO_BUILD=false astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && timeout 8 ros2 launch astrobot_launch phase2_slam.launch.py start_glim:=false start_bev_map:=false'
```

Record the Phase 2 random-long validation route:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab && source astrobot-lab/install/setup.bash && tools/record_phase2_bag.sh rocks_random_long /workspace/astrobot-lab/rosbags'
```

Record the shorter rocks map-eval route:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab && source astrobot-lab/install/setup.bash && tools/record_phase2_bag.sh rocks_map_eval /workspace/astrobot-lab/rosbags'
```

Replay an existing Phase 2 bag through GLIM and run all current metrics.
Prefer direct live-recorded TUM evaluation for current validation until the
`/clock` replay-shift issue is fixed:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab && source astrobot-lab/install/setup.bash && tools/run_glim_replay_eval.sh /workspace/astrobot-lab/rosbags/<bag_dir> /workspace/astrobot-lab/astrobot-lab/src/glim_slam_launch/config/glim_astrobot phase2_replay 1.0'
```

Convert GLIM IMU-frame pose to base-frame:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && BAG=/workspace/astrobot-lab/rosbags/<bag_dir> && ros2 run slam_eval transform_tum --input $BAG/glim_pose_corrected.tum --output $BAG/glim_pose_corrected_base.tum --xyz 0.264 0.017 -0.262 --quat 0 0 1 0'
```

Run the core Phase 2 metrics:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && BAG=/workspace/astrobot-lab/rosbags/<bag_dir> && ros2 run slam_eval compare_tum_trajectories --reference $BAG/gt_map_base_link.tum --estimate $BAG/glim_pose_corrected_base.tum --max-dt 0.1 --align first'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && BAG=/workspace/astrobot-lab/rosbags/<bag_dir> && ros2 run slam_eval compare_tum_trajectories --reference $BAG/gt_map_base_link.tum --estimate $BAG/glim_pose_corrected_base.tum --max-dt 0.1 --align yaw'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && BAG=/workspace/astrobot-lab/rosbags/<bag_dir> && ros2 run slam_eval evaluate_loop_closure --reference $BAG/gt_map_base_link.tum --estimate $BAG/glim_pose_corrected_base.tum --max-dt 0.1 --revisit-radius 0.75 --min-separation-sec 30 --sample-step 10'
```

Run supporting diagnostics:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && BAG=/workspace/astrobot-lab/rosbags/<bag_dir> && ros2 run slam_eval compare_bev_maps --bag $BAG/bag --reference-tum $BAG/gt_map_base_link.tum --estimate-tum $BAG/glim_pose_corrected_base.tum --align yaw --height-threshold 0.10 --min-points-per-cell 2 --dilation-radius 2 --feature-dilation-radius 2 --ignore-border-cells 2'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && BAG=/workspace/astrobot-lab/rosbags/<bag_dir> && ros2 run slam_eval compare_bev_accumulated --bag $BAG/bag --cloud-topic /astrobot_0/slam/aligned_points_corrected --reference-tum $BAG/gt_map_base_link.tum --estimate-tum $BAG/glim_pose_corrected_base.tum --align yaw --height-threshold 0.02 --min-points-per-cell 2 --dilation-radius 2 --feature-dilation-radius 2 --ignore-border-cells 2 --sample-stride 4'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && BAG=/workspace/astrobot-lab/rosbags/<generated_bev_bag> && ros2 run slam_eval compare_occupancy_maps --bag $BAG --pred-topic /astrobot_0/slam/bev_costmap --pred-occupied-threshold 50 --dilation-radius 2 --feature-dilation-radius 2'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && BAG=/workspace/astrobot-lab/rosbags/<generated_bev_bag> && ros2 run slam_eval evaluate_costmap_reachability --bag $BAG --pred-topic /astrobot_0/slam/bev_costmap --pred-blocked-threshold 80 --goal-count 200'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && BAG=/workspace/astrobot-lab/rosbags/<bag_dir> && ros2 run slam_eval analyze_pointcloud_height --bag $BAG/bag --topics /astrobot_0/lidar_0/pointcloud/raw /astrobot_0/lidar_0/pointcloud/filtered /astrobot_0/slam/aligned_points_corrected /astrobot_0/slam/glim_map --cell-size 0.5 --max-radius 15.0 --min-points-per-cell 4'
```

When replaying a bag to regenerate `/astrobot_0/slam/prob_voxel_map` and `/astrobot_0/slam/bev_costmap`,
use a moderate replay rate such as `--rate 5.0`. At `--rate 20.0`, the voxel
builder can drop enough point clouds to produce an incomplete costmap and weak
GT-map overlap scores.

Current rocks-collision Phase 2 validation references from 2026-05-10:

```text
phase2_rocks_random_long_20260510_015055:
  corrected/base yaw-aligned ATE rmse=0.0494 p95=0.0848 final=0.0806 after dropping one invalid GT (0,0) sample
  loop consistency p95=0.5779 close_rate_1m=1.0000

phase2_rocks_double_loop_20260510_023005:
  corrected/base yaw-aligned ATE rmse=0.1081 p95=0.2492 final=0.0476
  loop consistency p95=0.6324 close_rate_1m=1.0000
  filtered LiDAR count_above_0.20m=5457
  GLIM global map count_above_0.20m=1533
  rock-focused BEV feature_recall=0.218 feature_f1=0.141

phase2_rocks_map_eval_20260510_094803:
  GLIM global map messages=0, so it is not a reliable nav-map source yet.
  accumulated aligned corrected points, height_threshold=0.02,
  min_points_per_cell=2, sample_stride=4:
  BEV f1=0.300, iou=0.168, rock feature_recall=0.877,
  feature_precision=0.172, feature_f1=0.288.
  GT-registered filtered LiDAR upper-bound on the same route:
  BEV f1=0.278, iou=0.165, feature_recall=0.703, feature_f1=0.284.
  Runtime probabilistic voxel map -> BEV costmap initial slow replay
  baseline (`0.05 m` voxels, obstacle warn/lethal `0.02/0.20 m`,
  hard inflation): BEV f1=0.267, iou=0.154, feature_recall=0.603,
  feature_precision=0.172, feature_f1=0.267.

2026-05-13 BEV navigation-readiness candidate:
  default BEV costmap tuning is obstacle seed `0.10 m`, hard seed
  cost `100`, soft inflation radius `0.25 m`, soft inflation cost `60`,
  free-fill radius `0.20 m`.
  On the rocks map-eval bag with `5x` replay:
  soft cost coverage at threshold 50: feature_recall=0.691,
  feature_f1=0.297.
  hard-core reachability at threshold 80: reachable_gt_free_goal_rate=0.705.
  Phase 2 is accepted with this as the Phase 3 BEV traversability / Weighted A*
  starting point. Phase 3 owns the remaining safety/connectivity tradeoff.
```

The older 2026-04-29 Phase 2 pass is superseded for rock validation: GT `/gt/map`
contained rocks, but point-instanced rocks were not reliably visible to LiDAR.

Do not use `OMNILRS_GT_TF_ENABLED=false` or a SLAM-owned `map -> odom` for Phase 2 validation. That mixes GT-map goals with an independently drifting SLAM map frame. Phase 3 will plan in a SLAM-derived BEV map frame.

### Phase 3 SLAM-backed Nav2

Phase 3 disables simulator-owned GT `gt_map -> odom` and lets the SLAM stack own
the navigation frame through `map -> odom`.

```bash
OMNILRS_ROCKS_ENABLED=true OMNILRS_GT_TF_ENABLED=false \
  docker compose -f docker-compose.lunaryard.yml up -d --build --force-recreate
```

Launch GLIM, probabilistic voxel mapping, BEV costmap generation, the SLAM TF
bridge, Nav2, and `basic_control`:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && ros2 launch astrobot_launch phase3_nav.launch.py'
```

The launch defaults to the patched LiDAR/IMU GLIM profile `glim_astrobot` with
the IMU-frame base offset `slam_base_offset_x:=0.264`,
`slam_base_offset_y:=0.017`, and `slam_base_yaw_offset:=pi`. The LiDAR-only
profile `glim_astrobot_lidar_only` remains available as a fallback, but it must
use the LiDAR-frame base offset `0.150, 0.0, 0.0`.

The launch also defaults `slam_start_delay:=15.0` to keep GLIM from initializing
while the robot is still settling. If GLIM starts with a large initial pose
offset, Nav2 may reject all goals as outside the BEV map.

Run the short Phase 3 probe smoke test:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab && source astrobot-lab/install/setup.bash && python3 tools/phase3_probe_sweep.py --goal-timeout 90 --return-home-every 2 --relative-goal 0.6,0.0 --relative-goal 0.0,0.6 --output /workspace/astrobot-lab/rosbags/phase3_probe_summary.json'
```

Send a manual SLAM-frame goal:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && ros2 action send_goal /astrobot_0/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.6, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"'
```

Visualize with Foxglove fixed frame `map`:

```text
/tf
/gt/map
/gt/base_link_pose
/astrobot_0/odom
/astrobot_0/lidar_0/pointcloud/filtered
/astrobot_0/slam/pose_corrected
/astrobot_0/slam/aligned_points_corrected
/astrobot_0/slam/glim_map
/astrobot_0/slam/prob_voxel_map
/astrobot_0/slam/bev_costmap
/astrobot_0/cmd_vel_nav
/astrobot_0/cmd_vel
/astrobot_0/navigate_to_pose/_action/status
```

Notes:

- `/gt/map` is the simulator rock-obstacle reference map only. It no longer
  includes the Lunaryard terrain generation mask/slope raster.
- The active GLIM LiDAR/IMU profile uses `acc_scale=1.0` and
  `sensors.imu_gravity_magnitude=1.62`. The old `acc_scale=6.052` over-scaled
  dynamic acceleration and caused long-run drift. `validate_imu=false` remains
  the conservative live setting.

Current Phase 3 gate from 2026-05-17:

```text
nav_20_goal_gate_3m_lio_online_201648.json:
  profile=glim_astrobot LiDAR/IMU
  navigation_actions=20/20 succeeded
  return_home=5/5 succeeded
  endpoint_error_m min=0.085 median=0.214 p95=0.283 max=0.294
  residual risk: controller-loop timing warnings still appear under live GLIM + mapping load
```

### Phase 4 resource-aware exploration

Resource map coordinate system — `OMNILRS_RESOURCE_CENTERS` positions are in OmniLRS world frame
(same as `gt_map` ROS frame). For lunaryard_40m the robot starts at world ≈ (19.875, 18.133).
To place a peak at SLAM (sx, sy): world_x = 19.875 - sx, world_y = 18.133 - sy.

Start deterministic resources for smoke testing:

```bash
OMNILRS_ROCKS_ENABLED=true OMNILRS_GT_TF_ENABLED=false OMNILRS_RESOURCE_MODE=deterministic \
  docker compose -f docker-compose.lunaryard.yml up -d --build --force-recreate
```

Start with a single near-gradient peak for sample_return validation (peak at SLAM (8,6) = 10m):

```bash
OMNILRS_ROCKS_ENABLED=true OMNILRS_GT_TF_ENABLED=true \
OMNILRS_RESOURCE_MODE=deterministic OMNILRS_RESOURCE_CENTERS="11.875,12.133,4.0,1.0" \
  docker compose -f docker-compose.lunaryard.yml up -d --force-recreate
```

Start randomized multi-Gaussian resources for gate-style runs:

```bash
OMNILRS_ROCKS_ENABLED=true OMNILRS_GT_TF_ENABLED=false OMNILRS_RESOURCE_MODE=random OMNILRS_RESOURCE_SEED=1001 \
  docker compose -f docker-compose.lunaryard.yml up -d --build --force-recreate
```

Launch Phase 4 exploration:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && ros2 launch astrobot_launch phase4_exploration.launch.py mission_mode:=sample_return sample_success_threshold:=0.75'
```

Run the small sample_return validation (single clear gradient, peak at SLAM (8,6)=10m):

```bash
# Sim must be started with OMNILRS_RESOURCE_CENTERS="11.875,12.133,4.0,1.0"
# If launching after a previous run killed GLIM, use exploration_start_delay:=120.0
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && \
   ros2 launch astrobot_launch phase4_exploration.launch.py mission_mode:=sample_return \
   home_radius_m:=19.0 max_goal_distance_m:=5.0 max_duration_sec:=600.0 exploration_start_delay:=120.0'
```

Run the current hard seeded sample-return check (home_radius=19m = full yard, no artificial limit):

```bash
OMNILRS_ROCKS_ENABLED=true OMNILRS_GT_TF_ENABLED=false \
OMNILRS_RESOURCE_ENABLED=true OMNILRS_RESOURCE_MODE=random OMNILRS_RESOURCE_SEED=20260611 \
OMNILRS_RESOURCE_RANDOM_GAUSSIANS=8 OMNILRS_RESOURCE_RANDOM_SIGMA_MIN=5.0 OMNILRS_RESOURCE_RANDOM_SIGMA_MAX=10.0 \
docker compose -f docker-compose.lunaryard.yml up -d --force-recreate

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && \
   ros2 launch astrobot_launch phase4_exploration.launch.py mission_mode:=sample_return \
   home_radius_m:=19.0 max_goal_distance_m:=5.0 max_duration_sec:=900.0 exploration_start_delay:=35.0'
```

Run the latest hard seed-411 exploration diagnostic with the current GLIM IMU
config:

```bash
OMNILRS_ROCKS_ENABLED=true OMNILRS_ENVIRONMENT_SEED=411 OMNILRS_GT_TF_ENABLED=false \
OMNILRS_RESOURCE_ENABLED=true OMNILRS_RESOURCE_MODE=random OMNILRS_RESOURCE_SEED=1001 \
OMNILRS_RESOURCE_RANDOM_GAUSSIANS=4 OMNILRS_RESOURCE_RANDOM_SIGMA_MIN=6.0 OMNILRS_RESOURCE_RANDOM_SIGMA_MAX=9.0 \
OMNILRS_RESOURCE_RANDOM_AMPLITUDE_MIN=0.75 OMNILRS_RESOURCE_RANDOM_AMPLITUDE_MAX=1.0 \
docker compose -f docker-compose.lunaryard.yml up -d --force-recreate astrobot-dev astrobot-sim

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && ros2 launch astrobot_launch phase4_exploration.launch.py mission_mode:=explore_radius coverage_goal_fraction:=0.80 home_radius_m:=8.0 max_goal_distance_m:=8.0 mission_cell_size_m:=2.0 mission_candidate_limit:=120 exploration_start_delay:=35.0'
```

HISTORICAL diagnostic reference from 2026-05-30 (pre CPU-GLIM switch — the active odometry
config is now `config_odometry_cpu.json`; see "Phase 4 gate harness & suite" below for current state):

```text
GLIM config (as of 2026-05-30):
  config_ros.json acc_scale=1.0
  config_odometry_gpu.json validate_imu=false

hard seed 411, 8 m / 80% explore diagnostic:
  no "IMU prediction is not good" GLIM warnings
  max observed SLAM-vs-GT error ~= 0.62 m
  no slam_pose_error_drifting or slam_gt_increment_mismatch fault
  reached 27/51 = 51.9% coverage before return-home/max-goal behavior
  remaining issue appears to be exploration/reachability tuning, not SLAM divergence
```

Expected reference from `rosbags/phase4_abundant_sample_return_hard_seed411_20260525_212515`:

```text
best_sample=0.826
return_home=succeeded
nav2_failures=0
safety_cancellations=0
slam_jumps=0
max_slam_vs_gt_error=0.631 m
```

Run the current 8 m / 80% explore-radius check:

```bash
OMNILRS_ROCKS_ENABLED=true OMNILRS_ENVIRONMENT_SEED=411 OMNILRS_GT_TF_ENABLED=false \
OMNILRS_RESOURCE_ENABLED=true OMNILRS_RESOURCE_MODE=random OMNILRS_RESOURCE_SEED=20260611 \
docker compose -f docker-compose.lunaryard.yml up -d --force-recreate astrobot-dev astrobot-sim

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && ros2 launch astrobot_launch phase4_exploration.launch.py mission_mode:=explore_radius coverage_goal_fraction:=0.80 home_radius_m:=8.0 max_goal_distance_m:=8.0 mission_cell_size_m:=2.0 mission_candidate_limit:=120 exploration_start_delay:=35.0'
```

Expected reference from `rosbags/phase4_abundant_gate80_radius8_candidate_fix_20260524_232015`:

```text
coverage=42/52=80.77%
nav2_failures=0
slam_jumps=0
note=coverage gate only; return-home was stopped after threshold to avoid a long breadcrumb tail
```

Monitor a run:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab && source astrobot-lab/install/setup.bash && python3 tools/phase4_exploration_eval.py --duration 900 --output /workspace/astrobot-lab/rosbags/phase4_summary.json'
```

Foxglove topics:

```text
/gt/resource_maps/water
/astrobot_0/resource/water/sample
/astrobot_0/resource/water/sample_pose
/astrobot_0/exploration/frontiers
/astrobot_0/exploration/selected_goal
/astrobot_0/exploration/events
/astrobot_0/exploration/status
/astrobot_0/exploration/mission_grid
/astrobot_0/diagnostics
```

### Phase 4 gate harness & suite (the official PASS/FAIL — 2026-06-07)

Gate criteria (GT-scored; the mission's own success flag is NEVER trusted — SLAM drift produces
false successes): `sample_return` = best_sample >= 0.75 AND final GT pose <= 1 m from home (20,18);
`explore_radius` = coverage >= 0.70 AND mission finished. One retry per gate.

Run ONE gate (host-side; does its own fresh bring-up/teardown, auto-starts foxglove :8765 +
console backend :8080 so every run is watchable live):

```bash
tools/phase4_mission_gate_run.sh 42 sample_return --retries 1
tools/phase4_mission_gate_run.sh 7 explore_radius --retries 1
# optional: override the default artifact root (`rosbags/phase4_gate_artifacts/`):
tools/phase4_mission_gate_run.sh 7 sample_return --retries 0 --record my_diag_dir
```

Each attempt records navigation diagnostics inside the eval node, without extra
observer processes. Per-attempt artifacts live under
`rosbags/phase4_gate_artifacts/<mode>_seed<seed>_a<N>/` by default:

```text
eval.json
goal_events.jsonl
raw_exploration_events.jsonl
diagnostics.jsonl
gt_base.tum
slam_corrected.tum
snapshots/*.npz
mission_launch.log
eval_stdout.log
```

Summarize gate/navigation failure classes:

```bash
python3 tools/phase4_nav_diagnostics_summary.py rosbags/phase4_gate_suite.json
python3 tools/phase4_nav_diagnostics_summary.py rosbags/phase4_gate_explore_radius_seed42.json
```

Full suite (matrix + `rosbags/phase4_gate_suite.json`; ~2-3 h unattended):

```bash
tools/phase4_gate_suite.sh --seeds "42 411 7" --modes "sample_return explore_radius" --retries 1
```

Current matrix (tuned stack: CPU GLIM + real rocks + mapper retune + recovery-wiring fix):
sample_return 42 PASS / 411 FAIL (residual SLAM accuracy) / 7 PASS; explore_radius 0/3
(dense-cluster stall cascade — see PHASE_DEFINITIONS.md Phase 4 "Current status" for the full
fix menu). The mission console frontend should be kept running on the host:
`cd ui/astrobot-console && npm run dev -- --host` (port 5173; proxies the backend on :8080).

### SLAM divergence study (statistical characterization, not point-fixes)

```bash
# N repeats x seeds, one mission attempt each, fingerprint per run + aggregate clustering:
tools/slam_divergence_study.sh --seeds "42 411 7" --mode sample_return --reps 3 --tag mytag
# per-run fingerprint (diverged?, onset t/xy, type, turning?, imu warnings, max err):
python3 tools/slam_divergence_fingerprint.py rosbags/divergence_study/mytag/seed7_r1/<attempt_dir>
```

IMPORTANT: recording happens INSIDE the eval node (`--record-dir`) — do NOT add extra recorder
processes to measurement runs; 3 light recorders once amplified a 2.5 m drift into 8-23 m
divergences (observer effect, documented 2026-06-04). The Phase 4 gate harness follows
that rule: goal events, diagnostics, TUM traces, and stall/spin snapshots are collected by
`tools/phase4_exploration_eval.py`.

### BEV mapper evaluation & tuning (alignment-correct)

```bash
# live per-rock feature recall + cell precision vs /gt/map (run INSIDE dev, stack up):
python3 tools/bev_gt_eval.py --duration 60
# canonical map<->world alignment from a recorded run dir (least-squares, IMU->base aware):
python3 tools/map_gt_alignment.py rosbags/<run_dir_with_tums>
# offline param sweep: replay a recorded bag through the voxel mapper across hit x decay grids,
# score recall/precision/flicker per combo (run INSIDE dev; bag needs aligned_points_corrected
# + pose_corrected + gt topics + bev_costmap):
bash tools/bev_sweep.sh /workspace/astrobot-lab/rosbags/bev_tune_seed7 /workspace/astrobot-lab/rosbags/my_sweep
# score one bag (final-grid recall/precision + flicker toggles/min + transient blips):
python3 tools/bev_bag_score.py <bag_dir> --label mylabel --csv out.csv
```

Current mapper operating point (phase2_slam.launch.py, swept 2026-06-06): hit_log_odds 0.5,
decay_factor 0.998, max_voxels 6M -> rock recall 59% / precision 96% / flicker 635 toggles/min
(baseline 0.85/0.97/1.2M was 21% / 95% / 1929). Per-rock recall is the honest metric — GT marks
full rock footprints, LiDAR marks visible rims; naive cell-wise recall under-reads catastrophically.

### Gotchas that cost real debugging time (read before touching the sim/stack)

- `use_point_instancer` MUST stay False — point-instanced rocks are INVISIBLE to the RTX LiDAR.
- `slam/odom_corrected` is nav_msgs/Odometry; `slam/pose_corrected` is PoseStamped. A wrong-typed
  rclpy subscription receives NOTHING silently (ros2 echo/hz still work — they're type-agnostic).
- GLIM's pose_corrected is the IMU frame: simulator TF `base_link -> Imu_Sensor`
  is `(0.264, 0.017, 0.262, yaw pi)`. Convert IMU to base before comparing to
  GT (tools/map_gt_alignment.py does this).
- `kill -INT` on background jobs of a NON-interactive shell is a silent no-op (SIGINT inherited as
  ignored) — use SIGTERM to stop ros2 bag record etc., or bags never finalize (no metadata.yaml).
- behavior_server needs `remappings=[("cmd_vel", "cmd_vel_raw")]` (phase1_nav.launch.py) or
  recoveries fight the command_mux on the final cmd_vel topic and silently do nothing.
- explore_radius `coverage_goal_fraction` must EQUAL the gate threshold (0.70) — a stricter
  mission self-goal times out `not_finished` at gate-level coverage.

### RTK / absolute-position fusion (on by default in phase2/3/4)

Full design + "full authority" semantics + caveats: `astrobot-lab/src/synthetic_rtk/README.md`.

```bash
# Offline A/B replay through GLIM (no sim) on the divergence fixture, toggle RTK via the config:
tools/rtk_replay_ab.sh /workspace/astrobot-lab/rosbags/seed411_sparse_explore \
  /workspace/astrobot-lab/astrobot-lab/src/glim_slam_launch/config/glim_astrobot mytest
# Disable RTK for a run:  start_synthetic_rtk:=false   OR   config_ros abs_constraint.enable=false
```

- RTK = synthetic_rtk (sim source from GT; swap for a real beacon driver on hardware) -> `/rtk/pose`
  -> `abs_constraint_module` (glim_ros2, additive) -> GPSFactor (position) + yaw PriorFactor on X(current).
  Needs `init_T_world_imu` pinning (config_odometry) so the home-relative fix is a direct world prior.
- "Full authority" = `gate_dist=0` + `huber_k=0` = RTK is never gated/robust-rejected. It is NOT
  dominance: per-frame the scan prior (Precision 1e3) > RTK (~400); RTK bounds drift by accumulating
  absolute anchors on every pose + breaking the divergence feedback loop. To make RTK dominate
  per-frame you'd lower the scan prior `Isotropic::Precision(6, 1e3)`, not gate/huber.
- Validated: arrests catastrophic feature-poor divergence (9480m->~1m) + bounds long-drive drift to
  ~0.3m (vs 2.5m when RTK starves out of the graph). The "RTK not getting into the graph" bug was a
  too-tight `assoc_tolerance` (0.05->0.2s) + low publish rate (synthetic_rtk republishes at 30Hz).
- Caveat: gate/huber=0 has NO bad-fix protection -> add a kinematic RTK-to-RTK gate for real hardware.
