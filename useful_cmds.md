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
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && ros2 launch astrobot_launch astrobot_master.launch.py nav2_params:=/workspace/astrobot-lab/astrobot-lab/src/astrobot_launch/config/nav2_phase2_slam.yaml crop_min_x:=0.20 crop_max_x:=0.90 crop_min_y:=-0.50 crop_max_y:=0.50 crop_min_z:=-0.40 crop_max_z:=0.10'
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

Start deterministic resources for smoke testing:

```bash
OMNILRS_ROCKS_ENABLED=true OMNILRS_GT_TF_ENABLED=false OMNILRS_RESOURCE_MODE=deterministic \
  docker compose -f docker-compose.lunaryard.yml up -d --build --force-recreate
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

Run the current hard seeded sample-return check:

```bash
OMNILRS_ROCKS_ENABLED=true OMNILRS_ENVIRONMENT_SEED=411 OMNILRS_GT_TF_ENABLED=false \
OMNILRS_RESOURCE_ENABLED=true OMNILRS_RESOURCE_MODE=random OMNILRS_RESOURCE_SEED=20260611 \
OMNILRS_RESOURCE_RANDOM_GAUSSIANS=8 OMNILRS_RESOURCE_RANDOM_SIGMA_MIN=5.0 OMNILRS_RESOURCE_RANDOM_SIGMA_MAX=10.0 \
docker compose -f docker-compose.lunaryard.yml up -d --force-recreate astrobot-dev astrobot-sim

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && ros2 launch astrobot_launch phase4_exploration.launch.py mission_mode:=sample_return sample_success_threshold:=0.80 home_radius_m:=12.0 max_goal_distance_m:=8.0 resource_weight_scale:=7.0 resource_influence_radius_m:=5.0 resource_decay_radius_m:=2.0 max_resource_candidates:=30 resource_explore_radius_m:=4.0 mission_cell_size_m:=2.0 exploration_start_delay:=35.0'
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
/astrobot_0/exploration/status
/astrobot_0/exploration/mission_grid
/astrobot_0/diagnostics
```
