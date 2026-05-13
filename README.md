# astrobot-lab

## Quick Start

### 1. Prep the repo

```bash
git clone <your-fork-or-repo-url>
cd astrobot-lab
git submodule update --init --recursive
```

If needed, adjust container UID/GID defaults in `.env`.

### 2. Build the Isaac Sim image

```bash
./docker/build_omnilrs_base.sh
```

This builds `isaac-sim-omnilrs:latest` from `OmniLRS/omnilrs.docker/Dockerfile`.

### 3. Use a shared ROS 2 domain (177)

All compose files and devcontainers are pinned to:
- `ROS_DOMAIN_ID=177`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `CYCLONEDDS_URI=file:///config/cyclonedds_config.xml`

If you run OmniLRS directly (outside compose), export the same values:

```bash
export ROS_DOMAIN_ID=177
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///config/cyclonedds_config.xml
```

### 4. Run Lunaryard 40m + Space ROS (same Docker network)

Headless is the default:

```bash
docker compose -f docker-compose.lunaryard.yml up -d
docker compose -f docker-compose.lunaryard.yml logs -f astrobot-sim
```

For a local X11-backed Isaac Sim window, keep the same compose file and switch with env vars:

```bash
xhost +local:root
DISPLAY=${DISPLAY:-:0} OMNILRS_HEADLESS=false docker compose -f docker-compose.lunaryard.yml up -d
docker compose -f docker-compose.lunaryard.yml logs -f astrobot-sim
```

The sim service now uses:
- `OMNILRS_HEADLESS=true|false`
- `OMNILRS_ENVIRONMENT=lunaryard_40m|lunalab|...`
- `OMNILRS_ROCKS_ENABLED=true|false`
- `OMNILRS_GT_TF_ENABLED=true|false`

If GUI mode is requested without `DISPLAY`, the sim exits early with a clear error instead of silently starting with no window.

For Lunaryard runs, rock generation is best controlled at startup rather than with the runtime
visibility topic. Examples:

```bash
OMNILRS_ROCKS_ENABLED=true docker compose -f docker-compose.lunaryard.yml up -d --force-recreate astrobot-sim
OMNILRS_ROCKS_ENABLED=false docker compose -f docker-compose.lunaryard.yml up -d --force-recreate astrobot-sim
```

This sets `environment.rocks_settings.enable` in the selected OmniLRS environment config before the scene is built.

For Phase 2 SLAM evaluation, keep simulator GT enabled and use it only as the
safe driving/evaluation reference. Do not disable simulator-owned `map -> odom`
for Phase 2 runs; SLAM-frame navigation starts in Phase 3 after a SLAM-derived
BEV map exists.

Open a shell in the dev container:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash
source /etc/ros_setup.sh
export ROS_DOMAIN_ID=177
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///config/cyclonedds_config.xml
ros2 topic list
```

### 5. Open Foxglove from your local machine

`astrobot-dev` includes `ros-jazzy-foxglove-bridge`, and `docker-compose.lunaryard.yml`
publishes it on `127.0.0.1:8765` on the remote host.

For remote viewing, the current lunaryard Husky streams are tuned to roughly:
- `/front_camera/mono/rgb` at ~10 Hz
- `/pointcloud` at ~10 Hz

Start the bridge:

```bash
docker compose -f docker-compose.lunaryard.yml exec -d astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765'
```

Forward the port from your laptop:

```bash
ssh -L 8765:localhost:8765 <user>@<remote-host>
```

Then connect from Foxglove Desktop or `https://app.foxglove.dev` to:

```text
ws://localhost:8765
```

### 6. Prepare the local ROS workspace

`astrobot-dev` auto-builds the mounted ROS workspace at container startup. If you
change ROS code while the container is already running, recreate the dev service
instead of doing a manual post-launch build:

```bash
docker compose -f docker-compose.lunaryard.yml up -d --build --force-recreate astrobot-dev
```

### 7. Launch Phase 1 navigation

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && ros2 launch astrobot_launch phase1_nav.launch.py'
```

This brings up:
- OmniLRS simulator-side GT publishing `/map` and `map -> odom`
- `basic_control` muxing `/cmd_vel_nav` and `/cmd_vel_teleop` into `/cmd_vel`
- Nav2 `planner_server`, `controller_server`, and `bt_navigator`
- Lunaryard Husky drive tuning with:
  - high wheel damping for velocity control
  - XT32M2X-style lidar timing
  - simulator-side GT `map -> odom` publishing at 30 Hz

Send a simple goal:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 22.0, y: 18.0, z: 0.0}, orientation: {w: 1.0}}}}"'
```

Current Phase 1 notes:
- the execution stack is standard Nav2:
  - `SmacPlanner2D`
  - `RotationShimController`
  - `RegulatedPurePursuitController`
- simulator-side GT obstacle generation is tunable in `OmniLRS/src/environments_wrappers/ros2/gt_map_ros2.py`
- current GT tuning knobs are:
  - `mesh_obstacle_padding_m`
  - `point_instancer_scale_factor`
  - `point_instancer_padding_m`
- the Lunaryard Husky controller tuning is set in `OmniLRS/cfg/environment/lunaryard_40m.yaml`
- for `lunaryard_40m`, use `OMNILRS_ROCKS_ENABLED=true|false` at container startup to control whether rocks exist in the scene

Current rocks-enabled Phase 1 baseline, validated on 2026-04-26 in `docker-compose.lunaryard.yml`:
- rock sampling in `OmniLRS/cfg/environment/lunaryard_40m.yaml`:
  - `lambda_parent: 0.02`
  - `lambda_daughter: 6`
  - `sigma: 1.2`
  - uniform rock scale `4.0..10.0`
- clean GT-map sampled run:
  - command: `tools/phase1_goal_sweep.py --timeout 120 --auto-free 10 --auto-occupied 4 --min-distance 1.0 --max-distance 5.0 --clearance 0.60 --seed 278`
  - free goals: `10/10` succeeded
  - final endpoint error band: `0.116..0.158 m`, mean `0.144 m`
  - occupied goals: `4/4` blocked cleanly with Nav2 planner error `208` (`NO_VALID_PATH`)
- broader stress run:
  - command: `tools/phase1_goal_sweep.py --timeout 120 --auto-free 12 --auto-occupied 4 --min-distance 1.0 --max-distance 7.0 --clearance 0.45 --seed 177`
  - free goals: `11/12` succeeded, final endpoint error band for successes `0.121..0.170 m`, mean `0.148 m`
  - one free-space stress goal aborted with controller error `103` (`FollowPath.INVALID_PATH`)
  - occupied goals: `4/4` blocked cleanly with planner error `208`

### 8. Phase 2 GLIM SLAM Bring-Up

Phase 2 is validated as SLAM evaluation, not as GT-map localization. Nav2 still plans and controls from simulator GT `/map` and GT `map -> odom`; GLIM runs in parallel and is evaluated against true simulator pose and SLAM-frame revisit consistency. Phase 3 will consume a SLAM-derived BEV map and send goals in the SLAM map frame.

`astrobot-dev` installs GLIM from Koide's Jazzy/CUDA packages:

- `ros-jazzy-glim-ros-cuda12.6`
- `cuda-cudart-12-6`

Start/recreate the Docker services. `astrobot-dev` auto-builds the mounted ROS
overlay at container startup, so normal usage does not require a manual
post-launch `colcon build`.

```bash
OMNILRS_ROCKS_ENABLED=true docker compose -f docker-compose.lunaryard.yml up -d --build --force-recreate
```

Run the combined Phase 1 Nav2 + Phase 2 GLIM safe-eval launch:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && ros2 launch astrobot_launch astrobot_master.launch.py nav2_params:=/workspace/astrobot-lab/astrobot-lab/src/astrobot_launch/config/nav2_phase2_slam.yaml'
```

Record a reproducible Phase 2 bag while that launch is running:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab && source astrobot-lab/install/setup.bash && tools/record_phase2_bag.sh rocks_random_long /workspace/astrobot-lab/rosbags'
```

Current rocks-collision Phase 2 validation baseline from 2026-05-10:

- `rosbags/phase2_rocks_random_long_20260510_015055` confirms LiDAR-visible rocks after replacing point-instanced rocks with explicit collider-backed rock prims. Corrected/base yaw-aligned ATE with one invalid GT `(0,0)` sample removed: RMSE `0.049 m`, p95 `0.085 m`, final `0.081 m`. SLAM-frame revisit p95 is `0.578 m`, with `100%` of revisits within `1.0 m`.
- `rosbags/phase2_rocks_double_loop_20260510_023005` uses the denser GLIM map config. Corrected/base yaw-aligned ATE: RMSE `0.108 m`, p95 `0.249 m`, final `0.048 m`. SLAM-frame revisit p95 is `0.632 m`, with `100%` of revisits within `1.0 m`.
- Rock-height diagnostics on the tuned double-loop show `/pointcloud/filtered` contains `5457` samples above `0.20 m`, and `/glim_rosnode/map` contains `1533` samples above `0.20 m`.
- Rock-focused BEV comparison is now tracked separately. Tuned double-loop feature recall improved to `0.218`, but feature F1 remains low at `0.141`; treat this as a Phase 3 map-quality target, not as evidence that rocks are absent.
- Phase 2 is accepted for GLIM SLAM bring-up, loop/revisit consistency, and BEV map handoff. The current `/slam/bev_costmap` is the Phase 3 starting point for traversability and Weighted A*; Phase 3 owns final navigation-map safety/connectivity tuning.
- The older 2026-04-29 Phase 2 pass is superseded for rock validation because the prior point-instanced rocks were represented in GT `/map` but were not reliably visible to LiDAR.

Important Phase 2 details:

- Simulator GT remains evaluation-only for SLAM metrics.
- GLIM consumes `/pointcloud/filtered`, generated from `/pointcloud` by a small crop-box filter that removes robot-body returns.
- GLIM keeps raw points in its map output for visualization. Current mapping config reduces GLIM preprocessing/downsampling to preserve rock geometry (`downsample_resolution=0.25`, submap/global voxel resolution `0.25`).
- `tools/record_phase2_bag.sh` records true simulator pose from `/gt/base_link_pose`.
- Direct live-recorded TUM evaluation is currently the authoritative path. `ros2 bag play` replay can shift `/clock` on some recordings, which breaks timestamp matching for replay-generated GLIM poses.
- GLIM `T_lidar_imu` is IMU-frame to LiDAR-frame; the current config uses simulator TF `vlp16 <- Imu_Sensor = [0.414, 0.017, 0.153, 0, 0, 1, 0]`.
- GLIM ROS pose is treated as IMU-frame for metric conversion; `slam_eval transform_tum` converts it to base-frame with `[0.264, 0.017, -0.262, yaw=pi]`.
- For live Foxglove viewing, use fixed frame `glim_map` and start with `/glim_rosnode/aligned_points_corrected`; `/glim_rosnode/map` is GLIM's global-map PointCloud2 output and may update less continuously than the aligned local/submap cloud.
- The packaged GLIM build does not include `libimu_validator.so`; use GLIM's built-in validation logs unless `glim_ext` is built from source.
- Do not disable simulator GT TF or publish a SLAM-owned `map -> odom` while sending GT-map goals for Phase 2; that mixes coordinate frames and is intentionally not a supported path.

### 9. Optional local GUI smoke test

```bash
xhost +local:root
DISPLAY=${DISPLAY:-:0} OMNILRS_HEADLESS=false docker compose -f docker-compose.lunaryard.yml up -d
```

The remote workflow is preferred when your local machine does not have a supported RTX GPU.

`docker-compose.lunalab.yml` is still kept for the smaller indoor environment, but
`docker-compose.lunaryard.yml` is now the default workflow. Use `OMNILRS_HEADLESS=false`
when you want a local Isaac Sim window.
