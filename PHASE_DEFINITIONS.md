# Astrobot-Lab Implementation Plan

## Purpose

This document turns the high-level mission into an execution plan for the first practical milestone:

1. Run a remote, headless OmniLRS-based exploration stack.
2. Support more than one robot.
3. Build a LiDAR/IMU-first autonomy pipeline.
4. Expose the system remotely through a usable operations UI.
5. Add resource-finding logic and later advisory VLM reasoning.

This is not a white paper. It is the build order.

## Executive Summary

The right order is:

1. Remote observability and operator control.
2. Single-robot autonomy vertical slice.
3. Multi-robot namespacing and coordination.
4. Resource simulation and mission weighting.
5. Custom web UI.
6. VLM advisory loop.
7. Anomaly simulation and formal evaluation.
8. White paper.

The main reason for this order:

- Without remote observability, development speed is too low.
- Without a single-robot vertical slice, multi-robot work is mostly fake complexity.
- Without a stable map/planner stack, VLM feedback has nothing useful to influence.
- The VLM should start as an advisory system, not a control dependency.

## Current Starting Point

### Confirmed working today

- Headless OmniLRS Lunaryard 40m runs in `docker-compose.lunaryard.yml`.
- Space ROS and OmniLRS communicate over ROS 2 using CycloneDDS on domain `177`.
- Phase 0 remote observability baseline is now working through Foxglove Bridge over SSH tunnel.
- Lunaryard Husky now uses an XT32M2X-style lidar approximation:
  - 10 Hz rotation
  - 32-beam style vertical span
  - 10 Hz remote-friendly point cloud publishing
- Current live robot interface includes:
  - `/cmd_vel`
  - `/odom`
  - `/tf`
  - `/pointcloud`
  - `/imu`
  - `/front_camera/mono/rgb`
  - `/front_camera/mono/rgb_info`
- Scene/robot management topics already exist:
  - `/OmniLRS/Robots/Spawn`
  - `/OmniLRS/Robots/Teleport`
  - `/OmniLRS/Robots/Reset`
  - `/OmniLRS/Robots/ResetAll`
- Remote-friendly sensor tuning is now applied in Lunaryard 40m:
  - `/front_camera/mono/rgb` at about 10 Hz
  - `/pointcloud` at about 10 Hz
- Phase 1 point-to-point navigation is validated in Lunaryard from a known spawn patch with:
  - rocks disabled
  - rocks enabled
  - a conservative differential-drive follower
  - simulator-owned GT `/map` and `map -> odom`

### Confirmed gaps today

- No action-based robot autonomy API is present yet.
- No SLAM stack is integrated.
- No planning or exploration stack is integrated.
- No resource sensor simulation exists yet.
- No remote UI exists yet.
- No VLM pipeline exists yet.
- No evaluation harness exists yet.

## Design Principles

1. Keep `astrobot-sim` responsible for simulation truth and sensor generation.
2. Keep `astrobot-dev` responsible for autonomy, planning, UI backends, and evaluation.
3. Start centralized before decentralized for multi-robot coordination.
4. Use 3D perception, but project to a weighted 2D BEV for planning.
5. Treat the VLM as advisory first, never as the only decision-maker.
6. Prefer existing ROS 2 tooling where it reduces risk, but avoid taking hard dependencies on stale multi-robot packages if they conflict with the desired architecture.

## Responsibility Split

### `astrobot-sim` responsibilities

- OmniLRS runtime.
- Multi-robot spawn/reset/teleport support.
- Robot sensor publishing.
- Environment state changes.
- Hidden ground-truth resource field.
- Anomaly scenario injection.
- Ground-truth logs for evaluation:
  - robot pose
  - resource truth
  - terrain truth
  - fault injection truth

### `astrobot-dev` responsibilities

- Basic teleop and command muxing.
- LiDAR/IMU odometry and SLAM.
- BEV traversability map generation.
- Frontier selection and weighted A* planning.
- Multi-robot task allocation and map fusion.
- Remote API and GUI services.
- VLM orchestration and feedback gating.
- Experiment running, metrics, replay, bagging.

## Recommended Architecture

### Vehicle control

- Low-level robot motion remains topic-based through `/cmd_vel`.
- Build a `basic_control` package in `astrobot-dev` with:
  - teleop input
  - command mux
  - watchdog timeout
  - stop/hold behavior
  - optional rate limiter

This is enough for the first milestone. Do not start by designing a complex action interface.

### Mapping and planning stack

- Per robot:
  - LiDAR/IMU odometry + SLAM
  - local traversability map
  - local planner
- Shared:
  - global BEV cost/traversability map
  - fleet coordinator
  - frontier allocator
  - operator feedback and POI weighting interface

### VLM role

- Consume periodic snapshots and robot summaries.
- Produce:
  - anomaly flags
  - candidate resource regions
  - operator-facing summaries
  - optional POI score deltas
- Initial mode:
  - advisory only
  - never directly writes motion commands

## Recommended Order of Operation

## Phase 0: Remote Ops Baseline

### Goal

Make remote development fast enough that the rest of the roadmap is practical.

### Why this comes first

You are operating remotely and currently blocked on visibility. A GUI should be high priority, but the first version should be thin and operational rather than fully custom.

### Deliverables

- `astrobot-dev` publishes a remote observability stack.
- Live remote access to:
  - camera feeds
  - point cloud
  - TF
  - odometry
  - bag playback
  - robot status
- Operator can send `/cmd_vel` and reset/spawn commands remotely.

### Implementation

- Add Foxglove Bridge first for immediate ROS 2 observability.
- Add `web_video_server` for simple browser camera streaming if embedding in a custom page is useful.
- Expose a very small FastAPI backend for:
  - fleet status summary
  - robot metadata
  - start/stop recording
  - spawn/reset/teleport wrappers
  - health endpoints
- Delay a full React dashboard until the telemetry contract is clear.

### Validation

- Over SSH tunnel, operator can view live topics from a browser.
- Operator can drive a robot remotely with acceptable latency.
- Operator can record and download MCAP logs.
- No need to shell into the GPU host for routine monitoring.

### Current status

- Foxglove Bridge is installed in `astrobot-dev`.
- The bridge is reachable through `127.0.0.1:8765` on the remote host.
- Camera and point cloud rates were reduced to roughly 10 Hz to make remote viewing usable.
- This is sufficient to treat Phase 0 as complete for now.

### Metrics

- UI update latency.
- average camera stream FPS.
- control round-trip latency.
- dropped connection recovery time.

## Phase 1: Basic Control Module

### Goal

Establish a stable, reusable control and point-to-point navigation path for one robot using:

- a perfect static obstacle map
- perfect or near-perfect robot state
- Nav2 for planning and control

This phase is not SLAM. This phase is not exploration. This phase is "make the robot go to commanded poses reliably".

### Current status

Completed for the initial vertical slice in `docker-compose.lunaryard.yml`:

- `astrobot_launch` package brings up:
  - `planner_server`
  - `controller_server`
  - `bt_navigator`
  - lifecycle manager
- OmniLRS simulator-side GT publishing now provides:
  - `/map` as a transient-local `nav_msgs/msg/OccupancyGrid`
  - `map -> odom` derived from simulator robot world pose and live `/odom`
  - `map -> odom` at `30 Hz`
- `basic_control` arbitrates:
  - `/cmd_vel_nav`
  - `/cmd_vel_teleop`
  - final `/cmd_vel`
  - `/control/status`
- Validation completed:
  - no-rock Lunaryard sweep:
    - 5 free goals succeeded
    - 1 occupied goal failed cleanly at planning time
    - final error band about `0.12-0.19 m`
  - rocks-enabled Lunaryard sweep:
    - 5 free goals succeeded
    - 1 occupied goal failed cleanly at planning time
    - final error band about `0.12-0.20 m`
  - mux validation passed for nav pass-through, teleop override, and watchdog zeroing

Known limits from the current live run:

- GT map now comes from OmniLRS terrain data and stage geometry. Rock and mesh footprints are now explicit and tunable, but still heuristic rather than exact per-triangle occupancy.
- The current execution stack is standard Nav2 with conservative tuning:
  - `planner_server`
  - `controller_server`
  - `bt_navigator`
  - `basic_control/command_mux`
- The current costmaps use a rectangular Husky footprint and a static GT map only.
- There is still no live local obstacle layer from the point cloud in Phase 1.

### Key decisions

- The current Husky should be treated as a differential/skid-steer robot, not an Ackermann robot.
- The robot execution interface remains `geometry_msgs/msg/Twist` on `/cmd_vel`.
- Use the standard Nav2 planning and control stack for Phase 1.
- Use:
  - `SmacPlanner2D` for global planning
  - `RotationShimController`
  - `RegulatedPurePursuitController`
  - `basic_control/command_mux` for teleop override and watchdog behavior
- Start with a static GT obstacle map only.
- Do not use point cloud obstacle layers in this phase.

Reason:

- OmniLRS can provide simulator-owned terrain truth and robot world pose directly, which is the right bootstrap for Phase 1 validation.
- Nav2 can be brought up cleanly on a static GT map while keeping all GT generation in `astrobot-sim`.
- `RotationShimController + RegulatedPurePursuitController` gave the simplest maintained path-tracking stack that worked reliably on Lunaryard after plant and GT tuning.

### Deliverables

- `astrobot_launch` package in `astrobot-lab/src`:
  - Nav2 launch files
  - Nav2 params
  - lifecycle manager wiring
- `basic_control` package in `astrobot-lab/src`:
  - teleop input
  - command mux
  - watchdog
  - estop / hold behavior
  - final publish to `/cmd_vel`
- OmniLRS ROS wrapper:
  - GT map publisher in `OmniLRS/src/environments_wrappers/ros2/gt_map_ros2.py`
  - `map -> odom` publisher in the same sim-side node
  - environment accessors for DEM, mask, and robot world pose
- Robot status topic and/or service:
  - mode
  - last command time
  - estop state
  - connectivity
- Command source arbitration:
  - teleop
  - nav2 autonomy
  - replay or testing tools
- Validation path:
  - send a goal pose
  - planner computes a path
  - controller outputs compatible `Twist`
  - robot reaches goal without manual intervention

### Required interfaces

Minimum runtime interfaces for this phase:

- Existing from OmniLRS:
  - `/odom`
  - `/tf` with `odom -> base_link`
  - `/cmd_vel`
  - `/clock`
- New for this phase:
  - `/map`
  - `map -> odom`
  - `/goal_pose` or `NavigateToPose` action usage
  - `/cmd_vel_nav`
  - `/cmd_vel_teleop`
  - `/cmd_vel` as final muxed output
  - `/control/status`

### Package layout

Recommended first package set:

```text
astrobot-lab/
  src/
    astrobot_launch/
    basic_control/
```

Simulator-side GT support lives in:

```text
OmniLRS/
  src/
    environments_wrappers/ros2/gt_map_ros2.py
```

### Nav2 configuration strategy

Use the standard Nav2 binary install rather than trying to install only a few plugins.

Install:

- `ros-$ROS_DISTRO-navigation2`
- `ros-$ROS_DISTRO-nav2-bringup`

Reason:

- this pulls in the server set consistently
- it avoids package-selection churn
- it keeps the planner and controller dependency surface conventional

Core Nav2 choices:

- planner server:
  - `nav2_smac_planner::SmacPlanner2D`
- controller server:
  - `nav2_rotation_shim_controller::RotationShimController`
  - `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
- BT navigator:
  - minimal `NavigateToPose` tree with `ComputePathToPose` and `FollowPath`
- execution output remap:
  - `controller_server /cmd_vel -> /cmd_vel_nav`

Reason:

- the current OmniLRS robot subscribes to `geometry_msgs/msg/Twist`, not `TwistStamped`
- this stack stayed simple enough to debug while outperforming the custom follower on Lunaryard

### GT map strategy

SmacPlanner2D plans over a 2D costmap, not raw terrain meshes.

So Phase 1 needs a GT obstacle map in Nav2-compatible form:

- `nav_msgs/msg/OccupancyGrid`, or
- `map.yaml` + image for `nav2_map_server`

Recommended staged approach:

1. Version 0:
   - transient-local simulator-owned `OccupancyGrid`
   - terrain mask + slope filtering
   - boundary occupancy
   - stage obstacle rasterization
2. Version 1:
   - tune rasterization heuristics per environment family
   - move from obstacle-only occupancy toward weighted traversability costs

This is enough to start point-to-point navigation while keeping all GT ownership in `astrobot-sim`.

### GT state strategy

For Phase 1, use the current `/odom` and derive `map -> odom` from simulator world pose.

Simplest valid setup:

- publish `/map` from the simulator environment data
- compute `map -> odom` from GT robot world pose and the live odometry estimate

This keeps GT generation entirely on the simulator side and avoids dev-side frame hacks.

Later replacements:

- Phase 2:
  - replace GT `map -> odom` with SLAM/localization output
- Phase 2/3:
  - replace GT occupancy/traversability with perception-derived BEV

### What is intentionally deferred

Not in this phase:

- LiDAR obstacle layers
- voxel layers
- traversability slope/roughness penalties
- SLAM-generated map
- frontier exploration
- multi-robot coordination

Reason:

- the current transform tree is still not rich enough for sensor-driven costmaps everywhere
- static simulator-owned GT is enough to prove the basic control path without leaving GT logic behind in `astrobot-dev`

### Bring-up order

1. Install Nav2 binaries in `astrobot-dev`.
2. Create `astrobot_launch`.
3. Add simulator-side GT publishing in OmniLRS.
4. Publish a static GT occupancy grid from environment truth.
5. Publish GT `map -> odom`.
6. Bring up Nav2 with:
   - planner server
   - controller server
   - BT navigator
   - lifecycle manager
7. Configure local and global costmaps to use the static map.
8. Remap controller output to `/cmd_vel_nav`.
9. Insert `basic_control` mux/watchdog between `/cmd_vel_nav` and `/cmd_vel`.
10. Validate with simple point-to-point goals.

### Parameters that must exist before tuning

- robot footprint or conservative robot radius
- max linear velocity
- max angular velocity
- acceleration limits
- inflation radius
- costmap resolution
- goal tolerances

These values do not need to be perfect on day one, but they must be explicitly set.

### Validation

- Send a reachable goal in free space and obtain a valid path.
- Nav2 controller output emits `Twist` commands compatible with the Husky differential controller.
- Robot reaches the goal within configured tolerance.
- Robot does not collide with mapped obstacles.
- Teleop can take control away from autonomy.
- Watchdog stop works when command input stalls.
- Repeated 2 m Lunaryard goals from the validated spawn patch should land in the `~0.12-0.20 m` final-error band for the standard cardinal and diagonal checks.

### Exit criteria

Phase 1 is complete when:

1. a static GT map is available to Nav2
2. `map -> odom` is available
3. Nav2 can drive the Husky to commanded poses
4. `basic_control` safely arbitrates commands and publishes final `/cmd_vel`
5. the whole loop works headless in `docker-compose.lunaryard.yml`

### Metrics

- goal success rate
- planning latency
- controller loop stability
- max cross-track error
- step response time.
- max command dropouts.
- stop distance after watchdog timeout.

## Phase 2: Single-Robot LiDAR/IMU State Estimation

### Goal

Get one robot to build a usable map and localize reliably in the lunar environment.

### Build order

1. Freeze a Phase 1 baseline dataset and command interface.
2. Bring up LiDAR odometry against the current XT32M2X-style cloud and IMU.
3. Compare odometry directly against simulator GT pose.
4. Bring up mapping / loop-closure SLAM after odometry is trustworthy.
5. Swap navigation state inputs from GT to SLAM one interface at a time.

### Recommendation

Use a two-tier approach:

1. Fast baseline odometry first.
2. Heavier SLAM second.

### Baseline option

- KISS-ICP for fast ROS 2 LiDAR odometry integration.

Why:

- Very fast to bring up.
- Official ROS 2 wrapper exists.
- Good for validating point cloud quality, frame conventions, timing, and bagging.

### Primary SLAM target

- LIO-SAM ROS 2 branch.

Why:

- Better fit for LiDAR + IMU.
- More appropriate than 2D SLAM in visually sparse and geometrically repetitive terrain.
- Supports loop-closure style mapping workflows better than pure odometry.

### Non-goals for this phase

- Multi-robot map merge.
- resource reasoning.
- VLM integration.

### Deliverables

- `slam_bringup` package in `astrobot-dev`.
- `slam_eval` package in `astrobot-dev`.
- `state_source_selector` or equivalent pose-switch interface in `astrobot-dev`.
- Per-robot namespaced SLAM launch.
- Standard output topics:
  - `/robot_X/odom`
  - `/robot_X/map` or equivalent pose output
  - `/robot_X/path`
  - `/robot_X/local_cloud`
  - `/robot_X/global_cloud`

### Concrete Phase 2 interfaces

Keep the interfaces stable so GT can remain as a validator while SLAM is inserted:

- Inputs from `astrobot-sim`:
  - `/pointcloud`
  - `/imu`
  - `/odom` only for comparison, not as the primary estimate once Phase 2 is active
  - `/tf` and `/tf_static`
  - simulator-owned GT `/map` and `map -> odom` for evaluation only
- New Phase 2 outputs from `astrobot-dev`:
  - `/slam/odom`
  - `/slam/map_to_odom`
  - `/slam/path`
  - `/slam/local_map` or `/slam/cloud_registered`
  - `/slam/diagnostics`
  - `/slam/gt_error`

### Repo split

- `astrobot-sim` owns:
  - repeatable Lunaryard scenarios
  - rock/no-rock toggles
  - GT `/map`
  - GT `map -> odom`
  - baggable raw sensors `/pointcloud`, `/imu`, `/odom`
- `astrobot-dev` owns:
  - LiDAR odometry bring-up
  - LiDAR/IMU SLAM bring-up
  - GT-vs-SLAM evaluation
  - pose source switching for navigation
  - bag replay / offline metrics

### Concrete implementation plan

#### Phase 2A: Dataset and frame sanity

- Record repeatable Lunaryard bags from the validated Phase 1 spawn patch.
- Verify:
  - point cloud timestamp consistency
  - IMU timestamp consistency
  - `vlp16 -> base_link`
  - `odom -> base_link`
  - frame handedness and yaw sign conventions
- Create a small evaluator that computes:
  - ATE against simulator GT
  - RPE over fixed windows
  - drift per meter traveled

#### Phase 2B: LiDAR odometry baseline

- Start with KISS-ICP-style LiDAR odometry bring-up.
- Goal here is not loop closure. Goal is to answer:
  - is the cloud usable
  - are frames correct
  - is 10 Hz sufficient
  - how much drift do we see on representative Lunaryard trajectories

Exit criteria for 2B:
- repeatable odometry launch
- bounded drift on short Lunaryard loops
- evaluator outputs saved with each run
- no change to the Phase 1 command interface (`/navigate_to_pose` and `/cmd_vel`)

#### Phase 2C: LiDAR/IMU SLAM

- Bring up LIO-SAM-class LiDAR/IMU mapping after the odometry baseline is clean.
- Keep SLAM and GT running in parallel at first.
- Compare:
  - SLAM `map -> odom`
  - simulator GT `map -> odom`
- Do not remove the GT validator when SLAM first comes up.

Exit criteria for 2C:
- SLAM map is stable over longer Lunaryard runs
- loop closures are visible when the trajectory supports them
- ATE / RPE are materially better than the odometry-only baseline
- SLAM pose can run in parallel with GT for a full Phase 1 nav session without destabilizing Nav2

#### Phase 2D: Navigation handoff

- Introduce a state-source switch so Phase 1 navigation can use either:
  - simulator GT pose
  - SLAM pose
- Keep the GT map for this step.
- Only after the SLAM pose handoff is stable should the map source start changing in later phases.

### Validation

- Compare against simulator ground truth.
- Run repeated closed-loop trajectories in Lunaryard.
- Verify drift before and after loop closure.
- Verify operation in low-feature regions.
- Verify the navigation stack can consume SLAM pose without changing the command interface.

### Acceptance checkpoints

1. Frame sanity:
   - all LiDAR/IMU frames consistent
   - evaluator produces stable GT error plots
2. Odometry baseline:
   - KISS-ICP-style odom runs headless from bags and live sim
   - short loops are repeatable
3. SLAM baseline:
   - LIO-SAM-class mapping runs live
   - loop closure improves GT error on revisited tracks
4. Navigation handoff:
   - Phase 1 Nav2 stack can swap GT pose to SLAM pose without touching `/cmd_vel` wiring

### Metrics

- Absolute trajectory error (ATE).
- Relative pose error (RPE).
- loop closure frequency and success.
- CPU/GPU load.
- map update rate.
- robustness across repeated runs.
- navigation success rate using SLAM pose plus GT map.

### Decision

Do not make `slam_toolbox` the primary SLAM path.

Reason:

- It is fundamentally a 2D SLAM tool.
- Your sensor and environment are better matched to 3D LiDAR/IMU.

Keep it only as a debugging fallback if you need a quick planar mapping sanity check.

## Phase 3: BEV Traversability and Weighted A*

### Goal

Convert 3D sensing into a planner-friendly weighted 2D grid.

### Core idea

- Maintain a 3D local/global representation for perception.
- Derive a 2D BEV planning layer with per-cell weights.

Each BEV cell should contain at minimum:

- occupancy
- traversability
- slope penalty
- roughness penalty
- uncertainty
- semantic/resource prior weight

### Recommended planning approach

- Use Nav2 as the base navigation framework.
- Use Smac 2D Planner first for weighted A* on a cost-aware grid.
- Use behavior trees for higher-level mission logic.

### Why this is the right compromise

- It matches your stated preference for A* on a weighted BEV.
- It avoids building the whole navigation stack from scratch.
- It leaves room to replace the global planner later if needed.

### Deliverables

- `mapping_bev` package:
  - point cloud to grid projection
  - terrain slope/roughness estimation
  - weight layer composition
- `planning_bringup` package:
  - global planner
  - local controller
  - recoveries
- `mission_bt` package:
  - navigate to target
  - return home
  - pause/inspect

### Validation

- Known obstacles produce expected costs.
- Planner avoids high-penalty cells.
- Robot reaches operator-specified goals.
- Return-to-home works from arbitrary mapped locations.

### Metrics

- planning latency
- path cost vs path length
- goal success rate
- oscillation count
- recovery count

## Phase 4: Single-Robot Exploration

### Goal

Autonomously explore using the BEV map and frontier/interest weighting.

### Strategy

- Start with frontier-based exploration.
- Score frontiers by:
  - distance
  - expected information gain
  - terrain risk
  - current operator/VLM POI weights

### Deliverables

- `exploration_manager` package:
  - frontier extraction
  - frontier scoring
  - goal dispatch
  - stuck detection
  - return-to-home trigger

### Validation

- Robot increases map coverage without manual goals.
- Robot eventually returns home.
- Exploration stops correctly when time/battery budget is met.

### Metrics

- coverage over time
- distance traveled per new area discovered
- time to first resource candidate
- return success rate

## Phase 5: Multi-Robot Foundation

### Goal

Scale the single-robot stack to a fleet without rewriting the architecture.

### Hard rule

Start with centralized coordination and known initial poses.

Do not start with full decentralized map fusion and unknown-relative-pose collaboration.

### Deliverables

- multiple robots launched in OmniLRS with unique namespaces
- one SLAM instance per robot
- one local planner per robot
- one fleet coordinator node
- one shared fleet state topic/service API

### Shared fleet coordinator responsibilities

- robot registry
- robot health/status
- frontier/task assignment
- collision/conflict avoidance at task level
- return-home prioritization

### Global map management recommendation

Use a staged approach:

1. Shared global BEV with known initial transforms.
2. Shared 3D map products only if needed.
3. Full map merge optimization later.

### Important note on open-source shortcuts

`m-explore-ros2` is useful as a reference for frontier exploration and map merge ideas, but it should not be the long-term core of this project.

Reason:

- it is built around older assumptions
- its multi-robot map merge path remains 2D-centric
- `slam_toolbox` support is described there as experimental

Borrow ideas, not architecture.

### Validation

- two robots can run simultaneously without TF/topic collisions
- each robot gets distinct frontiers
- overlap is reduced vs naive independent exploration
- no fleet deadlock

### Metrics

- coverage gained per robot
- overlap percentage
- coordination latency
- collision count
- idle time per robot

## Phase 6: Resource Simulation

### Goal

Introduce mission value beyond pure coverage.

### `astrobot-sim` implementation

- Add hidden resource layers to the environment:
  - scalar field over terrain
  - optional classes, e.g. ice/mineral
- Add a resource sensor model:
  - footprint
  - noise
  - false positive/negative model
  - configurable sampling duration
- Expose outputs through ROS topics/services/actions.

### Start simple

Version 1 should be:

- a hidden geospatial scalar field
- sampled when the robot is stopped or moving slowly
- returned as a concentration/confidence estimate

Do not start with a detailed physics model of a mass spectrometer.

### `astrobot-dev` implementation

- Convert detections into POI updates.
- Allow the planner to trade off coverage vs expected resource utility.
- Add revisit/inspection behavior.

### Validation

- resource ground truth is reproducible from seed
- sensor outputs respond correctly over high and low concentration zones
- planner prioritizes useful areas when configured to do so

### Metrics

- resource hit rate
- false positive/false negative rate
- time to first true hit
- mission score = coverage + resource utility

## Phase 7: Custom Remote GUI

### Goal

Build the mission operations interface you actually want, but after the telemetry contract is stable.

### Recommendation

This should be "high priority, small first version" rather than "first giant subsystem".

### Recommended UI stack

- React frontend
- FastAPI backend
- live transport:
  - FastAPI WebSockets for mission state and commands
  - Foxglove Bridge for high-throughput ROS introspection
  - `web_video_server` for direct browser camera streaming if embedding video tiles is easier than decoding raw ROS image messages in your own frontend

### Backend responsibilities

- aggregate robot state
- expose fleet and mission REST endpoints
- relay operator commands
- persist annotations and POIs
- trigger bags, snapshots, and exports
- provide authentication if you later open access beyond SSH tunneling

### Frontend responsibilities

- fleet overview
- per-robot cards:
  - mode
  - health
  - battery/mission proxy
  - last update
- live camera tiles
- BEV map panel
- POI list
- anomaly/resource event feed
- operator commands:
  - pause
  - return home
  - reassign goal
  - mark POI

### Validation

- one browser over SSH tunnel can view the mission without shell access
- operator can issue commands and see state updates
- multi-camera view remains usable under real network conditions

### Metrics

- UI latency
- time-to-detect operator issues
- operator task completion time
- browser CPU/network load

## Phase 8: VLM Advisory Layer

### Goal

Use Cosmos Reason2 to improve operator awareness and suggest mission weight adjustments.

### Role of the VLM

Advisory, asynchronous, explainable.

Not:

- low-level control
- safety-critical obstacle avoidance
- sole authority for mission decisions

### Inputs

- periodic camera frames
- optional frame windows or short clips
- robot health summary
- BEV map snapshots
- current frontier list
- recent sensor hits

### Outputs

- mission summary
- anomaly alerts
- candidate POIs
- suggested weight changes for planner regions
- rationale text for operator review

### Recommended deployment strategy

- start with Cosmos Reason2 2B for iteration speed
- upgrade to 8B only if quality justifies the cost
- keep inference isolated in its own service

### Safety gating

- VLM suggestions update a candidate POI layer
- planner only consumes them through a bounded weighting transform
- optionally require operator approval at first

### Validation

- compare VLM suggestions against:
  - human operator labels
  - ground-truth resources
  - anomaly injection truth
- track false alarms carefully

### Metrics

- precision/recall on resource/anomaly suggestions
- operator agreement rate
- time saved to identify important events
- effect on mission score vs no-VLM baseline

## Phase 9: Anomaly Simulation

### Goal

Stress the autonomy and UI stack under failures.

### `astrobot-sim` anomaly types

- camera blackout
- LiDAR dropout
- IMU bias drift
- wheel slip increase
- delayed/fragmented comms
- terrain traps
- false resource signatures

### `astrobot-dev` response behaviors

- degraded mode
- confidence drop alerts
- planner fallback
- return-home trigger
- operator escalation

### Validation

- each anomaly is detectable
- system fails safe
- operator can understand what happened

### Metrics

- detection latency
- recovery success
- false recovery rate
- mission degradation under fault

## White Paper Preparation

Do not wait until the end.

Start collecting material from Phase 2 onward:

- architecture diagrams
- interface definitions
- evaluation methodology
- benchmark tables
- qualitative screenshots
- failure case studies

The final white paper should cover:

- problem statement
- system architecture
- simulator extensions
- autonomy stack
- multi-robot coordination
- resource reasoning
- VLM advisory loop
- evaluation results
- limitations and future work

## Concrete Recommended Build Order

If starting tomorrow, do this:

1. Remote ops baseline
   - Foxglove Bridge
   - optional `web_video_server`
   - tiny FastAPI status API
2. Basic control package
   - teleop
   - command mux
   - watchdog
3. Single-robot SLAM baseline
   - KISS-ICP first
4. Single-robot full SLAM
   - LIO-SAM ROS 2 branch
5. BEV traversability + Nav2 Smac 2D planner
6. Frontier exploration for one robot
7. Multi-robot namespacing and fleet coordinator
8. Shared map/traversability fusion
9. Resource field + sensor simulation
10. Custom React UI
11. Cosmos Reason2 advisory service
12. Anomaly simulation
13. White paper

## What Not To Do First

- Do not start with full multi-robot SLAM merge.
- Do not start with a polished React app before telemetry contracts exist.
- Do not start with VLM-driven autonomy in the loop.
- Do not start with detailed instrument-grade resource physics.

These are all later multipliers, not foundations.

## Open-Source Resources Worth Using

These are the highest-value accelerators for the current plan:

- Foxglove Bridge for remote ROS 2 visualization:
  - https://docs.foxglove.dev/docs/getting-started/frameworks/ros2
- Foxglove shareable layouts and links:
  - https://docs.foxglove.dev/docs/visualization/shareable-links
- `web_video_server` for browser camera streaming:
  - https://github.com/RobotWebTools/web_video_server
- `rclnodejs` if a Node/TypeScript ROS 2 service becomes preferable later:
  - https://github.com/RobotWebTools/rclnodejs
- `rosbridge_suite` if a JSON/WebSocket ROS bridge is needed:
  - https://index.ros.org/r/rosbridge_suite/
- KISS-ICP for fast ROS 2 LiDAR odometry bring-up:
  - https://github.com/PRBonn/kiss-icp
- LIO-SAM as the main LiDAR/IMU SLAM target:
  - https://github.com/TixiaoShan/LIO-SAM
- Nav2 Smac Planner for weighted A* style global planning:
  - https://docs.nav2.org/configuration/packages/configuring-smac-planner.html
- Nav2 behavior trees:
  - https://docs.nav2.org/behavior_trees/
- Nav2 costmap filters for operator/VLM-defined keepouts and speed zones:
  - https://docs.nav2.org/plugins/index.html
  - https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html
  - https://docs.nav2.org/tutorials/docs/navigation2_with_speed_filter.html
- `m-explore-ros2` as a reference implementation only:
  - https://github.com/robo-friends/m-explore-ros2
- Cosmos Reason2 docs:
  - https://docs.nvidia.com/cosmos/latest/reason2/index.html
- Cosmos Reason2 NIM API:
  - https://docs.nvidia.com/nim/vision-language-models/1.6.0/examples/cosmos-reason2/api.html

## Suggested Near-Term Repo Structure

Recommended additions under `astrobot-dev` work:

```text
astrobot-lab/
  control/
  slam_bringup/
  mapping_bev/
  planning_bringup/
  exploration_manager/
  fleet_coordinator/
  mission_msgs/
  mission_interfaces/
  ops_api/
  ops_ui/
  vlm_service/
  evaluation/
```

Recommended simulator-side additions:

```text
OmniLRS/
  cfg/
    environment/
    robots/
    sensors/
    resources/
    anomalies/
  src/
    sensors/
    resources/
    anomaly_injection/
```

## Immediate Next Step Recommendation

The next engineering sprint should be:

1. add remote observability stack
2. add teleop/watchdog/basic control
3. bring up KISS-ICP on the current point cloud + IMU

Phase 0 is now substantially in place, so the next active build step is Phase 1:

1. add teleop/watchdog/basic control
2. standardize robot status reporting
3. keep Foxglove as the operator-facing interface until a custom UI is justified
