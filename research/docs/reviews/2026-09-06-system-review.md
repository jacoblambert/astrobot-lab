# Astrobot system and performance review

Review date: 2026-09-06. Source baseline: root `dev/phase-5`, HEAD `40988b4`, **including the existing uncommitted working tree**. Latest mission artifacts examined: 2026-06-18. This is a diagnosis and decision document; no production fixes were applied.

## Executive assessment

The current bottleneck is a coupled perception, timing, recovery, and evaluation problem. There are several independently reproducible correctness defects upstream of controller tuning. The evidence does not yet justify replacing GLIM, Nav2, or MPPI.

The highest-value findings are:

1. The reference pose stream occasionally contains invalid zero-position samples. The synthetic absolute-position source accepts these and also makes stale measurements appear fresh. A large localization transient follows one such sample in a recent run.
2. The live mapper uses 10 cm voxels, while several perception experiments use 5 cm voxels. With the live configuration, requiring three occupied voxels per BEV cell imposes a vertical-support requirement that excludes small rocks before obstacle-height classification. A 14 cm ground-to-rock fixture is classified as free by the actual C++ implementation.
3. Backup collision checking uses `odom` coordinates against a `map`-frame local costmap. Separately, mission progress monitoring can cancel an ongoing backup before it has enough simulated time to finish.
4. The `map -> odom` bridge combines unmatched timestamps and continually recomputes a correction from stale SLAM poses. This can suppress the very odometric propagation the bridge is intended to preserve.
5. Return-home handling can declare completion away from home, hang after goal rejection, or discard every breadcrumb without attempting a local escape.
6. Some evaluation metrics and replay tools do not measure what their names imply. Previous negative experiments cannot reliably rule out promising perception changes.
7. Mapping has a demonstrated computational scaling problem: long exploration runs log about 243–247 ms median per cloud integration, before map projection. Python reachability also has an expensive full-grid path.

Recommended decision: first establish trustworthy inputs, frame/time contracts, and deterministic component evaluation; then repair perception and recovery; then optimize measured hot paths. Keep beacon-assisted surface navigation and beacon-denied autonomy as separate capability tracks.

## 1. What the project currently is

The active implementation is substantially beyond the Phase 1 description in `AGENTS.md`. The current stack includes:

- OmniLRS/Isaac Sim, a Husky rover, a 40 m Lunaryard, a remounted VLP16, IMU, cameras, simulator reference maps, and resource sampling.
- ROS 2 through the Space ROS/Jazzy environment, CycloneDDS, domain 177, and the headless `docker-compose.lunaryard.yml` workflow.
- A crop-filtered lidar/IMU pipeline into a modified CPU GLIM estimator, with a custom absolute-position/heading extension and synthetic positioning enabled in the current workflow.
- A custom probabilistic voxel map projected into a BEV traversability/cost grid.
- Nav2 SmacPlanner2D, MPPI, a lunar recovery behavior tree, velocity smoothing, and `basic_control/command_mux`.
- A Python exploration/resource/return-home manager and extensive ad-hoc evaluation scripts.
- A Phase 5 FastAPI/React console, including telemetry, maps, camera viewing, and click-to-goal commands.

The important data dependencies are:

```text
simulated lidar + IMU -> GLIM -> corrected clouds -> voxel/BEV map -> Nav2
                          ^                                      ^
simulator pose -> synthetic position + heading             exploration manager
       |
       +-> reference scoring and resource sampling

simulator chassis odometry + GLIM pose -> map/odom bridge -> navigation TF
Nav2 -> velocity smoother -> command mux -> simulated drivetrain
```

This means reference-data faults can affect both the autonomy under test and its evaluator. It also means successful runs currently demonstrate **assisted simulated navigation**, not independent lidar-inertial localization under cave-like positioning loss.

I inspected the active Husky USD offline. `/odom` is produced by `IsaacComputeOdometry` connected to the chassis prim, not by a modeled wheel-encoder estimator. Its outputs also drive `odom -> base_link`. The USD publishes the IMU mount in the robot TF tree. These are important experiment assumptions, not necessarily inappropriate choices for the current phase.

Sources: [active launch](/home/jacoblambert/astrobot-lab/astrobot-lab/src/astrobot_launch/launch/phase3_nav.launch.py), [mission launch](/home/jacoblambert/astrobot-lab/astrobot-lab/src/astrobot_launch/launch/phase4_exploration.launch.py), [robot asset](/home/jacoblambert/astrobot-lab/OmniLRS/assets/USD_Assets/robots/ros2_husky_PhysX_vlp16_mono_depth_imu.usd), [GLIM configuration](/home/jacoblambert/astrobot-lab/astrobot-lab/src/glim_slam_launch/config/glim_astrobot/config_ros.json).

### Recorded project status

The latest per-seed gate files report the following. These are six latest recorded attempts, **not a fresh validation of today's dirty tree or a reliability estimate**.

| Mission | Seed | Recorded result | Reported mission coverage | GT home distance |
|---|---:|---|---:|---:|
| Sample return | 42 | PASS | Not the mission gate | 0.786 m |
| Sample return | 411 | PASS | Not the mission gate | 0.706 m |
| Sample return | 7 | PASS | Not the mission gate | 0.707 m |
| Explore radius | 42 | FAIL | 38.5% | 3.606 m |
| Explore radius | 411 | PASS | 73.1% | 0.654 m |
| Explore radius | 7 | PASS | 71.2% | 0.739 m |

Thus the latest records are sample return 3/3 and exploration 2/3. The aggregate `phase4_gate_suite.json` is older and does not describe these latest attempts. Earlier same-day runs include different outcomes for the same seeds; configuration/scene changes prevent treating all those runs as identical repeated trials.

The sample-return implementation detects a resource threshold and returns home; these results do not demonstrate physical sample collection. The console exists and its TypeScript check passes, but that is not a full UI acceptance test.

Evidence: [sample 42](/home/jacoblambert/astrobot-lab/rosbags/phase4_gate_sample_return_seed42.json), [sample 411](/home/jacoblambert/astrobot-lab/rosbags/phase4_gate_sample_return_seed411.json), [sample 7](/home/jacoblambert/astrobot-lab/rosbags/phase4_gate_sample_return_seed7.json), [explore 42](/home/jacoblambert/astrobot-lab/rosbags/phase4_gate_explore_radius_seed42.json), [explore 411](/home/jacoblambert/astrobot-lab/rosbags/phase4_gate_explore_radius_seed411.json), [explore 7](/home/jacoblambert/astrobot-lab/rosbags/phase4_gate_explore_radius_seed7.json).

### Localization looks substantially better than some diagnostics suggest

I recomputed planar base-link error from the six latest TUM pairs using the configured IMU-to-base offset, the known home translation `(20,18)`, and timestamp interpolation. I did not fit the evaluated trajectory to the reference. Exact zero-XYZ reference rows were excluded **only for this diagnostic calculation** and their occurrences were counted separately.

| Mission / seed | Median XY error | 95th percentile | Maximum | Invalid zero-XYZ GT rows |
|---|---:|---:|---:|---:|
| Sample / 42 | 0.060 m | 0.091 m | 0.121 m | 1 |
| Sample / 411 | 0.037 m | 0.068 m | 0.101 m | 0 |
| Sample / 7 | 0.044 m | 0.073 m | 0.103 m | 0 |
| Explore / 42 | 0.092 m | 0.150 m | 0.187 m | 3 |
| Explore / 411 | 0.037 m | 0.077 m | 0.118 m | 5 |
| Explore / 7 | 0.079 m | 0.157 m | 1.164 m | 3 |

These are approximate planar diagnostics, not certified 6-DoF estimator accuracy: the reference itself has faults, full roll/pitch lever-arm correction was not used, and input integrity remains unresolved. Nevertheless, the failed latest seed-42 exploration has modest localization error. Localization divergence alone is not a sufficient explanation for that mission failure.

## 2. Confirmed bugs and immediate correctness risks

Priority terminology: **P1** should be addressed before trusting further performance conclusions; **P2** is important hardening or a conditional limitation. Confidence distinguishes reproduced behavior, source-confirmed defects, and unproven causal explanations.

### F01 — Invalid and stale reference poses enter the absolute-position estimator

**P1. Invalid archived samples and missing validation are confirmed; the simulator race explanation is a hypothesis.**

The GT publisher forwards a dynamic-control rigid-body pose without a validity or kinematic check. GT publication runs in a separate executor thread from the main simulation stepping path. In recent trajectories, isolated `(0,0,0)` positions appear while neighboring poses place the rover elsewhere in the yard.

For example, latest explore seed 7 contains a zero-XYZ GT sample at approximately simulation time 280.0333 s. Recomputed localization error is approximately 0.097 m at that time and 1.164 m at 280.0833 s, then decays. This temporal association is consistent with a bad absolute-position update; proving the causal chain requires logging the actual RTK message and factor association.

The synthetic positioning node stores every GT pose. At its default 30 Hz republish rate it takes the latest stored pose, adds new noise, and gives it the current timestamp. It will do this indefinitely if GT stops. Consequently, old or bad positions become apparently fresh independent measurements. The GLIM position innovation gate and robust loss are disabled in the active configuration.

**Potential solution:** capture a coherent pose snapshot on the physics-step side, carry its original measurement timestamp and validity into ROS, reject impossible samples with a reference-to-reference kinematic check, and stop emitting fixes when the source is stale. Add explicit dropout, delayed-fix, zero-pose, quaternion, and non-finite input fixtures. Record rejected reference samples as infrastructure faults; do not silently clean them out of official scores.

Sources: [robot pose read](/home/jacoblambert/astrobot-lab/OmniLRS/src/robots/robot.py:658), [GT publication](/home/jacoblambert/astrobot-lab/OmniLRS/src/environments_wrappers/ros2/gt_map_ros2.py:290), [synthetic positioning](/home/jacoblambert/astrobot-lab/astrobot-lab/src/synthetic_rtk/synthetic_rtk/synthetic_rtk_node.py:133), [affected trajectory](/home/jacoblambert/astrobot-lab/rosbags/phase4_gate_runs/explore_radius_seed7_20260618_115319/explore_radius_seed7_a1/gt_base.tum).

### F02 — Live voxel geometry excludes small obstacles before classification

**P1. Reproduced against the actual mapper implementation.**

`phase3_nav.launch.py` overrides `voxel_size` to 0.10 m. BEV resolution is also 0.10 m. The projection requires `min_occupied_voxels_per_cell=3` before assigning low/high surface heights. With these aligned resolutions, occupied voxels in one XY cell differ primarily in their vertical bin: three bins require a nominal 20 cm separation between the lowest and highest voxel centers.

That is a different requirement from the configured 8 cm obstacle-height threshold. A small rock with ground and rock-top returns in only two vertical bins is excluded before that threshold is evaluated. Single-bin flat-ground observations are also excluded from the local ground-height samples.

Diagnostic fixture, using the repository C++ core:

```text
14 cm ground-to-rock span, min_voxels=3: cost=0
Same evidence, min_voxels=2: cost=100
```

This does not establish that changing the parameter to two is a sufficient production fix. It establishes a deterministic blind spot and a confound in earlier experiments: several replay/A-B scripts use 5 cm voxels instead. Conclusions that lowering the support threshold cannot help are not transferable to the live configuration without retesting.

**Potential solution:** share one resolved mapper configuration between live and replay. Separate surface confidence/temporal support from the number of vertically distinct occupied bins. Evaluate 5–25 cm obstacles, slopes, flat ground, isolated noisy returns, range, and viewing angle. Consider a denser dedicated obstacle-mapping input instead of using exactly the same downsampled cloud as localization. Measure the accuracy/compute trade-off of 5 cm versus 10 cm explicitly.

Sources: [live override](/home/jacoblambert/astrobot-lab/astrobot-lab/src/astrobot_launch/launch/phase3_nav.launch.py:88), [support setting and prior A/B conclusion](/home/jacoblambert/astrobot-lab/astrobot-lab/src/astrobot_launch/launch/phase2_slam.launch.py:216), [support gate](/home/jacoblambert/astrobot-lab/astrobot-lab/src/slam_mapping/src/probabilistic_voxel_map.cpp:316), [replay configuration](/home/jacoblambert/astrobot-lab/tools/gt_isolated_eval.py:83).

### F03 — Recovery collision checking uses the wrong coordinate frame

**P1. Confirmed from local configuration and the installed Nav2 version's source.**

The local costmap uses `global_frame: map`. The behavior server sets `global_frame: odom` and leaves `local_frame` unset. In Nav2 1.3.11, the latter defaults to `odom`. Backup obtains its current pose in `local_frame` and passes its XY/yaw directly to the local costmap collision checker, which does not transform that pose.

Therefore, when `map` and `odom` differ, backup checks the wrong location. A small correction can produce misleading collision decisions near rocks; larger corrections can move the checked footprint substantially. This is an integration defect, not evidence of a defect in Nav2's collision checker.

**Potential solution:** enforce matching behavior/costmap frame contracts and add a test with a deliberately non-identity `map -> odom`. The minimal current-stack correction is to make the behavior's local frame agree with its map-frame local costmap. A longer-term odom-frame rolling local map is a separate architectural choice requiring correct map-layer transforms.

Sources: [behavior configuration](/home/jacoblambert/astrobot-lab/astrobot-lab/src/astrobot_launch/config/nav2_phase3_bev.yaml:24), [local costmap](/home/jacoblambert/astrobot-lab/astrobot-lab/src/astrobot_launch/config/nav2_phase3_bev.yaml:215), [Nav2 1.3.11 defaults](https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.11/nav2_behaviors/src/behavior_server.cpp), [backup pose handling](https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.11/nav2_behaviors/include/nav2_behaviors/plugins/drive_on_heading.hpp), [collision checker](https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.11/nav2_costmap_2d/src/costmap_topic_collision_checker.cpp).

### F04 — The TF bridge combines different measurement times and masks staleness

**P1. Source-confirmed.**

The bridge calculates:

```text
map_T_odom = latest_map_T_base * inverse(latest_odom_T_base)
```

The two inputs are independently cached and their stamps are not matched. It recalculates this expression on a wall timer and stamps the result with current ROS time plus tolerance. If SLAM is delayed, composing the result with current odometry approximately recovers the old SLAM pose, cancelling odometry's forward propagation. If SLAM stops entirely, the bridge can continue to appear fresh while the map-frame robot pose remains stale.

**Potential solution:** compute the correction at a common SLAM measurement time using odometry history/interpolation, hold that correction between localization updates, and let current odometry propagate the base pose. Track measurement age separately from transform publication age and fail safely on stale/non-finite localization.

There is also a TF ownership problem: GLIM publishes `slam_odom -> Imu_Sensor`, while the robot asset publishes the IMU mount in the robot tree. Preflight explicitly exempts this duplicate parent. It should instead enforce a single authority per child and use a separate estimator frame or disable the redundant estimator transform. ROS's frame convention requires one parent per frame and distinguishes continuous local odometry from globally corrected map poses. [REP-105](https://reps.openrobotics.org/rep-0105/)

Sources: [TF composition and restamping](/home/jacoblambert/astrobot-lab/astrobot-lab/src/slam_mapping/src/slam_map_to_odom_tf_node.cpp:113), [GLIM TF publisher](/home/jacoblambert/astrobot-lab/astrobot-lab/src/glim_ros2/src/glim_ros/rviz_viewer.cpp:187), [preflight exemption](/home/jacoblambert/astrobot-lab/tools/phase4_gate_preflight.py:69).

### F05 — Mixed clocks and competing progress monitors interrupt recovery

**P1. Source-confirmed and observed in archived logs.**

The exploration manager measures mission/progress durations with `time.monotonic()`, while Nav2 behaviors and physics run on simulation time. The latest exploration runs have real-time factors of approximately 0.446–0.456. Thus 12 wall seconds provide only about 5.4 simulation seconds of motion.

The lunar backup requests 0.65 m at 0.08 m/s: nominally 8.125 simulation seconds, before any following wait. In failed seed-7 run `20260618_084644`, backup starts at wall log time 1781741429.266 and the manager cancels the navigation goal for `no_progress` at 1781741441.273. The behavior tree then reports failure to obtain the backup result during halt. Further backups in the same run are also cancelled early.

Progress toward a goal is measured against the best remaining path distance. Backing away or replanning can increase that distance even when recovery is working. The mission manager and Nav2 therefore disagree about what constitutes useful progress.

**Potential solution:** use simulation time for simulated mission/motion budgets and a separate wall-clock watchdog for process/input freshness. Give recovery an explicit bounded phase with one owner and its own progress criterion. Test invariance across simulation speed, including pause/resume. Merely increasing all timeouts would leave the ownership and clock defects intact.

Sources: [manager cycle](/home/jacoblambert/astrobot-lab/astrobot-lab/src/exploration_manager/exploration_manager/exploration_manager_node.py:429), [progress monitoring](/home/jacoblambert/astrobot-lab/astrobot-lab/src/exploration_manager/exploration_manager/exploration_manager_node.py:783), [recovery tree](/home/jacoblambert/astrobot-lab/astrobot-lab/src/astrobot_launch/behavior_trees/navigate_to_pose_lunar_recovery.xml), [observed cancellation](/home/jacoblambert/astrobot-lab/rosbags/phase4_gate_runs/explore_radius_seed7_20260618_084644/explore_radius_seed7_a1/mission_launch.log:2139).

### F06 — Return-home state transitions are incomplete

**P1. Two cases reproduced by executing the repository's Python methods with action/message stubs; breadcrumb failure observed in logs.**

- The home waypoint can be projected to another free point within 1.5 m. Successful completion of the final return action sets the mission to finished without checking `_is_home_reached()`. A direct method fixture with a successful return goal 1.5 m from home reports `finished=True`, `home_reached=False`. Nav2's 0.65 m goal tolerance adds another source of terminal-position slack.
- A rejected return action clears `active_goal` but leaves `returning_home=True`. The main cycle then only publishes status and returns. No retry, continuation, or terminal failure occurs. An unavailable action server has a related continuation problem.
- In latest explore seed 42, the manager enters return mode, skips all seven breadcrumbs as unreachable in about 0.16 seconds, and fails. It does not first perform a bounded local escape to recover a valid planning start.
- Planner-precheck callbacks do not carry a request-generation identity. Timing out an old check and starting a new one can let a late response act on newer pending state.

**Potential solution:** make return states and transitions explicit: selecting route, checking, navigating, recovering, verifying home, done/failed. Preserve failed waypoints until an explicit decision is made; attach IDs to asynchronous work; handle rejection, server loss, cancellation, and timeout on every path. Make terminal home-distance verification a mandatory invariant. Treat a reachable home approach point and actual home arrival as different states.

Sources: [return waypoint selection](/home/jacoblambert/astrobot-lab/astrobot-lab/src/exploration_manager/exploration_manager/exploration_manager_node.py:577), [projection](/home/jacoblambert/astrobot-lab/astrobot-lab/src/exploration_manager/exploration_manager/exploration_manager_node.py:675), [action responses/results](/home/jacoblambert/astrobot-lab/astrobot-lab/src/exploration_manager/exploration_manager/exploration_manager_node.py:1007), [seed-42 return failure](/home/jacoblambert/astrobot-lab/rosbags/phase4_gate_runs/explore_radius_seed42_20260618_110709/explore_radius_seed42_a1/mission_launch.log:1327).

### F07 — Localization supervision is not an active-goal safety guarantee

**P1. Source-confirmed.**

The main cycle returns through the active-goal branch before the SLAM watchdog and mission-level resource/budget checks. The watchdog primarily checks map-to-odom translation magnitude; it does not establish pose freshness, finite values, yaw quality, or covariance consistency. Missing TF returns a non-diverged result.

Consequently, a zero `slam_diverged` count does not prove localization remained valid throughout a goal, and mission-level resource/budget conditions can be deferred until the current action ends.

**Potential solution:** run freshness and validity supervision independently of mission state, with explicit degraded/invalid behavior and a tested command-stop path. Keep localization quality separate from the magnitude of the map/odom correction.

Sources: [early returns](/home/jacoblambert/astrobot-lab/astrobot-lab/src/exploration_manager/exploration_manager/exploration_manager_node.py:429), [watchdog](/home/jacoblambert/astrobot-lab/astrobot-lab/src/exploration_manager/exploration_manager/exploration_manager_node.py:1220).

## 3. Evaluation defects that obstruct learning

### F08 — Coverage, path efficiency, and failure labels need corrected definitions

**P1. Source-confirmed; coverage and verdict edge cases reproduced.**

1. **Coverage denominator:** mission coverage divides visited cells by the maximum observed reachable-cell count, rather than a fixed externally defined mission area. A fixture with four visited/reachable coarse cells in a 52-cell mission reports 100%. Latest successful exploration runs happen to have denominators of 52, so this defect alone does not invalidate those specific reported percentages. It does make the metric vulnerable to map/reachability changes. Estimated goal targets can also count as visited after tolerant goal success.
2. **Coverage independence:** the gate checks the manager's own coverage output. Unlike home distance, coverage is not independently reconstructed from reference motion or sensor coverage. Observed terrain, visited terrain, and traversable terrain are different metrics and should remain separate.
3. **Verdict consistency:** `return_reason=slam_diverged` adds a failure reason but does not participate in the pass Boolean. A fixture returns `pass=True` alongside `reasons=['slam_diverged']`.
4. **False-success labels:** `returning_home=True` with a failed mission away from home is labeled `false_success`, even though the manager did not claim success. This makes ordinary return failures look like proven SLAM-induced false completion.
5. **Path efficiency:** the evaluator overwrites its ratio with the latest remaining plan length divided by initial goal distance. Values such as 0.33 are expected as a robot approaches its goal. They are not measurements of executed-route efficiency and should not drive a policy/controller replacement decision.
6. **Localization diagnostics:** some runtime diagnostics compare a base-link GT pose to an IMU SLAM pose using only an initial translation offset. The omitted rotating lever arm can create an apparent turn-dependent error approaching 0.53 m. Some speeds are distance divided by wall time, which scales them down with simulation real-time factor.

**Potential solution:** define fixed-area physical visitation, sensor-observation coverage, and reachable-area coverage separately; calculate official mission success from independently timestamped reference state; enforce `pass`/reason consistency; compute executed path length per action ID using valid reference poses; use one tested extrinsic/time-alignment implementation. Keep online estimates as diagnostics, not their own ground truth.

Sources: [coverage computation](/home/jacoblambert/astrobot-lab/astrobot-lab/src/exploration_manager/exploration_manager/exploration_manager_node.py:1604), [gate verdict](/home/jacoblambert/astrobot-lab/tools/phase4_exploration_eval.py:488), [path callback](/home/jacoblambert/astrobot-lab/tools/phase4_exploration_eval.py:252), [canonical alignment helper](/home/jacoblambert/astrobot-lab/tools/map_gt_alignment.py).

### F09 — GT-isolated mapper replay is neither equivalent nor deterministic

**P1. Source-confirmed.**

The intended separation of localization from perception is valuable, but the current `gt_isolated_eval.py` has several confounds:

- It associates using bag receive timestamps rather than consistently using message measurement stamps, with unbounded nearest matching.
- It converts SLAM XY from IMU to base but retains IMU Z, then reanchors to GT base Z. The resulting cloud-height offset is approximately the sensor lever-arm height under planar assumptions.
- It uses the GT base location as the ray origin rather than the lidar origin.
- It shifts the map to world coordinates with extent approximately `[0,40]`, but leaves mapper integration bounds at the default `[-25,25]` in XY. Points beyond 25 m are dropped; an out-of-bounds sensor origin drops the whole cloud.
- It uses 5 cm voxels, while live navigation uses 10 cm, and omits the live near-field persistence setting.
- It sends clouds with a fixed 20 ms sleep and no per-frame processing acknowledgment. The mapper's bounded queue and variable runtime mean a slower algorithm can consume a different subset of scans. A final fixed sleep does not establish complete integration.

These issues undermine claims that an algorithm family has been conclusively ruled out by the existing isolated replay.

**Potential solution:** deterministic in-process replay through the mapper core, or a sequence-number/acknowledgment protocol with exact integrated-frame counts. Use complete frame transforms at measurement time, matching live parameters and valid bounds. Separate cloud-geometry evaluation from ray-clearing evaluation, and archive per-stage counts. Rerun a small representative baseline before any large sweep.

Source: [isolated replay](/home/jacoblambert/astrobot-lab/tools/gt_isolated_eval.py).

### F10 — Experiment identity and gate aggregation are insufficient

**P1 for conclusions about improvement; source-confirmed.**

- Root and submodules have substantial existing modifications. The default entrypoint does not rebuild source GLIM unless explicitly enabled; prebuilt image GLIM and workspace overlays coexist. Editing a source file does not by itself prove the running executable changed.
- The gate does not archive a complete resolved configuration/build/image/asset manifest. Scene geometry and software have changed between trials.
- The suite defaults to a retry and treats any passing attempt as a passing seed. This is useful for smoke tests but is not per-attempt reliability.
- The suite prints failures but ends with successful summary/cleanup commands; it does not propagate an aggregate failure exit status.
- Preflight can continue after a soft settling warning. Default compose enables GT TF unless overridden, while Phase 3 only prints a warning about disabling it.
- Synthetic RTK's default RNG seed zero selects a nondeterministic generator. Resource layout is not independently diversified across the rock seeds used in the gate.
- The GT obstacle raster is a geometric rock approximation, not a physical collision/contact oracle. Current mission gates do not establish collision-free operation or negative-obstacle safety.

**Potential solution:** an immutable run manifest with dirty diffs, submodule commits, loaded executable/library paths, image IDs, resolved parameters, seeds, scene/asset hashes, TF authorities, and scenario contracts. Report all attempts, classify infrastructure-invalid trials separately, use a nonzero suite exit on failure, and fail closed on wrong-frame/GT-leak preconditions. Preserve the scenario while tuning the algorithm.

Sources: [entrypoint](/home/jacoblambert/astrobot-lab/docker/dev_entrypoint.sh:32), [compose](/home/jacoblambert/astrobot-lab/docker-compose.lunaryard.yml), [suite](/home/jacoblambert/astrobot-lab/tools/phase4_gate_suite.sh), [per-mission harness](/home/jacoblambert/astrobot-lab/tools/phase4_mission_gate_run.sh).

## 4. Computational and architectural bottlenecks

### F11 — Mapper work grows with accumulated map size

**P1 performance finding. Logged integration timings and implementation agree; an end-to-end profiler run is still needed.**

| Latest exploration | Median integration | 95th percentile | Logged samples | Real-time factor |
|---|---:|---:|---:|---:|
| Seed 42 | 102.4 ms | 206.4 ms | 60 | 0.446 |
| Seed 411 | 242.8 ms | 270.4 ms | 163 | 0.453 |
| Seed 7 | 246.9 ms | 282.6 ms | 107 | 0.456 |

These are throttled integration-log samples, not an unbiased profiler distribution, and exclude separate BEV publication work. At roughly 0.25 s per integration, a single-threaded executor can integrate at most about four clouds per wall second before projection/other callbacks. Corrected SLAM stamps in these runs advance at about 20 Hz simulated time, approximately nine Hz wall time. This creates a clear service-rate mismatch at sustained long-run costs; exact dropped-cloud counts are not instrumented.

The implementation decays the entire voxel hash map on each cloud, raycasts many points, then scans the accumulated map again for projection. BEV plane fitting also traverses the full 400-by-400 grid and local neighborhoods. This makes runtime depend on mission history rather than just current local scene complexity.

**Potential solutions, in increasing scope:** instrument receipt/integration/publication counts and measurement age; use time-based lazy decay; update dirty BEV regions; cache/reuse projection buffers; adopt spatial blocks and a rolling local obstacle map; keep longer-lived terrain/mission information in a separate representation. Preserve a deterministic baseline while measuring accuracy versus subsampling and map radius.

A map publication rate near 0.9 Hz wall time is consistent with the configured 2 Hz simulation-time timer at RTF 0.45. It is not by itself proof that the timer missed its target. What matters for control is map measurement age, integrated evidence, and end-to-end latency.

Sources: [integration](/home/jacoblambert/astrobot-lab/astrobot-lab/src/slam_mapping/src/probabilistic_voxel_map.cpp:174), [node execution/publication](/home/jacoblambert/astrobot-lab/astrobot-lab/src/slam_mapping/src/probabilistic_voxel_mapper_node.cpp:65), [seed-411 logs](/home/jacoblambert/astrobot-lab/rosbags/phase4_gate_runs/explore_radius_seed411_20260618_112044/explore_radius_seed411_a1/mission_launch.log).

### F12 — Reachability is expensive and disagrees with navigation geometry

**P1/P2. Host timing reproduced; geometric mismatch source-confirmed.**

Executing the production reachability method on a free 400-by-400 grid took about 0.83 seconds on this host. It flood-fills in Python while repeatedly examining clearance neighborhoods. Candidate/return evaluation repeats substantial work.

The mission's configured 0.90 m clearance uses a square neighborhood, approximately 1.8 m across, around cells above its threshold. Nav2 instead uses the rectangular footprint, padding, and inflation. These are not equivalent feasibility tests. The mission layer can reject routes or starts that Nav2 considers feasible. Conversely, relocating a blocked start to a nearby free cell can select a disconnected component without a physically valid path from the actual robot pose.

Stall blacklists filter candidate endpoints rather than making the failed physical corridor costly to the planner. Choosing another endpoint can send the robot through the same unseen obstacle again. Skipping a failed breadcrumb does not solve a local wedge at the current location.

**Potential solution:** cache a vectorized/compiled clearance mask and connected components per map version, with a shared footprint/clearance contract. Keep actual start connectivity distinct from projected candidate feasibility. Add a bounded local escape procedure and short-lived contact/stall evidence to the local navigation map, with explicit expiry and validation.

Sources: [reachability and safety neighborhood](/home/jacoblambert/astrobot-lab/astrobot-lab/src/exploration_manager/exploration_manager/exploration_manager_node.py:2136), [stall blacklist](/home/jacoblambert/astrobot-lab/astrobot-lab/src/exploration_manager/exploration_manager/exploration_manager_node.py:2078), [Nav2 footprint configuration](/home/jacoblambert/astrobot-lab/astrobot-lab/src/astrobot_launch/config/nav2_phase3_bev.yaml:181).

### F13 — BEV free-space and persistence semantics need a stronger terrain model

**P1 for extending to hazardous terrain; current behavior reproduced/source-confirmed.**

The current projection can label an XY column free without supported traversable ground. For example, one horizontal ray at 1 m height, with no ground observations, produces BEV cost zero along the ray. Missing surface/ground estimates also fall back to free. Thus clearing air and establishing drivable ground are conflated.

Additional problems:

- Rays originate from the latest IMU pose rather than a timestamp-matched lidar pose; the configured lidar/IMU translation is about 0.14 m. Independently cached cloud/pose callbacks add temporal mismatch.
- Occupied evidence receives at most one hit per cloud, while misses accumulate per ray. Three misses at -0.20 outweigh one +0.50 hit, so the belief depends strongly on scan density and ray overlap.
- With hit 0.5, threshold 1.0, and decay 0.999, two observations reach 0.9995, not occupancy; a third is required. Decay is per integrated cloud, so changing throughput changes persistence in time.
- Near-field persistence freezes every qualifying occupied voxel in its radius, including ground/noise; it has no separate obstacle identity or age criterion.
- At capacity, new voxels are ignored without spatial replacement, and free-ray allocations occur before endpoint hits. A five-voxel capacity fixture drops the actual obstacle endpoint. Recent logs do not establish that the live six-million-voxel limit was reached; this is a latent long-run failure mode.
- A single 2D height-cost layer does not explicitly represent missing support, rover clearance under overhangs, or distinct traversable floors. Slope lethality is not currently enforced by the disabled slope-cost term.

**Potential solution:** distinguish observed air, supported ground, obstacle volume, unknown, and unsafe slope/drop-off. Use matched lidar poses and time-based evidence updates. Separate ground support from above-ground obstacle confidence; cap/deduplicate evidence consistently; make persistence bounded and explainable. A local elevation/traversability map plus obstacle-volume layer is a plausible next architecture, but should be compared against a corrected baseline before replacement.

Sources: [ray and voxel updates](/home/jacoblambert/astrobot-lab/astrobot-lab/src/slam_mapping/src/probabilistic_voxel_map.cpp:108), [free fallback](/home/jacoblambert/astrobot-lab/astrobot-lab/src/slam_mapping/src/probabilistic_voxel_map.cpp:531), [pose callback](/home/jacoblambert/astrobot-lab/astrobot-lab/src/slam_mapping/src/probabilistic_voxel_mapper_node.cpp:165), [sensor extrinsics](/home/jacoblambert/astrobot-lab/astrobot-lab/src/glim_slam_launch/config/glim_astrobot/config_sensors.json).

### F14 — Fusion weights currently compensate for a measurement-model mismatch

**P2 design issue with P1 input-integrity dependencies. Source-confirmed; causal contribution to numerical failures needs controlled testing.**

The synthetic positioning noise grows with distance, but GLIM ignores message covariance and assumes fixed position sigma 0.05 m. At an approximately 8 m baseline the generator's horizontal sigma is about 0.13 m and vertical sigma about 0.26 m: variances approximately 6.8 and 27 times the fixed assumed variance. The configuration comment still describes sigma 0.025/precision 1600, while the actual value is 0.05/precision 400.

The absolute-factor extension uses nearest timestamp association; a fix can support more than one frame, and delayed measurements are not treated as a general out-of-sequence update to past smoother states. Combined with restamped repeated GT, this obscures measurement independence and freshness. Planar lever-arm handling also omits roll/pitch effects.

The modified CPU odometry adds both a fixed-precision between factor and a fixed-precision absolute prior derived from the same scan match. These are correlated evidence. The scan target is updated before subsequent smoother corrections, leaving another consistency issue to measure. Covariance extraction remains a TODO. These choices may stabilize a demo, but tuning scalar weights alone does not establish statistical consistency.

The new exception handler calls `fallback_smoother()` after a failed solve, but the CPU implementation of that method is empty. Catching an exception does not demonstrate recovery of the estimator state. Fault injection should verify finite output, key consistency, and downstream safe behavior.

**Potential solution:** first fix source validity and timestamps, then log innovations and factor associations. Use a justified measurement covariance, explicit outlier/fix-quality rules, one independent measurement identity, and a tested delayed-update policy. Compare fusion behavior on fixed replays with absolute position, heading, and dropout ablations. Validate CPU smoother recovery explicitly. Avoid further weight escalation as the first response to bad source data.

Sources: [absolute constraints/configuration](/home/jacoblambert/astrobot-lab/astrobot-lab/src/glim_slam_launch/config/glim_astrobot/config_ros.json:37), [CPU factors and empty fallback](/home/jacoblambert/astrobot-lab/astrobot-lab/src/glim/src/glim/odometry/odometry_estimation_cpu.cpp:196), [exception handler](/home/jacoblambert/astrobot-lab/astrobot-lab/src/glim/src/glim/odometry/odometry_estimation_imu.cpp:421).

### F15 — Additional hardening and scope limitations

**P2 unless explicitly noted.**

- **Reproduced memory safety defect:** plane-fit scratch arrays have capacity 256 with no bound check. `ground_window_radius=8` admits up to 289 samples. An AddressSanitizer fixture triggers a stack-buffer overflow at the array write. Default radius two is safe from this specific overflow. Fix dynamic sizing or validate the parameter before window-size tuning. [Array write](/home/jacoblambert/astrobot-lab/astrobot-lab/src/slam_mapping/src/probabilistic_voxel_map.cpp:348)
- **Downsampling knob ambiguity:** a positive `random_downsample_target=20000` takes precedence over `random_downsample_rate=0.1`. Changing the rate alone does not necessarily change preprocessing density. Inspect actual input/output counts at every downsampling stage before attributing losses to a nominal 90% cut. [Preprocess config](/home/jacoblambert/astrobot-lab/astrobot-lab/src/glim_slam_launch/config/glim_astrobot/config_preprocess.json)
- **Estimator capability boundary:** disabled global/submap optimization and synthetic absolute position plus heading do not establish loop-closure or beacon-denied cave capability. Position and heading availability must be an explicit sensor contract, not an implicit property of a generic beacon.
- **Controller optimization remains an experiment:** MPPI samples 2,000 trajectories of 40 states at a 10 Hz configured control rate. Its approximately 4 s horizon spans only 0.72 m at the 0.18 m/s maximum forward speed. This is worth profiling and testing, but inspected recent logs do not show broad controller-deadline failures that establish MPPI as the dominant cause. Correct perception, frames, and recovery ownership first.
- **Build optimization caveat:** the existing mapper build cache is Release with `-O3`. An unoptimized build is not the demonstrated explanation for current mapper costs. Clean-build configuration still needs pinning.
- **UI/command exposure:** the backend has no authentication and accepts navigation goals; compose publishes 8080 and 8765 on all interfaces. Appropriate for a controlled lab only with explicit network boundaries. Bind locally or add authenticated access before broader exposure. [API](/home/jacoblambert/astrobot-lab/astrobot-lab/src/astrobot_ui_backend/astrobot_ui_backend/ui_backend.py:673), [ports](/home/jacoblambert/astrobot-lab/docker-compose.lunaryard.yml:86)
- **Command validation:** the mux clamps commands but does not explicitly reject NaN/non-finite inputs. Its software stop path is not a substitute for testing downstream behavior when the mux/process/clock disappears. [Command mux](/home/jacoblambert/astrobot-lab/astrobot-lab/src/basic_control/basic_control/command_mux.py:148)
- **Documentation drift:** Phase 1 onboarding, current launch defaults, experiment comments, and recent gate artifacts describe different states. Treat executable configuration and timestamped artifacts as authoritative until the project status is consolidated.

## 5. Proposed order of decisions and evaluation

This is a recommended dependency order for the next planning session, not an implementation already undertaken.

| Stage | Narrow objective | Evidence needed before proceeding |
|---|---|---|
| 0. Freeze the baseline | Identify exactly what runs and what success means | Manifest, all-attempt results, fixed scenario/sensor contract, corrected metric definitions |
| 1. Establish valid state | Fix GT/RTK validity, stamp handling, TF ownership, matched-time correction, recovery frame alignment | Stationary/straight/turning fixtures; stale/delayed/bad input injection; non-identity map/odom test; safe invalid-state behavior |
| 2. Make component replay trustworthy | Eliminate frame/bounds/config/drop-selection confounds | Same ordered clouds integrated by every variant; matching live settings; exact counters and measurement ages |
| 3. Repair obstacle evidence | Remove the small-rock support blind spot and unsafe free-space assumptions | Height/range/angle recall, false obstacles on slopes/ground, persistence while approaching/reversing, contact outcomes |
| 4. Repair mission/recovery transitions | Consistent clocks, recovery ownership, return-state completeness, actual home verification | Goal rejection/server-loss/late-result tests; blocked start; all-breadcrumbs-unreachable; RTF-invariant backup; no false completion |
| 5. Optimize measured hot paths | Bounded mapping and reachability work without losing safety | Per-stage p50/p95/p99 latency, scan counts, map age, memory growth, unchanged or improved component accuracy |
| 6. Reassess architecture | Decide whether a richer terrain map or different localization/controller is necessary | Corrected-baseline failures isolated to a representation/algorithm limitation on held-out scenarios |

Some deterministic state-machine and memory-safety fixes can be developed independently once the baseline is archived. Mission comparisons should still introduce one behavior-changing category at a time, preserving the ability to attribute gains and regressions.

### Minimum evaluation matrix

- **Perception fixtures:** flat ground, smooth ramps, 5/8/10/15/20/25 cm rocks, sparse/grazing returns, near-field disappearance, reverse/reobservation, negative obstacles, and capacity pressure. Report observable versus unobservable obstacle cases separately.
- **Localization fixtures:** stationary, straight, turning, pitched/rolled motion, feature-poor regions, delayed/stale/outlier fixes, heading-only/position-only, and sustained absolute-sensor dropout. Record reference integrity and estimator health separately.
- **Navigation fixtures:** straight approach, narrow corridor, non-identity map/odom, obstacle near footprint corner, wedged start, interrupted backup, projected home, rejected return, and out-of-order action responses.
- **System tests:** fixed seeds with multiple predetermined repetitions, plus held-out seeds. Archive every attempt; do not select the best retry. Use scenario-specific success, physical home distance, fixed-denominator coverage, contact/stall events, executed distance, simulation duration, wall duration, and localization/map age.
- **Performance tests:** compare early and late mission windows. Record received/integrated/dropped clouds, per-stage wall runtime, simulated sensor rate, controller deadlines, CPU/GPU utilization, and memory. Repeat at different RTFs without changing logical outcomes.

Suggested acceptance invariants are stronger than a single aggregate score: no completion outside home tolerance; no fresh-looking state from stale measurements; no recovery pose checked in another frame; no supported obstacle fixture labeled free solely by the support-count gate; no unexplained integrated-frame mismatch between replay variants; and no failure reason accompanying a passing verdict.

### Decisions that remain open

1. Is the immediate target a beacon-assisted surface demonstrator, beacon-denied exploration, or both as distinct tracks?
2. What physically defines coverage: rover visitation, sensor observation, resource survey, or traversable-area discovery?
3. Which rock heights, slopes, drop-offs, and clearances must be treated as non-traversable for this rover and gravity/traction model?
4. Is real-time operation a hard requirement on this laptop, or is slower simulation acceptable if logic and evaluation are simulation-time invariant?

These choices materially affect the next architecture decision. They need not block repairing the confirmed correctness defects.

## 6. Verification performed and limitations

Performed without launching a mission or commanding motion:

- Read project status/history, active launch/configuration, custom ROS packages, GLIM modifications/extensions, simulator GT/sensor/robot paths, UI/backend paths, and evaluation harnesses; cross-checked recent successful and failed mission artifacts.
- Compiled and ran the existing probabilistic voxel-map tests against the current core: passed.
- Compiled direct core fixtures with AddressSanitizer/UndefinedBehaviorSanitizer: reproduced small-rock exclusion, unsupported free-space projection, capacity admission loss, three-hit confirmation, and the large-ground-window stack overflow.
- Executed actual exploration/evaluator methods extracted with Python AST and stubbed ROS/action objects: reproduced coverage inflation, return completion away from home, rejection dead-end, and inconsistent gate verdict. Timed the actual reachability routine.
- Recomputed the six latest planar localization-error summaries and checked reference zero samples, mapper integration timings, and simulation/wall-time ratios.
- Parsed 42 local-package Python files, active GLIM JSON configurations, and the recovery XML; TypeScript `tsc --noEmit --incremental false` passed.
- Inspected the dev image read-only through an ephemeral network-disabled container; identified installed Nav2 1.3.11 and cross-checked the relevant tagged upstream source. Inspected the active USD offline using a temporary dependency installation.

Diagnostic probes remain in `/tmp/astrobot-review-io5fvu/`; production files and existing user edits were not changed. This report is the only repository artifact added by the review.

Not performed: a new ROS/Isaac integration mission, full GLIM rebuild, GPU/CPU end-to-end trace, physical collision/contact validation, exhaustive third-party audit, or hardware acceptance test. No simulation/dev container was running during this review; an unrelated database container was left untouched. Archived runs do not prove that every current dirty source file was present in their running binaries.

The review therefore establishes multiple concrete defects and a well-supported next evaluation direction. It does not assign an exact percentage of mission failures to each defect or claim a replacement architecture has already been validated.
