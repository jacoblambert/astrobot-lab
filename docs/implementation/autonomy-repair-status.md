# Autonomy repair implementation status

Approved plan: conversation dated 2026-09-07; review: ../reviews/2026-09-06-system-review.md.

The starting tree contains extensive user changes in the root and submodules. Preserve them; do not reset, stage, or commit without instruction.

## Acceptance contract

- Assisted position/heading plus outage tests; bounded automatic SLAM recovery.
- Stop before restart, retain world/home frame and mission history, clear navigation evidence.
- Obstacles >=8 cm lethal; 5–8 cm soft risk; unsupported terrain unknown.
- Global home goal, no automatic breadcrumb fallback.
- Deterministic fixtures, short simulation courses, then seeds 42/411/7 for 20 simulation minutes each.
- Safe fault containment does not count as spontaneous-failure-free SLAM.

## Progress

- Review complete. Implementation now exists in the working tree; preserve it across interruptions.
- Batch A: fixed-reference coverage and verdicts, simulation-time evaluation budget, all-attempt failure semantics, provenance writer, and regression fixtures implemented. Shared-config step-and-ack mapper replay and full-SE(3) measurement-time GT isolation are validated for repeatability; obstacle recall scoring remains. The archived receipt-time/yaw-only evaluator now refuses gate use by default.
- Batch B: coherent physics-thread GT snapshots, validated synthetic positioning, covariance-aware/one-fix-per-key absolute constraints, same-time map/odom correction, bounded SLAM process supervision, epoch map reset, output gating, and fail-closed command mux implemented. Stationary startup and intentional estimator-exit recovery passed in the simulator. More outage/moving-fault tests remain.
- Batch C: actual surface extrema (not vertical voxel counts), temporal confirmation, deduplicated misses with hit priority, elapsed-time decay, unknown-without-ground semantics, checked dynamic plane-fit storage, and shared live mapper YAML implemented. Small-object/grid-offset/slope/noise/expired-evidence core fixtures pass. Capacity exhaustion latches a terminal motion hold; stale maps also hold localization. Bounded obstacle-only navigation memory is implemented and fixture-tested but staged OFF pending approach/live-cadence validation. A bounded-memory voxel admission/eviction policy remains.
- Batch D: global home goals (no breadcrumb execution), actual-home completion checks, stale-action generation guards, Nav2-owned bounded backup/replan recovery, matched behavior/costmap frame, fixed mission coverage denominator, and cached connected-component reachability implemented. Simulator motion/course validation remains blocked by unsupported spawn terrain described below.
- Three 20-simulation-minute seed soak runs: NOT RUN. This is not a validated mission baseline yet.

## Validation on 2026-09-07

- Full isolated ROS build passed, including modified `glim` and `glim_ros`; build/install/log under `/tmp/repair-{build,install,log}` inside task-owned `astrobot-repair-dev`.
- **44 Python tests passed**, covering core integrity, actual supervisor callbacks, command mux, mission actions, reachability, evaluator simulation-time/frame validation, and SE(3) alignment/reset-barrier contracts.
- **Six CTest programs passed**: `test_probabilistic_voxel_map`, `test_pose_history`, `test_surface_evidence`, `test_obstacle_memory`, `test_pointcloud_layout`, and `test_absolute_association`. Surface and obstacle-memory fixtures also passed under address/undefined-behavior sanitizers on host.
- First stationary integration attempt correctly held startup on stale GT. Root cause was our new snapshot timestamp using `World.current_time`, which resets during warm-up and lagged the ROS sensor clock by 1.783 s. Changed snapshot timestamp to Isaac core's monotonic simulation sensor clock.
- Corrected stationary run (45 wall seconds): 611 GT poses, 391 accepted SLAM poses, 891 commands, all zero; healthy epoch 1; GT displacement 0.00014 m; GT/SLAM age at most ~0.0167 s.
- Intentional termination of the owned GLIM child: STOPPING -> epoch 2 reset -> REINITIALIZING -> REBUILDING -> HEALTHY. All 900 observed commands zero; GT displacement 0.00033 m. This proves stationary fault containment/recovery, not fault-free localization under motion.
- Intentional 8-wall-second positioning-source pause: DEGRADED -> STOPPING (absolute_sensor_outage) -> epoch 3 reset/rebuild -> HEALTHY. All 900 commands zero; GT displacement 0.00030 m. No claim of moving/outage accuracy is made.
- Latest rebuilt stack stationary check passed again (45 wall seconds): 613 GT poses, 373 SLAM poses, 894 commands all zero; maximum pose age ~0.0167 s. TF preflight confirmed `map -> odom -> base_link` and no GT frame leak.
- Two isolated-domain replays each acknowledged all 184 clouds. **Every per-frame map hash matched**, final SHA256 `e81d8939f8f6b034ad3a6f399d5f69196c0a1b44782266eef5710e44054620c6`. Measured p95 step-and-ack latency was 66–68 ms, including projection and ROS transport. This is mapper repeatability, not GT recall or full live-history equality.
- Latest mapper with obstacle memory disabled matched all 184 original baseline frame hashes again. Three GT-isolated replays matched all 184 frame hashes with no excluded boundary scans; final SHA256 `19d94ad659f45847e01f24d1d20988d6ac07155ca6aa7811db016e3883356b45`. These use full translation/rotation, base-to-IMU lever arm, quaternion SLERP, and corrected LiDAR ray origin. Replay rejects extrapolation, reference gaps/jumps, duplicate pose stamps, and multiple recorded map epochs; missing health evidence is explicitly reported. This does not yet measure obstacle recall.
- Evaluator smoke check stopped at 5.000000261 simulation seconds / 10.764 wall seconds, with valid reference input and the expected unsuccessful verdict because no mission was started.
- Mapper reported no dropped clouds during these stationary runs and approximately 22–30 ms integration at ~100k voxels. This is not a long-run throughput result.
- Artifacts: `rosbags/autonomy_repair_20260907/` (ignored runtime outputs). Includes summaries, MCAP fixture, replay grids/hashes, source/config/binary manifests, and copied launch logs.

## Latest continuation changes

- Absolute association now retains keys even when no positioning fix is buffered, prunes using actual surviving smoother keys rather than a hardcoded lag, and selects deterministic closest-time one-to-one matches. This is a greedy nearest-first association, not a globally optimal assignment. Delayed existing-key updates now use the smoother pose for the same innovation gate as newly inserted keys. Timestamp/retention regressions pass; the full moving delayed-factor integration test remains outstanding.
- Association rebuild stationary probe passed: 611 GT poses, 301 accepted SLAM poses, 900 zero commands, maximum pose age ~0.0167 s, displacement 0.00032 m, healthy epoch 1.
- Final rebuild including delayed-key innovation gating passed another 45-wall-second probe: 615 GT poses, 368 accepted SLAM poses, 892 zero commands, displacement 0.00028 m, healthy epoch 1, maximum pose age ~0.0167 s. It still has zero clearance-safe cells; stationary success is not mission readiness. Summary and running-binary manifest: `stationary_continuation_final.json`, `continuation_final_manifest.json` in the artifact directory.
- Added checked PointCloud2 field/stride/data bounds and bounded finite BEV dimensions. Malformed clouds are rejected before unsafe allocation/read.
- Added navigation-only obstacle memory carrying actual elevated-return timestamps, not publication or subsequent ground-hit timestamps. The regression reproduces rays just above an 8 cm rock clearing its 10 cm voxel; navigation retains the confirmed obstacle briefly, then expires it. Tests cover reset, radius limits, no invented ground, no soft-cost-to-lethal promotion, and ground hits not rejuvenating a rock. `obstacle_memory_radius: 0.0` deliberately keeps this experiment disabled in the baseline. Replay projects each frame; live projection cadence must be tested before enabling it.
- Mission budget/resource checks now preempt an active navigation goal through cancellation before sending the home goal. Repeated terminal-failure events are suppressed.

## Newly exposed integration constraint

`spawn_support.json` shows **zero footprint-clearance-safe cells anywhere in the observed map**, not merely an inconvenient start cell. The local 3 m window contains a substantial self-filter/sensor blind region plus sparse ground returns. Under the corrected rule that an air ray does not prove ground support, the current mapper has ~3,300 free cells but none forms a fully observed 0.75 m-radius disk. Snapping to a nearby cell or marking the blind region free would hide this, not resolve it.

Keep autonomous motion disabled until a measured-ground support/coverage route is implemented and tested. Candidate routes: bounded interpolation between consistent observed ground (small gaps only), downward/near-field depth coverage, or a separately justified contact-supported starting footprint with collision-checked egress. The conservative circular reachability filter is not the exact oriented Nav2 footprint and should eventually share geometry/configuration directly with Nav2.

## Execution environment notes

- Task-owned dev container uses host networking. DDS interface override for the current simulator bridge: `br-f221da6afe07` (discover bridge again after network recreation; do not hardcode in production).
- Simulator is task-started `astrobot-sim-lunaryard`, seed 42, rocks enabled, GT TF disabled, camera streams disabled for these stationary checks.
- Existing unrelated postgres container was not touched.
- The long-lived dev container initially reported `NVML: Unknown Error`; restarting that task-owned container restored GPU access. The final stationary check ran after this refresh with no CUDA warning flood. Earlier CPU-only runs should not characterize GPU performance.
- Unchanged pointcloud preprocessing, diagnostics, UI, and slam_eval were inherited from the existing workspace install. The isolated overlay overrides repaired packages; manifests record actual runtime paths.
- At handoff, the owned Phase 3 launch and simulator are stopped. `astrobot-repair-dev` retains the isolated build/install and logs; source `/etc/ros_setup.sh` **then** `/tmp/repair-install/setup.bash` when resuming. No autonomous goals were sent during these checks.

Follow-on experiments and promotion gates: [autonomy-improvement-routes.md](autonomy-improvement-routes.md).

Architecture experiments are gated on a corrected, validated baseline. Do not report this entire plan complete while simulator gates or recovery integration remain pending.
