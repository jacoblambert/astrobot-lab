# CLAUDE.md — Astrobot Lab (single-robot lunar rover autonomy)

> **STALE (snapshot of 2026-06-18 state; flagged 2026-10-19).** Sept–Oct work changed GLIM core,
> association (`assoc_tolerance` 0.05), the LiDAR mount, exploration return-home and added the
> localization supervisor + layered terrain. Current state: `docs/implementation/implementation-plan.md`
> ("October 18 handover"). This file will be rewritten after the terrain robustness audit.

Transfer/onboarding doc. Self-contained: assume no prior workspace memory. Read this first.

## What this is

Single-robot lunar-rover autonomy in simulation: **OmniLRS / NVIDIA Isaac Sim** (lunar yard) + **ROS 2
Jazzy**. Stack = **GLIM LiDAR-IMU SLAM** → **BEV occupancy/cost mapping** → **Nav2** → **frontier /
resource exploration manager**, plus a web **Ops console** for monitoring + teleop-click goals. Goal is
to harden autonomy against the lunar failure modes (feature-poor SLAM divergence, rock fields, terrain).

- Active branch: **`dev/phase-5`**. Main: `main`. Commits: the user manages commits — **do not commit
  unless asked**. End commit messages with the Co-Authored-By trailer; PR bodies with the Generated-with line.
- Robot namespace: **`astrobot_0`**. World/sim arena: **lunaryard_40m** (40×40 m), spawn ≈ **(20, 18)**.

## Two-container architecture (docker compose)

`docker-compose.lunaryard.yml` defines two services:
- **`astrobot-sim`** (`isaac-sim-omnilrs:latest`): Isaac Sim + OmniLRS. Publishes `/astrobot_0/lidar_0/
  pointcloud/{raw,filtered}`, `/astrobot_0/imu`, `/astrobot_0/odom`, ground truth `/gt/base_link_pose`
  (PoseStamped, frame `gt_map`, 15 Hz), `/gt/map` + `/gt/resource_maps/*` (OccupancyGrid). Robot TF
  (odom→base_link→sensors). Subscribes `/astrobot_0/cmd_vel`. Slow to boot (~2–3 min). Key env:
  - `OMNILRS_ENVIRONMENT=lunaryard_40m`, `OMNILRS_ENVIRONMENT_SEED=42` (also 411, 7 used in gates)
  - `OMNILRS_HEADLESS=true` (set false + DISPLAY for the Isaac GUI)
  - `OMNILRS_ROCKS_ENABLED=true` (rocks; in `OmniLRS/cfg/environment/lunaryard_40m.yaml`: ThomasCluster
    density `lambda_parent` **0.015** / `lambda_daughter` 2 / `sigma` 3.0; `req_scale.xmin/xmax` **8/11** —
    rock size, raised from 7 so marginal rocks stand prouder of terrain mounds. yaml is mounted → fresh sim.)
  - **`OMNILRS_GT_TF_ENABLED` MUST be `false` during nav** (else gt_map→odom TF collides with SLAM's
    map→odom → split TF tree → nav breaks). Default true — always override false for any nav/SLAM run.
- **`astrobot-dev`** (`astrobot-dev:latest`): the ROS 2 stack + tooling. Workspace mounted at
  `/workspace/astrobot-lab` (host repo root). Entry point auto-builds the overlay (`ASTROBOT_AUTO_BUILD=
  true`). Ports **8765** (foxglove_bridge ws) and **8080** (Ops console backend) bound on all interfaces.

Bring-up / shells:
```bash
OMNILRS_ROCKS_ENABLED=true OMNILRS_GT_TF_ENABLED=false docker compose -f docker-compose.lunaryard.yml up -d
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash      # interactive shell
docker compose -f docker-compose.lunaryard.yml down                        # ALWAYS tear down after a run
# inside dev, always: source /etc/ros_setup.sh && source /workspace/astrobot-lab/astrobot-lab/install/setup.bash
```

## Repo layout

- `astrobot-lab/src/` — ROS 2 packages:
  - `astrobot_launch` — all launch files (see below) + `config/nav2_phase3_bev.yaml`.
  - `glim`, `glim_ros2` — **git submodules**, our forks (Open-Space-Systems/*). GLIM SLAM core + ROS wrapper.
  - `glim_slam_launch` — GLIM config (`config/glim_astrobot/`) + `glim_live.launch.py`.
  - `slam_mapping` — `probabilistic_voxel_mapper` (BEV cost map, C++) + `slam_map_to_odom_tf_node`.
  - `exploration_manager` — frontier/resource/teleop_click mission manager (Python).
  - `synthetic_rtk` — sim RTK source (see `synthetic_rtk/README.md`).
  - `astrobot_ui_backend` — Ops console backend (ROS↔web bridge, port 8080).
  - `astrobot_diagnostics` — GT-scored health/diagnostics node (publishes `/astrobot_0/diagnostics`).
  - `slam_eval` — TUM recorders, trajectory/ATE/loop-closure/BEV comparison tools.
  - `basic_control` (command_mux), `pointcloud_preprocessor`.
- `OmniLRS/` — sim submodule (env configs in `cfg/`).
- `ui/astrobot-console/` — web Ops mission-control frontend (Vite/React; `npm run dev`).
- `tools/` — eval/replay/diagnostic scripts (gate harness, RTK replay A/B, divergence study, etc.).
- `PHASE_DEFINITIONS.md` (phase plan + current Phase-4 status), `README.md`, `useful_cmds.md`
  (runnable command reference + gotchas), `MISSION.md`, `IMPLEMENTATION.md`.

## Launch chain

Includes nest: `phase2_slam` ⊂ `phase3_nav` ⊂ `phase4_exploration`.
- **`phase2_slam.launch.py`** — GLIM (`glim_live`) + `probabilistic_voxel_mapper` (BEV) + `synthetic_rtk`
  (`start_synthetic_rtk:=true` default). Does NOT launch `slam_map_to_odom_tf_node` (nav layer does).
- **`phase3_nav.launch.py`** — phase2_slam + Nav2 (controller_server[MPPI]/planner_server/bt_navigator/
  behavior_server) + `command_mux` + `slam_map_to_odom_tf_node` (publishes map→odom).
- **`phase4_exploration.launch.py`** — phase3_nav + `exploration_manager` (+ `astrobot_diagnostics` if
  `start_diagnostics:=true`). Arg **`mission_mode`** ∈ {`explore_radius`, `sample_return`, `teleop_click`}.
- **`phase5_gui.launch.py`** — Ops console backend on `:8080` (`host:=0.0.0.0 port:=8080`).
- foxglove: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765`.

### Common sessions (run inside dev, sourced)
```bash
# Teleop-click ops session (GLIM+Nav2+console, operator clicks goals; GT-free). NOT a lean glim_live.
ros2 launch astrobot_launch phase4_exploration.launch.py mission_mode:=teleop_click start_diagnostics:=true
ros2 launch astrobot_launch phase5_gui.launch.py host:=0.0.0.0 port:=8080
#   then on host:  cd ui/astrobot-console && npm run dev   -> click goals on the BEV map.
#   backend POST /api/robots/astrobot_0/teleop_goal -> /astrobot_0/teleop/goal (accepted only in teleop_click)

# Autonomous gate run (1 trial/env), self-managing bring-up/teardown, GT-scored verdict + snapshots:
tools/phase4_gate_suite.sh --retries 0                 # seeds 42/411/7 x explore_radius/sample_return
tools/phase4_mission_gate_run.sh 42 explore_radius --retries 0   # single (seed,mode)
```

## Build model (IMPORTANT — GLIM is special)

- The dev entry point runs `colcon build --symlink-install` on the workspace at start, but **ignores
  `glim` + `glim_ros`** by default (`ASTROBOT_BUILD_SOURCE_GLIM=false`) — GLIM is prebuilt into the dev
  **image** from the submodules (`docker/dev.Dockerfile` COPYs them to `/opt/astrobot/glim_source`).
- To iterate on the GLIM forks **without an image rebuild**, build them in the overlay:
  ```bash
  colcon build --symlink-install --allow-overriding glim glim_ros --packages-select glim glim_ros \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=ON -DBUILD_WITH_VIEWER=OFF \
    -DBUILD_WITH_OPENCV=ON -DBUILD_WITH_MARCH_NATIVE=OFF "-DCMAKE_PREFIX_PATH=/opt/ros/jazzy;/usr/local"
  ```
  (glim core first build ~minutes; glim_ros ~15 s.) Workspace packages (slam_mapping, etc.) build normally.
- Launch files / Python nodes / JSON configs are `--symlink-install`ed → edits take effect on node restart,
  no rebuild. GLIM reads its JSON config at node startup.

## SLAM = GLIM (CPU), config `glim_slam_launch/config/glim_astrobot/`

- **CPU odometry** (`libodometry_estimation_cpu.so`, `config_odometry_cpu.json`) — switched off GPU because
  GPU contention with Isaac Sim caused freezes/divergence (GPU is Isaac-only now).
- **Lunar gravity** `imu_gravity_magnitude: 1.62` in `config_sensors.json` (was THE whack-a-mole root cause
  vs GLIM's old hardcoded Earth 9.81).
- **Global loop closure is DISABLED** (commit c5f2da6) — it caused catastrophic divergence. RTK is the
  global anchor in its place.
- GLIM `pose_corrected` is the **IMU frame**: offset `(0.264, 0.017)` + yaw `π` from base_link
  (`slam_map_to_odom_tf_node` / `tools/map_gt_alignment.py` convert it).
- The CPU path injects the scan match into the smoother as a **hardcoded `Isotropic::Precision(6, 1e3)`**
  prior (it discards the GICP match covariance — there's a literal TODO). This is the GICP-overconfidence
  lever: to make an absolute source (RTK) dominate the scan per-frame you lower this, not the RTK gate.

## RTK / absolute-position fusion (VALIDATED, on by default)

Bounds SLAM drift + arrests catastrophic feature-poor GICP divergence. **Full design + the precise
"full authority" semantics + caveats: `astrobot-lab/src/synthetic_rtk/README.md` (READ IT).** Summary:
- `synthetic_rtk` (sim RTK from `/gt/base_link_pose`, 30 Hz, home-relative, noise+dropout) → `/rtk/pose`
  → `abs_constraint_module` (glim_ros2, additive `ExtensionModuleROS2`) → position `GPSFactor` + yaw
  `PriorFactor` on the live pose key. Needs `init_T_world_imu` frame pinning. GLIM core is **stock**.
- "Full authority" (`gate_dist=0`, `huber_k=0`) = RTK is **never gated/robust-rejected**, NOT that it
  outweighs the scan (scan prior 1e3 > RTK ~400 per-frame). It works by accumulated absolute anchors.
- Validated: catastrophic divergence **9480 m → ~1 m**; long-drive drift **bounded ~0.3 m**. Bug fixed:
  RTK starved out of the graph at `assoc_tolerance=0.05` → set **0.2 s** + 30 Hz republish.
- Hardware caveat: `gate/huber=0` has no bad-fix protection → add a kinematic RTK-to-RTK gate before a
  real receiver. On hardware, replace `synthetic_rtk` with the real beacon driver (same `/rtk/pose`).

## Current status (2026-06-18)

- Phases 1–4 substantially working. **SLAM is solved** for the divergence failure mode (RTK fusion on by
  default, 0 `slam_diverged`). **Gates: sample_return 3/3** (seeds 42/411/7), **explore_radius 2/3**
  (stochastic boundary — the failing seed varies run-to-run; coverage went from 0/3 to 2/3 this round).
- **The old #1 blocker (`gt_motion_stalled` "stuck on a rock") is SUBSTANTIALLY FIXED.** It was NOT a
  mid-path terrain bog in clear space; via bag/visual diagnosis it is the rover **wedging on rocks the BEV
  under-detected** — specifically *marginal* rocks sitting on **real gentle terrain mounds** (only ~0.14 m
  proud of their local ground, right at the detection gate AND the rover's ~0.11 m clearance). Fix:
  **`obstacle_height` 0.11→0.08** (measured against the local plane-fit ground, slope-detrended; GT-validated
  it does NOT add false obstacles) + **rock-size floor `scale.xmin` 7→8** in `OmniLRS/cfg/environment/
  lunaryard_40m.yaml` (rocks stand prouder of the mound + gate). Result: seed-7 explore nav2_failure 26→1,
  coverage 0.33→0.71; no over-marking (reachable stays healthy).
- **Nav hardened (this round):** unified inflation — BEV mapper `obstacle_inflation_radius`→**0** (detection
  only; rock cores stay cost-100), Nav2 `inflation_radius`→**0.75** (MUST be ≥ the **circumscribed radius
  0.737** — 0.60 floods `computeCircumscribedCost` and chokes the controller → return stalls),
  `footprint_padding`→0.10; MPPI `vx_min`→−0.05, `batch_size`→2000. **Precheck anti-trap:** distance-scaled
  `path_unknown_fraction` tolerance (near goals permissive, far strict) + a force-move backstop — killed the
  near-spawn `no_candidates` trap. **Stage-3 coverage ordering:** explore_radius sorts mission candidates
  nearest-unvisited (lawnmower sweep) instead of by global score.
- **Tried + REFUTED:** R-GPF (Patchwork lowest-seed ground fit) for the rock detection — marks 0/14 of the
  wedge-rock footprint at every window radius because the rock's local ground is a real elevated apron, so
  the fit anchors to the apron. Code kept param-gated (`use_gpf_ground`, **default false**). Only a *wide*
  low-floor reaches true ground (14/14) but it over-marks smooth curvature catastrophically (`use_ground_floor`
  also gated off). The rock-vs-curvature detection tradeoff is fundamental for these marginal rocks.
- **All of the above is UNCOMMITTED on `dev/phase-5`** (the user manages commits). New eval/diagnostic tools
  in `tools/`: `gt_isolated_eval.py`, `analyze_wedge_hits.py`, `diagnose_spawn_obstacles.py`,
  `spawn_rock_audit.py`, `_rgpf_ab.sh`, `explore_validate.sh`, `stage_recheck.sh`.

## Open issues / fix menu

1. **Return-home robustness (the new #1 blocker)** — the dominant explore_radius gate failure on seeds that
   cover well is now `return_failed` (rover hits coverage but can't get home ≤1.0 m); also the marginal
   axis on sample_return (home hovers near 1.0 m). Harden the breadcrumb / direct-return.
2. **Marginal small-rock detection** — rocks ~0.14 m proud on mounds, near the gate + rover clearance. R-GPF
   refuted (apron); wide low-floor over-marks curvature. The practical lever is the rock-size floor
   (`lunaryard_40m.yaml` scale) — but a floor of 9 over-walls seed-411's tight spawn (real rocks seal it,
   reachable→18/52 cells), so 8 is the compromise. Tradeoff, not solved.
3. **explore_radius stochastic 2/3** — the failing seed varies with the rock floor; tied to #1 + coverage
   variance, not a single deterministic bug.
4. **GT rock map is an INCOMPLETE eval reference** — `gt_map_ros2.py` filters rocks by `obstacle_z_min`
   (default 0.05 m, absolute world z) + footprint, so small/low rocks (and rocks in dips) are dropped from
   `/gt/map`. A BEV cell correctly marking a GT-excluded rock scores as a *false positive* → GT-based
   recall/precision is biased low. Pass a lower/terrain-relative `obstacle_z_min` for honest eval.
5. **resource_found trigger latency** — sample threshold checked only between goals (exploration_manager).

## Testing & eval

- **Gate harness**: `tools/phase4_mission_gate_run.sh <seed> <mode> [--retries N]` and
  `tools/phase4_gate_suite.sh` — fresh bring-up→GT-scored verdict→teardown per (seed,mode). Records
  `eval.json` + `.npz` diagnostic snapshots (e.g. `gt_motion_stalled_while_commanded`) under
  `rosbags/phase4_gate_runs/`. Gate thresholds: explore_radius coverage ≥0.70; sample_return best_sample
  ≥0.75 + home ≤1 m. `gt_motion_stalled` def (`astrobot_diagnostics`): `cmd_linear>0.08 AND gt_speed<0.03`.
- **Offline RTK/SLAM replay A/B (no sim)**: `tools/rtk_replay_ab.sh <bag_dir> <config_path> <name>` —
  replays raw LiDAR+IMU+GT through GLIM + synthetic_rtk, scores vs GT. Fixture:
  `rosbags/seed411_sparse_explore` (a reproducible catastrophic feature-poor divergence — sparse rocks
  `lambda_parent 0.003`). Toggle RTK via `abs_constraint.enable`.
- **Divergence study**: `tools/slam_divergence_study.sh` + `slam_divergence_fingerprint.py`.
- **BEV eval**: `tools/bev_gt_eval.py` (live recall/precision), `tools/bev_bag_score.py` (offline). CAVEAT:
  both align BEV→GT from SLAM-pose pairs (breaks when SLAM drifts — the planefit fixtures drift ~2 m → the
  fit fails) AND score against the **incomplete** `/gt/map` (see open issue #4). For a trustworthy, drift-free
  number use **`tools/gt_isolated_eval.py`** — it RE-ANCHORS each scan into the GT frame with the GT pose
  (`Δ = GT_base ∘ SLAM_base⁻¹`), drives the real mapper, and scores per-cell with identity alignment +
  recall-by-rock-height. `tools/analyze_wedge_hits.py` (radial ground profile + ground-method sweep on a
  recorded wedge) and `tools/diagnose_spawn_obstacles.py` (spawn BEV-vs-GT real-vs-false + reachable) are the
  rock-detection diagnostics.

## CRITICAL gotchas (read before touching the sim/stack — each cost real debugging time)

- **`use_point_instancer` MUST stay `False`** (`lunaryard_40m.yaml`) — point-instanced rocks are INVISIBLE
  to the RTX LiDAR (a "rocks-on" run with this True is silently bare terrain).
- **`OMNILRS_GT_TF_ENABLED=false` for any nav/SLAM run** (gt_map→odom vs map→odom split tree).
- **`slam_map_to_odom_tf_node` must be running** (publishes map→odom). A lean `glim_live` alone does NOT
  → map disconnected from base_link → broken TF / dead nav / blank console. The nav launches include it.
- **Don't restart GLIM mid-session without restarting the sim**: odom keeps accumulating while GLIM
  re-inits → large stale `map→odom` offset → the exploration `_slam_diverged` watchdog (reads that offset,
  threshold 2 m) **false-trips**. The watchdog is only valid from a fresh sim+GLIM start.
- **Topic types**: `slam/odom_corrected` = `nav_msgs/Odometry`; `slam/pose_corrected` = `PoseStamped`. A
  wrong-typed rclpy subscription silently receives NOTHING (`ros2 echo/hz` still work — type-agnostic).
- **`kill -INT` on a non-interactive shell's background job is a silent no-op** → use SIGTERM/`pkill -INT
  -f` the actual process, or `ros2 bag record` never finalizes (no `metadata.yaml`).
- **Observer effect**: never add recorder processes to a *behavior measurement* run — extra CPU load
  amplifies SLAM drift (3 light recorders turned 2.5 m drift into 8–23 m). Record inside the eval node, or
  only for fixture capture.
- **`behavior_server` needs `remappings=[("cmd_vel","cmd_vel_raw")]`** (phase1_nav) or recoveries fight the
  command_mux on the final cmd_vel and silently do nothing.
- **teleop_click is a mission mode**, not a lean setup: `phase4_exploration mission_mode:=teleop_click` +
  console — NOT `glim_live` + direct cmd_vel.
- **GT-score everything** — mission self-reports are untrustworthy under drift (false successes are real).
  Validate scene *content* visually, not metrics alone. Flip sim-fidelity switches only after checking
  Phase-2 notes. `docker compose down` after every run.

## Workflow preferences

- Cheapest/safest fix first; later stages conditional; skip to validation if an early fix resolves it.
- Short confirmation runs before any long/gate run; always bring up foxglove + Ops console; tear down after.
- The user manages commits — do not commit unless explicitly asked.
