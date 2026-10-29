# Detailed repository runbook

## October 18 handover resume checklist

Read the [current handover](implementation-plan.md#october-18-handover--start-here)
before using historical commands below. The latest baseline is v13 route
commitment with asynchronous post-motion recording finalization. It is **not a
navigation pass**. The first development task is the partial-cell safe-footprint
boundary issue, not another LiDAR mounting study. October 18 was documentation
and read-only inspection only; the commands below were not rerun then.

### Restore the environment before capture or replay

The running `astrobot-selected-dev` has a stale archive bind as of this handover:
host device ID 2065 versus container 2049. Matching pathname text in Docker's
mount listing is insufficient. Confirm no collector, mission, command publisher
or replay owns the container, then recreate the dev container using the existing
base + archive compose setup (the no-port service recipe is preserved below).
Do not repair host hardware, modify archived device contracts, or bypass checks.
Re-source/rebuild the required overlay if needed; preserve all workspace changes.

```bash
# Host: inspect, do not assume yesterday's mounted device or GPU state.
docker ps --format '{{.Names}} {{.Status}}'
findmnt -T /media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab
df -h /media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab
stat -c '%d %n' /media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab
docker exec astrobot-selected-dev stat -c '%d %n' /archive
# Require matching device IDs AFTER recreating the stale bind.
nvidia-smi
python3 docker/check_cuda_runtime.py
docker exec -u1000:1000 astrobot-selected-dev python3 /workspace/astrobot-lab/docker/check_cuda_runtime.py
```

The last working offline dev image ID was
`sha256:745b95aebd391cdb23c847c60583686c86a690c68eaa97ef4b6b6a3d67278fe3`.
Do not assume it remains installed. For exact offline replay, use its isolated
same-image envelope described below, with both `/archive` and the original
absolute host archive path bound to the **current** drive, network disabled,
UID/GID 1000, `OMP_NUM_THREADS=4`, `OPENBLAS_NUM_THREADS=1`. Verify external
Shapely staging `/archive/contact-deps-OXpt89ix` exists and retain the sourced
ROS `PYTHONPATH` when prepending it. Never inherit the currently stale bind.

### Focused regression entry point

```bash
# Host only. No ROS or simulator; this is not live navigation acceptance.
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 \
uv run --offline --no-project --with shapely==2.1.2 --with numpy==1.26.4 \
  --with scipy==1.11.4 --with pytest --with toml==0.10.2 --with pyyaml \
  python -m pytest -q -p no:cacheprovider \
  tools/test/test_turn_space_policy.py tools/test/test_route_commitment.py \
  tools/test/test_route_observation.py tools/test/test_costmap_scenarios.py \
  tools/test/test_costmap_trial.py tools/test/test_local_support_sweep.py \
  tools/test/test_capture_seal.py tools/test/test_natural_holdout.py
```

Add boundary-specific fixtures before the fix: use the latest saved refusal
grid plus independent synthetic grid phases and rock/outside-prior controls.
Then run the broader contact, route, storage and shutdown regressions. Isolated
ROS tests run in the sourced overlay with domain 179 and loopback, not live 177:
`test_lease_process_ros.py`, `test_lease_callbacks_ros.py`,
`test_capture_polling_ros.py`, `test_shadow_capture_ros.py` under `tools/test/`.
The October 9 counts (626 host / 54 ROS) are historical results, not a promise
that the smaller focused command above runs 626 tests.

### Fresh same-layout live comparison

Only after the bind, GPU, capacity and regression checks pass. Start the sole
recorder in one terminal; await recorder readiness before starting Isaac in
another. Use a fresh run, never resume a failed recording. Keep runtime source
unchanged through capture and applicable exact replay.

```bash
docker exec -u1000:1000 astrobot-selected-dev bash -lc '
source /etc/ros_setup.sh &&
source /workspace/astrobot-lab/astrobot-lab/install/setup.bash &&
cd /workspace/astrobot-lab &&
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 \
  ASTROBOT_SELECTED_LIDAR_PROFILE=selected_optical_v2 \
  PYTHONPATH=/archive/contact-deps-OXpt89ix:$PYTHONPATH &&
python3 tools/run_shadow_capture.py \
  --config /archive/revision-4c7b70a4acb44a07acb1bca0d63e3692/config \
  --archive-root /archive --require-physics-imu \
  --natural-motion costmap_detour8 --selected-lidar --terrain-v2 \
  --terrain-library /archive/revision-70647a2b742c446eb61bd3d7141ea011/liblocal_terrain.so \
  --contact-advisory --contact-startup-turn-area --contact-current-turn-area \
  --wall-seconds 3600'

# Separate host terminal, only after recorder readiness:
DEV_USER=jacoblambert ASTRO_UID=1000 ASTRO_GID=1000 \
SELECTED_EXECUTION_PROFILE=default \
SELECTED_DEMO_ROCK_PROFILE=single_40x60_detour_v1 \
SELECTED_ISAAC_EXPERIENCE=lean_v1 SELECTED_IMPORT_STAT_CACHE=false \
bash docker/run_selected_headless.sh start costmap_detour8
```

This invokes the policy in the current checkout. A boundary-policy change needs
its own declared version and fresh evidence, not relabeling the old v13 report.
Follow recorder/native/mapper IDs printed in this run. Do not run compression,
heavy diagnostics or replay beside capture. Stop the owned simulator after the
recorder reaches a terminal outcome; preserve its logs and failed artifacts.
The 32 GiB run quota and 50 GiB reserve require at least 82 GiB at startup;
allow additional capacity for the separate recorder/mapper outputs and audits.

For a **completed** acquisition, run in the isolated sourced offline environment:

```bash
python3 tools/assess_step_trial.py /archive/MAPPER --archive-root /archive
python3 tools/audit_completed_route.py /archive/NATIVE /archive/RECORDER --archive-root /archive
python3 tools/diagnose_costmap_refusal.py /archive/MAPPER --archive-root /archive
```

The last command applies only when refusal snapshots exist. It is a single-query
diagnostic, not proof that a route would complete. The latest baseline IDs are
in the handover table. `audit_completed_route.py` restores packed native inputs
with byte verification; the documented narrow ownership correction may be
needed for `native_chunks`. Cold-compressed `.mcap`/`native.bin` inputs separately
require `cold_archive.py restore` on explicit original host paths. Failed captures
stay ineligible for completed-run evaluators even after restoration. Exact replay
of a changed policy is not expected to match old decisions; preserve the frozen
baseline source/library and label changed-policy comparisons separately.

## October 20: SLAM + terrain robustness loop (C1-C4 scorecards)

Decision context: `docs/decision-2026-10.md`. Thresholds live in `tools/audit_suites/checks*.toml`.
All outputs are archive runs; scorecards are `scorecard.{md,json}` in the suite run.

```bash
# Host, after any terrain / gap-policy / scorer edit (~1 min; ~10 s with --reuse):
tools/uv_python.sh tools/terrain_audit_suite.py quick [--reuse <prior quick run>] [--baseline <scorecard.json>]
# Host, every capture x pose profile (parallel, one replay per core):
tools/uv_python.sh tools/terrain_audit_suite.py full --reuse <prior full run> --baseline <prior scorecard.json>
# Host, drives the dev container (astrobot-selected-dev) with docker exec; ~15 min:
tools/uv_python.sh tools/terrain_audit_suite.py c3
```

- `--reuse` re-scores cached layers (`jobs/<slug>/layers.npz`) when the replay key (source,
  profile, every, limit and replay code hash) is unchanged. Scorer or gap-policy changes then
  re-score without replaying the native library.
- Exit 0 = every check passes, 2 = a check failed or regressed (the scorecard is still written),
  1 = a job crashed. Do not run the C3 suite alongside host suites: CPU load changes GLIM
  latency (observer effect).
- Action availability is scored on the costmap Nav2 would see: `navigation_costs` (gap policy),
  then publisher-side `clear_footprint`, then Nav2 static/inflation, then the swept padded
  footprint (`tools/local_terrain/nav2_contract.py`). The contract matches installed Nav2
  inflation and swept area exactly (`NAV2_SHADOW_PROBE=... pytest tools/test/test_nav2_contract.py`
  in the container, probe built as described below).
- C3 replays use a shared per-recorder input (`audit_rtk_robustness.py <recorder> --make-base
  [--no-supervisor]`, ~3.5 GB once) re-timed onto the recorded `/clock`. Bare GLIM runs are
  exact at rate 2 with 4 in parallel; supervised runs need rate 1 (pose_stale is 0.5 sim-s).
- Terrain profile `glim:<c3 run>` replays the terrain capture with that C3 run's GLIM poses. It
  is only accepted for the capture whose `shadow_link.json` names the C3 recorder (loop16:
  terrain `0113a562` / recorder `d4d0f081`).
- RTK noise (user decision, Oct 20): synthetic_rtk defaults are now σ(d) = 0.02 + 0.001·d m
  (z ×2); the old 0.05 + 0.01·d (0.6 m at the 55 m lunaryard baseline) was not a hardware
  model. `c3.toml` sets `rtk_sigma = [0.02, 0.001]`: the base builder (`--make-base
  --rtk-sigma S0,K`) rescales every recorded fix's own noise sample to that model, so the
  recorded drive replays at the new level with the same noise realization. The runner refuses
  a pinned base built at another `rtk_sigma`.
- Nav2 planner costmap: `live_local_terrain.py` parameter `planner_gap_policy` (`""`,
  `isolated` = the old `isolated_gap_costs`, or `nav2_bounded_shadow`; recorder
  `--planner-gap-policy nav2_bounded_shadow`) publishes `/terrain_shadow/planner_costmap` via
  `nav2_contract.planner_costs`, the function the suites score: gap policy, then the rover's
  own unpadded footprint cleared at the record pose. Policies live in
  `navigation_cost.PUBLISHED_POLICIES`.
- Supervisor RTK gates are per axis (horizontal vs vertical σ, never a 3-D trace). While GLIM
  runs, each fix is gated against GLIM's position at the fix stamp (0.1 m + 0.5 m/s over any
  uncovered time + 6σ). Otherwise it is gated fix to fix at 0.5 m/s. A persistent outlier is
  rejected until the 2 s outage fault holds motion. Supervised replays also write
  `status.json`, the supervisor state timeline.

### Archive cleanup (Oct 20): replacement config and library

The Oct 20 cleanup removed every archive run older than Oct 19 ~19:00, plus the
`terrain_entrypoints/` index and `contact-deps-OXpt89ix`. The commands in this runbook name
deleted IDs; substitute:
- config `revision-4c7b70a4…/config` → **`revision-ed871eabc2e64d1eb657d292e5d9ac77/config`**.
  It is the frozen config (byte-identical copy `5c636f93`; three recorders' `config/` copies
  agree) with only its two plugin paths moved to the rebuilt plugins below. `5c636f93` alone
  does not run: GLIM exits on the deleted plugin paths (recorder `ae9ba662`, `estimator_exited`).
- GLIM plugins `revision-009c6a6d…/{librevision_target_cpu,libclosed_window_abs}.so` →
  **`revision-9f0c2307848742b4953ffc3847065065`**: built from `tools/glim_shadow` targets
  `revision_target_cpu` / `closed_window_abs`, from sources last modified Sept 21 (before the Sept 22
  original build). Sizes equal the originals' exactly (438,760 / 1,082,096 bytes); hashes differ.
  `test_closed_window` and `test_revision_target` pass; every linked library was also loaded by
  the live GLIM with the originals (recorder `10f448bf` runtime manifest).
- library `revision-70647a2b…` → **`revision-06a82bedebf641418a4d6d420447cef2`**: same
  `build_local_terrain.py` flags, all 17 sources hash-identical. The `.so` hash differs
  (`cf7098b6…` vs `e452b2b6…`), but an exact receipt-order replay of capture `4ac1810d`, made
  with the old library, reproduces all 1,163 recorded projections byte for byte.
- `terrain_entrypoints/`: the HEAD-commit (`40bf078`) `live_local_terrain.py`,
  `contact_advisory.py` and `contact_runtime.py` are re-archived from git (runs `340f63d3`,
  `38ae287b`, `85c1b56f`), as is the Oct 20 worktree `live_local_terrain.py` (`e9500543`).
- Shapely staging: not rebuilt. Only `--contact-advisory` needs it; restage Shapely 2.1.2 to
  a new directory before using that flag.

**Lost fixtures.** Every Oct 19 audit fixture lost its native capture run and pose export:
detour `3ac40f8a` (quick suite), `236b00c7`, `0113a562` (loop16), `b47a9b95`, `f3b5f4d7`, and
C3 recorder `d4d0f081`. Their terrain runs remain but can't be scored or exact-replayed. The
quick, full and c3 suites need fresh captures; the Oct 19 numbers in `docs/decision-2026-10.md`
stand as recorded but can't be reproduced.

### New stress drives (Oct 20): `transit50`, `approach_field`

Waypoint loops in `tools/natural_motion.py::WAYPOINTS` (seed 43, as every natural capture).
Their clearance survey is a corridor of `ROTATION_RADIUS` (1.35 m) about the route, not the
bounding box; each leg clears every seed-43 rock box by ≥ 1.43 m. Preflight PhysX overlap uses
per-leg boxes plus chord strips per turn disc (`corridor_boxes`). The per-step footprint
guard is unchanged.

**Corridor floor is recorded, not gated (user decision, Oct 21).** The first transit50
attempt (sim `9147eafa`, recorder `757ae0ea`) was refused preflight by the 2 cm detrended
roughness limit (2.14 cm peak over 25 of 47k cells), and the recorder then sat out its 2 h
wall limit. Sim captures are GT-scored; their guards exist for convenience, not safety. So
for corridor routes (`floor_gate = "recorded_only_v1"`): incline, step and roughness are
measured and listed under `floor_limit_violations` in `clearance.json`, but only the GT rock
checks (PhysX overlap, first-hit terrain per cell) and a valid floor hit gate motion. The
legacy box survey keeps its 2 cm / 0.35 limits. The recorder now fails after
`--request-seconds` (900 s) without a sim capture request.

Two more transit50 blockers, fixed Oct 21:
- `EncodedFrames` capped capacity at 40,000 while transit50 declares 55,000 native frames
  (sim `c051a745`, preflight). The cap, and `restore_native_archive.py`'s, is now 100,000.
- At 510 sim-s (sim `6479a129`, recorder `e19170de`) the recorder spent 0.63 s in one loop
  pass: a 0.25 s decode of a 4,878-frame GLIM revision (revisions carry the whole
  trajectory, so they grow with the drive) plus a 0.38 s storage check. That starved the
  0.5 s wall heartbeats (terrain status, physics IMU). All lease/heartbeat windows (recorder
  checks, `PhysicsIMUReadiness`, `ShadowLease`, `LeaseProcess` receipt age, sim drive
  watchdog) now use `shadow_capture_gate.HEARTBEAT_S` = 2 s. The 0.5 s IPC query deadline
  is unchanged.
- `transit50`: 50.5 m, 7 legs at 0.18 m/s (Nav2 `vx_max`), ~430 sim-s; crosses itself once at
  (0, −8.5). C2 tilt/z at speed and C3 drift. The rock 1.4 m west of the spawn leaves only
  south-east departures and a due-south return.
- `approach_field`: 9.5 m out at 0.15 m/s past demo profile `approach_field_v1`, then 9.5 m
  in reverse on the same lane (`return_reverse`, action `"reverse"`): rocks of 0.12 / 0.20 /
  0.30 / 0.40 m, edges 0.95 m from the centreline (0.61 m from the body, inside the 1 m C4
  danger range, outside the 0.82 m guard envelope), each passed twice. The survey exempts only
  `/DemoApproachRocks/`; `validate_demo_clearance` rechecks the authored rocks preflight.

**In-place turns are limited to 120° (`MAX_WAYPOINT_TURN`).** The skid-steer base turns about
a point 9–13 cm behind `base_link` (least-squares fit to the loop16 and approach_field motion
logs), so `base_link` orbits the waypoint. Past ~120° the rest of that orbit is sideways, which
the base cannot correct before the turn ends. The first approach_field capture (`6bf40592`,
out-and-back with a 180° turn) left the 25 cm skid bound at 141°. Waypoint loops declare
`turn_translation_policy = anticipatory_corner_feedback_v3` (`waypoint_turn_lead`): turns
≤ 90° keep loop16's lead; larger turns hold the lead along the current heading and fade it
over the last 30°; the first rotation has no lead. loop16 keeps v2, unchanged. Kinematic
model with 0–20 cm offsets: ≤ 0.17 m drift on every waypoint-loop turn.

Same procedure as `costmap_detour8` above (recorder first, then the headless sim), with
`--natural-motion transit50 --wall-seconds 7200` (or `approach_field --wall-seconds 3600`) and
`SELECTED_DEMO_ROCK_PROFILE=approach_field_v1` for `approach_field` (empty for `transit50`):
`bash docker/run_selected_headless.sh start transit50`. The capture, recorder and live
terrain runs each take the route quota (`costmap_scenarios.WAYPOINT_ARCHIVE_GIB`: 40 GiB
transit50, 12 GiB approach_field) plus the 50 GiB reserve at allocation; a 50 m drive writes
~40 GB in total, so check archive space first. The recorder's wall limit is the route's
`max_wall_s`. The selected dev container is in the same compose project: stop only the sim
(`docker compose -f docker-compose.lunaryard.yml -f docker-compose.archive.yml -f
docker-compose.selected-headless.yml stop astrobot-sim`, then `rm -f astrobot-sim`); `down`
would also remove the dev container. Then add the capture/recorder to `full.toml` and
`c3_transit.toml`.

### Nav2 offline replay (Oct 22): recorded costmap through the real planner

`tools/nav2_replay_check.py` runs two `planner_server`s (SmacPlanner2D, StaticLayer and
InflationLayer from `nav2_phase3_bev.yaml`, wall time), one on the recorder bag's
`/terrain_shadow/planner_costmap` and one on a GT rock-core grid. It scores every shadow path's
swept body against GT and writes `summary.json`, `plans.jsonl` (with the paths) and the master
costmaps. It needs the capture's pose audit (`pose_arrays.npz`). No sim. Run it in the dev
container, with a separate `ROS_DOMAIN_ID` for each concurrent run:

```bash
docker exec -e ROS_DOMAIN_ID=183 astrobot-selected-dev bash -c 'source /etc/ros_setup.sh;
  export PYTHONPATH=/archive/contact-deps-oct21:/workspace/astrobot-lab/tools:$PYTHONPATH;
  cd /workspace/astrobot-lab; python3 tools/nav2_replay_check.py --recorder revision-2623e294 --every 2'
# --global-map: plan on the mosaic of all windows so far (local_terrain/global_stitch.py),
#   to home and to points --back 5 10 20 40 m along the route; use --every 10-20.
```

Pass: 0 `shadow_transit_hits_*` (contact before the 0.65 m goal tolerance) and
`shadow_found_gt_blocked` 0. Arrival contacts are a goal-placement/controller matter. Results
are in `docs/decision-2026-10.md` item 4.

### v2 Nav2 stack, closed loop (Oct 22)

v2 is the only map on this stack:
- the sim nav stream (`OMNILRS_SELECTED_NAV`) feeds GLIM and the v2 publisher;
- `live_local_terrain.py deployment:=nav` serves `slam/reset_map` and publishes the window plus
  `/terrain_shadow/global_costmap` (the mosaic);
- the supervisor's `map_topic` is the window;
- Nav2 reads both through `nav2_v2.yaml`.

Bring-up:

```bash
C="docker compose -f docker-compose.lunaryard.yml -f docker-compose.archive.yml -f docker-compose.selected-nav.yml"
OMNILRS_ENVIRONMENT_SEED=43 $C up -d --no-deps astrobot-sim   # wait for "SELECTED_NAV tile" in its log
docker exec -d astrobot-selected-dev bash -c 'source /etc/ros_setup.sh; source /workspace/astrobot-lab/astrobot-lab/install/setup.bash;
  ros2 launch astrobot_launch phase3_nav.launch.py slam_stack:=v2 slam_start_delay:=5.0 \
    nav2_params:=$(ros2 pkg prefix astrobot_launch)/share/astrobot_launch/config/nav2_v2.yaml > /tmp/v2.log 2>&1'
# wait for /astrobot_0/localization/status reason "validated", then e.g.
tools/v2_nav_trial.py --scene-run <SELECTED_NAV run> --goals 10    # in the container, PYTHONPATH as above
$C stop astrobot-sim; $C rm -f astrobot-sim                         # never `down`
```

Options and gotchas:
- **Faults:** `rtk_occlusion_period_s:=150.0 rtk_occlusion_duration_s:=10.0` (RTK lost while
  sim % period < duration).
- **Raw archive:** `terrain_archive_raw:=true` archives native bytes for an audit-suite
  replay, at ~28 MB per sim-s.
- **Gates:** `tools/v2_gate_run.sh <seed> <explore_radius|sample_return>` runs one attempt with
  the legacy mission parameters, a GT-scored eval on the mosaic, and offline GT rock contacts
  (`tools/score_gt_contacts.py`). Everything goes to one archive run, then the sim is
  stopped and removed.
  The eval starts before the mission and runs with `--home-from-gt-start`: coverage and
  home distance are scored about the rover's spawn GT pose (as the manager anchors its
  grid), with the nominal-home figures kept in `verdict.metrics` for legacy comparison.
  Contacts with rocks already under the spawn body are reported as `spawn_rock_contacts`.
  Nav runs allocate with a 20 GiB free-space reserve (gate run, sim nav stream, nav
  publisher `archive_reserve_gib`); a run writes MBs. Captures keep the 50 GiB default.
- **Archive reconnects:** if `/archive` gives EIO inside `astrobot-selected-dev` while the
  host mount works, the container holds a stale bind; `docker restart astrobot-selected-dev`
  (idle) re-binds it.
- **Exploration on v2:** use `exploration_map_topic:=/terrain_shadow/exploration_map` (the
  class mosaic), `reveal_distance_m:=1.5`, `reachable_seed_distance_m:=1.0` and
  `home_approach_radius_m:=0.5` with
  `return_behavior_tree:=<share>/behavior_trees/navigate_to_pose_lunar_recovery_precise.xml`
  (its 0.35 m goal checker keeps a return success inside the 0.9 m home radius; other goals
  keep 0.65) and `nav_through_poses_bt_xml:=<share>/behavior_trees/navigate_through_poses_v2.xml`.
  `nav2_v2.yaml` loads two goal checkers, so every BT's `FollowPath` must name one; the
  phase3 BT (`navigate_to_pose_lunar_recovery.xml`) names `general_goal_checker`, the only
  checker of the legacy configs. New BT files need `colcon build --packages-select
  astrobot_launch` to install. `mission_visit_margin_m:=0.15` keeps the manager's coverage
  count at or below GT (a cell counts only when the pose is 0.15 m inside it).
  The cost mosaic's taper and gap-policy values
  are wrong for the manager's free/clearance thresholds, and a stationary rover has nothing
  reachable until it moves.
- **Launch-argument names:** `v2_slam.launch.py` arguments must not reuse phase3 names.
  Launch configurations are shared with included files, so a reused name such as
  `glim_config_path` silently takes the legacy default (T1 first ran GLIM on the legacy
  config and topics).

## October 6: experimental planner costs and path checks

This is costmap-to-path testing, not Nav2 command execution. Core files:
`tools/local_terrain/navigation_cost.py` (isolated-gap penalty),
`cost_planner.py` (bounded footprint-aware search), `planner_evaluation.py`
(independent GT scoring). Never import the evaluator into the planner.
Measured classes and the existing controller remain unchanged.

Host analytic cases require NumPy 1.26.4, SciPy 1.11.4 and Shapely 2.1.2:

```bash
OPENBLAS_NUM_THREADS=1 uv run --offline --no-project \
  --with numpy==1.26.4 --with scipy==1.11.4 --with shapely==2.1.2 \
  python tools/run_navigation_cost_cases.py --archive-root <external-archive>
```

For native/GLIM recordings use the sourced dev image, staged Shapely,
`OMP_NUM_THREADS=4`, and **both** `/archive` and the original absolute host
archive path bound to the same external device. Stop Isaac first. The selected
recordings also require the original `/workspace/astrobot-lab` source alias.
See the contact replay section for this isolated Docker environment.

```bash
python3 tools/assess_cost_paths.py /archive/<completed-mapper> --archive-root /archive
# Additional high-cost missing-ground control; strictly offline, never commands:
python3 tools/assess_cost_paths.py /archive/<completed-mapper> --archive-root /archive \
  --optimistic-control
python3 tools/benchmark_navigation_cost.py --library /archive/<frozen-build>/liblocal_terrain.so \
  --archive-root /archive --profile layer_capacity
python3 tools/benchmark_navigation_cost.py --library /archive/<frozen-build>/liblocal_terrain.so \
  --archive-root /archive --profile dense_flat
python3 tools/check_navigation_cost_equivalence.py /archive/<completed-mapper> \
  --reference-cost-file /archive/<prior-assessment>/source/local_terrain/navigation_cost.py \
  --archive-root /archive
python3 tools/render_cost_paths.py --cases /archive/<analytic-cases> \
  --replay /archive/<short-drive-assessment> --replay /archive/<turn-assessment> \
  --archive-root /archive
```

Every replay checks the original terrain/advisory outputs exactly. All maps
contribute cost statistics; the first query in each fixed five-second bin plans
the same twelve 0.3/0.5/1 m cardinal goals. Keep failed goals and missing GT.
Scene bounds give conservative possible collisions, not mesh contact; outside
the surveyed floor stays unverified. Hidden-hazard analytic controls deliberately
expose risk. No returned paths is **not** a safety pass. The eight-heading search
can miss routes, especially when its initial heading connector is unavailable.

Live opt-in publication: add `--isolated-gap-costs` to the existing v2 guarded
recorder command (or `isolated_gap_costs:=true` to `terrain_shadow.launch.py`).
It publishes `/terrain_shadow/planner_costmap` and
`/terrain_shadow/uncertainty_mask`, invalidated to unknown on fault/shutdown.
Do not substitute the optimistic offline control here. These OccupancyGrid
values are costs, not probabilities; an eventual Nav2 adapter must preserve
graded costs, unknown policy and the published grid transform rather than
assuming default binary static-map semantics.

Build `slam_mapping astrobot_launch` after adding Python modules. Focused tests:
`test_navigation_cost.py`, `test_cost_planner.py`, `test_planner_evaluation.py`,
`test_local_costmap.py` plus contact/support/controller regressions. In sourced
ROS domain 179 run `test_live_local_publish_ros.py test_shadow_capture_ros.py`.
The publication fixture tests the actual method and serialized message types;
it does not claim live DDS/GLIM scheduling or vehicle tracking acceptance.
The HTML renderer also needs both archive aliases when mixing host-created
analytic cases and container-created replays; do not alter their archive
contracts to make a host-only invocation work. It verifies archived grid/path
hashes and includes every saved case, time bin and goal.

## Optical-origin audit and live v2 costmap (September 28)

### October 4: false-dropoff fix and matched reassessment

Use the explicit pointwise-boundary candidate for the next guarded advisory
capture; [results, limits and artifact IDs](false-dropoff-fix.md). The later
[three online scenarios](online-terrain-scenarios.md) complete and preserve the
fix, but expose near-wheel visibility limits. Neither result qualifies Nav2.

```bash
python3 tools/build_local_terrain.py --archive-root /archive \
  --patch-normal-cache --sparse-native-patch --defer-native --rolling-grid --pointwise-boundary
OMP_NUM_THREADS=4 python3 tools/reassess_local_terrain.py \
  /archive/revision-282097f0d80a4d98aa4275dd31f7d395 \
  /archive/revision-70647a2b742c446eb61bd3d7141ea011/liblocal_terrain.so --archive-root /archive
```

Run in the offline dev image with the external archive aliases verified, NumPy
and SciPy; no ROS graph or simulator is needed. A rebuilt library gets a new
run ID; substitute that explicit output when testing it. Original native inputs,
poses and schedules stay fixed. A baseline mirror must reproduce all recorded
layer hashes before candidate results count. Reports reuse the original floor
and motion-query denominators and retain every projection.

For branch attribution build with `--diagnostics` **instead of** `--defer-native`.
Then pass `--diagnostics --compare-library <uninstrumented-candidate-library>`
to the reassessment tool: diagnostic and ordinary candidate outputs must also
match byte for byte. `--require-exact` is for instrumentation of the original
algorithm, not for a deliberately changed candidate. Instrumented timing is not
compute acceptance. Add `--serial` for the non-OpenMP build.

```bash
python3 tools/render_dropoff_fix.py <original-assessment> <matched-comparison> --archive-root /archive
OMP_NUM_THREADS=4 python3 tools/stress_terrain_resources.py \
  --library <uninstrumented-candidate-library> --archive-root /archive --profile layer_capacity
OMP_NUM_THREADS=4 python3 tools/stress_terrain_resources.py \
  --library <uninstrumented-candidate-library> --archive-root /archive --profile dense_flat
```

Run resource tests without other CPU experiments. New density/outlier/expiry
fixtures: set `TERRAIN_LIBRARY` to the original `revision-9eadef...` library and
`TERRAIN_POINTWISE_LIBRARY` to the candidate, then run
`python3 -m pytest tools/test/test_pointwise_boundary.py`.
The historical `test_terrain_two_path.py` suite instead requires legacy parent
`revision-bf1fde8ac66a45979c61a8404232e8f0/liblayered_terrain.so` as `TERRAIN_LIBRARY`
and the new candidate as `TERRAIN_TWO_PATH_LIBRARY`; its pre-two-path false-normal
control cannot use the already-fixed modern parent. Run ordinary calibration,
slopes, local/rolling and worker tests with the candidate as `TERRAIN_LIBRARY`.
Use `TERRAIN_REFERENCE_LIBRARY` for immediate/deferred/serial comparisons via
`test_local_native_patch.py`, and `TERRAIN_DIAGNOSTIC_LIBRARY` for
`test_terrain_diagnostics.py`. Report tests are in `test_dropoff_reassessment.py`.

### Earlier optical-origin workflow

- Stationary full-body sensor fixture: stop owned simulator/dev inspection
  containers, then `python3 tools/run_local_isaac_fixture.py optical`. Host and
  image CUDA preflights, external archive, network-none, 240 s deadline. Copies
  active body/wheel collision geometry; compares native rays to optical-origin
  first hits. Every raised zero-origin control must pass; original embedded
  origin is a negative control. No rover/navigation qualification.
- Corrected runtime, current frozen config/library and start order:
  [live inspection guide](live-inspection.md#corrected-mounting--native-glimv2-costmap).
  Requires `slam_mapping` built with `SLAM_MAPPING_BUILD_TERRAIN_SHADOW=ON` and
  `astrobot_launch` rebuilt; existing NumPy/SciPy/ROS/custom-interface overlay.
  `selected_optical_v2` is distinct from the historical `selected_generic_v1`.
- Inside sourced dev: `python3 tools/probe_live_costmap.py` checks ten received
  class/taper pairs within 20 s, archives a snapshot, never sends commands.
  Host exact replay: `python3 tools/replay_live_local_terrain.py <mapper-run>`.
  Requires completed sealed evidence and unchanged hashed runtime inputs;
  an unsealed/failed source is not eligible. Receipt order and every projection
  must match; wall compute timing alone is excluded from equality.
- Translating corrected-mount diagnostics: replace **both** route arguments in
  the inspection guide with `straight_slow` (2 m out/back at 0.05 m/s), or
  `combined10` (turn, 2 m out/back, return heading at 0.10 m/s). Keep 900 s
  recorder deadline and all original guards. `loop16` is not yet a cleared
  terrain mission; long-run GC and non-ODOM revisions remain separate barriers.
- Completed-live sampled floor diagnostic in the isolated dual-archive-alias
  dev container described below:
  `python3 tools/score_live_floor_samples.py /archive/<mapper-run>
  /archive/<native-capture-run> --archive-root /archive`. Both original archive
  aliases must exist; a plain host invocation correctly refuses a recorded
  `/archive` device contract it cannot resolve. This exact-replays every projected
  map and queries every original active-PhysX clearance sample, including
  outside/unknown and the invalidated completion map. Pre-projection startup
  publications are explicitly not scored. Full world/home/map/local transforms;
  no fitted alignment.
  It measures point-sampled floor support/false blocking, **not** continuous
  cell support, obstacle recall, false-safe rate or safe-motion qualification.
  Tests: `python3 -m pytest tools/test/test_live_floor_samples.py`.
- Paired pose attribution, after exact floor replay and completed pose audit,
  in that same isolated dual-alias container:
  `python3 tools/attribute_live_floor_support.py /archive/<floor-score-run>
  /archive/<measure-exported-glim-run>`. Runs an original-SLAM mirror (mandatory
  per-projection exact layer hashes) and an explicitly offline GT-pose branch
  on identical native input, live grid origins, query stamps and evidence ages.
  Outputs every floor-sample count/reason and last valid spatial layers. It does
  not alter the mapper, substitute GT terrain labels, claim compute acceptance
  or treat agreement with the GT-pose map as safety truth. Do not run these CPU
  replays beside live captures when measuring resource performance.
- Spatial view (host, matplotlib/NumPy):
  `python3 tools/plot_live_floor_attribution.py <external-archive/attribution-run>`.
  Verifies the archived last-valid layer hash and creates a new external run
  containing `paired-floor-maps.png` and provenance. Dots are independently
  surveyed floor points; outline/arrow are the estimated chassis/front, not a
  footprint-clearance certificate. This post-hoc view does not redefine gates.
- Focused host tests: `python3 -m pytest tools/test/test_local_costmap.py
  tools/test/test_chassis_returns.py tools/test/test_optical_origin_audit.py
  tools/test/test_local_terrain_seal.py tools/test/test_selected_lidar_profile.py
  tools/test/test_generic_lidar_adapter.py`. Add the existing local-core suite
  with explicit `TERRAIN_LIBRARY` to avoid counting skips as validation.
- Actual ROS shutdown regression: sourced dev, set `TERRAIN_LIBRARY` to the
  archived rolling library and `TERRAIN_LIVE_REQUEST` to an archived corrected
  recorder's `terrain_request.json`, then run
  `python3 -m pytest tools/test/test_live_local_shutdown_ros.py`.
  The subprocess uses domain 179, publishes only unknown advisory grids, gets
  SIGINT, and must seal closed with zero samples and `experiment_completed=false`.
  It explicitly uses loopback, enforces the launcher's five-second shutdown
  budget, and compares saved output digests to the actual disk bytes.
  Archive mounted/verified first; no simulator required and no motion authority.
- A frozen config prepared on the host records the host archive path in its
  device contract. For offline ROS audits, use a disposable dev container with
  the verified external archive mounted at BOTH `/archive` and its original
  host path; repo read-only at `/workspace/astrobot-lab`, network none, host
  UID/GID, normal sourced ROS/overlay. Do not rewrite historical contracts,
  manufacture symlinks to workspace storage or weaken device checks. The
  completed September 28 audit used this declared alias, not a recapture.

This reference preserves the supplied `AGENTS.md` content, with navigation headings added. Its dated results and “current” statements are migration-snapshot context, not newly verified qualification. Consult the implementation ledger before acting on historical status.

Use the root [AGENTS.md](../../AGENTS.md) for standing instructions and completion policy. Read only the relevant recipe below. Preserve each recipe's prerequisites, safety stops and evidence limits; task persistence does not authorize bypassing them. The referenced repository, tests and implementation documents were not supplied or executed as part of this edit.

## Contents
- [Local terrain v2](#local-terrain-v2)
- [Project structure](#project-structure--module-organization)
- [Build and development](#build-test-and-development-commands)
- [Coding conventions](#coding-style--naming-conventions)
- [Core tests and revision fixtures](#core-tests-revision-fixtures-and-provenance)
- [Terrain calibration](#terrain-calibration-and-frozen-fixtures)
- [Native timing and resources](#native-timing-support-and-resource-gates)
- [Acquisition and coverage](#acquisition-diversity-coverage-and-diagnostics)
- [GPU preflight](#gpu-preflight-and-offline-documentation)
- [Shadow capture and IMU](#shadow-capture-pose-qualification-and-imu-checks)
- [GLIM experiments](#glim-replay-accuracy-and-candidate-experiments)
- [Natural holdouts and motion](#natural-holdouts-controlled-motion-and-terrain-isolation)
- [Terrain candidates and Nav2](#terrain-candidates-boundary-evidence-and-nav2-probes)
- [Live advisory terrain](#live-advisory-terrain-provenance-and-acquisition-loops)
- [System field guide](#system-field-guide)
- [Commits and PRs](#commit--pull-request-guidelines)
- [Environment and Phase 1 tuning](#environment-notes)

## Project Structure & Module Organization
- `OmniLRS/` holds the simulator codebase. Core Python modules live in `OmniLRS/src/`, configuration in `OmniLRS/cfg/`, and assets in `OmniLRS/assets/`.
- `OmniLRS/WorldBuilders/` contains terrain/world-generation utilities and ad-hoc test scripts (e.g., `test_*.py`).
- `docker/` and `docker-compose.yml` define dev and simulation containers; `config/` contains middleware config like `cyclonedds_config.xml`.
- `astrobot-lab/` is the ROS workspace for local autonomy code. Current Phase 1 packages live in `astrobot-lab/src/`:
  - `basic_control`
  - `astrobot_launch`
- Simulator-owned GT map and `map -> odom` publishing now live in `OmniLRS/src/environments_wrappers/ros2/gt_map_ros2.py`.
  Current Phase 1 controller tuning lives in:
  - `astrobot-lab/src/astrobot_launch/config/basic_control_phase1.yaml`
  - `astrobot-lab/src/astrobot_launch/config/nav2_phase1.yaml`
  - `OmniLRS/cfg/environment/lunaryard_40m.yaml`

## Build, Test, and Development Commands
- `docker compose up astrobot-dev` starts the Space-ROS dev container (requires `DEV_USER`, `ASTRO_UID`, `ASTRO_GID`, `DISPLAY`). Example: `DEV_USER=$USER ASTRO_UID=$(id -u) ASTRO_GID=$(id -g) docker compose up astrobot-dev`.
- `docker compose up astrobot-sim` starts the simulation container (requires NVIDIA runtime and X11).
- `OmniLRS/omnilrs.docker/build_docker.sh` builds the Isaac Sim image; `OmniLRS/omnilrs.docker/run_docker.sh` runs it with GPU/X11 mounts.
- `python OmniLRS/run.py` launches the simulator using Hydra configs in `OmniLRS/cfg/`.
- `docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc 'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && colcon build --symlink-install'` builds the local ROS workspace overlay.
- `docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc 'source /etc/ros_setup.sh && ros2 launch astrobot_launch phase1_nav.launch.py'` launches the current Phase 1 navigation stack.

## Coding Style & Naming Conventions
- Python uses Black with line length 120 (`OmniLRS/pyproject.toml`). Use 4-space indentation.
- Prefer `snake_case` for functions/variables and `PascalCase` for classes. Keep filenames lowercase with underscores.
- Configuration is YAML via Hydra; keep new config files under `OmniLRS/cfg/` matching existing folder structure.

## Testing Guidelines
### Core tests, revision fixtures and provenance

- Pure integrity/evaluation contracts: `PYTHONPATH=astrobot-lab/src/astrobot_core python3 -m pytest astrobot-lab/src/astrobot_core/test` (host Python 3 + pytest, no ROS/simulator).
- Mapper and absolute-association tests: `colcon test --packages-select slam_mapping glim_ros && colcon test-result --verbose` after a ROS overlay build with `BUILD_TESTING=ON`. Mapper core sources can also be compiled directly with C++17 for sanitizer runs.
- Simulator/ROS integration gates require the headless Lunaryard containers and NVIDIA runtime; passing host tests does not constitute mission validation.
- ROS callback/mission/recovery contracts (after sourcing the built overlay): `python3 -m pytest astrobot-lab/src/astrobot_core/test astrobot-lab/src/localization_supervisor/test astrobot-lab/src/basic_control/test astrobot-lab/src/exploration_manager/test` from the repository root. When checking uninstalled changes, put those four package source directories on `PYTHONPATH`.
- Stationary, read-only simulator probe: `python3 tools/localization_smoke.py --wall-seconds 45 --output <new-summary.json>`; launch Phase 3 without any exploration/goal sender first.
- Deterministic mapper replay: `python3 tools/replay_surface_fixture.py <mcap-bag> --output <new-directory> --domain-id 179`. Requires exact measurement-stamp pairs of corrected aligned clouds and corrected LiDAR poses. Compare `frame_map_sha256` across repeats; this is not GT-isolated recall scoring.
- Add `--gt-isolate` for full-SE(3), measurement-time GT re-anchoring of both clouds and LiDAR origins. Requires corrected IMU poses and `/gt/base_link_pose`; configure the fixed home and base/IMU extrinsic for other robots. Rejects interpolation gaps, extrapolation, duplicate pose stamps, and multiple recorded localization epochs. This is mapper isolation, not obstacle recall scoring. Replay projects every frame; obstacle-memory promotion also requires live-cadence tests.
- Runtime provenance: `python3 tools/write_run_manifest.py --output <new-manifest.json> --seed 42 --launch-command '<actual command>'` inside the same environment as the running stack. Existing artifacts are not overwritten.
- Evaluator time/frame and SE(3)/reset-barrier contracts: `python3 -m pytest tools/test` in the sourced ROS environment (NumPy and SciPy required).
- Revision/archive reference tests (host Python + NumPy + pytest, no ROS): `python3 -m pytest tools/test/test_revision_evidence.py tools/test/test_archive_guard.py`. ROS serialization/TF audit tests also live in `tools/test` and require the freshly built `astrobot_interfaces` overlay.
- Revision foundation integration: build `astrobot_interfaces`, `glim`, `glim_ros`, `slam_mapping`, `localization_supervisor`, and `glim_slam_launch` with `BUILD_TESTING=ON`; run `colcon test --packages-select glim_ros slam_mapping && colcon test-result --verbose`. These include actual GLIM callback, ROS cloud/pose, retained-observation and snapshot-service tests.
- External-drive stationary fixture: mount the drive with `-f docker-compose.archive.yml`, source the overlay, then run `python3 tools/run_stationary_revision_check.py --wall-seconds 60` with the simulator already running and GT TF disabled. Do not run alongside another navigation stack. It starts a bounded Phase 3 launch, records first, sends no goals, and preserves failures.
- Validate its printed run path with `python3 tools/audit_revision_fixture.py <run>`, `python3 tools/audit_tf_fixture.py <run>`, and `python3 tools/check_revision_reconstruction.py <run>`. Outputs are exclusive-create; do not overwrite failed attempts. These are wiring/reference-reconstruction checks, not terrain or autonomy acceptance. See `docs/implementation/revision-aware-terrain-status.md` for limits and remaining gates.
- Prefer the host UID/GID when running archive tools in Docker so artifacts remain user-owned (for example, `docker compose exec --user "$(id -u):$(id -g)" astrobot-dev bash`). The archive mount is opt-in and must already exist on the external drive.
- Ad-hoc world-builder scripts remain under `OmniLRS/WorldBuilders/test_*.py`.
### Terrain calibration and frozen fixtures

- Experimental layered terrain contracts: build `slam_mapping` with `BUILD_TESTING=ON`, then run `ctest --test-dir <build>/slam_mapping --output-on-failure`. For the 60 analytic small-object cases and Python contracts, set `TERRAIN_LIBRARY=<build>/slam_mapping/liblayered_terrain.so` and run `PYTHONPATH=astrobot-lab/src/astrobot_core python3 -m pytest -p no:cacheprovider tools/test/test_layered_terrain_calibration.py tools/test/test_raw_terrain_layout.py tools/test/test_terrain_metrics.py tools/test/test_terrain_gt_probe.py`. NumPy/SciPy/pytest required; these tests do not establish simulator or navigation acceptance.
- Stationary raw GT replay (sourced ROS overlay, external archive mounted): `python3 tools/replay_raw_terrain.py <bag> --library <built-liblayered_terrain.so> --archive-root /archive`. Preserves every finite XYZ return, validates recorded extrinsics, refuses moving bags until timing/deskew validation, allocates a fresh run, and reports coverage/runtime only. See `docs/implementation/gt-terrain-status.md`.
- Opt-in simulator collision probe: add `-f docker-compose.archive.yml` and set `OMNILRS_TERRAIN_GT_PROBE=true OMNILRS_GT_TF_ENABLED=false` when starting only `astrobot-sim`. Captures one stationary 10x10m / 2cm PhysX collision tile on the simulation thread; never supplies navigation labels. Do not issue movement commands during capture. `TERRAIN_GT_PROBE COMPLETE` means raw capture completed, not that the reference passed calibration or route clearance validation.
- If you add tests, document the runner and expected environment in this file.
- Controlled terrain geometry/driver/scorer contracts (host Python + NumPy + pytest): `python3 -m pytest -p no:cacheprovider tools/test/test_terrain_calibration.py tools/test/test_calibration_drive.py tools/test/test_calibration_score.py tools/test/test_raw_return_filter.py tools/test/test_terrain_gt_probe.py tools/test/test_terrain_sensor_audit.py tools/test/test_archive_guard.py`. C++ terrain tests additionally cover immutable bucket expiry, monotonic projection time, height overflow, shallow hole walls, insufficient XY support and two-direction interpolation.
- Opt-in physical calibration capture: with the external archive mounted and **no navigation/other command publisher**, run `OMNILRS_TERRAIN_GT_PROBE=true OMNILRS_TERRAIN_CALIBRATION=true OMNILRS_TERRAIN_DRIVE=true OMNILRS_GT_TF_ENABLED=false docker compose -f docker-compose.lunaryard.yml -f docker-compose.archive.yml up -d --no-deps astrobot-sim`. This teleports once to the isolated analytic platform before measurement, then actually drives a bounded 2 m out-and-back route. `OMNILRS_TERRAIN_DRIVE_PATTERN=diagonal15` selects the separately declared 43-view diagonal experiment, still under validation; default straight has 41 views. Never move manually alongside this collector.
- Score only a completed, audited capture: `python3 tools/score_terrain_calibration.py /archive/<capture-run> --multiview --library /archive/<frozen-library-run>/liblayered_terrain.so` in the sourced ROS dev container. Allocates a fresh score run, preserves failed attempts and reports worst dropoff false-free over all frames. `python3 tools/plot_terrain_calibration.py <capture> <score-run>` adds an exclusive-create diagnostic figure. GT labels stay in the scorer; only settled scans are mapped. See `docs/implementation/gt-terrain-status.md` for actual results and blockers.
- Archive failure is a hard stop for captures/replay, never authorization to write the same workload onto the workspace/root filesystem. Verify the external device is mounted before starting; recreate stale container bind mounts after reconnecting it.
- Sensor-mount experiments are opt-in: `OMNILRS_TERRAIN_LIDAR_PITCH_DEG=10` rotates the live `vlp16` parent frame, not the asset file or only its LiDAR child. Every settled viewpoint must match published `base_link -> vlp16` TF to USD physics geometry. Score with `--reference-capture /archive/<original-untilted-capture>`; the scorer refuses tilted captures without this fixed reference and TF audit. `OMNILRS_TERRAIN_DRIVE_PATTERN=spin360` selects 25 settled views at 15-degree increments, zero commanded translation, a 25 cm drift limit and full-circle inflated-footprint clearance checks covering that entire drift envelope about the original pose. It is a calibration acquisition experiment, not a navigation recovery policy. The first 4 cm-limit attempt stopped safely on skid drift and remains a failed artifact.
- Run new TF/reference/rotation/gate/figure contracts with `python3 -m pytest -p no:cacheprovider tools/test/test_terrain_sensor_audit.py tools/test/test_calibration_drive.py tools/test/test_calibration_score.py tools/test/test_terrain_gates.py tools/test/test_terrain_plot.py` (NumPy/pytest; Matplotlib for figure contracts). Diagnostic figures verify and retain the scored reference even when archive paths move from Docker to host.
- `python3 tools/terrain_sampling_study.py /archive/<completed-straight-capture>` records an ideal-flat-plane screen of 0/5/10/15-degree pitches using captured sensor angles and poses. It ignores occlusion, is never mapper input, and cannot establish acceptance. Real captures and unchanged safety gates are still required.
- Exact frozen-fixture regression: `python3 tools/compare_terrain_replays.py <baseline-score> <candidate-score>` (NumPy). Requires identical input/reference provenance and all per-frame records and final layered arrays, allowing library changes but excluding nondeterministic timing. It rejects failed/incomplete/mismatched fixtures; use current scorer output format for both runs. Contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_terrain_replay_compare.py`.
### Native timing, support and resource gates

- Native-tick GT isolation: add `OMNILRS_TERRAIN_NATIVE_CAPTURE=true` to a guarded physical capture. `python3 tools/audit_native_terrain.py <completed-capture>` verifies all native hashes and reports actual scan cadence/current-versus-previous GT pose geometry. Only after that gate passes, run `python3 tools/replay_native_terrain.py <capture> --reference-capture <original-reference> --library <frozen-library> --archive-root /archive`. Every native chunk is integrated; projection and full-sequence positive/dropoff scoring run at 10 Hz. GT terrain remains evaluator-only. The scorer refuses failed/incomplete captures, wrong cadence, skipped native ticks, missing surface rays, or incorrect poses.
- Native replay `--grid-phase` accepts only the declared 0/0.025/0.05/0.075 m XY offsets. Evaluate all four with one frozen core, keeping the same raw inputs, GT reference, 10 cm resolution and gates. This is grid-alignment stress on development data, not new physical or natural-terrain holdout evidence. Manifest/figures record the shifted geometry; exact replay comparison rejects different grids.
- ROS full scans remain un-deskewed. Native-enabled viewpoint audits check all exact native XYZ acquisition candidates (including ambiguous times) against active surfaces and published extrinsic TF. They do not supply a single valid scan pose: the ordinary stop-scan scorer rejects this audit mode. Failed raw inputs and every attempted run remain archived.
- `OMNILRS_TERRAIN_DRIVE_PATTERN=centered` declares the separate 41-view route `0 -> +1 -> -1 -> 0` m, with the same 4 m travel as the original route. It retains per-step inflated-footprint/floor queries, sole command ownership, heading/lateral limits and watchdog. This is a GT-controlled acquisition experiment, not a navigation recovery policy or authorization for unguarded reverse motion.
- `OMNILRS_TERRAIN_DRIVE_PATTERN=centered_diagonal15` declares the same centered 4 m route along a 15-degree heading, with bounded origin rotations (43 views total). This tests lateral viewpoint diversity with the original fixed reference, unchanged evidence lifetime and safety gates; it is development data, not a held-out acceptance run.
- `OMNILRS_TERRAIN_DRIVE_PATTERN=centered_sweep15` requires native capture and follows that same 4 m/15-degree path with six endpoint/rotation views, avoiding intermediate acquisition stops. Its 2 m leg has a 20 s deadline; 0.15 m/s speed, per-step footprint/support, 1.5 s stall and 0.5 s wall-heartbeat guards are unchanged. All moving native chunks are audited/scored; no longer evidence lifetime or relaxed denominator is permitted.
- The experimental terrain library uses OpenMP when available, at most four workers, only for independent cell classification and clearance checks. Set `OMP_NUM_THREADS=1` for serial regression or a smaller CPU budget; non-OpenMP builds remain supported. Exact replay comparison and separate resource stress are required after parallelization changes.
- Timing/native/regression contracts (host Python + NumPy/pytest): `python3 -m pytest -p no:cacheprovider tools/test/test_simulation_timing.py tools/test/test_terrain_native_capture.py tools/test/test_native_timing_audit.py tools/test/test_native_replay.py tools/test/test_terrain_sensor_audit.py tools/test/test_calibration_drive.py tools/test/test_calibration_score.py tools/test/test_terrain_gates.py`. Source `/etc/ros_setup.sh` and the built overlay for the complete `tools/test` suite.
- Resource stress: `python3 tools/stress_terrain_resources.py --library <frozen-library> --archive-root /archive` (NumPy, external archive). Fixed 200x200 cells, four height layers, 320 inputs at 10 Hz, 48k points/input; stops at 2 GiB diagnostic RSS, and independently gates 1 GiB / 50 ms p95 / 100 ms p99. This adversarial resource profile is not a terrain-accuracy or live-ROS test. Profile contract: `python3 -m pytest -p no:cacheprovider tools/test/test_terrain_stress.py`.
- Also run resource stress with `--profile dense_flat`: same 320 inputs, 10 Hz and 48k points/input, but one broad ground layer exercises connectivity and whole-cell support. Both profiles count; an occupied-grid-only pass cannot establish free-surface compute performance.
- Whole-cell support contracts in `tools/test/test_layered_terrain_calibration.py` include unsupported hole rims across four grid phases and a rotated hole, spatial-certificate expiry, and worker-count determinism. C++ contracts distinguish center-only brackets (insufficient for full-cell support) from an actually supported collinear-return gap. XY-presence/corner-sector metadata is evidence, not GT labels or vertical voxel occupancy; 10 cm output cells and the 8 cm height requirement remain separate.
- Experimental rotation-invariant angular support is tested in `test_layered_terrain` (same CTest runner): six-spoke rotations, missing 90/120-degree wedges, uncertain XY-region bounds and excessive pairwise span. Existing Python hole/expiry/worker tests must still pass. This candidate supplements fixed sectors with actual bounded bearings; it is not promoted until frozen native replays pass every sequence-safety gate. Track outcomes in `docs/implementation/implementation-plan.md`.
- The same C++ runner exhaustively checks all 256 true-sector masks against 256 within-sector boundary placements for the early gap-rejection optimization, including circular wraparound. Mask rejection is never a support certificate. Require exact frozen replay comparison and both resource profiles after changes; no timing retry replaces a failed attempt.
- The rejected boundary-crossing fallback prototype is retained only in `test/angular_boundary_fixture.hpp`, exercised by the same `test_layered_terrain` CTest runner: X-axis six-spoke prefilter reproducer, missing wedges, uncertainty regions crossing sectors, excessive span and regions containing the corner. It adds no final reference coverage and fails native latency; the active mapper does not use this pool. Supplemental masks alone never certify support.
### Acquisition diversity, coverage and diagnostics

- `OMNILRS_TERRAIN_DRIVE_PATTERN=wide_sweep15` declares a separate **4.8 m** native-only experiment: origin yaw 0, rotate to 15 degrees, translate `0 -> +1.2 -> -1.2 -> 0`, stop at the travel heading (five views). Its longitudinal bound is +/-1.28 m; the 0.08 m lateral bound, 0.15 m/s speed, 20 s longest-leg deadline, footprint/floor checks and watchdogs remain unchanged. Driver/scorer contracts are in `tools/test/test_calibration_drive.py` and `tools/test/test_calibration_score.py`. The September 11 capture passed native audit but failed all four coverage offsets (91.64–92.55%); safety passed. Never silently substitute its larger acquisition dose for the original 4 m comparison.
- `wide_sweep15_return` is the separately declared terminal-turn experiment: exactly the five `wide_sweep15` targets, then a bounded origin turn back to yaw 0. Translation remains 4.8 m and all guards/lifetime/reference stay fixed; score all four offsets with the same frozen baseline. Driver/scorer tests verify the identical prefix, required final view, native-only capture and no off-origin rotation. Its first attempt failed before measurement with host CUDA error 999. After user recovery, the September 13 capture passes native audit and retains the exact original 1,152-chunk prefix; all four baseline offsets still fail coverage (94.23–94.87%) while passing sequence safety. Consult `docs/implementation/implementation-plan.md`; failed inputs remain ineligible.
- Coverage attribution: `python3 tools/diagnose_terrain_coverage.py <capture> <score-run>` (NumPy/Matplotlib). Verifies the scored reference, final cost/reason hash and metrics, then exclusively creates `coverage_diagnostics.json`. Counts every original fine flat-reference cell by cost/reason, including unknown and outside-map cells; GT remains evaluator-only. Contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_terrain_coverage_diagnostics.py`.
- Coverage diagnostics also partition the fixed calibration-platform XY quadrants, retaining the global flat denominator in every subgroup. These are explanatory regions, never candidate-dependent visibility masks or separate acceptance gates. The same diagnostic contracts cover all four grid phases, axis centers and empty quadrants.
- Opt-in rejection telemetry: explicitly build `layered_terrain_diagnostics` in the `slam_mapping` CMake build. It is excluded from normal builds/install and does not alter the six map outputs. Set `TERRAIN_LIBRARY=<default-library>` and `TERRAIN_DIAGNOSTIC_LIBRARY=<diagnostic-library>` for `python3 -m pytest -p no:cacheprovider tools/test/test_terrain_diagnostics.py` (NumPy/pytest, no ROS). Native replay requires `--diagnostics` for instrumented libraries and excludes their compute acceptance. Require exact per-frame/final-array comparison to the frozen uninstrumented replay; telemetry is never support evidence or GT input.
- Boundary regression fixtures cover mirrored upward/downward faces, ground occluded by a face, four grid phases, three rotations, slopes -0.15/0/+0.15 and immutable expiry in that same diagnostic suite. Independent continuous wall/hole probes must never be free. The signed-only wall candidate passed recorded courses but failed a rotated wall sliver; its runtime hook is removed and its prototype is test-only. On slopes, below-plane flags are relative to the estimated plane, not GT: a pre-existing low-wall ground-fit error remains open. See `docs/implementation/implementation-plan.md`; do not remove the conservative extent guard based only on wall sign.
- Native audit outliers retain point indices, original ray/endpoint, direction norm and up to eight ordered hit identities/positions/normals. A later matching floor hit does not replace an incorrect first hit; the 1 mm / zero-missing gate is unchanged. Historical recordings remain unchanged and failed recordings remain ineligible.
### GPU preflight and offline documentation

- Check `nvidia-smi` before new Isaac capture. A driver/kernel-library mismatch requires host repair; do not change drivers, unload modules or reboot implicitly. Background system-update compilation also invalidates assumptions of an idle timing benchmark; retain failed results and report contention, never select the best run as a robust pass.
- Also run `python3 docker/check_cuda_runtime.py`: NVML visibility alone does not establish working CUDA. The normal Isaac launcher now requires this bounded context/allocation probe before world construction. Host-only mocked contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_cuda_runtime.py` (pytest, no GPU/ROS). Error 999 with a kernel recovery request is a host-repair blocker, not permission to reset hardware or repeatedly launch captures. Preserve failures; see `docs/implementation/implementation-plan.md`.

- Offline algorithm walkthrough contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_terrain_algorithm_overview.py` (host Python + pytest; no ROS, GPU or archive). Checks embedded fixture provenance, retained/rejected cell outcomes, PNG completeness and local links. Browser interaction/layout checks are separate from these static contracts; this page is not a new terrain-acceptance run.

### Shadow capture, pose qualification and IMU checks

- Shadow timing/mount contracts (host NumPy/SciPy/pytest): `python3 -m pytest -p no:cacheprovider tools/test/test_shadow_terrain.py tools/test/test_shadow_inputs.py`. Set `TERRAIN_LIBRARY=<frozen-liblayered_terrain.so>` to include C++ reconstruction/expiry tests. `tools/export_shadow_trajectory.py <completed-revision-run>` requires the sourced `astrobot_interfaces` ROS overlay and external archive; it preserves source hashes and clock-bounded receipt times, not just final poses. `tools/audit_shadow_inputs.py <native-capture> --prepare-config` freezes an opt-in mount-corrected configuration, never changes production, and does not establish live TF/timing readiness.
- Guarded shadow-capture contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_shadow_capture_gate.py` on the host (NumPy/SciPy/pytest); add `tools/test/test_shadow_capture_ros.py` in the sourced ROS overlay on isolated DDS domain 179 for real MCAP serialization, recorder failure paths and zero-command lease enforcement. Build `localization_supervisor`, `glim_slam_launch`, and `astrobot_launch` first. The opt-in `terrain_shadow.launch.py` has no mapper/Nav2/command publisher and always withholds navigation permission. `tools/run_shadow_capture.py --config <audited-5-degree-config> --archive-root /archive` records before starting that launch, waits for post-teleport calibration readiness, and grants a capture-specific dual-clock lease only after live USD/TF/GLIM mount checks and runtime provenance. Start it before the simulator with the existing guarded `wide_sweep15_return`/5-degree/native flags plus `OMNILRS_TERRAIN_SHADOW=true`; GT TF must remain disabled. This is collection infrastructure, not moving-terrain acceptance. Never run beside navigation or another command publisher. Recheck archive binds after drive reconnects; old containers can retain stale mounts.
- Paired shadow-pose qualification: after native audit and trajectory export, `python3 tools/audit_shadow_pair.py <native-capture> <recorder-run> <trajectory-export> --archive-root /archive` verifies cross-run identities, hashes, full-route coverage and every native chunk's pose association. Reports fixed-home first-published versus retrospective corrected SE(3) errors without fitted alignment, dropping chunks, or extrapolation. It does not score terrain or evaluate causal map projection. Host contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_shadow_pair.py` (NumPy/SciPy/pytest). Live mount checks use the precise settled USD/TF IMU origin; the older millimetre-rounded offset misses it by 0.547 mm. The old authored asset pose is not the resolved fixed-joint pose. See the implementation ledger for failed attempts and frozen candidate configs; never relax the live matrix tolerance to make a config pass.
- Shadow viewpoint audits process at most 100 ray pairs per stationary tick, retaining all selected rays and every ambiguous native candidate. Two scans are reserved per view; unselected later scans are counted explicitly. This changes held acquisition duration, not the 0.5 s watchdog or any geometry/safety gate. `tools/test/test_terrain_sensor_audit.py` covers bounded work and the frame quota. `tools/test/test_calibration_drive.py` additionally checks atomic/exclusive completion markers; the ROS shadow tests check sealing without navigation authority and reject post-finish motion. A failed recorder remains ineligible even if the simulator independently completed its route.
- Offline pose-only terrain sensitivity: add `--shadow-pose-audit <qualified-pair-run> --shadow-pose-mode first_published` (or `final_revised`) to `tools/replay_native_terrain.py`. Same finite native returns, fixed reference, 10 cm grid, acquisition timestamps, projection schedule and safety gates; only poses change. Both modes deliberately exclude availability-delay scheduling and revision rebuild cost, so compute/live-causal acceptance is disabled. All chunks and pose-file hashes must match the qualified pair. Contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_shadow_pose_replay.py tools/test/test_native_replay.py tools/test/test_terrain_replay_compare.py`. Exact replay comparison rejects different pose modes/source provenance even if final arrays happen to coincide.
- Registration/IMU diagnostic contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_imu_consistency.py tools/test/test_terrain_registration.py` (host NumPy/SciPy/pytest; set `TERRAIN_LIBRARY=<frozen-core>` for terrain cases). Four strict registration **xfails** are an open false-obstacle failure, not acceptance. In the sourced ROS overlay, `python3 tools/audit_shadow_imu.py <native-capture> <recorder-run> <qualified-pair-run> --archive-root /archive` exclusively archives fixed-clock kinematic residuals and independent frame checks. It verifies recorded hashes, reports all code-derived clock hypotheses and never fits/applies an offset or supplies GT to GLIM. Component clocks and stationary physics-rate delivery have been checked, but moving accuracy remains unvalidated; no blind IMU retimestamping or mapper threshold relaxation is authorized by this diagnostic.
- Physics-IMU contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_physics_imu.py` (host NumPy/pytest, no ROS/GPU). Opt-in `OMNILRS_PHYSICS_IMU=true` with the archive compose overlay adds a **separate** `/astrobot_0/imu_physics` stream and its `/clock` diagnostics; it does not replace legacy IMU or change GLIM input. The sourced-ROS `tools/check_physics_imu_capture.py --archive-root /archive` records a bounded stationary 10 s comparison, refusing any command publisher. Start recorder before the simulator; no navigation or driver alongside this test. Isolated `tools/isaac_imu_clock_probe.py` runs in Isaac Python after same-container CUDA preflight, with external archive, no ROS/robot and no estimator. It retains reset-clock and every-physics-step observations on a rotating cube. Sensor orientation can lag while gyro is current; never infer a whole-message offset from orientation alone. See the implementation ledger for actual qualification status.
- Installed Nav2 primitive probe is standalone/non-installed: after `/etc/ros_setup.sh`, use `AMENT_PREFIX_PATH=/opt/ros/jazzy CMAKE_PREFIX_PATH=/opt/ros/jazzy cmake -S tools/nav2_shadow -B /tmp/nav2-shadow-jazzy-build -DCMAKE_PREFIX_PATH=/opt/ros/jazzy`, then `cmake --build /tmp/nav2-shadow-jazzy-build -j2`. This explicit single-prefix build avoids mixed Space ROS/Jazzy type-support exports; it is not a change to mission launch setup. Run `LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$LD_LIBRARY_PATH NAV2_SHADOW_PROBE=/tmp/nav2-shadow-jazzy-build/nav2_shadow_probe TERRAIN_LIBRARY=<retained-library> REJECTED_TERRAIN_LIBRARY=<rejected-library> python3 -m pytest -p no:cacheprovider tools/test/test_nav2_shadow.py`. Probe subprocesses use loopback DDS domain 179 and disable external log files. Two frozen wall-safety cases currently **xfail**; that is a failed safety gate, not acceptance. `tools/run_nav2_shadow_check.py --library <library> --probe <executable>` archives the measurements and exits nonzero for those failures. See the implementation ledger for scope and artifacts.

- Paired moving IMU contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_paired_imu.py` (host NumPy/SciPy/pytest). Use `tools/run_shadow_capture.py --require-physics-imu` with the declared guarded shadow route and `OMNILRS_PHYSICS_IMU=true` to require both streams before motion; GLIM still consumes legacy. After native/pair qualification, `tools/audit_shadow_imu.py <capture> <recorder> <pair> --compare-physics --archive-root /archive` compares identical GT windows, without fitted offsets or selected residuals. Missing ticks/clock faults invalidate the comparison. The 30 Hz GT sampling is a diagnostic limitation, not justification for using GT in estimation. Actual ROS message/MCAP contracts remain in `tools/test/test_shadow_capture_ros.py` (sourced overlay, DDS domain 179).

### GLIM replay, accuracy and candidate experiments

- Same-recorded-input GLIM A/B contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_paired_glim_replay.py` (host NumPy/SciPy/pytest). `tools/glim_shadow` is a standalone non-installed C++ probe linking the installed GLIM callbacks, with a required CV-enabled ABI check. Build against one Jazzy prefix plus the existing `glim_ros`, `glim`, and `astrobot_interfaces` overlays; mixed Space ROS/Jazzy CMake discovery fails. In the sourced overlay, `tools/run_paired_glim_replay.py prepare <capture> <recorder> <qualified-pair>` freezes original post-calibration CDR/order and embedded launch-time configs on `/archive`; `run <prepared> --mode legacy|physics --probe <executable>` creates an exclusive attempt; `compare <legacy-run> <physics-run>` rejects any other config/input/binary difference. Replays publish no GT or motion command and never regenerate synthetic RTK. Callback counts, full native pose brackets and all source hashes are mandatory. Output availability is offline bookkeeping, not live timing acceptance. See the implementation ledger for artifact IDs and actual terrain outcomes.

- Initial-origin diagnostic contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_glim_anchor.py tools/test/test_paired_glim_replay.py` (host NumPy/SciPy/pytest). `tools/audit_glim_anchor.py <native-capture> <recorder> <qualified-pair> [<qualified-pair> ...] --archive-root /archive` reports identical physical LiDAR/base/IMU origins and first-sample-only world/body offset hypotheses, never a full-trajectory fit or mapper correction. The standalone replay's opt-in `--mode lidar_only` is a separate CT-GICP backend ablation: only recorded clouds/clock, no IMU or RTK, no local/global mapping, identity LiDAR-centered state. It retains the recorded global-shutter approximation and rejects nonidentity export extrinsics. Its `lidar_only_audit.json` is diagnostic, not a qualified terrain pose source or IMU-only A/B. No GT is supplied to GLIM; the first-native-pose alignment is evaluator-only. Production configuration is unchanged.

- GT-start accuracy contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_glim_gt_start.py tools/test/test_glim_accuracy.py tools/test/test_glim_surfaces.py tools/test/test_paired_glim_replay.py` (host NumPy/SciPy/pytest). Rebuild the standalone `tools/glim_shadow` probe before using `run_paired_glim_replay.py run <prepared> --mode legacy --initial-state-source gt-start --probe <executable>`; add `--trace` for read-only stage/factor callbacks. GT seeds only the first cloud using recorded `gt_map` poses and verified mounts, not the later native-scoring start. A 1 s stationary window and first-output <=1 cm / 0.1 degree are mandatory. The loaded GLIM/GTSAM lunar-gravity probe and `/proc/self/maps` library hashes are archived; production is not installed or changed.
- `tools/diagnose_glim_accuracy.py <capture> <qualified-pair>` archives full-sequence base-frame absolute/1 s relative errors, GT-motion phases, figures and (when available) estimator-stage errors. `tools/compare_glim_diagnostics.py <capture> <untraced> <traced> <untraced-repeat>` reports every repeat and trace difference, plus a 100.1 ms endpoint-motion bound at 5 m. No fitted alignment, selected best run or timing acceptance. `tools/diagnose_glim_surfaces.py <capture> <qualified-pair>` evaluates GT-selected top/side returns separately from flat-floor Z error. The analytic course has small boxes and an unobserved large plateau; this cannot establish natural large-rock recall or localization acceptance. All three require the external archive; accuracy plotting also needs Matplotlib. See the ledger for remaining gates and replay variability.

- GLIM replay repeatability contracts (host NumPy/SciPy/pytest): `python3 -m pytest -p no:cacheprovider tools/test/test_glim_trace_comparison.py tools/test/test_glim_accuracy.py tools/test/test_paired_glim_replay.py`. In the sourced ROS overlay, `tools/compare_glim_traces.py <left-run> <right-run>` archives processed-cloud, consumed-IMU and GPS-factor differences; this is attribution, not acceptance.
- Standalone revisable-target candidate: configure `tools/glim_shadow` with the same explicit single Jazzy/installed-GLIM prefixes as the replay probe, then build `--target test_revision_target` and run `test_revision_target <frozen-run/config>`. This exercises actual target reconstruction, full-SE(3) corrections, immutable sensor evidence, repeat rebuilds, bounded expiry and marginalized-frame lifetime. The candidate plugin is non-installed. Replay it only with `--execution-profile serial --initial-state-source gt-start --mode legacy --cpu-plugin <librevision_target_cpu.so> --target-policy smoothed-window`; retain a same-plugin `--target-policy frozen` control and every failed attempt. No terrain or navigation acceptance follows from these contracts.
- `--absolute-schedule measurement` is an explicitly non-causal offline diagnostic for those serial GT-start replays: recorded RTK CDR is supplied by measurement-time snapshot before each solve rather than by wall arrival. Never claim live accuracy from it. Build `--target test_absolute_schedule` and run `test_absolute_schedule <prepared-run/inputs>` for exact-CDR, no-future-fix, no-duplicate and monotonic-cutoff contracts. The normal/default schedule remains recorded receipt.
- GT-start candidate replay now accepts `--mode physics` as a separately declared same-recording sensor ablation; never combine it silently with a core change. `--scan-information geometry` is a separate opt-in candidate shaping unchanged scalar scan precisions by mean point-to-plane Jacobian information. Build/run `test_scan_geometry` for planar nullspace, rotational equivariance, duplicate/sign invariance and invalid-geometry contracts. Build `glim_geometry_probe` and run `tools/diagnose_glim_geometry.py <qualified-serial-run> --probe <executable>` to archive all-scan normal/geometry spectra without GT; spectra are not calibrated covariance or correspondence-quality proof.

- Causal absolute-association experiment: build `closed_window_abs` and `test_closed_window` in the standalone GLIM probe build; run the latter directly. `--absolute-plugin <libclosed_window_abs.so> --association-policy closed-window` keeps original recorded receipt delivery and waits for the full existing association window from an ordered accepted-fix source. It cannot be combined with the noncausal measurement schedule. Repeatability is not live timing acceptance.
- `--backend gpu` is a separate installed-backend ablation using frozen GPU registration settings and unchanged sensor/seed/lag policy; it cannot be combined with CPU-specific candidates. The September 21 trial fails horizontal/yaw gates despite good plane alignment. `--scan-information registration-mean` is a non-promoted CPU unary-curvature experiment, normalized by source count; `test_scan_geometry` also covers positive-semidefinite curvature, duplicate normalization and invalid input. Neither candidate is enabled in production; preserve every failure and consult the implementation ledger.

- Fixed-deadline pose diagnostic: `python3 -m pytest -p no:cacheprovider tools/test/test_glim_latency.py` on host; then `tools/diagnose_glim_latency.py <native-capture> <qualified-pair>` in the sourced ROS environment/external archive. It evaluates all acquisitions at exactly +200 ms (two declared scan periods), using only revisions available by that deadline. Missing brackets/tail coverage fail the whole run; final revisions cannot leak backward. This is offline receipt-bound sensitivity, not a live delay policy or terrain acceptance.

### Natural holdouts, controlled motion and terrain isolation

- Independent natural holdout (no navigation/command publisher, mounted archive):
  start `tools/run_shadow_capture.py --config <frozen-passing-run/config> --require-physics-imu --natural-stationary --wall-seconds 900`
  in the sourced ROS overlay, then start only the simulator with
  `OMNILRS_NATURAL_HOLDOUT=true OMNILRS_ENVIRONMENT_SEED=43 OMNILRS_ROCKS_ENABLED=true OMNILRS_PHYSICS_IMU=true OMNILRS_GT_TF_ENABLED=false`
  and calibration/drive/probe/native-capture flags false. This opt-in collector uses
  the same 5-degree mount, never teleports or commands motion, and requires 30 s
  stationary native evidence with the unchanged 0.5 s lease and first-hit gates.
  Export with `tools/export_shadow_trajectory.py <recorder>`, then qualify using
  `tools/audit_natural_holdout.py <capture> <recorder> <export> --frozen-candidate <frozen-passing-run>`.
  Live initialization remains the supervisor's normal start; GT-start is a
  separate qualified replay. Never label stationary success moving acceptance.
  Host contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_natural_holdout.py tools/test/test_runtime_manifest.py tools/test/test_shadow_capture_gate.py tools/test/test_terrain_native_capture.py`.
  Actual recorder contracts: `tools/test/test_shadow_capture_ros.py` in the sourced
  ROS overlay, isolated DDS domain 179. All failed attempts remain ineligible.
  Optional first-hit identity diagnostics stream to hashed per-chunk sidecars;
  retaining their nested object graphs caused a measured 816 ms Python GC pause.
  Do not disable GC or relax leases to mask it. `tools/diagnose_natural_surfaces.py
  <capture> <qualified-pair>` reports observed surface placement, not terrain recall.
  Native audit frames now retain immutable encoded JSON, with only the bounded
  16-frame association window decoded; the same native-capture contracts verify
  2,400-frame round-trip identity and mutation isolation. Global GC stays enabled.
  Host contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_natural_surfaces.py tools/test/test_terrain_native_capture.py`.
  Natural mode publishes GT poses immediately after immutable simulation-tick
  snapshots, disabling only their timer to prevent rasterization from dropping
  GT samples. Default mode is unchanged. This also changes synthetic RTK input
  cadence: do not compare different recordings as a same-input A/B. Host contracts:
  `python3 -m pytest -p no:cacheprovider tools/test/test_gt_pose_publication.py tools/test/test_glim_gt_start.py`.

- Pure-yaw drift reproduction (October 8): same recorder-first selected headless
  workflow, route `turn45`, recorder `--execution-profile turn45_bounded_v1
  --wall-seconds 900`, host `SELECTED_EXECUTION_PROFILE=turn45_bounded_v1`.
  Existing +45°/-45°/home acquisition has zero commanded translation and no
  SLAM centering; GT controls yaw endpoints only. The new profile changes only
  the archive quota to a stricter 16 GiB; the 50 GiB reserve, 900 s envelope,
  route hash, physics and all motion guards are unchanged. Optional
  `/astrobot_0/joint_states` is recorded if published, never required for GT/SLAM
  readiness. Start-point natural terrain is not the later detour location or
  a flat-plane comparison. Tests: `test_natural_motion.py`,
  `test_inspection_config.py`, `test_costmap_scenarios.py`, and sourced domain-179
  `test_shadow_capture_ros.py`. Failed captures remain diagnostic-only.
- Controlled natural motion contracts (host NumPy/SciPy/pytest, no ROS/GPU):
  `python3 -m pytest -p no:cacheprovider tools/test/test_natural_motion.py tools/test/test_natural_holdout.py`.
  ROS recorder/supervisor contracts additionally require the freshly built
  `astrobot_core`, `localization_supervisor`, `glim_slam_launch`, `astrobot_launch`
  overlay: run `tools/test/test_shadow_capture_ros.py` and
  `astrobot-lab/src/localization_supervisor/test` under pytest in DDS domain 179.
  Opt-in recorder `--natural-motion straight_slow --require-physics-imu` pairs with
  simulator `OMNILRS_NATURAL_HOLDOUT=true OMNILRS_NATURAL_MOTION=straight_slow`,
  seed 43, rocks/physics IMU enabled and calibration/GT TF/navigation disabled.
  Other separately declared patterns are `turn15`, `turn45`, `combined10`,
  `combined15`. No commands until collision clearance, immutable live GT seed,
  first-output verification and recorder readiness all pass. Failed survey tiles
  remain archived. The user-approved `relative_surface_v2` separates 2 cm
  tangent-compensated steps/plane-fit roughness from a 0.35 absolute incline cap;
  tests include smooth ramps and +/-8 cm steps on those ramps. This experimental
  incline criterion is not vehicle traction/tipping certification. (Oct 21, user:
  sim acquisition guards are convenience only and may be relaxed to keep testing; sim
  world settings such as rock size and terrain need care. Corridor routes now record
  these floor criteria without gating; see "New stress drives".)
  `bounded_minimum_v1` turn control uses a 0.04 rad/s minimum outside the original
  0.012 rad arrival tolerance, with the original 0.10 rad/s cap. Signed endpoint
  and lossy-actuator fixtures run in `test_natural_motion.py`; they are not live
  traction or stall-watchdog acceptance. Each control policy is route-hashed.
  After completion, export the recorder trajectory, then run
  `tools/audit_natural_holdout.py <capture> <recorder> <export> --motion <pattern>
  --frozen-candidate <candidate>`; accuracy/surface/deadline diagnostics follow.
  This is acquisition testing, not navigation acceptance. See implementation ledger.
- Relative-surface mapper fixtures: `TERRAIN_LIBRARY=<frozen-liblayered_terrain.so>
  python3 -m pytest -p no:cacheprovider tools/test/test_relative_terrain_surface.py`
  (host NumPy/pytest, no ROS/GPU). Exercises smooth inclines and 8 cm obstacles on
  those inclines across four grid phases; does not establish scan coverage or
  natural-scene acceptance.
- Terrain mesh storage contracts: `python3 -m pytest -p no:cacheprovider
  tools/test/test_terrain_mesh_storage.py` (host NumPy/pytest, no Isaac). Executes
  the actual grid-building method in isolation and verifies exact vertices,
  triangle ordering, UVs and numeric/untracked index storage. A fresh PhysX/native
  audit remains required after mesh-storage changes; passing host tests alone
  does not qualify simulator timing or collision geometry.
- Natural pose-to-terrain isolation: `python3 tools/replay_natural_terrain.py
  <completed-natural-motion-capture> <qualified-pair> --library <frozen-library>`
  in the archived/sourced dev environment. Integrates every native chunk into
  GT-pose, first-published and final-revised branches with the same 20 m grid,
  10 cm cells, 30 s lifetime and 10 Hz projection. Full-grid class transitions
  include unknown cells. This is offline pose sensitivity, **not terrain GT,
  detection recall, causal scheduling or compute acceptance**. Host contracts:
  `python3 -m pytest -p no:cacheprovider tools/test/test_natural_terrain_replay.py`.
  Strict-deadline diagnostics retain every missed sample in
  `deadline_coverage.json`, including nanosecond next-clock bound excesses;
  `tools/test/test_glim_latency.py` tests that no rounding/trim creates a pass.
  Live exports retain the audited final `/clock` independently of the final
  revision receipt; only hashed export metadata can extend the deadline observer
  horizon. This never creates a pose bracket or permits extrapolation.
  Terrain replay also attributes GT-pose-free to unknown changes to every
  mapper reason code; this remains diagnostic comparison, not a GT label set.

### Terrain candidates, boundary evidence and Nav2 probes

- Normal-confidence candidate gate (host NumPy/pytest, no ROS): set `TERRAIN_LIBRARY` to a frozen candidate and `BASELINE_TERRAIN_LIBRARY` to the retained core, then run `python3 -m pytest -p no:cacheprovider tools/test/test_terrain_normal_confidence.py`. Includes sparse-ring registration thickness and rough/partially observed wall/hole cases across four grid phases and three rotations. Both September 21 candidates were rejected and their runtime hooks removed; archived source/binaries remain reproducible. Expect failures, not acceptance: the suite separately gates absolute hazard safety and newly false-free probes, including regressions inside already-failing cases. Run existing relative-surface, small-object, boundary, expiry and worker tests too. See the implementation ledger for hashes and outcomes.
- Natural replay attribution: `python3 tools/replay_natural_terrain.py <capture> <qualified-pair> --library <diagnostic-library> --diagnostics --reference-run <frozen-uninstrumented-run>` requires the external archive and NumPy/SciPy. It rejects any changed six-output projection hash, input, grid or schedule; telemetry timings are not compute acceptance. Contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_natural_terrain_replay.py`. GT-pose map agreement is not terrain truth or obstacle recall.

- Observation-aware experiment contracts: explicitly build non-installed `layered_terrain_observations` (unchanged maps plus telemetry). Set `TERRAIN_LIBRARY=<frozen-baseline>`, `TERRAIN_OBSERVATION_LIBRARY=<diagnostic-library>` and optionally `TERRAIN_OBSERVATION_CANDIDATE=<archived-candidate-library>`, then run `python3 -m pytest -p no:cacheprovider tools/test/test_terrain_observations.py` (NumPy/SciPy/pytest, no ROS). The September 21 cell-local candidate is rejected: three repeated/jittered-face regressions; its runtime hook/target are removed. `test_observation_moments` is in the mapper CTest runner and checks rank, planarity, incline, translation/yaw equivariance, exact scatter merging and bounded contextual support. Native rigid inputs only; ROS concatenated scans are not qualified. Natural diagnostic replay must still match every frozen baseline projection. Contextual normals use a fixed 9x9-cell native-only neighborhood and a conservative 5 mm center-return residual bound; they never modify map classes or prove whole-cell clearance. Candidate qualification requires per-probe partial-face nonregression, absolute safety, frozen motion/GT and both resource profiles; no promotion from instrumentation or synthetic recovery alone.

- Two-path surface/boundary experiment: explicitly build non-installed `layered_terrain_two_path`; set `TERRAIN_LIBRARY=<frozen-parent>` and `TERRAIN_TWO_PATH_LIBRARY=<candidate>`, then run `python3 -m pytest -p no:cacheprovider tools/test/test_terrain_two_path.py` (NumPy/pytest, no ROS/GPU). Tests 288 partial-face geometries under both stationary repetition and rigid registration perturbations, every-frame absolute safety and per-probe nonregression, a coherent whole-frame flat-gauge counterexample, and immutable boundary expiry. Run existing small-object, slope, hole and worker tests with `TERRAIN_LIBRARY=<candidate>` too. Native bounded-gradient residual is unknown/boundary evidence, not a lowered 8 cm occupied threshold; the earlier context-offset warning is rejected. This opt-in prototype requires frozen replay and resource gates; production stays unchanged until acceptance.
- With `BUILD_TESTING=ON`, opt-in `SLAM_MAPPING_TEST_TWO_PATH=ON` adds the `test_layered_terrain_two_path` CTest target with matching public feature definitions; manual C++ clients must also define both `TERRAIN_OBSERVATIONS` and `TERRAIN_TWO_PATH` to avoid class-layout mismatch. The September 21 candidate remains unpromoted: better natural pose sensitivity, but worse analytic flat coverage and failed resource gates. Coverage/natural diagnostic contracts now distinguish bit-8 native boundary from pooled normal/below-plane evidence: `python3 -m pytest -p no:cacheprovider tools/test/test_terrain_coverage_diagnostics.py tools/test/test_natural_terrain_replay.py` (NumPy/SciPy/pytest; Matplotlib for coverage module imports).

- Pose-conflict regression tracing: `python3 tools/diagnose_terrain_pose_conflicts.py <natural-capture> <qualified-pair> <completed-natural-replay>` in the archived dev environment. Verifies every original chunk and recorded output hash; preserves original point indices and both measurement-time transforms for every final GT-pose-occupied/SLAM-pose-free conflict. Diagnostic selection is not a terrain-truth or recall mask, and the full timeline is retained (not asserted to equal final bucket membership). Contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_terrain_pose_conflicts.py` (NumPy/SciPy, no ROS/GPU).
- Compact two-path evidence is exhaustively checked over 65,536 event sequences and nine bucket splits by `test_observation_moments` (same CTest runner). Full observation counts remain diagnostic-only; same-input moments are stored separately from retained cells and patches are cached only within a native input. Both complete frozen analytic/natural replays must remain exact after representation changes. `test_terrain_two_path.py` additionally covers 24 mixed resolved-end/partial-wall sequences; an occupied source must never clear adjacent unresolved geometry.
- The two-path experiment also parallelizes independent native patch/classification and immutable bucket expiry, max four workers. Within-cell merge order is unchanged; expiry uses a minimum reduction and per-layer votes have unique writers. Existing worker-count/expiry tests must pass along with exact replay and both stress profiles. Optional non-installed `layered_terrain_resolved_boundary` adds `TERRAIN_RESOLVED_BOUNDARY`: only already-occupied sources omit the supplemental native halo; unresolved faces retain it and all pre-existing guards remain. `SLAM_MAPPING_TEST_TWO_PATH=ON` includes its separate C++ contract runner too. It still fails analytic coverage; never enable it in navigation based on synthetic or observed-object success alone.
- Nav2 shadow probe now reports an additional **offline-only** `area_accepted` veto using convex rectangular footprint/cell-area overlap, including lethal/unknown interiors. Original center/perimeter `accepted` remains unchanged; `run_nav2_shadow_check.py` retains its original failure/exit status and separately reports `area_policy`. Rebuild the standalone probe before tests. `tools/test/test_nav2_shadow.py -k 'not rotated_wall'` isolates the six primitive/area contracts; original wall checks remain separate failed gates, not ignored acceptance. September 21 candidate area policy blocks all three measured primitive wall collisions while retaining useful poses, but has no swept-motion or live controller qualification and is not installed.
- Extended offline footprint contracts: rebuild the standalone probe, set `NAV2_SHADOW_PROBE` and Jazzy `LD_LIBRARY_PATH` as above, then run `python3 -m pytest -p no:cacheprovider tools/test/test_nav2_swept_area.py` (NumPy/pytest). Four grid phases, four static yaws, lethal/unknown interiors, endpoint-clear translation/rotation tunneling, explicit full-turn winding, boundary contact, map bounds and deterministic random segments are covered. `swept_area_clear` bounds linear XY / unwrapped-yaw segments with midpoint Minkowski envelopes; it is a collision veto, not a dynamic/kinematic planner or live controller. The 4,096-subinterval budget fails closed. Original primitive safety failures are unchanged and separate. Do not apply straight-segment qualification to curved trajectories without a proven enclosing bound.
- Projection profiling is opt-in at compile time with `TERRAIN_PROFILE` and `include/slam_mapping/terrain_profile.hpp`; each projection writes one stage-timing JSON line to stderr. No production target defines it. Use a frozen instrumented library with the fixed resource input generator, archive stderr, and exclude instrumented timings from compute acceptance. The two-path compact lowest-point cache must retain exact analytic/natural outputs and pass both uninstrumented resource profiles. No thresholds or evidence lifetimes change.
### Live advisory terrain, provenance and acquisition loops

- Live advisory terrain contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_live_terrain_shadow.py` (host NumPy/SciPy; set `TERRAIN_LIBRARY` for real-core replay). Build/install `slam_mapping` with explicit `SLAM_MAPPING_BUILD_TERRAIN_SHADOW=ON` to install the non-authoritative ROS wrapper and candidate. Normal navigation remains unchanged. `terrain_shadow.launch.py terrain_mapping:=true` requires an explicit frozen library, map origin and sensor min/max ranges. The guarded recorder's `--terrain-library <frozen-so>` derives those from the already-declared initial seed/sensor manifest; Isaac additionally needs `OMNILRS_TERRAIN_NATIVE_PUBLISH=true`. No GT pose/labels enter native transport. First-published mapping uses a 200 ms delayed acquisition watermark; non-ODOM revisions, sequence/clock faults and overflow invalidate the map. No revised/global reconstruction is claimed.
- Actual ROS shadow contracts: after sourcing ROS and the `astrobot_interfaces` overlay, run `ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=1 TERRAIN_SHADOW_ROS_TEST=1 TERRAIN_LIBRARY=<frozen-so> python3 -m pytest -p no:cacheprovider tools/test/test_live_terrain_shadow_ros.py`. Requires the external archive and uses only isolated synthetic publishers, never commands/Isaac. Records actual callback order, verifies all six-output projection hashes and checks fail-closed unknown maps. `python3 tools/replay_live_terrain_shadow.py <completed-shadow-node-run>` verifies source/library/input hashes and exact callback-order outputs; this does not establish Isaac/native transport timing, GT terrain accuracy or concurrent GLIM resource acceptance. Recorder integration contracts remain in `tools/test/test_shadow_capture_ros.py`; absent native publication or a mapper fault must not be accepted as a qualified capture.
- Failed-prefix diagnosis is separate: `python3 tools/replay_live_terrain_shadow.py <shadow-node-run> --diagnose-heartbeat-prefix` accepts only an archived `simulation wall heartbeat lost` termination, verifies every retained projection/association and the exact terminal state, and preserves failed qualification. It does not reproduce a wall-clock watchdog offline or qualify the missing route tail. Ordinary replay still rejects failed inputs. Eligibility/source-identity contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_live_shadow_provenance.py` (host NumPy/SciPy/pytest; no ROS).
- Live GLIM accuracy diagnosis: `python3 tools/measure_live_glim_accuracy.py <native-capture> <recorder-run> <shadow-node-run>` in the sourced ROS/interfaces environment with the external archive. Preserves failed-run status; checks fixed-frame base_link GT against native sensor GT, first-published 200 ms-deadline associations and retrospective revisions, with missing tails explicit. No fitted alignment/time shift and no acceptance promotion. Pure contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_live_glim_accuracy.py tools/test/test_glim_accuracy.py` (host NumPy/SciPy/pytest).
- `OMNILRS_NATURAL_MOTION=loop16` / recorder `--natural-motion loop16 --wall-seconds 1800` declares a separate 16 m rectangle (2 m forward, 6 m right, 2 m back across, 6 m home), four 90° turns and original final heading. GT-controlled acquisition only; full live floor/overlap preflight before grant. At most 0.10 m/s / 0.10 rad/s; existing watchdog, stall, footprint, lateral/heading and turn-skid guards unchanged. Route-specific 85 s translation / 75 s rotation deadlines and 1,800 s wall cap do not qualify old heartbeat failures. Tests: `python3 -m pytest -p no:cacheprovider tools/test/test_natural_motion.py tools/test/test_natural_loop.py` (host NumPy/SciPy/pytest). Never replace shorter-route evidence with this larger acquisition dose.
- First `loop16` attempt failed safely at the first corner: 24.2 cm legal turn skid leaves 20.0 cm cross-track error for the next 8 cm-bounded straight leg. The recorded endpoint is a regression in `test_natural_loop.py`; no full-loop acceptance. Live GLIM diagnostics use the existing exact GT-snapshot contract (2 ns round/truncate tolerance, not missing-tick interpolation), and report interrupted, unprojected integration tails separately from verified map inputs. See the implementation ledger before retrying; corner rejoin and incomplete-shutdown invalidation remain open.
- September 22 `loop16` correction declares `frozen_post_turn_connection_v2` (new route hash): freeze the actual first pose of each translation, connect to the unchanged next waypoint, retain 8 cm tracking around that fixed segment and the original surveyed center envelope. Command/reached logs include `segment_start`; completion checks reject changing/fabricated anchors. Same natural-loop/motion test runners apply. This changes the acquisition controller, not SLAM, mapper evidence or safety timing. Earlier failed v1 artifacts remain failed and are not scored as v2 runs.
- The first v2 loop rerun clears the formerly failing turn-to-straight transition and drives 1.99 m of the next leg with 1.98 mm maximum cross-track error, then fails the unchanged simulator heartbeat guard. It is not a completed loop. Preserve that artifact; see the September 22 ledger for accuracy and concurrent p95/p99 compute failures before starting another attempt.

### System field guide

- System field guide contracts: `python3 -m pytest -p no:cacheprovider tools/test/test_system_overview.py tools/test/test_terrain_algorithm_overview.py` (host Python + pytest; no ROS, GPU, archive or motion). Covers local links/anchors, embedded-only assets, accessible controls, source-backed topic/reason/frame distinctions, retained failed-run provenance and shell syntax without executing runbook commands. Browser rendering/search/copy checks are separate; this documentation does not establish runtime acceptance.

## Local terrain v2

- Current direction: [implementation-plan-v2.md](implementation-plan-v2.md).
  Prototype contract: [local package README](../../tools/local_terrain/README.md).
  Historical sweeps are indexed in [tools/legacy](../../tools/legacy/README.md);
  old forwarding paths are not safe current capture runners.
- Host NumPy/SciPy/pytest, no ROS/GPU/motion: `python3 -m pytest -p no:cacheprovider tools/test/test_local_terrain.py`.
  Set `TERRAIN_LIBRARY` to the retained `libresolved_boundary.so` to exercise real
  C++ reconstruction. Legacy forwarding is mocked; original scripts never run.
- Host C++ geometry contracts require g++/Eigen3, no ROS/GPU. Compile
  `astrobot-lab/src/slam_mapping/test/test_layered_terrain.cpp` together with
  `astrobot-lab/src/slam_mapping/src/layered_terrain.cpp`, using `-std=c++17 -O2`,
  `-I/usr/include/eigen3 -Iastrobot-lab/src/slam_mapping/include` and all three
  `-DTERRAIN_OBSERVATIONS -DTERRAIN_TWO_PATH -DTERRAIN_RESOLVED_BOUNDARY` definitions.
  Write the executable into a fresh temporary build directory. Test a build with
  `-fopenmp` under `OMP_NUM_THREADS=1` and `4`, and a separate build without OpenMP.
  Archived binaries can require a newer libstdc++; rebuild locally instead of
  replacing host libraries. These geometry contracts do not qualify local replay timing.
- New offline replay: `OMP_NUM_THREADS=4 python3 tools/replay_local_terrain.py <shadow-node-run> <native-capture> --archive-root <external-archive> --window 2`.
  Window choices are 2/5/30 s. Failed recordings require explicit
  `--diagnose-failed-prefix`; bounded development tests require explicit
  `--max-projections N`. Neither option grants acceptance. Verifies original
  stream/runtime/library hashes, reconstructs callback order, retains failures
  and allocates a fresh external run. No original runtime module is changed.
  Add `--policy first_published --window 30` for the original baseline: every
  recorded six-layer hash/state/association must match before reporting new
  body-relative availability. `--profile` instruments only the local policy and
  cannot establish compute acceptance. Source dependencies are frozen in each run.
- Controlled ablations use the same local runner: `--observation-policy coherent`
  (default) versus `--observation-policy frozen_observation`, `--cells 60|200`,
  and `--window 2|5|30`. Change one factor at a time, fixing the input prefix,
  classifier and current-rover pose. The fixed 6 m query region never shrinks with
  available evidence. Frozen observation means first-used local placement, not the
  historical first-publication runtime. Original baseline rejects these overrides.
- Compare two retained arms: `python3 tools/compare_local_ablation.py <left-run> <right-run> --archive-root <external-archive>`.
  Requires identical frozen candidate dependencies, source data, exposure and one
  changed factor. Produces a fresh diagnostic comparison, not truth/compute acceptance.
- Exact preparation experiment: add `--backend prepared` to the local replay;
  default `reference` remains the naive oracle. Run matching arms with identical
  source versions, window, grid and prefix. The comparison runner checks every
  layer hash, association, origin and availability field for backend-only pairs.
  Additional numerical cache is capped at 64 MiB; terrain integration is still a
  full ordered rebuild. `--profile` remains a separate instrumented diagnostic.
  Default preparation retains filtered native coordinates, not transformed clouds:
  current revision traffic invalidated every exact transform-cache entry in the
  diagnostic. To compare a later semantics-preserving implementation to an archived
  one, use `compare_local_ablation.py --exact <left-run> <right-run>`: differing
  retained implementation sources are declared, but all algorithm factors, inputs,
  poses/associations, layers, origins and query results must remain identical.
  This flag never relaxes geometry equivalence or grants runtime acceptance.
- V2 evaluator/cache host fixtures: `python3 -m pytest -p no:cacheprovider
  tools/test/test_local_ablation.py tools/test/test_local_motion_evaluation.py
  tools/test/test_local_reference_mesh.py tools/test/test_local_pose_lookup.py
  tools/test/test_local_return_cache.py` (NumPy/SciPy/pytest; no GPU/ROS).
  The mesh helper needs explicit collision triangles in a validated reference frame;
  it neither reconstructs missing topology nor certifies full wheel/body safety.
  OmniLRS's `meshSimplification` collider can differ from authored USD/DEM triangles.
  The installed Isaac image has PhysX 107.3.26; inspect its `_physx.pyi`/cooking
  stubs before using APIs from newer documentation. The documented
  [107.3 collision representation API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/extensions/runtime/source/omni.physx/docs/api/python.html)
  exposes convex representation requests, not a validated terrain-triangle export.
  Active scene-query validation or a verified cooked-geometry adapter is required;
  do not switch simulator collision approximations to simplify evaluation.
- Exact local C++ candidates: `python3 tools/build_local_terrain.py --archive-root
  <external-archive> --patch-normal-cache` freezes sources, compiler flags, C++
  fixtures and shared library in a fresh archive directory. Add
  `--sparse-native-patch` for the experimental ordered active-neighbor index;
  `--defer-native` (requires normal cache) snapshots immutable native moments and
  defers only classification votes, allowing parallel work across small inputs.
  Votes flush before projection/expiry and reach their original buckets; ordered
  geometry integration is unchanged. Its 64 MiB / 2048-input work bound does not
  replace the total-memory gate. It intentionally rejects diagnostic-ABI builds.
  `--serial` builds without OpenMP. Test binaries retain assertions even though
  the shared library is release. Run both binaries, layered tests at OMP=1/4,
  and serial tests. `--candidate-library <build-run>/liblocal_terrain.so` is allowed
  only for local replay, never the original baseline. Compare all outputs with
  `compare_local_ablation.py --exact`, then both `stress_terrain_resources.py`
  profiles (`layer_capacity`, `dense_flat`); isolated core timings do not qualify
  repeated local reconstruction. Build/provenance fixtures:
  `python3 -m pytest tools/test/test_local_build.py tools/test/test_local_ablation.py`.
  Exact randomized stencil/expiry fixtures additionally need
  `TERRAIN_LIBRARY=<candidate> TERRAIN_REFERENCE_LIBRARY=<dense-reference>
  python3 -m pytest tools/test/test_local_native_patch.py`. Include these in both
  OpenMP and serial checks; missing library variables cause skips, not acceptance.
- Incremental v2 candidates are a **separate algorithm**, not exact rebuilds.
  Build with `--patch-normal-cache --sparse-native-patch --defer-native --rolling-grid`
  (also build `--serial`). `replay_local_terrain.py ... --window 30 --backend prepared
  --observation-policy frozen_observation --map-policy rolling_incremental
  --candidate-library <build>/liblocal_terrain.so` uses a 6 m rolling grid.
  `--map-policy fixed_incremental --cells 200` is the distinct fixed 20 m control.
  New cells start unknown; there is no raw-history backfill or revised-pose repair.
  Persistent immutable-bucket expiry differs from reconstructing a raw window.
  `compare_local_ablation.py` intentionally refuses these as exact/one-factor
  reconstruction comparisons. Use `TERRAIN_ROLLING_LIBRARY=<rolling-library>`
  for `tools/test/test_local_rolling.py`; ordinary `TERRAIN_LIBRARY` additionally
  enables `tools/test/test_local_incremental.py`'s real direct-stream expiry check.
  Run both resource profiles and uninstrumented replay separately from telemetry.
  Current incremental code uses a FIXED map/local reference and an oldest-frame
  witness; witness changes invalidate the candidate. Earlier recent-anchor
  incremental artifacts have a confirmed frame defect and are rejected.
  `--window 5 --evidence-lifetime 30|60` explicitly decouples raw retention from
  persistent evidence for the rolling variant. A 60 s lifetime is an experimental
  contract change; resource passes at 30 s do not qualify every 60 s workload.
- Completed-loop navigation-gap assessment (October 4; read actual MCAP maps,
  no simulator, no motion commands): in the sourced ROS development image,
  use the offline dual archive alias setup described later in this runbook if
  the loaded-contact run records the original host archive path.

  ```bash
  python3 tools/assess_navigation_gaps.py \
    /archive/revision-d4d0f081b64f47caab3831bf2154ab08 \
    /archive/revision-0113a56294cf4d8f817fe1404a8c820f \
    /archive/revision-070d4d8a70084dbdb5414ad9dc945b35 \
    /archive/revision-3ba6642d9fa64823b47392e8dc757cad --archive-root /archive
  OMP_NUM_THREADS=4 python3 tools/explain_navigation_gaps.py <assessment-run> --archive-root /archive
  python3 tools/assess_grounded_start.py <assessment-run> --archive-root /archive
  python3 tools/render_navigation_gaps.py <assessment-run> --archive-root /archive \
    --explanation <explanation-run> --grounded <grounded-comparison-run>
  ```

  Requires NumPy/SciPy, rosbag2_py, rclpy and nav_msgs. Exact explanation also
  requires the frozen terrain library and unchanged recorded runtime sources.
  Every command allocates fresh external output; preserve earlier results.
  Motion sampling is fixed 1 Hz; floor scores use all recorded valid maps.
  October 4 online follow-up: `assess_navigation_gaps.py` also accepts completed
  short natural routes. It uses their distance/heading waypoints and their own
  archived survey, not loop XY targets. Loop goals and all candidate motions
  remain unchanged. Candidates omit reverse, so a short out/back return may
  have a clear reverse route without scoring forward progress. Turn availability
  is still useful but does not prove that turning reveals enough new ground.
  Loaded wheel locations are measured, 80×120 mm support patches are assumed.
  The grounded-start comparison is explicitly optimistic, not a new policy.
  Run focused tests with `python3 -m pytest tools/test/test_navigation_gaps.py`;
  also run gap-export, support-sweep, floor, costmap and natural-motion regressions.
- Continuous assumed-wheel-patch diagnostics:
  `python3 tools/diagnose_local_support.py <mapper-run> <capture> <loaded-contact-run>
  --window 5|30 --max-projections 320 --candidate-library <library>
  --diagnose-failed-prefix`. Optional `--map-policy fixed_incremental|rolling_incremental`
  selects those explicitly distinct 30 s candidates. Freeze dimensions/motions
  before inference; 80x120 mm patches are assumptions, not measured contact areas.
  All stationary/forward/turn queries including startup remain in `support.jsonl`.
  Timing is instrumented and ineligible for compute acceptance. This checks map
  class coverage only, not body clearance, support age, uncertainty or terrain truth.
  Host tests: `tools/test/test_local_support_sweep.py` and
  `tools/test/test_local_support_diagnostic.py`. Do not reinterpret mapper age/plane
  sigma channels as dependency-aware freshness or pose-uncertainty certificates.
- Frozen-placement GT audit: `python3 tools/diagnose_local_frozen_poses.py
  <incremental-run> <accuracy-run> <capture>`. Requires same-native-tick GT poses,
  audited base/LiDAR mount and matching archived provenance. The reference base
  endpoint is the latest assimilated native acquisition, not the nominal 100 ms
  mapper query stamp. Report its offset and missing GT; never broaden the 2 ns
  representation tolerance or substitute a nearby sample. Age bins count pose
  opportunities, not actual surviving C++ cell dependencies. No alignment, GT
  correction, covariance calibration or consumer-time error pass is implied.
  `--placement current_gauge_of_first_map` is an **attribution-only** comparison
  using the same first map poses transported through the current gauge; it does
  not repair the stored map or handle unsupported global changes. Tests:
  `python3 -m pytest tools/test/test_local_frozen_pose_audit.py`.
- Independent active body reference: `tools/probe_local_physx_reference.py` is an
  isolated static-box Isaac fixture, not a rover/motion collector. First verify
  archive device/bind, host `nvidia-smi` and CUDA allocation in the Isaac image.
  Use `isaac-sim-omnilrs:latest`, `--network none --gpus all`, accepted EULA/privacy,
  explicit `--mount type=bind` archive at `/archive`, repository `tools` read-only
  at `/workspace/terrain-tools`, and `OmniLRS` read-only at `/workspace/omnilrs`.
  Entrypoint `/isaac-sim/python.sh`, script
  `/workspace/terrain-tools/probe_local_physx_reference.py`. Retain container
  stdout/stderr and exit status in a separate fresh archive run; bound startup,
  and stop only the owned probe container if its client times out. No ROS/domain,
  rover, command publisher or terrain modification is involved. Scene queries
  run on the simulation thread at a fixed epoch/tick. Geometry test command:
  `python3 -m pytest tools/test/test_local_physx_reference.py`.
  A body-envelope clear result never certifies wheel support or a safe motion.
- Preferred isolated-fixture launcher: `python3 tools/run_local_isaac_fixture.py
  body|contact|contact_awake --archive-root <external-archive>`. It performs host/image CUDA
  preflight, uses a unique owned container, archives launch/log/exit/inner result,
  and bounds startup/run to 240 s. **An exit code of zero is insufficient**:
  Isaac shutdown has hidden a Python fixture exception. The launcher requires
  exactly one inner run, a successful diagnostic report and no failure marker.
  Contract tests: `python3 -m pytest tools/test/test_local_isaac_fixture.py`.
  The contact fixture loads the actual rover on a known planar floor with all
  command/sensor graphs disabled, zero wheel targets and lunar gravity. It keeps
  all 480 steps, enforces 5 cm base-XY drift and finite-state bounds, and examines
  the final 60 steps for settling/upward wheel contacts. This is a separately
  declared stationary physical diagnostic, not motion acquisition, braking,
  loaded contact-patch extent or a full mission/collision-model qualification.
  Never start fixtures alongside a mission; stop only the uniquely owned fixture
  container on timeout. Compare inner failure/result files, not just console text.
  The launcher refuses already-running astrobot/OmniLRS/Isaac task containers.
  Contact forces can disappear when an articulation sleeps: the `contact_awake`
  protocol explicitly wakes the base every tick and records sleep state. This is
  **instrumented force characterization**, not unchanged mission physics or a
  runtime benchmark. RigidPrim force tracking sets body sleep thresholds to zero;
  stabilization is explicitly left enabled instead of accepting the SDK default
  that disables it. Do not select earlier force-active samples or treat absent
  impulses as unsupported ground. Keep the default-protocol failure separate.
- Relative-motion diagnostic: `python3 tools/diagnose_local_motion.py <accuracy-run> --archive-root <external-archive>`.
  Requires completed accuracy diagnostic output (which may itself describe a
  failed source), NumPy/SciPy, no ROS/GPU. Preserves failed-source status and
  missing endpoints; reports first-publication and retrospective errors separately.
- Flat-ground sensing bound: `python3 tools/diagnose_local_observability.py
  <native-capture> <successful-awake-contact-run> --diagnose-failed-prefix`.
  Requires host NumPy/SciPy, matching asset hashes, native angle provenance and
  the complete 480-step loaded fixture. Archives a fresh diagnostic, no simulation.
  It assumes level ground, constant 0.10 m/s straight motion and a continuous
  angular cone, ignoring occlusion/discrete rings. Native Isaac zenith is positive
  down. A too-short window is ruled out for direct wheel-center ground observation;
  a sufficiently long window is **not** qualified sensing/support. Never use the
  counterfactual flat plane as terrain labels. Analytic contracts:
  `python3 -m pytest tools/test/test_local_observability.py`.
- Bounded mount screen (host NumPy/SciPy; external archive only):
  `python3 tools/diagnose_local_mounting.py <observability-run> --diagnose-failed-prefix`.
  Reads and rechecks the preceding diagnostic's input hashes, uses recorded
  native elevations/column count and an explicitly synthetic azimuth phase, and
  compares absolute pitches +5/+10/+15/+20/+30/-10/-20/-30 degrees. Positive pitch
  points the front beams down. All 10 cm grid phases and band/sector denominators
  stay fixed. The 25-view virtual-yaw arm changes acquisition and is NOT a robot
  command protocol; hit cells are not fully supported/drivable cells. The source
  recording's failure remains explicit. No production asset or sensor TF changes.
- Native mount geometry fixture: `python3 tools/run_local_isaac_fixture.py mounting
  --study-run <completed-mount-screen> --archive-root <external-archive>`.
  Uses the same isolated launcher/preflight/deadline as body/contact probes.
  There is **no rover, ROS, SLAM or motion publisher**. Each pitch is stationary
  in separate analytic flat/8 cm box scenes; records six actual 10 Hz native
  chunks after six warmup ticks, retaining every case. Audits actual USD sensor
  transform, native angles/column counts, <=5 m return masks and ray-depth/XYZ
  against the analytic scene (2 mm maximum error). Raw chunks and hashes are
  archived. It tests geometry and sampling, not classifier detection, optical
  self-occlusion, actual mounting clearance, or moving acquisition. Synthetic
  full-view sampling and native acquisition have separate declared doses.
  Host contracts: `python3 -m pytest -p no:cacheprovider
  tools/test/test_local_mounting.py tools/test/test_local_observability.py
  tools/test/test_local_isaac_fixture.py`.
- Single-LiDAR fixed-budget ray redesign: `python3 tools/diagnose_local_ray_design.py
  <completed-mount-screen> --diagnose-failed-prefix --archive-root <external-archive>`.
  Host NumPy/SciPy, external archive only. Explicit **zero-pitch/roll/yaw mount**,
  preserving measured translation. Original +5-degree calibration remains only
  as source provenance; v2 level manifests cannot silently inherit it.
  32 elevations × 1999 columns, unchanged horizontal coverage and range limits.
  Tests asymmetric uniform and nonuniform elevations (sensor-local, positive up),
  all old ground regions/phases plus two declared 128-placement 8 cm box banks.
  It records all candidates/misses and minimum-range clipping. This is a flat,
  no-self-occlusion diagnostic; no synthetic rays are fed to GLIM or the mapper.
- Custom-ray simulator validation: `python3 tools/run_local_isaac_fixture.py ray_design
  --study-run <completed-ray-screen> --archive-root <external-archive>`.
  Same archive/GPU/container ownership/deadline guards as other isolated fixtures.
  Uses the installed Isaac **Generic range sensor**, not the legacy symmetric
  uniform LiDAR interface; the live native capture adapter is NOT converted.
  Twelve measured 60 Hz ticks after six warmup ticks, fixed nominal 639,680
  samples/s for all candidates. Exactly one active sensor; fresh sensor identity
  per layout/scene avoids stale queued ray tables across `world.reset()`.
  Native raw data is archived before angle/geometry validation. All 63,968
  supplied column/ring identities must appear, actual sample-count sequences
  must match across cases, all <=5 m masks must agree with the analytic scene,
  and depth/XYZ errors must stay <=2 mm. No discarding invalid chunks or extending
  warmup until a preferred result. This does not establish mapper/GLIM runtime,
  moving acquisition, optical self-occlusion or safe-motion acceptance.
  The fixture also reads back and archives actual USD mounting transforms; v2
  level studies reject any nonidentity sensor/base rotation. Earlier v1 tilted
  runs remain historical and can be validated explicitly, not silently substituted
  for the requested level comparison.
  Tests: `python3 -m pytest -p no:cacheprovider tools/test/test_local_ray_design.py
  tools/test/test_local_mounting.py tools/test/test_local_isaac_fixture.py`.
- Quick far-structure retention: `python3 tools/measure_ray_features.py <level-ray-screen>`.
  Host NumPy, archive, local `astrobot-dev:latest` image with gtsam_points headers/
  library and compiler required. Private network-none CPU container compiles
  `tools/glim_shadow/ray_sampling_probe.cpp` against the installed sampler; no
  ROS/mission or GPU kernel workload. Frozen 64 analytical box/ground geometries,
  two layouts, five seeds, exact GLIM random-grid parameters then range filter.
  Retention/extent go-no-go is not SLAM registration accuracy or runtime acceptance.
- Current classifier on level native rays: `python3 tools/assess_ray_terrain.py
  <completed-native-ray-fixture> --library <archived-build>/liblocal_terrain.so`.
  Host NumPy; library provenance must pass `build_local_terrain.validate_build`.
  Baseline/mid-dense, flat/eight-box scenes, all native ticks and four grid phases;
  both cost 0 and cost 20 count as traversable. GT labels enter scoring only.
  Whole-cell positive-area object overlap is excluded from free truth; no inflated
  halo is credited as object detection. Fresh archive includes every map and score.
- Settled-view terrain diagnostic: `python3 tools/run_local_isaac_fixture.py
  ray_terrain_views --study-run <level-ray-screen>`, then `python3
  tools/assess_ray_terrain.py <inner-native-run> --multiview --library
  <archived-build>/liblocal_terrain.so`. Same guarded launcher and fixed deadline.
  Six sensor-only XY placements over known geometry, no rover or ROS commands.
  Native multiview v2 uses closed triangle-mesh boxes at the same analytic bounds;
  convex-cube v1 failed on a grazing return and is ineligible for terrain scoring.
  Depth/mask tolerances are unchanged, with all native samples retained.
  Each records six warmup plus twelve measured ticks; native time stays continuous
  within each layout/scene. Fresh sensor identity at each placement, one active
  sensor. Both layouts receive identical increased dose; do not equate six views
  with a stationary run or a physically safe/live startup maneuver. Assessor rejects
  reset epochs, missing/reordered views and mismatched mounts without retimestamping.
  Contracts: `python3 -m pytest -q -p no:cacheprovider tools/test/test_ray_features.py
  tools/test/test_ray_terrain.py tools/test/test_local_ray_design.py
  tools/test/test_local_isaac_fixture.py`.
- Selected development sensor is frozen in `tools/local_terrain/selected_lidar.json`:
  mid-dense +10..-60, 0-degree mount, original translation/ranges/count/rate.
  Level captures and terrain assessment reject incompatible geometry; this does
  not configure the old live uniform-LiDAR adapter. Keep mount and base-world yaw
  separate, and rotate the off-center origin with base yaw.
- Settled yaw diversity: `python3 tools/run_local_isaac_fixture.py ray_terrain_yaw
  --study-run <level-ray-screen>`, followed by `OMP_NUM_THREADS=4 python3
  tools/assess_ray_terrain.py <inner-run> --observation-yaw --library
  <archived-build>/liblocal_terrain.so`. Twenty-four fixed base yaws (0..345 by 15),
  selected layout only, two scenes, twelve measured ticks after six warmup per
  view. No robot or commands. Continuous clock per scene, all native samples,
  triangle-box geometry, unchanged strict range/mask gate and archive/preflight.
  Distinct acquisition dose; do not call it a robot spin or a timing benchmark.
  Range reports are additive cell-center bands 0–1/1–2/2–3 m relative to the fixed
  original base, plus forward 90-degree sectors; unknown cells remain counted.
- Nonuniform native -> GLIM input contract: `python3 tools/check_generic_glim_input.py
  <completed-native-yaw-run>`. Host NumPy, external archive and local dev image.
  Isolated CPU-only/network-none Docker probe uses one Jazzy header prefix and
  `/opt/astrobot/glim_source`; it changes no mission overlays or ROS topics.
  `generic_lidar_adapter.py` validates every 10,661-sample tick, beam continuity,
  sequence and native 60 Hz cadence; six whole ticks produce a bounded timed
  PointCloud2-compatible window (63,966 native samples, not a rounded full scan).
  XYZ stays sensor-local, misses/range-invalid returns are excluded from GLIM,
  ring/beam identity retained. The original raw terrain stream stays separate.
  Native tick times are rounded to ROS nanoseconds, not subdivided into invented
  per-ray times. No GT pose/labels, interpolation, registration or synthetic IMU.
  The copied experimental sensor config disables global shutter and preserves
  relative point times; production configuration remains unchanged. Installed
  ROS converter, TimeKeeper and preprocessor must preserve coordinates/times and
  acquisition extent for all 96 windows. A separate global-shutter negative control
  must reject collapsed timing. This does not run odometry, estimator callbacks,
  DDS, moving acquisition or runtime qualification. Contracts:
  `python3 -m pytest -q -p no:cacheprovider tools/test/test_generic_lidar_adapter.py`.
- Actual causal geometry comparison: `python3 tools/diagnose_local_reconstruction.py <local-run> <accuracy-run> <native-capture> --archive-root <external-archive>`.
  Uses identical one-second native endpoint pairs, verifies output/input hashes and
  publication availability, and compares coherent poses against first publication.
  Failed-source and bounded-prefix status remain explicit; no terrain-truth claims.
- USD-only inspection: `python3 tools/local_terrain/inspect_vehicle.py <asset.usd>` needs
  pxr, not a simulator world. In the existing Isaac image, use `/isaac-sim/python.sh`
  with `PYTHONPATH=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311`
  and the corresponding `.../bin` in `LD_LIBRARY_PATH`; mount the repository read-only,
  set container network to `none`, and archive stdout/stderr in a fresh external run.
  Plain Isaac Python without this extension path fails to import pxr; do not start
  an Isaac application just to read the USD asset. Geometry bounds do not establish
  loaded wheel contact, braking or safe attitude limits.

## Commit & Pull Request Guidelines

- Commit messages are short and descriptive (examples in history: `devcontainer wip`, `update OmniLRS submodule`). Keep them concise and imperative.
- PRs should include: a brief summary, the commands run (or note if untested), and screenshots/video when changing simulation visuals.

## Selected level LiDAR — moving GLIM experiment (opt-in)

- Live v2 unchanged-revision integration (September 29 evening): the wrapper
  explicitly enables `LocalConfig.allow_bookkeeping`; the default offline core
  remains ODOM-only. A bounded 8,192-frame canonical ODOM ledger verifies entire
  SUBMAP/GLOBAL deltas at 1e-10 rounding tolerance and advances the chain without
  modifying poses or resurrecting old interpolation frames. Snapshots, changed
  poses/extrinsics/stamps, missing/duplicate IDs and deskew invalidation still
  fail closed. This is not general global-correction support.
  Tests: `TERRAIN_LIBRARY=<frozen-so> python3 -m pytest -q -p no:cacheprovider
  tools/test/test_local_bookkeeping.py tools/test/test_local_terrain.py
  tools/test/test_local_terrain_seal.py`. Host `python3
  tools/benchmark_bookkeeping.py` allocates a fresh external run and retains all
  32 timings for 8,192-frame batches; no live/concurrent acceptance is implied.
- Explicit changed-runtime comparison: `OMP_NUM_THREADS=4 python3
  tools/replay_live_local_terrain.py /archive/<sealed-v2-mapper>
  --archive-root /archive --bookkeeping-candidate`. It declares current core/live
  wrapper hashes versus recorded ones, verifies all other dependencies/inputs,
  and compares every original projection exactly except wall timing. Besides
  completed inputs, ONLY the archived `only contiguous ODOM deltas are supported`
  failure is eligible for this **failed-prefix diagnostic**. Its original fault
  remains recorded; the missing route is not replayed or accepted. Ordinary
  replay without this flag retains strict source hashes and completed-only
  eligibility. Historical library manifests may use the host archive path:
  mount both that exact path and `/archive` in the offline dev container, not a
  rewritten archive contract or symlink workaround.

- Isolated native precision diagnosis: stop the task stack (including idle dev
  and inspection containers for the isolated-fixture ownership guard), then
  `python3 tools/run_local_isaac_fixture.py native_precision --rejection-run
  <host-archive/native-failure>`. Requires preserved same-tick
  `rejected_native_recast.json`. Runs only the seeded static terrain; no ROS,
  rover, commands or resumed failed recording. Keeps original/centered/
  unsimplified collider cases and a float64 local-triangle reference. Local
  reference is not complete scene truth. Success means the diagnostic finished,
  not that native ray consistency or rover acceptance passed. Run
  `tools/test/test_ray_triangle_precision.py` for analytic and saved-ray cases.
- Native writer byte-equivalence: in the Isaac image's Python environment, with
  archive/tools/OmniLRS binds and OmniLRS working directory, run
  `tools/benchmark_native_archive_io.py /archive/<native-capture>`. Does not
  start Isaac or require a GPU. Uses the first 100 recorded ticks, keeps both
  legacy and candidate files, independent hashes and every timing. Failed input
  is permitted only as a byte-copy diagnostic and remains ineligible. Host
  regression: `python3 -m pytest -p no:cacheprovider tools/test/test_native_archive_io.py`.
- Headless **coupled selected optical v2** run (September 29): start the sole
  recorder first in the sourced `astrobot-selected-dev` overlay as UID/GID 1000:
  `ASTROBOT_SELECTED_LIDAR_PROFILE=selected_optical_v2 python3
  tools/run_shadow_capture.py --config /archive/<frozen-glim>/config
  --archive-root /archive --require-physics-imu --natural-motion loop16
  --selected-lidar --terrain-v2 --terrain-library /archive/<frozen-build>/liblocal_terrain.so
  --wall-seconds 1800`. Then on the host use
  `DEV_USER=jacoblambert ASTRO_UID=1000 ASTRO_GID=1000 bash
  docker/run_selected_headless.sh start loop16`. The helper uses base + archive +
  `docker-compose.selected-headless.yml`, checks CUDA allocations and archive
  device/capacity, rejects an active simulator, and requires exactly one recorder.
  Selected headless compose sets `OMNILRS_DISABLE_VIEWPORT_UPDATES=true`;
  `RendererConf` forwards it to SimulationApp and rejects it for GUI runs.
  Confirm `VIEWPORT_UPDATE_AUDIT ... updates_enabled=False` in the preserved
  simulator log. GUI/default remains false. This does not change physics/render
  stepping or authorize skipping sensor ticks. It is not a proven stall fix;
  see the [capture I/O audit](capture-io-audit.md) and implementation ledger.
  No X11, navigation, GT TF/grid, cameras or resource map; physical IMU, GT pose
  initialization, native terrain input and synthetic RTK assistance remain.
  Stop this simulator after the recorder seals and preserve its log. A longer
  route is NOT enabled just by increasing wall time: route/clearance/acquisition
  limits and storage bounds need their own explicit declaration and fixtures.
- Same physical loop on a slow host (October 3, opt-in, not yet live-qualified):
  recorder adds `--execution-profile loop16_slow_host_v1 --wall-seconds 3600`.
  Host startup adds `SELECTED_EXECUTION_PROFILE=loop16_slow_host_v1` to the
  headless command above. Both sides must match; the request archives the
  profile and 3,600 s overall envelope. Default remains unchanged. No change to
  route/hash, native dose, physics cadence, per-leg deadlines or heartbeat/lease
  limits. Tests: `test_natural_motion.py`, `test_inspection_config.py` and sourced
  domain-179 `test_shadow_capture_ros.py`. Do not edit an active run's budget.
- Simulator heap diagnosis (October 4): `OMNILRS_HEAP_PROFILE_ONLY=true` is
  strictly pre-request/pre-lease and deliberately aborts with `stationary heap
  diagnostic complete; no motion requested` after writing `heap_profile.json`.
  It needs the same selected headless world, GPU/allocation/archive preflight,
  but no recorder or motion grant. Start as a uniquely named compose `run
  --no-deps` with this explicit environment override; monitor its failure marker
  and stop that exact container within 240 wall seconds. This marker is an
  intentional diagnostic termination, NOT completed capture evidence. Native
  archives may be root-owned: preserve Docker logs in a separate UID-owned
  archive, and put shutdown in `finally` so log-copy failure cannot skip it.
  The profiler neither disables/freezes GC nor changes collection thresholds.
- Opt-in startup comparison (October 4, not yet moving-qualified): headless
  compose accepts `SELECTED_IMPORT_STAT_CACHE=false` and
  `SELECTED_ISAAC_EXPERIENCE=lean_v1`. Defaults are `true` / `default` and GUI
  startup is unchanged. The first disables Kit's import-stat exception cache;
  the second selects only required direct extensions while preserving merged
  upstream base/Python settings. `selected_experience.py` generates the `.kit`
  and its hash/input record in a fresh external archive; native manifest records
  the startup declaration and generated input hashes. This changes the runtime
  configuration and must be declared, not pooled with baseline timing. Tests:
  `test_simulator_heap_profile.py`, `test_selected_experience.py`,
  `test_inspection_config.py`, plus natural-motion/sensor contracts.
- Selected native timestamp regression (September 30): host
  `tools/test/test_generic_lidar_adapter.py` and `tools/test/test_local_terrain.py`
  cover Isaac 5.1 clock-compatible `int(t*1e9)` encoding and the real saved
  one-nanosecond future-tick failure, with unchanged deadline enforcement.
  Sourced ROS domain 179: `python3 -m pytest -q -p no:cacheprovider
  tools/test/test_selected_transport_ros.py` checks both legacy nearest-ns and
  declared `isaac_clock_ns_v2` window transport through serialization/MCAP.
  Do not rewrite historical evidence timestamps; policy is capture provenance.
- Recorder readiness scheduling regression: host
  `python3 -m pytest -q -p no:cacheprovider tools/test/test_capture_polling.py
  tools/test/test_shadow_capture_gate.py`; sourced ROS domain 179 adds
  `tools/test/test_capture_polling_ros.py` and `tools/test/test_shadow_capture_ros.py`.
  The 32-topic DDS fixture reproduces starvation from 110 ms checks under the old
  start-based schedule and checks the completion-based callback reservation.
  It uses only synthetic `/polling_fixture/*` topics, no simulator or commands.
  These are scheduling regressions, not long-capture throughput acceptance.
  October 4 candidate: `StorageCheck` runs exactly one filesystem/config check
  concurrently with ROS callbacks; each grant consumes a fresh result. The old
  cadence fixtures remain as historical regression controls. Include
  `test_shadow_capture_gate.py` to verify expired leases remain terminal.
- Short live GC scheduling stress (October 4): same recorder-first selected
  headless workflow, but use route `turn15`, recorder
  `--execution-profile turn15_gc_stress_v1 --wall-seconds 900`, and host
  `SELECTED_EXECUTION_PROFILE=turn15_gc_stress_v1 SELECTED_ISAAC_EXPERIENCE=lean_v1
  SELECTED_IMPORT_STAT_CACHE=false`. Forces full GC after native ticks 240, 480,
  720 and adds 110 ms to every actual storage check. GC remains enabled with
  default thresholds; wall/lease/geometry/stamp guards remain active. The exact
  injection list/cost is in the matched execution declaration and completed
  injections are recorded in `acquisition_timing.json`. Both sides must match.
  This is deliberately instrumented and rejected by nominal holdout qualification;
  it cannot establish normal compute/terrain acceptance. Inspect all three
  injections and capture/seal results before a fresh nominal full-loop retry.
  Add `test_lease_callbacks_ros.py` to isolated domain-179 regressions. It checks
  independent reception, sender loss/terminal expiry, executor shutdown, and a
  separate-process DDS sender with a fixed 400 ms GIL-held pause plus I/O.
  The fixed-pause fixture is not a substitute for the actual Isaac stress run.
- October 4 simulator critical-path regression: `test_lab_manager_factory.py`
  verifies lazy selection of Lunalab/Lunaryard/LargeScale and custom registrations
  without importing unselected worlds. `test_natural_motion.py` checks immutable
  survey lookup against exact brute-force argmin, ties and survey boundaries.
  The floor raycasts/overlap queries and every guard threshold remain unchanged.
  Heap-only same-scene comparison must precede a moving retry after changing
  startup imports; compare scene, clearance, sensor audit and graph contract.
- Offline cross-host/container audit: older frozen candidates may record the
  external archive's original host path, while new captures record `/archive`.
  Preserve both contracts. Use a separate same-image offline container with
  `--volumes-from astrobot-selected-dev`, plus a bind of the actual
  external archive at its original absolute host path. Use `--network none`,
  host UID/GID, sourced ROS overlay, and `/archive` for fresh output. Verify both
  binds refer to the intended external device. The current `check_run` contract
  requires a writable archive root even for input validation, so a read-only
  alias fails before scoring. The evaluator still never edits its inputs.
  Do not edit archived contracts,
  weaken device/quota checks, or recreate the live development container merely
  to accommodate an offline audit.
- Actual grid gallery, **offline after sealing**: in the sourced ROS dev container,
  `python3 tools/export_gap_examples.py /archive/<recorder> /archive/<mapper>
  --archive-root /archive`. Failed sources require `--diagnose-failed-prefix` and
  stay ineligible. This extracts MCAP `/terrain_shadow/classes`, checks full grid
  pose against receipts, retains missing publications and terminal invalidation,
  and uses fixed display times plus the last available grid. NumPy/SciPy and
  `rosbag2_py`, `nav_msgs` are required. On the host run `python3
  tools/render_gap_report.py <host-archive/grid-export> [<more-exports> ...]
  --pose-reports <matching-pose-json> [<more-pose-json> ...]`; omit the entire pose
  option if unavailable. Output is fresh external-archive `gap-report.html`, a
  self-contained inline-SVG gallery requiring no web server. Pure tests:
  `python3 -m pytest -q -p no:cacheprovider tools/test/test_gap_examples.py
  tools/test/test_native_rejection.py tools/test/test_inspection_config.py`.
- Derived-report storage (October 3): `diagnose_failed_glim.py`,
  `export_gap_examples.py` and `render_gap_report.py` allocate a stricter 1 GiB
  per-report quota, not the 64 GiB raw-capture quota. External-only storage and
  the 50 GiB free-space reserve remain unchanged; no effect on capture preflight.
  Final output quota checks are mandatory. Host regressions:
  `python3 -m pytest -q -p no:cacheprovider tools/test/test_diagnostic_report_budget.py
  tools/test/test_archive_guard.py tools/test/test_failed_glim.py tools/test/test_gap_examples.py`.
- Rejected selected native ticks now preserve `rejected_native_tick.npz/json`
  separately, then re-raise the unchanged adapter failure. These raw bytes and
  diagnostic poses are NOT valid native chunks, published windows, replay inputs
  or evidence that a stream can resume. Never fabricate/drop the rejected tick.
  If present, inspect them offline with `python3 tools/diagnose_native_rejection.py
  /archive/<failed-native> --archive-root /archive` in the dev container. For a
  sensor association failure, `rejected_native_recast.json` additionally retains
  same-tick independent PhysX recasts along beam and XYZ directions (at most 20
  offending rays), including the physics hit's distance/position self-consistency.
  Motion commands stop first; physics is not stepped and no returned value repairs
  or replaces the rejected native tick. Diagnostic errors are separately recorded
  without suppressing the original sticky failure. These are evaluator-only.
  For a revision-kind failure, `python3 tools/diagnose_revision_batches.py
  /archive/<sealed-mapper> --archive-root /archive` checks the sealed event hash,
  chain and differences from preceding poses/extrinsics/stamps/deskew flags.
  This uses offline full pose history (bounded at 40,000 IDs); its output is NOT
  a live no-change certificate or permission to bypass the core's revision gate.
  NumPy/SciPy are required; ROS is not needed for these two JSON/NPZ diagnostics.

- Live v2 archive sealing now uses `local_terrain_seal.OutputDigest` to hash bytes
  during writes, keeping shutdown independent of payload length. Install with
  `colcon build --symlink-install --packages-select slam_mapping --cmake-args
  -DSLAM_MAPPING_BUILD_TERRAIN_SHADOW=ON`. Independent seal verification still
  re-hashes every file. A missing/empty seal stays ineligible; never repair one
  retroactively. In the dev container, `python3 tools/benchmark_local_seal.py
  /archive/<mapper>/native.bin --archive-root /archive` copies the entire payload
  into a fresh archive run, checks the streamed hash against independent reads
  of both files and times writer close. Requires that much extra archive space;
  this is a writer fixture, not a live timing or completed-capture gate.
  Regression: `python3 -m pytest tools/test/test_local_terrain_seal.py`.

- GUI/Foxglove procedure: [live-inspection.md](live-inspection.md). Use the
  `docker-compose.inspect.yml` overlay and `bash docker/run_selected_gui.sh
  preflight|start loop16|turn15`, not a legacy navigation launch. Current
  session `DISPLAY`/`XAUTHORITY` are required; the helper checks actual container
  Xlib authorization, archive and CUDA, and requires a recorder before start.
  Bridge service `inspection-bridge` binds **127.0.0.1:8766** with only graph/time
  capabilities. `python3 tools/probe_inspection_bridge.py --handshake-only`
  works without a simulator; omit that flag during live GLIM to receive the
  five declared cloud/status/GT/request streams (20 s bound). Installed bridge
  3.2.6 needs SDK WebSocket negotiation; old v1-only probes get HTTP 400.
  Host fixtures: `python3 -m pytest -q -p no:cacheprovider
  tools/test/test_inspection_config.py tools/test/test_inspection_probe.py`
  (PyYAML/pytest). GUI output and browsing add load; they do not establish
  headless resource equivalence. The opt-in coupled v2 path now publishes
  `/terrain_shadow/classes` and `/terrain_shadow/costmap`; see the live inspection
  guide and ledger for current qualification limits, not the old mapper.

- Freeze `tools/local_terrain/selected_lidar.json`; do not reuse the legacy
  uniform/5-degree protocol. Generic returns feed six-tick windows on
  `/astrobot_0/lidar_selected/points` using real 60 Hz acquisition tick times.
  No GT enters this adapter. Physical IMU, one-time GT initialization and
  synthetic RTK assistance remain explicitly declared experimental inputs.
- Fresh dev containers need `astrobot_interfaces`, `astrobot_core`, `glim`,
  `glim_ros`, `localization_supervisor`, `glim_slam_launch`, `astrobot_launch`, and
  `synthetic_rtk` built. Source `/etc/ros_setup.sh` and the resulting overlay.
  A stale CMake cache can select old image GLIM headers; after building workspace
  `glim`, pass `-Dglim_DIR=/workspace/astrobot-lab/astrobot-lab/install/glim/share/glim`
  to the `glim_ros` build. Its generated `share/glim/cmake` directory lacks the
  target export. Keep `BUILD_TESTING=ON`; use at most two compiler workers.
- If unrelated software owns the UI ports, do not kill it. The no-port service
  form is `ASTROBOT_AUTO_BUILD=false docker compose -f docker-compose.lunaryard.yml
  -f docker-compose.archive.yml run -d --no-deps --name astrobot-selected-dev
  astrobot-dev`. Verify the actual archive bind and absence of mission command
  publishers before capture.
- `python3 tools/prepare_selected_glim.py /archive/<completed-physical-IMU-recorder>
  --archive-root /archive` freezes a new level/timed configuration. Historical
  read-only imports may cross Linux device renumbering; preserve the old contract
  and hashes, never rewrite/resume its lease. Output gets a fresh device lease.
- Start `python3 tools/run_shadow_capture.py --config /archive/<profile>/config
  --archive-root /archive --require-physics-imu --natural-motion straight_slow
  --selected-lidar --wall-seconds 900` before Isaac, as host UID/GID. Then launch
  **only** `astrobot-sim` with the archive overlay and these settings:
  `OMNILRS_NATURAL_HOLDOUT=true`, `OMNILRS_NATURAL_MOTION=straight_slow`,
  `OMNILRS_SELECTED_LIDAR=true`, `OMNILRS_PHYSICS_IMU=true`, seed 43, rocks true,
  GT TF false, camera false, calibration/drive/probe/native-publish flags false.
  Native capture is automatic in this branch. Launcher opts into 60 Hz rendering,
  one step per physics tick; legacy rendering defaults are unchanged.
- Isolated localization resource ablation additionally sets
  `OMNILRS_GT_MAP_ENABLED=false` and `OMNILRS_RESOURCE_ENABLED=false`. This removes
  unused raster/resource work, **not GT poses** or native first-hit audits. The
  request/timing artifact records this workload; not concurrent terrain/navigation
  compute acceptance. Never relax the 500 ms watchdog.
- Stop Isaac after recorder completion to freeze its postroll log. Only completed
  captures are eligible: `export_shadow_trajectory.py <recorder>`, followed by
  `audit_natural_holdout.py <capture> <recorder> <trajectory> --motion straight_slow
  --selected-lidar --frozen-candidate <profile>`. The new audit checks every native
  tick, current first hits (1 mm), exact GT snapshots, recorded timed-window bytes
  including boundary windows, motion artifacts, and config/plugin provenance.
  Historical binary equivalence is not claimed for a rebuilt environment.
  `diagnose_glim_accuracy.py <capture> <qualified-pair>` reports first-published
  and retrospective base-frame errors without fitted alignment. Failed runs
  remain ineligible; consult the ledger for outcomes.
- For an explicitly separate **completed-route pose diagnostic**, use
  `measure_exported_glim.py <capture> <recorder> <exported-trajectory>`. It checks
  every native/GT tick, timed-window bytes, exported source hashes and required
  pose brackets, but reports the native surface gate independently. Its output
  always has `acceptance=false` and cannot feed qualified terrain replays. This
  is useful for distinguishing GLIM error from a sensor-reference query failure;
  it does not authorize ignoring a failed first-hit check.
- `diagnose_glim_revision_delay.py <capture> <recorder> <exported-trajectory>`
  repeats those complete-recording checks and compares fixed 0.3/0.5/1.0-second
  acquisition-to-receipt deadlines. It never uses revisions after each deadline,
  and refuses aggregate scoring if any tick/deadline lacks a pose or recorded
  tail. Offline delayed acquisition poses are not consumer-time robot poses or
  live compute qualification. Regression: `test_glim_revision_delay.py`.
- Run `test_generic_lidar_adapter.py` on **both** host and container Python:
  NumPy 1.21 and 1.26 differ in structured concatenation padding. Also run
  `test_selected_lidar_profile.py`, `test_selected_native_audit.py`,
  `test_gt_pose_publication.py`, `test_terrain_gt_probe.py`, and existing motion/
  lease tests. Sourced ROS domain 179 adds `test_selected_transport_ros.py` and
  `test_shadow_capture_ros.py`. Transport tests exercise actual MCAP bytes,
  malformed stride, beam corruption and missing boundary-window tails.
- Also run `test_archive_guard.py`, `test_stationary_audit_seal.py`,
  `test_measure_exported_glim.py`, and core `test_shadow_dump.py`. Stationary
  audits are validated once and hash-sealed; live exact archive quota walks use
  streaming directory entries to avoid a growing temporary Path heap. Archive
  checks, normal GC, all acquisition samples and watchdogs remain enabled.
- Selected-sensor long acquisition uses existing `loop16` and recorder
  `--wall-seconds 1800`. Explicit request/native metadata declare up to 660 s /
  40,000 retained native ticks; short-route limits remain 200 s / 20,000 ticks.
  Driver leg/heartbeat/clearance/skid deadlines are unchanged. Strict audit uses
  `--motion loop16 --selected-lidar`; do not relabel short-route evidence.
- September 27 loop acquisition declares `anticipatory_corner_feedback_v2`:
  during a turn, at most 0.03 m/s along-heading correction (gain 0.6/s) toward
  a fixed command reference 0.15 m ahead of the nominal corner along the declared
  entry heading. Arrival heading and radial guard still use the nominal corner;
  no reanchoring. This supersedes the failed reactive-only
  `bounded_corner_feedback_v1`. Same radial 0.25 m
  envelope and all existing safety/deadline bounds. This is a changed, hashed
  GT acquisition-controller protocol, never a navigation policy. The recorded
  25.0058 cm second-corner escape remains a failed regression fixture.
- `diagnose_failed_glim.py <failed-native-capture> <failed-recorder>` is an
  explicitly failed-prefix-only diagnostic. It retains all native indices and
  missing GT/pose-tail counts, does not extrapolate, and emits no qualified-pair
  or completed-route artifact. Contracts: `tools/test/test_failed_glim.py`.
- `assess_glim_stability.py <completed-pose-diagnostic>` checks the immutable
  output of `measure_exported_glim.py`, plots fixed-frame trajectories/errors and
  reports sustained >0.5 m / >5-degree errors and >0.5 m / >5-degree changes in
  GT-relative error (native tick and 100 ms scan scales). These are catastrophic
  error screens, not navigation bounds. Normal accuracy targets, geometry audit
  failures and first-published/final-revised distinctions remain visible.
  Host fixture: `python3 -m pytest -p no:cacheprovider tools/test/test_glim_stability.py`.
- Recorder revision validation uses `LiveRevisionGate`, not the all-history
  offline timeline. It has no pose-query API and retains all immutable frame
  timestamps/revision digests. Submap assignment can deliver a large historical
  batch even with global/submap optimization disabled. Pure parity fixtures:
  `python3 -m pytest -p no:cacheprovider tools/test/test_live_revision_gate.py`;
  the isolated ROS recorder suite includes a 1,835-pose batch.
- In the sourced ROS environment, `python3 tools/profile_revision_validation.py
  /archive/<recorder>` compares reference/new validation state on all actual
  revision messages, including failed captures, in a fresh external run. Timing
  excludes bag reads and is diagnostic only; no live deadline or pose-accuracy
  acceptance. Collection `recorder_processing` retains per-stage maxima and the
  last 128 >50 ms events (plus the full slow-event count); `recorder_source_sha256`
  identifies validator/recorder dependencies. Keep native watchdogs unchanged.

## Environment Notes

- Containers assume NVIDIA GPUs and X11 forwarding. Check `docker-compose.yml` for required env vars and `config/cyclonedds_config.xml` for DDS settings.
- Current working headless stack is `docker-compose.lunaryard.yml` with CycloneDDS on `ROS_DOMAIN_ID=177`.
- In `astrobot-dev`, use `source /etc/ros_setup.sh` rather than sourcing only Space ROS or Jazzy directly.
- Foxglove Bridge is installed in `astrobot-dev` and is typically exposed on `127.0.0.1:8765` for SSH tunneling.
- Current validated Phase 1 motion stack is:
  - `planner_server`
  - `controller_server`
  - `bt_navigator`
  - `basic_control/command_mux`
- Current validated Lunaryard Husky drive tuning includes:
  - `maxAngularSpeed=4.0`
  - `maxWheelSpeed=12.0`
  - `wheel_drive_stiffness=0.0`
  - `wheel_drive_damping=10000.0`
## Contact-support experiments (October 4)

### Startup observation extension

Dimensioned coupled-rock fixture (October 5): start the ordinary selected optical
v2 recorder with terrain/contact advisory and fixed startup-turn area, then set
`SELECTED_DEMO_ROCK_PROFILE=rounded_10x15_v2` when invoking
`docker/run_selected_headless.sh start turn15` or `turn360` (also declared for
`peek_reverse20`). The default empty value leaves the original world unchanged.
For the selected headless tests also explicitly set
`SELECTED_ISAAC_EXPERIENCE=lean_v1 SELECTED_IMPORT_STAT_CACHE=false` on the
simulator launch. The helper defaults to the full experience/cache enabled;
omitting these previously validated switches caused the October 5 full-turn
attempt to exceed its unchanged heartbeat limit during full Python collection.
This is startup/resource configuration, not a sensor or classification change.
This adds eight static rounded meshes, 10 cm above local terrain and 15 cm wide;
it does not resize or remove the natural rocks. The west test object is 2.65 m
from startup to avoid a natural boulder; others use +/-1.8 m axis/diagonal offsets.
Geometry is cooked and checked before sensor audit and motion. Records:
`demo_rocks_created.json`, independently verified `demo_rocks.json`, and any
`demo_rocks_failure.json`. The request contains the immutable geometry hash;
the mapper never consumes those labels.

After completed capture and simulator shutdown, in the sourced dev container
with both archive aliases and Shapely 2.1.2:

```bash
python3 tools/assess_demo_rocks.py /archive/<completed-live-mapper> --archive-root /archive
python3 tools/assess_startup_observation.py /archive/<completed-live-mapper> --archive-root /archive
python3 tools/export_shadow_trajectory.py /archive/<recorder> --archive-root /archive
python3 tools/measure_exported_glim.py /archive/<native> /archive/<recorder> /archive/<trajectory> --archive-root /archive
python3 tools/render_demo_rocks.py /archive/<mapper> /archive/<object-assessment> \
  /archive/<observation-assessment> /archive/<pose-diagnostic> --archive-root /archive
# If the strict reference audit fails, inspect every saved recast disagreement:
python3 tools/diagnose_demo_rock_rays.py /archive/<native> --archive-root /archive
```

Object detection requires an occupied cell overlapping the actual projected
rock footprint, not an inflated halo. All valid maps and all eight objects stay
in the denominator. Report final and ever-detected counts separately; free cells
on a rounded object's low skirt are not automatically a false-safe whole object.
Keep independent clear-floor support and supported exits separate from object
recall. Contract tests: `tools/test/test_demo_rock_profile.py`.

Host regression dependencies matching the October 5 live container:
`numpy==1.26.4 scipy==1.11.4 shapely==2.1.2 toml==0.10.2`, plus pytest.
Use these explicit `uv run --no-project --with ...` pins for the selected
experience, paired-IMU, contact and fixture tests. Unpinned resolution selected
SciPy 1.18.1, which rejects the older single-axis angle-array fixture; that is
a dependency compatibility failure, not an observed live IMU/SLAM failure.
The runtime environment and existing assertions were not changed.

Near-field hazard visibility screens (October 5) are CPU-only analytic tools,
not Isaac captures or motion policies:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 uv run --offline --no-project --with numpy --with scipy \
  python tools/assess_nearfield_hazards.py
# Sourced dev container, with both archive aliases and original source-path alias:
python3 tools/assess_recorded_hazard_views.py /archive/<hazard-screen> /archive/<completed-turn-capture> \
  --archive-root /archive
# Separately declared, post-baseline rounded-rock size sweep:
python3 tools/assess_recorded_hazard_views.py /archive/<hazard-screen> /archive/<completed-turn-capture> \
  --archive-root /archive --size-sweep
```

The first tool generates 1,728 fixed placements; the second adds recorded XY/yaw
at 15-degree crossings and start/end, **not** original tilt/Z, native surfaces,
noise or GLIM errors. `--size-sweep` declares 1,536 rounded-rock cases with
8/10/12/16 cm heights and 10/12/15/20 cm widths. Compare all original eligible
placements AND `matched_size_summary` (common positions across sizes).
`report.html`, `report.json` and every per-view count in `placements.jsonl`
remain in a fresh external archive directory. These counts measure observable
differences from ideal flat ground, not the terrain classifier's obstacle recall.

Original native archive contracts may reference `/archive`, while host screens
reference the long host archive path. Run the recorded comparison in the dev
container with both aliases. When host-generated source hashes name
`/home/jacoblambert/astrobot-lab`, also bind that repository read-only at the same
path; do not edit archived contracts or bypass device checks to resolve aliases.
The initial host invocation on October 5 failed before allocating output because
`/archive` was absent; the correctly aliased container run completed.
Regression file: `tools/test/test_nearfield_hazards.py`.

`--contact-startup-turn-area` declares the user-approved, fixed deployment
turning sweep: physical half-extents 0.5037 x 0.3454 m, radius 0.6107494 m.
An inscribed 16-sided polygon stays inside that circle and the existing geometry
work budget. No extra skid margin, moving the prior, free-grid painting or hazard
override. The previous rectangle remains the default for old experiments.

Selected headless route `turn360` is a separately hashed acquisition dose, not
navigation: 15 cm preparatory translation, eight 45-degree arcs with bounded
observation-orbit feedback, then return to the initial position. Version 3 pauses
yaw for straight recentering above an 18 cm radius
when the heading can reduce the error. It resumes the same arc without moving
the origin. The new protocol explicitly declares 420 s / 26,000 native frames
as storage/audit bounds, preserving every 40 s leg cap. At most 3 cm/s
linear and 0.1 rad/s angular; unchanged 25 cm origin/skid limit, lease, floor,
stall and leg guards. Recorder `--natural-motion turn360 --wall-seconds 1800`
must precede `docker/run_selected_headless.sh start turn360`. Keep the selected
sensor, v2 library, contact advisory and explicit turning-area flag; core support
is not needed for this experiment. Check the ledger: first version failed skid;
neither a model regression nor a failed-prefix result qualifies this route.

Separate fore–aft observation protocols: `peek_reverse` requests 15 cm backward
with the legacy 2 cm arrival tolerance; `peek_reverse20` requests 20 cm backward
with an explicitly declared 5 mm tolerance. Both dwell 3 s, return to startup,
keep heading fixed and cap speed at 3 cm/s. Their longitudinal bounds are
[-17, +2] cm and [-22, +2] cm respectively; existing lateral/heading/floor guards
remain. Match recorder `--natural-motion` to the headless route and retain
`--contact-advisory --contact-startup-turn-area`. These are GT acquisition
motions, not commands selected by the contact advisory. Compare actual
displacement, not just the nominal route name.

After stopping Isaac, use the sourced ROS dev image and both archive aliases:

```bash
python3 tools/assess_startup_observation.py /archive/<completed-live-mapper> --archive-root /archive
python3 tools/render_startup_observation.py /archive/<observation-assessment> --archive-root /archive
python3 tools/diagnose_startup_visibility.py --archive-root /archive
```

The visibility screen is only a flat-plane/chassis-box model. It traces the
first collision before software range filtering, includes the selected beam
budget, and separately reports parked/reverse/hypothetical-side-mount cases.
It excludes wheels, mast, relief and SLAM; it neither changes the real mount
nor establishes native-sensor or navigation acceptance.

The assessment exact-replays raw maps AND original live advisory rows, then
tests fixed 10–100 cm forward/reverse paths offline. An exit must require new
wheel support outside both the un-eroded startup area and current wheel patches;
small movement inside assumed support does not count. All valid maps and original
1 Hz decision bins remain. Independent surveyed clear-floor points stay fixed
and evaluator-only; this is not obstacle recall or executed goal progress.
Quarter-turn raw grids and the final grid are saved for the HTML report.
Exact advisory replay reads the declared startup policy from the source manifest.
An older source can separately compare the new assumption using
`replay_contact_advisory.py --candidate --startup-turn-area`; never relabel it
as original live behavior.

Focused contracts: `tools/test/test_startup_observation.py`, plus existing
natural-motion/loop, contact-advisory/runtime/replay and capture-gate tests.

These are offline/advisory tools, not a command-permission or Nav2 plugin.
Keep both original archive aliases mounted, use the existing sourced ROS overlay
and stop Isaac before replay. The new geometric backend requires **Shapely
2.1.2** (tested with its bundled GEOS 3.13.1), in addition to NumPy/SciPy.
Install it in an isolated environment, not by changing the mission ROS setup.
The experiment records the geometry library versions and native-library hashes.

Current-pose integration adds `local_terrain/contact_advisory.py` (shared query)
and `local_terrain/contact_runtime.py` (opt-in mapper callback adapter). The live
output `/terrain_shadow/contact_advisory` is a JSON String: timestamp/pose age,
retained support status, and five candidate checks at 1 Hz. `motions: null`
means that map update did not schedule a new candidate batch, **not** that no
motion is available. `valid: false` invalidates earlier answers. This topic has
no command authority and is not a Nav2 costmap replacement. Default history is
LiDAR-confirmed wheel patches, not the earlier all-wheels-grounded ablation.

After a scoped rebuild of `slam_mapping` and `astrobot_launch`, add
`--contact-advisory` to the existing `run_shadow_capture.py --terrain-v2`
recorder command. The runner declares a clear guarded deployment and requires
both the original mapper seal and the separate contact-advisory seal. Keep
recorder-first startup and the selected headless launch unchanged. The callback
adapter requires Shapely in the recorder/launch process's Python path. The
October 4 staged dependency is `/archive/contact-deps-OXpt89ix` (Shapely 2.1.2,
bundled GEOS); after sourcing ROS and overlay, prepend that directory to
`PYTHONPATH`. It is an isolated dependency directory, not a changed ROS prefix.
Both version and native-library hashes are captured by the advisory.

Before changing `live_local_terrain.py`, run
`python3 tools/terrain_entrypoint_archive.py --archive-root <external-archive>`
to preserve its source by SHA-256. `--source local_terrain/contact_advisory.py`
and `--source local_terrain/contact_runtime.py` cover the two non-mapper
advisory adapters. Older exact **raw-map** replays may verify these saved
sources, which raw reconstruction does not execute. No reconstruction source
or library hash is exempted; all replayed layers/associations remain exact.
This does not repair arbitrary historical source changes or claim an older
advisory's decisions are unchanged. The separate advisory replay below checks
those decisions, or explicitly declares a changed candidate.

```bash
# Sourced offline dev image; archive mounted at /archive and original host path.
python3 tools/assess_contact_support.py /archive/<navigation-gap-assessment> \
  /archive/<exported-trajectory> --archive-root /archive --history-xy-erosion 0.02
# Separate, explicitly optimistic/sensitivity experiments: 0.01 and 0 metres.
# Separate, UNQUALIFIED partial-support policy: append --support-core.
# Full wheel/body hazard vetoes remain; positive support uses a 40x60mm central region.
# Recent-path withdrawal is reported separately; it never counts as goal progress.
python3 tools/render_contact_support.py /archive/<support-run> /archive/<another-support-run> \
  --archive-root /archive
python3 tools/compare_contact_support.py /archive/<before> /archive/<after> --archive-root /archive
python3 tools/audit_support_pose_age.py /archive/<mapper-run> --archive-root /archive
python3 tools/replay_contact_advisory.py /archive/<mapper-with-advisory> --archive-root /archive
# Changed advisory only: append --candidate, and optionally --support-core.
# Raw maps/pose/time exposures still match; this does NOT qualify a failed native audit.
# A fresh experimental live core run additionally uses --contact-support-core
# on run_shadow_capture.py; default live history remains LiDAR-confirmed.
# Single-candidate synthetic kernel timing only, not a coupled resource gate:
python3 tools/benchmark_contact_query.py --archive-root /archive

# Host pure tests with the geometry dependency, no ROS/simulator:
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run --no-project --with shapely==2.1.2 \
  --with numpy --with scipy --with pytest python -m pytest -q -p no:cacheprovider \
  tools/test/test_contact_support.py tools/test/test_contact_support_assessment.py \
  tools/test/test_contact_index.py tools/test/test_contact_support_comparison.py \
  tools/test/test_contact_support_report.py \
  tools/test/test_contact_manifold.py tools/test/test_navigation_gaps.py \
  tools/test/test_local_support_sweep.py tools/test/test_local_isaac_fixture.py

# Host launcher; stop the task stack first, including the idle selected dev container.
# Performs host/image CUDA preflights, launches a zero-command isolated fixture,
# records contact points in one JSONL file, preserves failures, and exits within 240 s.
python3 tools/run_local_isaac_fixture.py contact_points --archive-root <external-archive>
# Six simultaneous isolated loaded-rover cases; all 480 ticks retained per case:
python3 tools/run_local_isaac_fixture.py support_hazards --archive-root <external-archive>
```

Support remains separate from original terrain classes. Startup uses chassis plus
the existing **assumed** wheel patch extents, not the acquisition safety guard.
History is placed using mapper receipt order and its exact IMU/base transform
chain. Missing historical poses withhold support; no future revisions or short
time expiry. The 0/1/2 cm erosion values are sensitivity parameters, **not**
calibrated uncertainty limits. Manifold samples do not certify filled support
area between contacts. Neither tool establishes autonomous navigation safety.

## Bounded terrain-controlled trial (October 6)

Normal-terrain current-turn-area variant: add `--contact-current-turn-area` to
the recorder below. It uses the same 61.07 cm nominal turning footprint at
each **actual current** consumer pose, in addition to the unchanged fixed
startup support. It never grants support at hypothetical future poses or
retains the moving disk as measured history. Existing occupied/drop-off vetoes
and all driver guards remain. No demo-rock profile is selected. The flag is
opt-in and recorded in `contact_manifest.json`; exact replay restores it.
`replay_contact_advisory.py --candidate --current-turn-area` is the corresponding
offline changed-policy comparison. Do not label that comparison an exact
advisory replay; only its original raw map replay is exact.

For the normal-terrain map/path diagnostic, use the same recorder flags and
replace `terrain_step` with `straight_slow` on **both** recorder and simulator
launch. Keep `SELECTED_DEMO_ROCK_PROFILE=` empty. This is the original guarded
2 m out / 2 m back GT-guided acquisition, not terrain-controlled navigation.
After completed acquisition and simulator shutdown, in the isolated replay
environment with both archive aliases:

```bash
python3 tools/assess_cost_paths.py /archive/MAPPER_RUN --archive-root /archive --include-short-goals
python3 tools/render_cost_paths.py --replay /archive/PATH_ASSESSMENT_RUN --archive-root /archive
```

`--include-short-goals` adds 10/20 cm to every original 30/50/100 cm cardinal
goal; it does not remove harder goals or failed bins. The report can omit
synthetic cases entirely. It restores the recorded current-turn assumption,
archives strict/isolated grids separately and draws the actual support polygon.
Both stages reject incomplete acquisitions. The October 6 4 m attempt stopped
on a simulator heartbeat failure; the published normal-terrain report instead
uses the completed `terrain_step` recording and labels that limited scope.

`terrain_step` is a separate opt-in execution experiment, not an exploration
controller or the older GT waypoint acquisition. It attempts a 30 cm goal in
5 cm increments at 2 cm/s, using only causal estimated pose and contact support
to choose commands. The existing GT survey and footprint monitor can veto
motion. Startup turning-area support and full 80 × 120 mm wheel patches remain
unchanged. Persistent terrain refusal is a completed **refused** trial, not a
successful route. The 2 cm stopping allowance is experimental, not calibrated.

Use the selected headless/archive preflight and recorder-first startup described
above. In the sourced `astrobot-selected-dev` environment, with Shapely 2.1.2
available through the existing archive dependency directory:

```bash
export OMP_NUM_THREADS=4
export ASTROBOT_SELECTED_LIDAR_PROFILE=selected_optical_v2
export PYTHONPATH=/archive/contact-deps-OXpt89ix:$PYTHONPATH
python3 tools/run_shadow_capture.py \
  --config /archive/revision-4c7b70a4acb44a07acb1bca0d63e3692/config \
  --archive-root /archive --require-physics-imu --natural-motion terrain_step \
  --selected-lidar --terrain-v2 \
  --terrain-library /archive/revision-70647a2b742c446eb61bd3d7141ea011/liblocal_terrain.so \
  --contact-advisory --contact-startup-turn-area --wall-seconds 900
```

Then on the host (do not enable a second command publisher):

```bash
DEV_USER=jacoblambert ASTRO_UID=1000 ASTRO_GID=1000 \
SELECTED_DEMO_ROCK_PROFILE= SELECTED_ISAAC_EXPERIENCE=lean_v1 \
SELECTED_IMPORT_STAT_CACHE=false \
bash docker/run_selected_headless.sh start terrain_step
```

Stop the owned simulator after collection. Use the documented isolated replay
container with **both archive aliases** to run:

```bash
python3 tools/assess_step_trial.py /archive/MAPPER_RUN --archive-root /archive
```

The audit requires completed acquisition and exact maps, advisory decisions and
controller proposals. It reports GT displacement separately from estimated goal
completion. Consumer-time pose error includes pose age; it is not same-time
SLAM accuracy. `step_trial.jsonl` contains all proposals; the native capture's
`motion.jsonl` contains executed proposals and GT evaluation poses. First and
terminal refusal JSON/NPZ retain exact unconfirmed wheel fragments and maps.
Missing/stale proposals produce logged zero-command waits, never continued
motion. Pose-noise-induced off-axis step targets stop and replan without moving
the fixed goal or counting progress. Invalid proposals still abort. Both
waiting ticks and nonzero-command ticks remain in the audit denominator.
The generic GT-waypoint route audit is not the trial audit. These checks do not
replace native-ray, hazard-recall, stopping-margin or resource qualification.

## Longer costmap route experiments (October 7)

### Restoring cold recordings after user-authorized cleanup

October 7 cleanup uses lossless Zstandard cold storage, not deletion of failed
experiments. A cold file has adjacent `FILE.zst` and `FILE.cold.json` containing
original size/SHA-256/mode/mtime and compressed SHA-256. Reports, manifests,
bag metadata, run IDs and failure outcomes stay unchanged. Replay deliberately
cannot use a cold run until its data files are restored; do not skip missing
files, rewrite input hashes, or silently substitute another recording.

On the **host**, run `python3 tools/cold_archive.py restore /absolute/archive/
revision-ID/bag/bag_N.mcap` for each cold bag segment (or the run's `native.bin`).
Use the real external mount path recorded in `.cold.json`, not container
`/archive`. The tool checks the compressed checksum, restores to a temporary
file, verifies the original checksum, then activates without overwriting.
It keeps the compressed copy. Both pack/restore require the normal 50 GiB free
reserve plus temporary working space. Never launch capture/replay beside pack
or restore. `pack` accepts only explicit MCAP/native.bin files in closed runs;
no automatic pruning. Current navigation evidence, libraries/configs and all
October recordings were excluded from the first cleanup batch. Exact run IDs
and measured recovery are in the implementation ledger. Tests:
`python3 -m pytest tools/test/test_cold_archive.py` (host Python + zstd).

### Running longer routes

Use the recorder-first command below with `--natural-motion costmap_return4`,
`costmap_square8` or `costmap_detour8`, `--contact-current-turn-area` and
`--wall-seconds 3600`. Launch exactly the same route with the headless helper.
For `costmap_detour8` ONLY, set
`SELECTED_DEMO_ROCK_PROFILE=single_40x60_detour_v1` on the simulator startup;
the other two use an empty profile. All use level `selected_optical_v2`, the
same frozen GLIM config/library and one-time GT initialization, never GT steering.
The detour's `wide_goal_south_v2` layout goes to fixed initial-frame (3.5,-1.5) m
and back home (7.616 m total). Its former (4,0) m goal was inside the margin of
a natural background rock; no live result used that rejected layout. The inserted
rock remains 1.8 m ahead, 0.4 m high × 0.6 m wide; the new direct path still
requires a detour. Survey rectangle and physical limits are unchanged.

These are opt-in `costmap_route_trial_v13` experiments with fixed initial-SLAM-frame
goals and exact home, not the generic GT route follower. Underlying floor survey
may include rocks elsewhere in the region; first-hit/footprint checks still veto
driving into them. Routes, bounds, recording limits and stricter 32 GiB artifact
quotas are in `tools/costmap_scenarios.py`; the 50 GiB archive reserve is unchanged.
No cleanup or root-filesystem fallback. Keep all failures.
Version 2 raises only the turn cap from 0.04 to 0.1 rad/s after very slow
measured turning; forward speed stays 0.05 m/s and checked turns stay 5 degrees.
The old single-goal `costmap1/2` protocol retains 0.04 rad/s.
Version 3 counts stall time only while motion is commanded and gives negative
edges wheel-radius clearance; raised/occupied hazards keep body clearance.
It retains the exact immediate wheel/body sweep and frozen skid references.
Version 4 extends wheel-extent routing to all explicit unknown support conflicts,
matching the immediate sweep. Occupied cells retain the original full-body
circle; an inscribed-body alternative was rejected by the rock-margin fixture.
Version 4's live square stops on side two because start-cell rounding rejects
an actually clear wheel envelope. Version 5 checks exact circular clearance for
the start connector and support-hazard shortcuts, retaining occupied-cell
padding and the full immediate sweep. See the ledger for live qualification;
neither a passing snapshot query nor an ideal plant establishes a completed route.
Version 6 adds SLAM-only turn centering: pause yaw for a checked signed 32 cm
straight sweep at 5 cm/s when along-heading displacement from the frozen turn
anchor exceeds 6 cm; resume yaw at 2 cm. Turn ID/anchor persist through reverse,
forward corrections and waits. Driver keeps a separate frozen GT turn envelope
across those mode switches; 25 cm drift, heading/lateral, footprint, floor,
stall and freshness limits are unchanged. `test_turn_recentering.py` exercises
these boundaries. Report reverse-command time separately. This changed controller
requires fresh live qualification; the failed v5 square is not rescored as v6.
The unpromoted v7 heading-specific goal approach failed its small motion fixture
and was removed. Version 8 instead fixes occupied-padding start entrapment:
allow only a continuously circle-clear connector out of the current padded
cell to a padded-free cell; retain every other padded cell and actual 3-D sweep.
It does not implement the rejected goal-approach workaround.

Version 9 implements the user's current-turn-footprint precedence policy:
uncertain cells wholly within the actual current 61.08 cm nominal turning circle
are allowed for navigation, never changed into measured free terrain. Occupied
cells always veto. Future routes and the commanded 32 cm segment retain full-turn
clearance; the prior never follows hypothetical nodes or enters history. Raw
terrain and contact-advisory topics remain unchanged; each route proposal records
`current_turn_assumption`, including every overridden cell. This is a declared
policy change, not equivalent replay of v8. Fixtures: `test_turn_space_policy.py`.
Version 10 permits a checked clockwise observation turn when no goal route exists
but the current turning envelope is clear. It keeps the same centering/GT turn
anchor and a 90-s simulation-time budget including waits and transient routes.
Only 30 cm estimated translation resets the episode; total route time is still
600 s. Neither missing routes nor the current-footprint assumption overrides an
occupied detection. Proposals record `observing_for_route` and
`observation_started_ns`. Fixtures: `test_route_observation.py`.
Version 11 also tries the same checked observation when a proposed motion loses
turn clearance, rather than only when A* has no route. Blocked centering may defer
to independently checked yaw, keeping its frozen anchor and original 25 cm GT
skid veto. The 32 cm translation sweep and all occupied-cell tests stay unchanged.

Version 13 retains a fixed-local-frame route and steering endpoint while its
remaining segments and current connector stay clear. It does not rerun A* for
cost-only changes or newly visible alternatives. It replans for a new geometric
blocker or failed immediate sweep, extends a consumed local partial route toward
the same final goal, and starts a new route on actual goal arrival. Retained
tentative segments outside the rolling map are not automatically blocked; every
observed hazard and the original inflated clearance still apply. The immediate
3-D sweep retains its in-map, freshness and collision requirements. Proposals
record `route_policy`, `route_generation`, `route_action`, `route_waypoints_xy`
and `route_goal_xy`; exact replay must reproduce this state. The command-age
clock policy is unchanged. Fixtures: `tools/test/test_route_commitment.py` plus
the existing scenario/turn/observation tests. Historical v12 captures are not
equivalent replay inputs for this policy change.

Version 12 is the explicitly user-authorized drifting-turn experiment: no
centimeter-scale recentering or reverse correction. Track heading from current
SLAM pose, allow at most 75 cm cumulative turn translation about the frozen
anchor (SLAM and independent GT), and add 5 cm planar slip clearance to both
all-heading body/wheel envelopes for paths and commands. The driver enforces
5 cm per-proposal GT translation, including sequence transitions, without using
GT to choose commands. Existing body inflation, occupied veto, current-footprint
prior, lease/freshness, actual footprint/floor, route bounds and budgets remain.
Legacy acquisition/short-route turn limits are unchanged. This declared change
does not qualify v11 failures. Tests: `test_turn_recentering.py` now checks
absence of recentering, frozen anchors, slip allowance and new-limit enforcement;
its earlier v11 source is archived in the implementation ledger.

October 8 lease-reception candidate: `tools/lease_process.py` runs the grant-only
ROS receiver in a separate Python interpreter, because Isaac GC blocks every
Python thread. `natural_holdout_capture.py` launches it with Isaac's Python and
inherited ROS import paths. The child never publishes grants/commands or touches
physics. Both child and parent check the original lease; parent IPC queries have
a 200 ms socket timeout, identity/sequence matching and a fresh age check. One
fresh query may replace a reply made stale by parent GC, within a 500 ms total
read bound; no child failure is retried and no stale snapshot authorizes motion.
Queries are serialized with the independent motion watchdog. Child exit,
hang, revocation and expired continuity fail closed. This is capture scheduling,
not relaxed robot clearance or localization acceptance. Isolated tests:
`tools/test/test_lease_process_ros.py` with domain 179 / localhost and the normal
sourced dev overlay. Run alongside the existing lease/capture ROS fixtures.
Before a nominal route retry, use matching recorder/simulator
`turn15_gc_stress_v1` with the existing three forced collections and 110 ms
storage-check injection; this instrumented run cannot qualify nominal timing.

Mapper shutdown defers normal SIGINT until the active callback completes, then
seals with ROS still alive. Keep the recorder's existing 20 s shutdown deadline.
Unexpected empty-message exceptions must produce nonempty sticky faults; a
legacy `failure=""` recording is failed, even if its old completion Boolean
incorrectly says true. Tests: `test_local_shutdown.py`, `test_local_terrain_seal.py`,
and the steady-clock SIGINT callback test in `test_shadow_capture_ros.py`.
Real core tests use `TERRAIN_LIBRARY`; rolling tests additionally require
`TERRAIN_ROLLING_LIBRARY` (the selected rolling-enabled library supports both).

After stopping Isaac, run `assess_step_trial.py` in the isolated replay container
with both archive aliases. It replays every proposal/map exactly, and reports
each reached goal's GT error, actual return-home distance, time and unknown-cell
exposure. The single-rock scenario also reports minimum XY clearance from the
inflated robot footprint to that rock at every executed tick. This does not
measure clearance to every natural scene rock or qualify braking. Then export
trajectory and run `measure_exported_glim.py` for native/transport and same-time
pose diagnostics. Fixtures: `test_costmap_scenarios.py` plus the existing
costmap, driver, runtime, audit, natural-motion and selected-sensor suites.

For bounded diagnostic storage, use `audit_completed_route.py CAPTURE RECORDER
--archive-root /archive` in the same sourced, isolated replay container. It calls
the unchanged trajectory/pose/transport evaluators and allocates two strict
1 GiB outputs with the normal 50 GiB reserve. This does not change capture quotas.
`diagnose_costmap_refusal.py MAPPER --archive-root /archive` checks frozen
refusal snapshots against sealed map/proposal records and evaluates one
counterfactual command; it cannot claim route completion. Run it in the container
so the original `/archive` contract path exists. Keep failed diagnostic outputs.

Selected native capture storage (October 8 candidate): live `native_chunks/`
contains one append-only `native.tar`, not two files per physics tick. The TAR
stores the unchanged NPZ/first-hit JSON bytes; `native_storage.json` seals its
count/size/hash. Do not extract during acquisition. After both native capture and
recorder have closed, run `python3 tools/restore_native_archive.py /archive/CAPTURE`
in the sourced container before legacy native-file evaluators or direct
`selected_native_audit.audit_transport` calls. `audit_completed_route.py` does
this automatically. Isaac may create `native_chunks` as root. After closure,
inspect that exact directory (reject symlinks), then use container root to change
**only that directory's ownership**, not source file contents, to the host
UID/GID before user-owned restoration:
`docker exec -u0:0 astrobot-selected-dev chown 1000:1000 /archive/CAPTURE/native_chunks`.
Resolve the actual host UID/GID; never use recursive/broad ownership changes.
Restoration verifies every logical hash and available
capacity first, creates exact copies without overwrite, and retains the source
TAR. It temporarily needs space for both representations within the original run
quota and 50 GiB reserve. Failed recordings remain failed after restoration.
The mapper's separate `native.bin` and all ROS topics are unchanged. Fixtures:
`test_native_archive_io.py`, `test_packed_native_archive.py`,
`test_terrain_native_capture.py`. All-tick storage-only comparison:
`benchmark_packed_native.py CAPTURE --archive-root /archive` (8 GiB bound).

Selected moving capture completion (October 9): commands/native acquisition stop
on the physics thread, which announces the exact final native tick and freezes
producer/timing metadata. `capture_seal.BackgroundSeal` then writes audit files
and closes lease receivers in one bounded worker, while actual sensor/clock
ticks continue for GLIM and the mapper's final pose brackets. Completion is
published only after files and cleanup succeed. The original 20 s sealing
deadline and mapper heartbeat remain unchanged; errors/late results stay failed.
There is no extra native tick, extrapolated pose, synthetic clock or resumed
command authority. Tests: `test_capture_seal.py`, `test_natural_holdout.py`,
`test_terrain_native_capture.py` and the isolated ROS lease/capture suites.

## Exploration-entry diagnostics (October 6)

Unknown-permitted live costmap trial (separate from `terrain_step`): use the
recorder-first command above with `--natural-motion costmap1` (or `costmap2`),
`--contact-current-turn-area` and `--wall-seconds 1800`; launch the same name
with `docker/run_selected_headless.sh start costmap1`. Original startup-turn
and full-patch options remain required. No second command publisher or concurrent
replay. `tools/local_terrain/costmap_trial.py` selects a fixed observed-free goal,
uses a 2-D body-inflated cost grid with finite unknown cost, and checks immediate
motion for hazards without demanding measured ground under every wheel. GT
survey/footprint checks only veto. `costmap1` / `costmap2` are different declared
1 / 2 m experiments, not longer versions of the original 30 cm trial.

The existing `step_trial.jsonl`, `step_trial_collection.json` and native
`step_trial_outcome.json` filenames are retained; the proposal protocol is
`costmap_unknown_trial_v1`. Stop the simulator after collection and use
`assess_step_trial.py` on the new mapper run for exact controller/map replay,
GT distance to the fixed goal, command-time accounting and live controller
timing. Controller tests: `tools/test/test_costmap_trial.py`; retain the original
step-driver, contact runtime, trial-audit and natural-motion regressions.
Clear-terrain arrival does not qualify blind-obstacle detection or hole safety.

For `costmap1/2`, `ContactRuntime` selects the explicitly tagged `HazardAdvisory`
mode. It retains coherent current pose/map and hazard checks but does not build
historical wheel-support unions that these controllers do not use. Original
`terrain_step` retains its positive-support advisory. `contact_manifest.json`
records `hazard_only` / `support_history_used`; replay restores that selection.
Observed-wheel flags remain reporting-only and raw terrain classes are unchanged.

Natural captures now record passive `stage_timing` in `acquisition_timing.json`:
per-stage counts/maxima and twelve worst intervals, including world step,
sensor processing, archive checks and command delivery. Nested intervals are
not additive. A run-wide maximum may belong to stationary startup, so inspect
its stamp before blaming a motion-time simulator stall. Compare mapper
`events.jsonl` wall receipts with advisory `update_ms` / `total_ms`: a busy
single-threaded mapper can starve its own clock callback and report a heartbeat
fault while simulation continues. Test helper: `test_simulation_stage_timing.py`.

Two-horizon path test (October 6): this is **offline route intent plus locally
checked prefix**, not a command controller. `tools/local_terrain/horizon_planner.py`
keeps separate route/local grids: missing-evidence cells cost 90 on the route
only; occupied and explicit conflicts remain vetoes. It requires an initially
observed-free goal, and checks the returned local prefix with the original
wheel/body geometry and support index. Up to 0.5 m is checked; 2 cm is withheld
as the existing experimental stopping reserve. The assumption stays at the
actual current pose. No raw classes, GLIM or existing live controller changed.

```bash
# Same isolated sourced environment / Shapely / both archive aliases as above.
# No Isaac concurrently. All maps replay exactly; every five-second bin counts.
python3 tools/assess_horizon_paths.py /archive/MAPPER_RUN --archive-root /archive
# Explicit second protocol: observe five seconds, then freeze all goal choices.
python3 tools/assess_horizon_paths.py /archive/MAPPER_RUN --archive-root /archive --goal-selection-delay-s 5
python3 tools/render_cost_paths.py --replay /archive/FIRST_MAP_GOALS --replay /archive/OBSERVED_GOALS --archive-root /archive
```

Selection uses the nearest measured/inferred-free cell within 30 cm of each
1/2 m cardinal target, without GT or search-success input. Goals stay fixed;
missing rear goals and pre-selection bins stay visible. Actual lattice endpoints
must also remain on observed free cells inside the original 7.5 cm tolerance.
The endpoint precheck only detects when all eight arrival headings are blocked;
it does not certify reachability or raise the 4,000-state search budget.
Report full-route geometry separately from prefix geometry. Neither a returned
route nor repeated prefix availability proves executing/reaching the goal.
Run the host fixture `tools/test/test_horizon_planner.py` with the pinned geometry
dependencies below, plus the existing cost/planner/contact/sweep regressions.

Recorded navigation bottleneck diagnosis (no commands or changed policy):

```bash
# Same isolated sourced replay environment, Shapely and both archive aliases.
python3 tools/assess_navigation_bottleneck.py /archive/MAPPER_RUN \
  /archive/EXACT_STEP_TRIAL_AUDIT /archive/HORIZON_ASSESSMENT --archive-root /archive
```

Requires the completed acquisition, exact trial audit and matching completed
horizon assessment. Verifies input hashes, replays all original maps/advisories,
and preserves every planning bin. Outputs time-weighted executed-command states,
wall/simulation freshness ages, evaluator-only GT bearing disagreement, raw
wheel-support fractions, diagnostic 1 cm edge subdivisions and two `.prof`
files. Instrumented profiles and isolated query timings are not live compute
acceptance; neither the subdivisions nor GT bearings change a controller.
Run `tools/test/test_navigation_bottleneck.py` with the pinned host dependencies,
plus the horizon/planner/contact/sweep/trial regression tests.

These tools add offline experiments without changing the mapper, selected
sensor, startup radius or live movement rules. Stop Isaac before replay; use
the sourced ROS dev image with both archive aliases and the staged Shapely
dependency described above. A completed replay is not navigation acceptance.

```bash
# Optimistic causal support retention: falsification test, never a live policy.
python3 tools/assess_support_retention.py /archive/<mapper-with-advisory> --archive-root /archive
# Four fixed grid phases: identical native returns/poses/classifier/lifetime.
python3 tools/assess_grid_phase_support.py /archive/<mapper-with-advisory> --archive-root /archive
# Preserve the original straight queries; additionally test complete turn/drive paths.
python3 tools/assess_turn_exits.py /archive/<mapper-with-advisory> --archive-root /archive
# Host-only analytic screen, including chassis first hits. No sensor/asset writes.
PYTHONPATH=tools python3 tools/diagnose_steep_beams.py --archive-root <external-archive>
```

Use matching NumPy/SciPy for historical transform fixtures. An unpinned SciPy
upgrade changes an existing Euler-angle fixture and is not a mapper regression:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 \
uv run --offline --no-project --with shapely==2.1.2 --with numpy==1.26.4 \
  --with scipy==1.11.4 --with pytest --with toml==0.10.2 \
  python -m pytest -q -p no:cacheprovider \
  tools/test/test_support_retention.py tools/test/test_phase_support.py \
  tools/test/test_turn_exits.py tools/test/test_steep_beams.py
```

Results and failed hypotheses belong in the implementation ledger. The
hypothetical 52 cm forward optical origin is **not** a selected sensor profile;
its analytic result excludes wheels/mast and needs a separate native test.

October 6 correction: that forward placement was not mechanically qualified
and is withdrawn as the next recommended mount. To reproduce the new audit:
`tools/audit_lidar_attachment.py <asset.usd> --archive-root /archive` uses the
USD-only environment above, with the repository read-only and external archive
mounted. No GPU/Isaac application is needed. It compares actual transformed
mesh vertices and the selected parent's translation; positive separation proves
an attachment gap, while overlapping bounds would not certify attachment.

The separate ideal active-observation screen is host-only (NumPy/SciPy/Shapely,
same pinned versions above):

```bash
OMP_NUM_THREADS=4 python3 tools/diagnose_active_startup.py \
  --library <frozen-liblocal_terrain.so> --archive-root <external-archive>
python3 -m pytest -q -p no:cacheprovider tools/test/test_active_startup.py
```

This uses analytic flat-ground/chassis rays, perfect poses and one complete scan
per endpoint. All proposed observation segments are checked before supplying
their new observations; refused moves stop the sequence. It is neither native
sensor validation nor a live adaptive controller, and cannot qualify hazards.
