# CLAUDE.md — Astrobot

Single-rover lunar autonomy in simulation: OmniLRS / Isaac Sim (lunar yard) + ROS 2 Jazzy on a
Space ROS base image. GLIM SLAM (RTK-anchored) → astrobot_terrain → Nav2 (MPPI / Smac 2D) →
mission manager (pluginlib strategies), plus a web Ops console. Release 0.1 = sim/demo; 0.2 =
Space ROS compliance of the core packages. README.md, docs/architecture.md, docs/extending.md,
docs/running.md describe the system; docs/decision-2026-10.md is the history of the v2 stack.

## Rules

- The user manages git: never commit, tag, push or branch. Prepare a script instead.
- Branches: `dev/phase-5` holds everything, research and debug tooling under `research/`; `main`
  is cut from it without `research/` (research/scripts/cut_release.sh, run by the user).
- Simulator teardown: `docker compose stop astrobot-sim && docker compose rm -f astrobot-sim`.
  Never `docker compose down` (it would remove astrobot-dev too).
- `OMNILRS_GT_TF_ENABLED=false` for every SLAM/nav run (SLAM owns map -> odom).
- No extra recorders or load in behaviour runs (the eval records in its own node).
- Sim checks are convenience: relax capture/test guards freely and use ground truth; change the
  simulated world (rocks, terrain, LiDAR) only with care.
- Every rover node runs in `astrobot_0`; terrain topics are `/astrobot_0/terrain/*`.

## Layout

- `astrobot-lab/src/` — release packages: astrobot_interfaces, astrobot_localization (C++),
  astrobot_glim_extensions (C++, GLIM RTK constraint + revision publisher), astrobot_terrain (C++),
  astrobot_exploration (C++), astrobot_control (C++), astrobot_bringup (launch, Nav2 and GLIM
  config), astrobot_sim / astrobot_eval / astrobot_ui_backend (Python, sim and tooling only), glim
  and glim_ros2 (fork submodules, no astrobot dependency).
- `OmniLRS` (fork submodule): the nav stream is `src/environments_wrappers/ros2/selected_nav_*.py`,
  its LiDAR profile `cfg/sensors/selected_lidar_optical.json`; it writes into `OMNILRS_RUN_DIR`.
- `scripts/run_mission.sh <seed> <mission> [--eval] [--console] [--camera]`, `scripts/check.sh`.
- `docker-compose.yml` (astrobot-sim, astrobot-dev), `docker/` (images, entrypoints).
- `ui/astrobot-console` — web console (Vite/React).
- `research/` (dev only): `tools/` (capture harness, audits, parity tools), `docs/` (experiment
  ledger, runbook), `legacy/` (replaced packages, COLCON_IGNORE: the Python reference
  implementations of terrain, supervisor, mux and exploration), `docker/` (capture overlays).

## Build and test

Inside astrobot-dev (`docker compose exec astrobot-dev bash`):
`source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && colcon build --symlink-install`.
`scripts/check.sh` = build + unit tests + linters (must print CHECK PASSED).

Parity tools (research, dev branch) — rerun after touching the ported logic. Their native parts
(`terrain_replay`, `libastrobot_terrain_c.so`, `candidates_tool`) live in `research/tools/native`
and build against the installed workspace (commands in its CMakeLists.txt):
- `research/tools/check_terrain_library_parity.py old.so new.so` (layered terrain library, bitwise);
- `research/tools/nav_terrain_parity.py generate|reference|compare` with `terrain_replay`
  (full terrain pipeline vs the Python reference; exact on synthetic streams);
- `research/tools/replay_terrain_ros.py` (the ROS node against terrain_replay);
- `research/tools/exploration_parity.py <candidates_tool>` (goal ranking vs the Python manager).

## Gotchas

- ament_uncrustify (Jazzy config) removes the `;` after a brace-init in a one-line block
  (`{return {};}` → `{return {}}`): write `return Type{};` or use a multi-line block.
- `#pragma omp` lines over 100 columns get split by uncrustify; keep them short (hoist the `if`
  condition into a `const bool`).
- The base image mixes Space ROS (ament_lint 0.11.2 fork) and Jazzy apt packages:
  - apt `nav2_msgs` needs Jazzy message packages, so astrobot_localization and
    astrobot_exploration prepend `/opt/ros/jazzy` to CMAKE_PREFIX_PATH;
  - the Space ROS linters do not run cleanly on this toolchain, so check.sh calls the Jazzy
    linters (plus Space ROS clang-tidy) directly. Aligning the image is 0.2 work.
- The localization supervisor restarts GLIM itself; never restart GLIM by hand mid-run.
- GLIM's `pose_corrected` is the IMU frame (base <- IMU in config/robot.yaml).
- docker/dev.Dockerfile builds GTSAM 4.3a0 and gtsam_points v1.2.1 from source: Koide's PPA
  republishes newer GTSAM snapshots under the same version (GLIM then fails on `ImuFactorT`).
- `/etc/ros_setup.sh` sources the workspace `install/` when present, which overrides the image's
  GLIM (`/opt/astrobot/glim_source`); building GLIM in the workspace needs
  `-DCMAKE_PREFIX_PATH=/opt/ros/jazzy;/usr/local` (see docker/dev_entrypoint.sh).
- `use_point_instancer` must stay False in the lunaryard config (instanced rocks are invisible
  to the RTX LiDAR).

## Open items before 0.1

- Done 2026-11-21: six gates, RTK fault run, console smoke (headless API/WebSocket; a visual
  check in a browser is still to do), fresh image build (`astrobot-dev:fresh`, clean build and
  tests pass). Done 2026-11-22: cleanup pass (console without mock data, dead code and parity
  tools out of the release packages), verified by a live console smoke (teleop goal) and gate
  seed 42 explore_radius (PASS, coverage 0.77, home 0.43 m, 0 contacts). Pending: switch
  `astrobot-dev:latest` to the fresh image, then cut the release.
