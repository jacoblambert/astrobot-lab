# Changelog

## 0.1.0 (unreleased)

First release: lunar-rover autonomy in simulation (OmniLRS / Isaac Sim, ROS 2 Jazzy on a
Space ROS base image).

- **Localization** (`astrobot_localization`, C++): supervisor owning GLIM LiDAR-IMU SLAM, RTK
  innovation gating, epoch-based restart with map reset, motion permission; map->odom TF.
- **Terrain** (`astrobot_terrain`, C++): layered geometric terrain from native LiDAR returns,
  bounded gap policy, Nav2 local window cut from a persistent map-frame mosaic, exploration
  map. Output identical to the research reference implementation on synthetic streams.
- **Missions** (`astrobot_exploration`, C++): Nav2 mission manager with pluginlib strategies
  `explore_radius`, `sample_return`, `teleop_click`; planner precheck, reveal goals, home
  approach, coverage with a localization margin. Operator (teleop) goals skip the precheck by
  default (`teleop_precheck_enabled`), so the rover can leave an unmapped spawn.
- **Control** (`astrobot_control`, C++): command mux with localization gating (acceleration
  shaping is Nav2's velocity smoother).
- **GLIM extensions** (`astrobot_glim_extensions`, C++): RTK absolute constraint and SLAM
  revision publisher as GLIM extension modules (moved out of the glim_ros2 fork, which no longer
  depends on astrobot_interfaces); the validated GLIM configuration is in astrobot_bringup.
- **Bringup** (`astrobot_bringup`): one `mission.launch.py`, shared robot geometry, Nav2 MPPI /
  Smac configuration and behaviour trees; `scripts/run_mission.sh` and `scripts/check.sh`.
- **Simulation and evaluation** (`astrobot_sim`, `astrobot_eval`, Python): synthetic RTK, GT
  diagnostics, GT mission verdict and rock-contact scoring.
- **Ops console**: web console (`ui/astrobot-console`) and backend (`astrobot_ui_backend`):
  terrain, mission and GT layers, supervisor and terrain status, teleop goals; ground truth in
  the exact RTK home frame. It shows an offline frame and reconnects while the backend is down.
- **Images**: the dev image builds GTSAM 4.3a0 and gtsam_points 1.2.1 from pinned tags (no PPA
  drift) and the GLIM forks at their pinned commits; a `.dockerignore` keeps the build context small.
- **Validation** (2026-11-21, C++ stack): six mission gates pass (seeds 42 / 411 / 7 ×
  explore_radius / sample_return, 0 rock contacts); RTK-outage fault run holds within 2 s and
  recovers; terrain output identical to the Python reference on a recorded run (1,521 windows).

The Python predecessors of the flight-path nodes, the legacy BEV mapping stack and the research
tooling stay on the development branch under `research/`.

Known limitations: simulation only (RTK from ground truth, native simulator LiDAR stream).
Space ROS linter compliance (its lint fork, SARIF, Cobra, IKOS) is 0.2.
