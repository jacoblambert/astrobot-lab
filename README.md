# Astrobot

Single-rover lunar autonomy in simulation: a rover explores a rock-strewn lunar yard in
[OmniLRS](https://github.com/OmniLRS/OmniLRS) (NVIDIA Isaac Sim), localizes with GLIM LiDAR-IMU
SLAM anchored by RTK-style absolute fixes, builds a local geometric terrain map from raw LiDAR
returns, drives with Nav2, and runs a mission (coverage of a disc around home, or a resource
search and return) through a pluggable mission manager.

Release 0.1 is a **simulation and demo** release. It is not ready for hardware: the absolute
fixes come from simulator ground truth and the terrain input is the simulator's native LiDAR
stream. The flight-path packages are C++ (ROS 2 Jazzy on a Space ROS base image); making them
Space ROS compliant is the goal of 0.2.

| Mission gates (seeds 42 / 411 / 7), 2026-11-21 | Result |
|---|---|
| explore_radius: coverage >= 0.70 of an 8 m disc, home <= 1 m | 3 / 3 (coverage 0.77-0.81, home 0.29-0.77 m) |
| sample_return: sample >= 0.75, home <= 1 m | 3 / 3 (sample 0.751-0.757, home 0.47-0.81 m) |
| GT rock contacts (>= 0.10 m rocks) | 0 |

An RTK outage (10 s every 300 sim-s) holds motion within 2 s, restarts SLAM and resumes the
mission ([docs/running.md](docs/running.md#fault-injection-rtk-outage)).

## Quick start

Requirements: an NVIDIA GPU with the container toolkit, Docker Compose v2, about 40 GB disk.

```bash
git clone --recursive <repo> astrobot && cd astrobot
./docker/build_omnilrs_base.sh                  # Isaac Sim + OmniLRS image (isaac-sim-omnilrs)
docker compose build astrobot-dev                 # ROS 2 workspace image (astrobot-dev)
scripts/run_mission.sh 42 explore_radius --console
```

`run_mission.sh <seed> <mission> [--eval] [--console] [--camera]` starts the workspace
container (building the packages on first start), brings up the lunar yard for the seed, waits
for the simulator stream, launches the mission and, with `--console`, starts the console backend. Then run the frontend
and open http://localhost:5173:

```bash
cd ui/astrobot-console && npm install && npm run dev
```

Missions: `explore_radius`, `sample_return`, `teleop_click` (click goals in the console).
`--eval` scores the run against ground truth and writes `runs/<run>/result.json`; Ctrl-C tears
the simulator down. See [docs/running.md](docs/running.md).

## Packages

| Package | Language | Role |
|---|---|---|
| `astrobot_interfaces` | IDL | Messages and services (SLAM revisions, localization status, map reset) |
| `astrobot_localization` | C++ | Localization supervisor (owns GLIM, gates RTK fixes, motion permission) and map->odom TF |
| `astrobot_glim_extensions` | C++ | GLIM extension modules: RTK constraint on the odometry smoother, SLAM revision publisher |
| `astrobot_terrain` | C++ | Layered geometric terrain from native LiDAR chunks: Nav2 local window, global cost mosaic, exploration map |
| `astrobot_exploration` | C++ | Mission manager on Nav2 with pluginlib mission strategies |
| `astrobot_control` | C++ | Velocity command mux (navigation, teleop, e-stop, localization permission) |
| `astrobot_bringup` | launch / YAML | `mission.launch.py`, `console.launch.py`, robot geometry, Nav2 parameters, behaviour trees |
| `astrobot_sim` | Python | Simulation only: synthetic RTK and GT diagnostics |
| `astrobot_eval` | Python | Simulation only: GT mission verdict and rock-contact scoring |
| `astrobot_ui_backend` | Python | Ops console backend (`ui/astrobot-console` is the web frontend) |
| `glim`, `glim_ros2` | C++ | GLIM forks (submodules): lunar gravity, IMU/odometry fixes, clean extension teardown |

[docs/architecture.md](docs/architecture.md) describes the node graph, frames and the
localization state machine; [docs/extending.md](docs/extending.md) covers new missions, seeds,
world settings and swapping components.

## Development

- `scripts/check.sh` builds the release packages and runs the unit tests and linters.
- Research, capture and debugging tooling lives on the development branch (`dev/phase-5`)
  under `research/`; `main` is cut from it without `research/` (`research/scripts/cut_release.sh`).
- Licence: Apache-2.0 ([LICENSE](LICENSE)). GLIM is MIT, OmniLRS BSD-3-Clause (submodules).
