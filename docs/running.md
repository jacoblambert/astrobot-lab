# Running missions

```bash
scripts/run_mission.sh <seed> <explore_radius|sample_return|teleop_click> [--eval] [--console] [--camera] \
    [<launch_arg>:=<value> ...]
```

Arguments of the form `name:=value` are passed to `mission.launch.py`.

Each run writes `runs/<stamp>-seed<seed>-<mission>/`: `mission.log`, and with `--eval`
`eval.json` (verdict), `record/` (GT and SLAM trajectories), `contacts.json` and `result.json`;
`sim/` holds the simulator's scene inventory, terrain tile and LiDAR window log.

The simulator boots in 2–3 minutes. SLAM, terrain and Nav2 start 15 s after launch, the mission
manager after 35 s; the localization supervisor then needs a few seconds stationary to start
GLIM and validate the map before the rover moves.

## Gates

`--eval` runs `astrobot_eval mission_eval` from before the mission starts (its first GT pose is
the spawn) and `score_contacts` afterwards:

| Mission | Pass |
|---|---|
| explore_radius | GT coverage >= 0.70 of the 2 m cells in an 8 m disc around the start, back within 1 m of home |
| sample_return | best sample >= 0.75, back within 1 m of home |
| both | no GT contact between the unpadded body and a rock >= 0.10 m above local ground (rocks under the body at spawn are reported apart) |

The C++ stack passed all six gates on 2026-11-21 (seeds 42, 411, 7 × both missions, 0 contacts;
`research/scripts/run_gates.sh` runs them in sequence). Behaviour runs must not add extra load
(recorders, the camera): the eval records inside its own node.

## Fault injection (RTK outage)

```bash
scripts/run_mission.sh 42 explore_radius --eval rtk_occlusion_period_s:=300 rtk_occlusion_duration_s:=10
```

The synthetic RTK drops every fix while `sim_time % period < duration`. Expected: the supervisor
holds motion ~2 s into the outage (`absolute_sensor_outage`), cancels the goal, resets the map,
restarts GLIM once RTK is back and is HEALTHY again ~15–30 s after the outage began; the mission
then resumes. On 2026-11-21 all three outages behaved this way (hold +1.98 s, rover still by
+2.35 s) and the mission met its coverage goal with 0 contacts. The verdict reads FAIL
(`localization_fault_observed`) by design: gate runs must not fault.

## Manual control

```bash
docker compose up -d astrobot-dev
OMNILRS_ENVIRONMENT_SEED=42 docker compose up -d --no-deps astrobot-sim
docker compose exec astrobot-dev bash -lc "ros2 launch astrobot_bringup mission.launch.py mission:=teleop_click"
docker compose exec astrobot-dev bash -lc "ros2 launch astrobot_bringup console.launch.py"
```

Stop the simulator with `docker compose stop astrobot-sim && docker compose rm -f astrobot-sim`
(`astrobot-dev` can keep running).

## Configuration sources

- GLIM: `astrobot_bringup/config/glim/` (override with `glim_config:=<dir>`).
- Simulated LiDAR (pattern, mount, chassis exclusion): `OmniLRS/cfg/sensors/selected_lidar_optical.json`;
  the terrain node's copy of the mount and chassis box is `astrobot_bringup/config/robot.yaml`.
- The simulator writes the evaluator geometry of each run to `runs/<run>/sim/`
  (`OMNILRS_RUN_DIR`; default `/runs/sim-<capture id>` when the simulator is started by hand).
