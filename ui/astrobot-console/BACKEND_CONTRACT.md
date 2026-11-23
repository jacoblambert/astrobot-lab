# Astrobot Console Backend Contract

The frontend consumes `astrobot.console.v1` from `WS /ws/telemetry` (a frame every 0.5 s) and
`GET /api/state`. The backend is `astrobot_ui_backend` (`ros2 launch astrobot_bringup
console.launch.py`); all topics below are relative to the robot namespace (`/astrobot_0`) unless
they start with `/`.

## Frame

- `schemaVersion`: `astrobot.console.v1`.
- `stamp`: backend wall-clock seconds. `connected`: always `true` from the backend (the frontend
  shows an offline frame while the WebSocket is down).
- `simTime`, `simSpeed`: sim clock seconds and sim seconds per wall second.
- `robots`: array of robot states (one robot for now; kept an array for multi-robot support):
  - `pose`, `trajectory`: SLAM pose (`slam/pose_corrected`, converted from the IMU frame to
    base_link) and its trail;
  - `goal` (`exploration/selected_goal`), `plan` (`plan`), `candidates` (`exploration/frontiers`,
    the exploration manager's ranked goal candidates, best first);
  - `footprint` `{halfLength, halfWidth}`;
  - `gtPose`, `gtTrajectory`: ground truth (`/gt/base_link_pose`) in the map frame, map = world -
    home (`home_xy` parameter, default (-20, -20)). Evaluator overlay only, never fed back.
  - Trails keep a point per 0.1 m of travel (newest pose last, up to 3000).
- `mission`: summary of `exploration/status`; `mission.details` is the raw status (the mission disc
  comes from `home_x`, `home_y`, `home_radius_m`). `elapsedSec` is mission sim time once the
  manager's events (`exploration/events`) give its start.
- `localization`: `{state, motionPermitted, reason, epoch, stamp}` from `localization/status`.
- `terrain`: `{valid, failure, epoch, received, projected, skippedSequences, lateChunks,
  evidenceLagSec, peakRssMib, stamp}` from `terrain/status`.
- `maps`: `nav_msgs/OccupancyGrid` layers as JSON (below). WebSocket frames carry only the layers
  that changed since the last frame sent to that client; the frontend keeps the rest. Grids over
  `max_cells` are block max-pooled, so small obstacles survive downsampling.
- `gtRocks`: `[{x0, y0, x1, y1}]` rock footprints in the map frame from the sim's scene inventory
  (`gt_scene_path`, the gate's contact reference); empty when not given.
- `series`: numeric time series for the plots; `rateSeries`: the series to show as topic rates, in
  order. Rates are arrivals per sim second over the last 10 sim seconds, sampled once per wall
  second (`topics[].hz` uses the same window).
- `topics`: freshness and rate per subscribed topic. `cameraTopics`: camera topics seen so far.
- `events`: current status events, then the recent event log (mission events, supervisor
  transitions, terrain validity), newest first.
- `dds`: `{domain, rmw}`.

## Map layers

- `obstacle`: `terrain/global_costmap` (Nav2 costs on the persistent mosaic)
- `explore`: `terrain/exploration_map` (0 free, 100 occupied, -1 unknown)
- `local`: `terrain/local_costmap` (the window Nav2's local costmap reads)
- `mission`: `exploration/mission_grid` (visited / seen / unseen mission cells)
- `water`: `/gt/resource_maps/water`; `gt`: `/gt/map` (sim ground truth)

Rates come from small topics only (raw subscriptions: `imu_physics`, the costmaps, the SLAM pose;
the native chunk rate comes from the terrain status counter): the console never subscribes to point
clouds, so it adds little load to a run.

## Endpoints

- `GET /api/health`, `GET /api/state?include_maps=false`, `GET /api/robots`.
- `GET /api/maps`, `GET /api/maps/metadata`, `GET /api/maps/{layer}`.
- `GET /api/cameras`, `GET /api/cameras/{camera_key}/snapshot.jpg?max_width=640&quality=72`:
  camera metadata and the latest frame as a JPEG (`camera_0/image_raw`, with
  `run_mission.sh --camera`). Raw images never go through the WebSocket.
- `POST /api/robots/{robot}/teleop_goal` with `{ "x": number, "y": number, "yaw"?: number,
  "frame_id": "map" }`: the only command endpoint. It publishes `teleop/goal`; the exploration
  manager accepts it only with `mission_mode: teleop_click`.

## Rules

- Prefer adding optional fields over renaming existing ones.
- Use Foxglove or rviz for high-bandwidth debugging (point clouds, images).
