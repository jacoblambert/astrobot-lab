# Astrobot Console Backend Contract

The frontend consumes `astrobot.console.v1` from `WS /ws/telemetry` and `GET /api/state`.

## Top-level frame

- `schemaVersion`: currently `astrobot.console.v1`.
- `stamp`: backend wall-clock seconds.
- `connected`: `true` for live backend frames; frontend mock mode uses `false`.
- `robots`: array of robot state objects, single robot for now.
- `mission`: normalized mission summary.
- `maps`: named `nav_msgs/OccupancyGrid` layers. WebSocket frames may omit maps between refreshes; the frontend preserves the last map per layer.
- `cameraTopics`: selectable camera topic names.
- `series`: numeric time-series used by diagnostics panels.
- `topics`: topic freshness and estimated rate, keyed by backend topic key.
- `events`: latest compact status/warning/critical events.

## Optional fields (newer backends)

- `stack`: `legacy` or `v2` (`phase5_gui.launch.py stack:=...`).
- `simTime`, `simSpeed`: sim clock seconds and sim seconds per wall second.
- `rateSeries`: the `series` names to show as topic rates, in order. Rates are arrivals per sim second
  over the last 10 sim seconds, sampled once per wall second (`topics[].hz` uses the same window).
- `localization`: `{state, motionPermitted, reason, epoch, stamp}` from `localization/status` (v2).
- `terrain`: `{valid, failure, epoch, received, projected, skippedSequences, lateChunks, evidenceLagSec, peakRssMib, stamp}`
  from `/terrain_shadow/status` (v2).
- `dds`: `{domain, rmw}`.
- `robots[].footprint` `{halfLength, halfWidth}`; `robots[].gtPose` and `gtTrajectory`: ground truth in
  the display map frame (evaluator overlay; v2 uses the exact RTK-home transform map = world + (20, 20)).
  `trajectory` and `gtTrajectory` keep a point per 0.1 m of travel (newest pose last, up to 3000).
- `robots[].candidates`: `[{x, y}]` the exploration manager's ranked goal candidates (best first,
  `exploration/frontiers`).
- `gtRocks`: `[{x0, y0, x1, y1}]` rock footprints (world bounds, display map frame) from the sim's
  evaluator scene inventory (`gt_scene_path`, the gate's contact reference); empty when not given.
- `mission.details.home_x`, `home_y`, `home_radius_m`: the mission disc (exploration manager status).
- Downsampled grids (over `max_cells`) are block max-pooled, so small obstacles survive.
- `mission.elapsedSec`: mission sim time once the manager's events give its start.
- `events`: current status events, then the recent event log (newest first).
- WebSocket frames carry only map layers that changed since the last frame sent to that client.

## v2 layers (`stack:=v2`)

- `maps.obstacle`: `/terrain_shadow/global_costmap` (Nav2 costs on the persistent mosaic)
- `maps.explore`: `/terrain_shadow/exploration_map` (0 free, 100 occupied, -1 unknown)
- `maps.local`: `/terrain_shadow/planner_costmap` (the window Nav2's local costmap reads)
- `maps.mission`, `maps.water`, `maps.gt`: as below.

Rates come from small topics only (raw subscriptions, `imu_physics`, the costmaps, SLAM pose; native
chunk rate from the terrain status counter): the console never subscribes to point clouds.

## Initial live layers (legacy)

- `maps.obstacle`: `/astrobot_0/slam/bev_costmap`
- `maps.mission`: `/astrobot_0/exploration/mission_grid`
- `maps.water`: `/gt/resource_maps/water`
- `maps.gt`: `/gt/map`

Map component endpoints:

- `GET /api/maps/metadata`: layer dimensions, resolution, stamps, and cell counts.
- `GET /api/maps/{layer}`: one complete `OccupancyGrid` JSON layer.
- `GET /api/maps`: all currently cached layers.

The frontend can consume `OccupancyGrid` JSON directly because the BEV renderer
paints into canvas. If map sizes or update rates grow, add a PNG/tile endpoint
for rendered rasters and keep JSON for metadata/debug inspection.

## Initial live status inputs

- Robot pose: `/astrobot_0/slam/pose_corrected`
- Plan: `/astrobot_0/plan`
- Selected goal: `/astrobot_0/exploration/selected_goal`
- Mission summary: `/astrobot_0/exploration/status`
- Diagnostics: `/astrobot_0/diagnostics`
- Camera freshness: `/astrobot_0/camera_0/image_raw`

Topic-rate diagnostics currently track:

- `/astrobot_0/lidar_0/pointcloud/filtered` -> `PointCloud Hz`
- `/astrobot_0/imu` -> `IMU Hz`
- `/astrobot_0/slam/pose_corrected` -> `SLAM pose Hz`
- `/astrobot_0/slam/prob_voxel_map` -> `Voxel map Hz`
- `/astrobot_0/slam/bev_costmap` -> `BEV map Hz`
- `/astrobot_0/diagnostics` -> `Diagnostics Hz`
- `/astrobot_0/camera_0/image_raw` -> `Camera Hz`

Camera component endpoints:

- `GET /api/cameras`: discovered camera topics and latest image metadata.
- `GET /api/cameras/{camera_key}/snapshot.jpg?max_width=640&quality=72`: latest camera frame as throttled JPEG snapshot.

Do not stream raw ROS images through JSON/WebSocket. Use snapshots for the first
live test, then choose MJPEG/WebRTC/compressed-image transport if sustained video
is needed.

## Backend rules

- Monitoring data remains read-mostly, but teleop click mode exposes one command endpoint:
  `POST /api/robots/{robot}/teleop_goal` with `{ "x": number, "y": number, "yaw"?: number, "frame_id": "map" }`.
  The backend publishes `/astrobot_0/teleop/goal`; the exploration manager only accepts it in `mission_mode:=teleop_click`.
- Preserve array-shaped `robots` for future multi-robot support.
- Prefer adding new optional fields over changing existing field names.
- Use Foxglove for raw high-bandwidth debugging until browser image streaming is deliberately selected.
