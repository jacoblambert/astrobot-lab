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

## Initial live layers

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
