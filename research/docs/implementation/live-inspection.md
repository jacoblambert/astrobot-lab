# Live GUI + Foxglove inspection

This is the **selected level, mid-dense +10..−60° single-LiDAR experiment**,
not the legacy Phase 3 navigation launch. GT-controlled acquisition, one-time
GT initialization, physical IMU and synthetic RTK assistance are declared inputs.
No autonomous navigation, teleop, terrain-v1 substitution or GT TF is enabled.

## What is ready, and what is not

- September 27 smoke check: visible Isaac window, completed guarded `turn15`,
  healthy epoch 1, all recorder-required topics and five host WebSocket streams
  received. Recorder `revision-d908d0e395e64cc0892f2eefba9d8204` contains the
  screenshot and transport report. This is not long-route/terrain qualification.
- Dedicated read-only Foxglove bridge: **ws://localhost:8766**. Port 8765 belongs
  to another process; leave it alone. The Docker port is host-loopback-only.
- Isaac GUI uses `DISPLAY` (default `:1`) and the current session's read-only
  `XAUTHORITY` cookie mount, with cameras/resource raster/GT grid
  disabled to preserve the current localization workload. GUI rendering itself
  is an added workload, so this is not equivalent to a headless timing test.
- The long protocol is the existing **16 m rectangle with four 90° turns**,
  0.10 m/s / 0.10 rad/s maxima, unchanged clearance/lease/heartbeat/skid guards,
  660 s native-storage bound and 1,800 s recorder wall deadline.
- **Long-run blocker remains:** the last headless run stopped safely after a
  410 ms Isaac generation-2 GC pause. GC and the 500 ms watchdog are unchanged.
  A short GUI pass cannot qualify this long route.
- **Opt-in live v2 is now implemented** for the corrected optical-origin
  profile below. The historical pose-only commands still produce no terrain
  grid. The older `live_terrain_shadow.py` is NOT v2; verify status reports
  `policy: live_local_v2`. No navigation authority is granted.

## Corrected mounting + native GLIM/v2 costmap

For the October 8 **pure-turn drift demonstration**, change the recorder route
below to `--natural-motion turn45` and add
`--execution-profile turn45_bounded_v1` (keep `--wall-seconds 900`). Then launch:

```bash
SELECTED_EXECUTION_PROFILE=turn45_bounded_v1 \
ASTROBOT_SELECTED_LIDAR_PROFILE=selected_optical_v2 INSPECT_TERRAIN_V2=true \
INSPECT_IMPORT_STAT_CACHE=false bash docker/run_selected_gui.sh start turn45
```

This runs +45°, -45°, then returns to the original heading, with zero commanded
translation and no SLAM-driven centering. Select `/Robots/astrobot_0` in Stage
and press **F** over the viewport to frame it. Camera navigation is fine; do not
teleoperate, move prims, reset the world or pause the simulation during capture.
The controller stops after the declared sequence. Graphical rendering is extra
load; this is not equivalent to a headless performance run. The bounded profile
has a stricter 16 GiB quota and retains the separate 50 GiB disk reserve.

Use this branch for new terrain inspection. Original 0.25 m-height mounting
placed the optical origin inside the chassis. Native `minRange=0.5` also skipped
near occluders even after raising it. Corrected profile `selected_optical_v2`
uses **[0.4, 0, 0.45] m in base_link, zero roll/pitch/yaw**, the same 32-ring
mid-dense +10..-60-degree design, and casts from range zero. The software
measurement cutoff stays 0.5 m. Calibrated chassis hits are excluded, without
recovering any hypothetical points behind them; raw archives remain untouched.

October 4 candidate update: the command below selects the pointwise-boundary
fix, validated on the complete recorded loop and existing hazard/resource tests.
Fresh straight, turn-and-drive and turn-only captures now complete with this
candidate. The false-dropoff fix holds, but near-wheel visibility still limits
short-route motion availability; see [online results](online-terrain-scenarios.md).
This remains an advisory experiment, not navigation authority. For an explicitly declared old
baseline comparison use `revision-9eadef1010bf433583ce71a351502ab4` instead.
See [the fix report](false-dropoff-fix.md).

After the normal archive/GPU/X11 prerequisites, Terminal A:

```bash
docker exec --user 1000:1000 \
  -e ASTROBOT_SELECTED_LIDAR_PROFILE=selected_optical_v2 astrobot-selected-dev bash -lc '
  source /etc/ros_setup.sh &&
  source /workspace/astrobot-lab/astrobot-lab/install/setup.bash &&
  cd /workspace/astrobot-lab &&
  python3 tools/run_shadow_capture.py \
    --config /archive/revision-4c7b70a4acb44a07acb1bca0d63e3692/config \
    --archive-root /archive --require-physics-imu --natural-motion turn15 \
    --selected-lidar --terrain-v2 \
    --terrain-library /archive/revision-70647a2b742c446eb61bd3d7141ea011/liblocal_terrain.so \
    --wall-seconds 900'
```

Terminal B (retain the existing DISPLAY/XAUTHORITY/user variables):

```bash
ASTROBOT_SELECTED_LIDAR_PROFILE=selected_optical_v2 INSPECT_TERRAIN_V2=true \
  bash docker/run_selected_gui.sh start turn15
```

For the guarded translating diagnostic, change **both** `turn15` arguments to
`straight_slow`: 2 m forward and 2 m reverse at 0.05 m/s. After inspecting its
results, `combined10` adds a 15-degree turn and a faster out/back traversal.
Both retain the 900 s deadline. These are acquisition experiments, not terrain-
controlled navigation. The old loop revision/GC barriers have been addressed
and a baseline loop completed; follow the runbook's explicit slow-host profile.
The current priority is near-wheel startup support, not declaring the long loop
representative of every short-route start.

In Foxglove (`ws://localhost:8766`), use a 3D panel in **map** and enable:

| Topic | Meaning |
| --- | --- |
| `/terrain_shadow/classes` | Native v2 classification: -1 unknown, 0 measured support, 20 inferred support, 100 occupied. 60×60 cells, 10 cm, rolling 6 m window. |
| `/terrain_shadow/costmap` | Same grid plus a 1 m distance taper around occupied cells. Only supported cells receive the gradient; unknown never becomes free. Advisory, not a footprint-safe Nav2 map. |
| `/terrain_shadow/status` | `valid`, `failure`, `completed`, evidence timestamp, received/integrated counts, library hash, archive identity; no motion authority. |
| `/astrobot_0/slam_raw/aligned_points_corrected` | GLIM visualization, randomly thinned. **Not** the terrain input. |

Terrain consumes acquisition-stamped **per-tick native returns**, associates
them with causal GLIM revision state, and uses the existing fixed-reference
rolling v2 classifier (5 s raw window / 60 s evidence). Startup, faults and
completion publish unknown. Do not expect a stale map to remain marked valid
after the experiment stops. The recorded MCAP retains class/costmap topics.

Read-only runtime check, inside the sourced dev container:
`python3 tools/probe_live_costmap.py` (20 s maximum; creates a fresh archive).
Exact sealed replay on host:
`python3 tools/replay_live_local_terrain.py <external-archive/v2-mapper-run>`.
Neither is terrain-truth or navigation acceptance. Global/submap update handling
is still fail-closed; do not launch a long terrain mission on this basis.

Verified September 28 recording: external archive
`revision-1cf991e7efeb4c83b53849bc74e8e103/bag/` contains the GLIM, classes and
costmap topics (both `bag_0.mcap` and `bag_1.mcap`). The experiment completed and
Isaac was stopped; open the recordings in Foxglove for offline inspection or
use the commands above for a fresh guarded run. V2 mapper archive is
`revision-d52e36b1a46546498ea81c23923c4435`; all 328 outputs exactly reproduce
from receipt-order replay. First-published GT/base horizontal error was
8.46 cm p95, but pitch and concurrent compute targets still fail. See the ledger
for retained failures and the distinction between sensor, pose and terrain gates.

Longer corrected-mount recordings, September 28:

| Recording folder under the external archive | Protocol and result |
| --- | --- |
| `revision-6a3e3e9e2192436c8b379917597fa04c/bag/` | Slow 2 m out/back; 3.951 m actual, 81.883 s, 820 terrain maps. |
| `revision-ea53750e680247e4b6e7e5f52328f46d/bag/` | Turn + 2 m out/back + return heading; 4.094 m actual, 63.167 s, 633 terrain maps. |

Both recordings completed; whole-input audits and exact terrain replay pass.
Live horizontal GLIM error p95 is 11.21 / 7.87 cm respectively, and compute p95
50.053 / 50.316 ms still fails the 50 ms target. Independent sampled floor has
no occupied classifications but significant unknown support, concentrated near
the startup/return footprint; exact GT-pose substitution does not remove it.
See the implementation ledger for all denominators and paired spatial plots.
These are not autonomous navigation or independent obstacle-recall qualifications.

## Historical pose-only start order

Run from the repository root. The existing `astrobot-selected-dev` container
must have the built overlay and verified external `/archive` bind. See the
runbook's selected-LiDAR section if it needs recreation after drive reconnect.
Do not start the default `astrobot-dev` service (its UI ports may conflict).

First check and open the bridge (this does not start Isaac or move anything):

```bash
bash docker/run_selected_gui.sh preflight loop16
docker compose -f docker-compose.lunaryard.yml -f docker-compose.archive.yml \
  -f docker-compose.inspect.yml up -d --no-deps inspection-bridge
```

In Foxglove, choose a **Foxglove WebSocket** connection at `ws://localhost:8766`.
Use a current Foxglove client: installed bridge 3.2.6 negotiates
`foxglove.sdk.v1`, not the legacy-only `foxglove.websocket.v1`. A custom probe
using only the latter gets HTTP 400 despite the server being healthy. See the
[upstream protocol clarification](https://github.com/orgs/foxglove/discussions/1131).
For a remote workstation, tunnel `ssh -L 8766:127.0.0.1:8766 <robot-host>` and
use the same URL locally. It will initially show no sensor messages.

Verify the handshake without Isaac using
`python3 tools/probe_inspection_bridge.py --handshake-only`. During live GLIM,
omit `--handshake-only` to verify actual messages on five inspection streams.
The probe is bounded to 20 seconds and never publishes ROS data or commands.

Terminal A: start the recorder **before** Isaac. Every invocation allocates a
fresh external archive; no existing evidence is overwritten.

```bash
docker exec --user "$(id -u):$(id -g)" astrobot-selected-dev bash -lc '
  source /etc/ros_setup.sh &&
  source /workspace/astrobot-lab/astrobot-lab/install/setup.bash &&
  cd /workspace/astrobot-lab &&
  python3 tools/run_shadow_capture.py \
    --config /archive/revision-1545bcc16e884638ae30b137cacb17ea/config \
    --archive-root /archive --require-physics-imu \
    --natural-motion loop16 --selected-lidar --wall-seconds 1800'
```

Terminal B, once Terminal A prints `Run: /archive/revision-...`:

```bash
bash docker/run_selected_gui.sh start loop16
docker logs -f astrobot-sim-lunaryard
```

These are two separate commands. For a single-line paste, use
`bash docker/run_selected_gui.sh start loop16 && docker logs -f astrobot-sim-lunaryard`.
Keep the matching recorder in Terminal A running; if it timed out while
troubleshooting, start a fresh recorder first. The launcher does not require
host `rg` and reports ROS discovery errors separately from zero/duplicate recorders.

The startup helper verifies **container** X11 authorization (a host-only display
check is insufficient), host/dev CUDA, external capacity/bind and the
presence of exactly one recorder. Isaac runs its own CUDA allocation check
before world creation. The recorder and simulator then check the route match,
sensor frames, IMU, sole command ownership and surveyed clearance before motion.
Do not use Isaac's Play/Pause/Reset or move the robot manually during capture.

For the shorter smoke protocol, change **both** `loop16` arguments to `turn15`
and the recorder deadline to `900`. This executes the declared ±15°/return
sequence, not the long loop. A route mismatch is rejected, not silently adapted.

After recorder completion or failure, preserve its printed archive ID and stop
only Isaac (do not `compose down`, which would affect other services):

```bash
docker stop --time 20 astrobot-sim-lunaryard
```

To abort, interrupt Terminal A first so the recorder revokes its lease, then
stop Isaac. A GUI close, abort, missed heartbeat or reset is a failed capture.
The bridge can remain up for the next run. To stop it independently:

```bash
docker compose -f docker-compose.lunaryard.yml -f docker-compose.archive.yml \
  -f docker-compose.inspect.yml stop inspection-bridge
```

## Suggested Foxglove panels

In Isaac, the initial perspective may look away from the rover. Expand `Robots`
in the Stage panel, select `astrobot_0`, then frame the selection in the viewport
with **F**. Camera framing is fine; do not drag/transform the selected robot,
change physics, or use simulation Play/Pause/Reset during recording.

| Panel | Topics | Interpretation / frame caveats |
| --- | --- | --- |
| 3D, fixed/display frame `map` | `/astrobot_0/slam_raw/aligned_points_corrected`, `/astrobot_0/slam_raw/lidar_pose_corrected` | Corrected cloud and corresponding **LiDAR** pose; start with 0–1 s cloud decay. A long display trail does not retroactively correct old rendered points. |
| 3D, frame `vlp16` | `/astrobot_0/lidar_selected/points` | Native input bundled over six ticks, with per-point relative times. Foxglove does not deskew this display; never transform the whole window as one timestamp to construct terrain. |
| 3D, frame `gt_map` (separate panel) | `/gt/base_link_pose` | Simulator **base_link** GT, not GLIM's IMU center. Do not overlay directly onto `map`. The experimental map has a declared home translation; no fitted alignment or fake TF is added. |
| Raw Messages | `/astrobot_0/slam_raw/revisions` | Session, epoch, revision, update type and frame batch. SUBMAP/GLOBAL bookkeeping events can occur with optimization disabled. Avoid expanding huge historical arrays continuously. |
| Raw Messages | `/astrobot_0/localization/status`, `/terrain_shadow/request`, `/terrain_shadow/grant` | Health, initialization, route phase and lease state. `motion_permitted=false` is expected for the supervisor in shadow mode; only the separate guarded acquisition driver owns motion. |
| Plot | `/astrobot_0/cmd_vel.linear.x`, `/astrobot_0/cmd_vel.angular.z` | Observe commanded speed/turns; the bridge cannot publish commands. |
| Plot / Raw Messages | `/astrobot_0/imu_physics`, `/astrobot_0/imu_physics/clock`, `/clock` | Physical IMU and simulator cadence; native stamps unchanged. |
| Raw Messages | `/rtk/pose`, `/astrobot_0/localization/absolute_pose` | Synthetic RTK / supervisor absolute **constraint inputs**, not an independent GLIM pose estimate. |
| Optional supervised cloud/poses | `/astrobot_0/slam/aligned_points_corrected`, `/astrobot_0/slam/pose_corrected`, `/astrobot_0/slam/lidar_pose_corrected` | Supervisor-forwarded output. `pose_corrected` is the **IMU**, not base_link. |
| Raw Messages / TF panel | `/tf`, `/tf_static`, `/astrobot_0/odom` | Existing simulator transforms/odometry only. This shadow launch does not promise a connected `map → base_link` TF tree. Do not insert GT `map → odom` to make it look connected. |

The whitelist also exposes existing `slam_raw` observations/points/status and
terrain topics **if a publisher exists**, but does not create them. Use the
corrected cloud for map-frame viewing; the local `points_corrected` topic is
not a second global map. Keep high-volume cloud subscriptions limited: viewing
both clouds and huge observation/revision arrays is additional compute/network
load, not a free observation of baseline timing.

The bridge's read-only connection can remain open while fresh captures start;
clear display history between simulator sessions so trails from two epochs do
not appear to be one trajectory. Numerical error assessment still uses archived
same-time, same-body-frame poses rather than eyeballing two different centers.

## Remaining qualification work

Handle large SUBMAP/GLOBAL bookkeeping callbacks without future-pose leakage
or preserving evidence across an unhandled correction. Requalify terrain truth
and coverage with the corrected optical geometry, as well as both resource
profiles and concurrent runtime. Long-run Isaac GC remains an independent
blocker. Six-tick clouds at one pose and the legacy mapper are not shortcuts.

Current verification and artifact IDs are in [the progress ledger](implementation-plan.md).
