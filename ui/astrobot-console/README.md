# Astrobot Console

Lightweight Phase 5 mission console prototype.

## Local mockup

```bash
cd ui/astrobot-console
npm install
npm run dev
```

Open:

```text
http://localhost:5173
```

If the FastAPI backend is not running, the frontend uses deterministic mock
telemetry so the layout, theme, and responsive behavior can be reviewed without
Isaac Sim or ROS.

## Live v2 session (review / demo)

```bash
tools/v2_demo.sh 42 explore_radius            # or sample_return, teleop_click; add --eval for a GT verdict
                                              # --camera also renders the sim front camera (extra Isaac load)
# then, on the host, with the IP the script prints (astrobot-selected-dev publishes no ports):
cd ui/astrobot-console && ASTROBOT_BACKEND=http://172.19.0.2:8080 npm run dev
```

Open http://localhost:5173. Ctrl-C in the demo terminal tears down the sim (stop + rm) and the stack.
The demo runs the gated mission configuration (`tools/v2_mission_env.sh`), but the console is an
extra process: gate numbers come from `tools/v2_gate_run.sh` only.

What the v2 console shows:

- Map layers: **Cost** (Nav2 costs on the persistent v2 mosaic, `/terrain_shadow/global_costmap`),
  **Explored** (observed free/occupied/unknown, `/terrain_shadow/exploration_map`), **Local** (the 6 m
  window Nav2's local costmap reads, `/terrain_shadow/planner_costmap`), mission grid, water, GT.
- Views: **Arena** (fits the GT arena) or **Follow** (14 m around the rover); mouse wheel zooms.
- Overlays: GT rocks (footprints from the sim's scene inventory, the same reference the gate scores
  contacts against), GT pose + trail (amber, evaluator only), SLAM trail, Nav2 plan, goal, the rover
  at its true 0.99 x 0.67 m footprint, and **goal candidates** with the explore_radius mission disc.
- How explore_radius picks goals: coverage is the share of 2 m mission cells inside the disc
  (8 m around home) that the rover has physically *visited*; seeing a cell on the Explored layer is
  not enough. Each cycle the next goal (#1) is the nearest unvisited reachable cell (distance in
  0.5 m buckets, ties by score); the mission returns home at 70 % coverage. The **Grid** layer shows
  visited / seen-not-visited / not-yet-seen cells.
- Topic rates are per sim second (Isaac runs below real time; the sim speed is shown separately).
- Panels (in place of the cameras when no camera topic exists; with `--camera`, the camera plus the
  supervisor panel): localization supervisor (state,
  reason, motion permitted, GLIM epoch, sim speed) and terrain publisher (valid, epoch, chunks,
  skipped/late, evidence lag, RSS). The robot panel shows SLAM error against GT.
- Event log: mission events (goals, reveals, home approach, failures), supervisor transitions,
  terrain validity, diagnostics flags.
- Bookmarkable views: `?layer=explore&view=follow&rocks=1&gt=1` (`candidates=0` hides candidates).

## Live ROS bridge (legacy stack)

Launch the autonomy stack first, then start the backend:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && ros2 launch astrobot_launch phase5_gui.launch.py'
```

Run the frontend locally:

```bash
cd ui/astrobot-console
npm run dev
```

The Vite dev server proxies `/api` and `/ws` to `127.0.0.1:8080`.

Current live features:

- BEV overlays (legacy, `stack:=legacy`, the default) from `/astrobot_0/slam/bev_costmap`, `/astrobot_0/exploration/mission_grid`, `/gt/resource_maps/water`, and `/gt/map`.
- Robot pose, trajectory, selected goal, and Nav2 plan.
- Mission status from `/astrobot_0/exploration/status`.
- Diagnostic status and plots from `/astrobot_0/diagnostics`.
- Camera topic freshness metadata for `/astrobot_0/camera_0/image_raw`.

Camera image decoding/streaming is intentionally not implemented yet; the cards
reserve the UX space and show topic availability until the preferred streaming
path is selected.
