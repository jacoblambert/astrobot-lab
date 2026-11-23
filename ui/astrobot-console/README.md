# Astrobot Console

Web Ops console (Vite + React) for a running mission. The backend is the `astrobot_ui_backend`
package (FastAPI + WebSocket, read-only apart from teleop goals); the data it serves is described in
[BACKEND_CONTRACT.md](BACKEND_CONTRACT.md).

## Run

```bash
scripts/run_mission.sh 42 explore_radius --console   # or sample_return, teleop_click; --camera
                                                     # also renders the sim front camera
cd ui/astrobot-console && npm install && npm run dev
```

Open http://localhost:5173. The Vite dev server proxies `/api` and `/ws` to `127.0.0.1:8080`
(astrobot-dev publishes the port); set `ASTROBOT_BACKEND=http://<host>:8080` for a backend on
another machine. Ctrl-C in the mission terminal tears down the simulator. Until the backend answers,
the console shows an offline frame and reconnects every 2 s.

The console is an extra process beside the stack: gate numbers come from `--eval` runs without it.
`--camera` adds Isaac render load and is never used for gates.

## What it shows

- Map layers: **Cost** (Nav2 costs on the persistent terrain mosaic, `terrain/global_costmap`),
  **Explored** (observed free/occupied/unknown, `terrain/exploration_map`), **Local** (the 6 m
  window Nav2's local costmap reads, `terrain/local_costmap`), **Grid** (mission cells), water, GT.
- Views: **Arena** (fits the GT arena) or **Follow** (14 m around the rover); the mouse wheel zooms.
- Overlays: GT rocks (footprints from the sim's scene inventory, the same reference the gate scores
  contacts against), GT pose + trail (amber, evaluator only), SLAM trail, Nav2 plan, goal, the rover
  at its true 0.99 x 0.67 m footprint, and the goal candidates with the explore_radius mission disc.
- How explore_radius picks goals: coverage is the share of 2 m mission cells inside the disc
  (8 m around home) that the rover has physically *visited*; seeing a cell on the Explored layer is
  not enough. Each cycle the next goal (#1) is the nearest unvisited reachable cell (distance in
  0.5 m buckets, ties by score); the mission returns home at 70 % coverage. The **Grid** layer shows
  visited / seen-not-visited / not-yet-seen cells.
- Teleop click (teleop_click mission): tick *Teleop click* and click the map to send a goal.
- Panels: localization supervisor (state, reason, motion permitted, GLIM epoch, sim speed) and
  terrain node (valid, epoch, chunks, skipped/late, evidence lag, RSS); with `--camera`, the camera
  replaces the terrain panel. The robot panel shows the SLAM error against GT.
- Topic rates are per sim second (Isaac runs below real time; the sim speed is shown separately).
- Event log: mission events (goals, reveals, home approach, failures), supervisor transitions,
  terrain validity, diagnostics flags.
- Bookmarkable views: `?layer=explore&view=follow&rocks=1&gt=1` (`candidates=0` hides candidates).
