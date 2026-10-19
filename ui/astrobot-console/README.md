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

## Live ROS bridge

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

- BEV overlays from `/astrobot_0/slam/bev_costmap`, `/astrobot_0/exploration/mission_grid`, `/gt/resource_maps/water`, and `/gt/map`.
- Robot pose, trajectory, selected goal, and Nav2 plan.
- Mission status from `/astrobot_0/exploration/status`.
- Diagnostic status and plots from `/astrobot_0/diagnostics`.
- Camera topic freshness metadata for `/astrobot_0/camera_0/image_raw`.

Camera image decoding/streaming is intentionally not implemented yet; the cards
reserve the UX space and show topic availability until the preferred streaming
path is selected.
