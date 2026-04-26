Quick start commands

### Docker compose

```bash
docker compose -f docker-compose.lunaryard.yml up -d
docker compose -f docker-compose.lunaryard.yml logs -f astrobot-sim
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc 'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && colcon build --symlink-install'
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash
source /etc/ros_setup.sh
ros2 topic list
```

### Local GUI

```bash
xhost +local:root
DISPLAY=${DISPLAY:-:0} OMNILRS_HEADLESS=false docker compose -f docker-compose.lunaryard.yml up -d
docker compose -f docker-compose.lunaryard.yml logs -f astrobot-sim
```

### Rocks on/off

```bash
OMNILRS_ROCKS_ENABLED=true docker compose -f docker-compose.lunaryard.yml up -d --force-recreate astrobot-sim
OMNILRS_ROCKS_ENABLED=false docker compose -f docker-compose.lunaryard.yml up -d --force-recreate astrobot-sim
DISPLAY=${DISPLAY:-:0} OMNILRS_HEADLESS=false OMNILRS_ROCKS_ENABLED=true docker compose -f docker-compose.lunaryard.yml up -d --force-recreate astrobot-sim
```

### Foxglove

```bash
docker compose -f docker-compose.lunaryard.yml exec -d astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765'

ssh -L 8765:localhost:8765 <user>@<remote-host>
```

```text
ws://localhost:8765
```

### Phase 1 nav

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && ros2 launch astrobot_launch phase1_nav.launch.py'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 22.0, y: 18.0, z: 0.0}, orientation: {w: 1.0}}}}"'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && ros2 topic pub --once /OmniLRS/Terrain/EnableRocks std_msgs/msg/Bool "{data: true}"'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && ros2 topic pub --once /OmniLRS/Robots/Teleport geometry_msgs/msg/PoseStamped "{header: {frame_id: husky}, pose: {position: {x: 20.0, y: 18.0, z: 0.5}, orientation: {w: 1.0}}}"'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && python3 /workspace/astrobot-lab/tools/phase1_goal_sweep.py --timeout 90 --relative-goal=2.0,0.0 --relative-goal=0.0,2.0 --relative-goal=1.5,1.5'
```

### Phase 1 baseline validation

Rocks-enabled final Phase 1 baseline:

```bash
OMNILRS_ROCKS_ENABLED=true docker compose -f docker-compose.lunaryard.yml up -d --force-recreate astrobot-sim astrobot-dev

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && colcon build --symlink-install'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && ros2 launch astrobot_launch phase1_nav.launch.py'

docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && PYTHONUNBUFFERED=1 python3 /workspace/astrobot-lab/tools/phase1_goal_sweep.py --timeout 120 --auto-free 10 --auto-occupied 4 --min-distance 1.0 --max-distance 5.0 --clearance 0.60 --seed 278'
```

Expected result for the 2026-04-26 baseline:

```text
SUMMARY_FREE attempted=10 succeeded=10 min_err=0.116 max_err=0.158 mean_err=0.144
SUMMARY_OCCUPIED attempted=4 blocked=4 error_codes=[208]
```

Broader rocks stress run:

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash -lc \
  'source /etc/ros_setup.sh && cd /workspace/astrobot-lab/astrobot-lab && source install/setup.bash && PYTHONUNBUFFERED=1 python3 /workspace/astrobot-lab/tools/phase1_goal_sweep.py --timeout 120 --auto-free 12 --auto-occupied 4 --min-distance 1.0 --max-distance 7.0 --clearance 0.45 --seed 177'
```

Expected stress result from 2026-04-26:

```text
SUMMARY_FREE attempted=12 succeeded=11 min_err=0.121 max_err=0.170 mean_err=0.148
SUMMARY_OCCUPIED attempted=4 blocked=4 error_codes=[208]
```

### Bags

```bash
docker compose -f docker-compose.lunaryard.yml exec astrobot-dev bash
source /etc/ros_setup.sh
ros2 bag record -o /workspace/astrobot-lab/rosbags/lunaryard_$(date +%Y%m%d_%H%M%S) \
  /front_camera/mono/rgb /front_camera/mono/rgb_info /imu /odom /pointcloud /tf
```
