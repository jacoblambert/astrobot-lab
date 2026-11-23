# Extending Astrobot

## Change the world: seed, rocks, resources

The simulator reads its settings from the environment of `docker compose up astrobot-sim`
(`docker-compose.yml`); `scripts/run_mission.sh <seed>` sets the seed.

| Setting | Where |
|---|---|
| Terrain and rock seed | `OMNILRS_ENVIRONMENT_SEED` (run_mission.sh's first argument) |
| Rock density and size | `OmniLRS/cfg/environment/lunaryard_40m.yaml` (`lambda_parent`, `req_scale`). Keep `use_point_instancer: False`: instanced rocks are invisible to the RTX LiDAR. |
| Resource field | `OMNILRS_RESOURCE_CENTERS=x,y,peak,sigma` (sample_return sets 11.875,12.133,8.0,1.0), or `OMNILRS_RESOURCE_MODE` / `OMNILRS_RESOURCE_SEED` for a random field |
| Spawn | `OmniLRS/cfg/environment/lunaryard_40m.yaml`; home moves with it, the RTK home in `astrobot_bringup/config/sim.yaml` must follow |

`OMNILRS_GT_TF_ENABLED` must stay `false`: SLAM owns `map` -> `odom`.

## Change mission objectives

Mission parameters live in `astrobot_exploration/config/<mission>.yaml` (radius, coverage
goal, sample threshold, durations, weights). Pass another file with
`mission_params:=/path/to/file.yaml`. Common goal-selection parameters (clearance, precheck
limits, blacklists) are declared by the node; their defaults are the validated gate values.

## Add a mission

A mission is a `pluginlib` class deriving from `astrobot_exploration::MissionStrategy`
(`include/astrobot_exploration/mission_strategy.hpp`):

```cpp
class PatrolWaypoints : public astrobot_exploration::MissionStrategy
{
public:
  void initialize(rclcpp::Node & node) override {/* declare your parameters */}
  // Ranked goals, best first. Empty: the manager tries reveal goals, then returns home.
  std::vector<Candidate> candidates(MissionContext & c) override;
  // Mission complete between goals (or urgent_return to preempt an active goal).
  std::optional<std::string> return_reason(const MissionContext & c) override;
};
PLUGINLIB_EXPORT_CLASS(PatrolWaypoints, astrobot_exploration::MissionStrategy)
```

`MissionContext::planner` gives the map, rover pose, home, reachability, frontier and coverage
candidates, failure memory and goal checks (`mission_planner.hpp`). Export the class in your
package's `plugins.xml` with `<class name="my_pkg/PatrolWaypoints" ...>` and
`pluginlib_export_plugin_description_file(astrobot_exploration plugins.xml)`, then select it
with `mission_plugin: my_pkg/PatrolWaypoints` in a mission YAML.

## Change the rover

`astrobot_bringup/config/robot.yaml` holds the geometry the core nodes share: base <- IMU,
base <- LiDAR, the chassis self-return box and the padded footprint. Keep the Nav2
`footprint` / `footprint_padding` in `config/nav2.yaml` equal to it. The LiDAR model and mount
used by the simulator come from the selected LiDAR profile (`selected_optical_v2`).

## Swap a component

Nodes talk through namespaced topics, so a replacement only has to keep the contract:

- **Terrain**: publish `terrain/local_costmap`, `terrain/global_costmap`,
  `terrain/exploration_map` (`nav_msgs/OccupancyGrid`, frame `map`) and serve
  `slam/reset_map` (`astrobot_interfaces/srv/ResetMap`). The supervisor's `map_topic` and
  the Nav2 static layers name these topics.
- **SLAM**: the supervisor starts `glim_ros/glim_rosnode` with the configuration in
  `glim_config`; another estimator must publish `slam_raw/pose_corrected` (IMU frame) and
  `slam_raw/revisions` (`astrobot_interfaces/msg/SlamRevision`) for the terrain node.
- **Absolute fixes**: anything publishing `/rtk/pose` (`PoseWithCovarianceStamped`, frame `map`)
  replaces `synthetic_rtk`.
