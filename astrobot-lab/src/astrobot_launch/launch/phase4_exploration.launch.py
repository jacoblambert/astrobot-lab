from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_namespace = LaunchConfiguration("robot_namespace")
    start_exploration = LaunchConfiguration("start_exploration")
    start_diagnostics = LaunchConfiguration("start_diagnostics")
    exploration_start_delay = LaunchConfiguration("exploration_start_delay")
    mission_mode = LaunchConfiguration("mission_mode")
    exploration_map_topic = LaunchConfiguration("exploration_map_topic")
    home_radius_m = LaunchConfiguration("home_radius_m")
    coverage_goal_fraction = LaunchConfiguration("coverage_goal_fraction")
    sample_success_threshold = LaunchConfiguration("sample_success_threshold")
    max_goals = LaunchConfiguration("max_goals")
    max_duration_sec = LaunchConfiguration("max_duration_sec")
    goal_timeout_sec = LaunchConfiguration("goal_timeout_sec")
    goal_timeout_per_meter_sec = LaunchConfiguration("goal_timeout_per_meter_sec")
    goal_timeout_max_sec = LaunchConfiguration("goal_timeout_max_sec")
    max_goal_distance_m = LaunchConfiguration("max_goal_distance_m")
    blacklist_radius_m = LaunchConfiguration("blacklist_radius_m")
    blacklist_timeout_sec = LaunchConfiguration("blacklist_timeout_sec")
    min_goal_distance_m = LaunchConfiguration("min_goal_distance_m")
    recent_goal_radius_m = LaunchConfiguration("recent_goal_radius_m")
    recent_goal_timeout_sec = LaunchConfiguration("recent_goal_timeout_sec")
    distance_weight = LaunchConfiguration("distance_weight")
    over_distance_weight = LaunchConfiguration("over_distance_weight")
    resource_weight_scale = LaunchConfiguration("resource_weight_scale")
    resource_influence_radius_m = LaunchConfiguration("resource_influence_radius_m")
    resource_decay_radius_m = LaunchConfiguration("resource_decay_radius_m")
    max_resource_candidates = LaunchConfiguration("max_resource_candidates")
    resource_explore_radius_m = LaunchConfiguration("resource_explore_radius_m")
    resource_min_explore_step_m = LaunchConfiguration("resource_min_explore_step_m")
    use_resource_candidates_in_explore_radius = LaunchConfiguration("use_resource_candidates_in_explore_radius")
    planner_precheck_enabled = LaunchConfiguration("planner_precheck_enabled")
    planner_precheck_timeout_sec = LaunchConfiguration("planner_precheck_timeout_sec")
    planner_precheck_max_cost = LaunchConfiguration("planner_precheck_max_cost")
    planner_precheck_high_cost = LaunchConfiguration("planner_precheck_high_cost")
    planner_precheck_max_high_cost_fraction = LaunchConfiguration("planner_precheck_max_high_cost_fraction")
    goal_clearance_radius_m = LaunchConfiguration("goal_clearance_radius_m")
    goal_clearance_cost_min = LaunchConfiguration("goal_clearance_cost_min")
    breadcrumb_return_enabled = LaunchConfiguration("breadcrumb_return_enabled")
    breadcrumb_spacing_m = LaunchConfiguration("breadcrumb_spacing_m")
    breadcrumb_max_home_radius_factor = LaunchConfiguration("breadcrumb_max_home_radius_factor")
    return_breadcrumb_stride_m = LaunchConfiguration("return_breadcrumb_stride_m")
    return_breadcrumb_min_distance_m = LaunchConfiguration("return_breadcrumb_min_distance_m")
    mission_cell_size_m = LaunchConfiguration("mission_cell_size_m")
    mission_coverage_min_known_fraction = LaunchConfiguration("mission_coverage_min_known_fraction")
    mission_candidate_weight = LaunchConfiguration("mission_candidate_weight")
    mission_candidate_limit = LaunchConfiguration("mission_candidate_limit")
    resource_names = LaunchConfiguration("resource_names")
    resource_weights = LaunchConfiguration("resource_weights")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("robot_namespace", default_value="astrobot_0"),
            DeclareLaunchArgument("start_exploration", default_value="true"),
            DeclareLaunchArgument("start_diagnostics", default_value="true"),
            DeclareLaunchArgument("exploration_start_delay", default_value="35.0"),
            DeclareLaunchArgument("mission_mode", default_value="explore_radius"),
            DeclareLaunchArgument("exploration_map_topic", default_value="slam/bev_costmap"),
            DeclareLaunchArgument("home_radius_m", default_value="16.0"),
            DeclareLaunchArgument("coverage_goal_fraction", default_value="0.30"),
            DeclareLaunchArgument("sample_success_threshold", default_value="0.75"),
            DeclareLaunchArgument("max_goals", default_value="30"),
            DeclareLaunchArgument("max_duration_sec", default_value="900.0"),
            DeclareLaunchArgument("goal_timeout_sec", default_value="120.0"),
            DeclareLaunchArgument("goal_timeout_per_meter_sec", default_value="18.0"),
            DeclareLaunchArgument("goal_timeout_max_sec", default_value="300.0"),
            DeclareLaunchArgument("max_goal_distance_m", default_value="5.0"),
            DeclareLaunchArgument("blacklist_radius_m", default_value="0.75"),
            DeclareLaunchArgument("blacklist_timeout_sec", default_value="0.0"),
            DeclareLaunchArgument("min_goal_distance_m", default_value="0.75"),
            DeclareLaunchArgument("recent_goal_radius_m", default_value="0.75"),
            DeclareLaunchArgument("recent_goal_timeout_sec", default_value="240.0"),
            DeclareLaunchArgument("distance_weight", default_value="0.8"),
            DeclareLaunchArgument("over_distance_weight", default_value="2.0"),
            DeclareLaunchArgument("resource_weight_scale", default_value="5.0"),
            DeclareLaunchArgument("resource_influence_radius_m", default_value="4.0"),
            DeclareLaunchArgument("resource_decay_radius_m", default_value="1.75"),
            DeclareLaunchArgument("max_resource_candidates", default_value="20"),
            DeclareLaunchArgument("resource_explore_radius_m", default_value="3.0"),
            DeclareLaunchArgument("resource_min_explore_step_m", default_value="0.75"),
            DeclareLaunchArgument("use_resource_candidates_in_explore_radius", default_value="false"),
            DeclareLaunchArgument("planner_precheck_enabled", default_value="true"),
            DeclareLaunchArgument("planner_precheck_timeout_sec", default_value="8.0"),
            DeclareLaunchArgument("planner_precheck_max_cost", default_value="70"),
            DeclareLaunchArgument("planner_precheck_high_cost", default_value="50"),
            DeclareLaunchArgument("planner_precheck_max_high_cost_fraction", default_value="0.04"),
            DeclareLaunchArgument("goal_clearance_radius_m", default_value="0.75"),
            DeclareLaunchArgument("goal_clearance_cost_min", default_value="80"),
            DeclareLaunchArgument("breadcrumb_return_enabled", default_value="true"),
            DeclareLaunchArgument("breadcrumb_spacing_m", default_value="0.75"),
            DeclareLaunchArgument("breadcrumb_max_home_radius_factor", default_value="1.25"),
            DeclareLaunchArgument("return_breadcrumb_stride_m", default_value="1.5"),
            DeclareLaunchArgument("return_breadcrumb_min_distance_m", default_value="0.75"),
            DeclareLaunchArgument("mission_cell_size_m", default_value="1.0"),
            DeclareLaunchArgument("mission_coverage_min_known_fraction", default_value="0.30"),
            DeclareLaunchArgument("mission_candidate_weight", default_value="3.0"),
            DeclareLaunchArgument("mission_candidate_limit", default_value="80"),
            DeclareLaunchArgument("resource_names", default_value="water"),
            DeclareLaunchArgument("resource_weights", default_value="water:1.0"),
            LogInfo(
                msg=(
                    "Phase 4 exploration requires rocks-enabled Lunaryard and "
                    "OMNILRS_GT_TF_ENABLED=false. Resource GT maps are evaluation-only."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [PathJoinSubstitution([FindPackageShare("astrobot_launch"), "launch", "phase3_nav.launch.py"])]
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "robot_namespace": robot_namespace,
                }.items(),
            ),
            TimerAction(
                period=exploration_start_delay,
                actions=[
                    PushRosNamespace(robot_namespace),
                    Node(
                        package="astrobot_diagnostics",
                        executable="diagnostics_node",
                        name="diagnostics",
                        output="screen",
                        condition=IfCondition(start_diagnostics),
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "publish_topic": "diagnostics",
                                "period_sec": 2.0,
                                "path_corridor_radius_m": 0.35,
                                "goal_radius_m": 0.75,
                                "progress_timeout_sec": 35.0,
                            }
                        ],
                    ),
                    Node(
                        package="exploration_manager",
                        executable="exploration_manager",
                        name="exploration_manager",
                        output="screen",
                        condition=IfCondition(start_exploration),
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "robot_namespace": robot_namespace,
                                "mission_mode": mission_mode,
                                "map_topic": exploration_map_topic,
                                "pose_topic": "slam/pose_corrected",
                                "action_name": "navigate_to_pose",
                                "planner_action_name": "compute_path_to_pose",
                                "frame_id": "map",
                                "resource_names": resource_names,
                                "resource_weights": resource_weights,
                                "home_radius_m": ParameterValue(home_radius_m, value_type=float),
                                "coverage_goal_fraction": ParameterValue(
                                    coverage_goal_fraction, value_type=float
                                ),
                                "sample_success_threshold": ParameterValue(
                                    sample_success_threshold, value_type=float
                                ),
                                "max_goals": ParameterValue(max_goals, value_type=int),
                                "max_duration_sec": ParameterValue(max_duration_sec, value_type=float),
                                "goal_timeout_sec": ParameterValue(goal_timeout_sec, value_type=float),
                                "goal_timeout_per_meter_sec": ParameterValue(
                                    goal_timeout_per_meter_sec, value_type=float
                                ),
                                "goal_timeout_max_sec": ParameterValue(
                                    goal_timeout_max_sec, value_type=float
                                ),
                                "max_goal_distance_m": ParameterValue(max_goal_distance_m, value_type=float),
                                "blacklist_radius_m": ParameterValue(blacklist_radius_m, value_type=float),
                                "blacklist_timeout_sec": ParameterValue(
                                    blacklist_timeout_sec, value_type=float
                                ),
                                "min_goal_distance_m": ParameterValue(min_goal_distance_m, value_type=float),
                                "recent_goal_radius_m": ParameterValue(
                                    recent_goal_radius_m, value_type=float
                                ),
                                "recent_goal_timeout_sec": ParameterValue(
                                    recent_goal_timeout_sec, value_type=float
                                ),
                                "distance_weight": ParameterValue(distance_weight, value_type=float),
                                "over_distance_weight": ParameterValue(
                                    over_distance_weight, value_type=float
                                ),
                                "resource_weight_scale": ParameterValue(
                                    resource_weight_scale, value_type=float
                                ),
                                "resource_influence_radius_m": ParameterValue(
                                    resource_influence_radius_m, value_type=float
                                ),
                                "resource_decay_radius_m": ParameterValue(
                                    resource_decay_radius_m, value_type=float
                                ),
                                "max_resource_candidates": ParameterValue(
                                    max_resource_candidates, value_type=int
                                ),
                                "resource_explore_radius_m": ParameterValue(
                                    resource_explore_radius_m, value_type=float
                                ),
                                "resource_min_explore_step_m": ParameterValue(
                                    resource_min_explore_step_m, value_type=float
                                ),
                                "use_resource_candidates_in_explore_radius": ParameterValue(
                                    use_resource_candidates_in_explore_radius, value_type=bool
                                ),
                                "planner_precheck_enabled": ParameterValue(
                                    planner_precheck_enabled, value_type=bool
                                ),
                                "planner_precheck_timeout_sec": ParameterValue(
                                    planner_precheck_timeout_sec, value_type=float
                                ),
                                "planner_precheck_max_cost": ParameterValue(
                                    planner_precheck_max_cost, value_type=int
                                ),
                                "planner_precheck_high_cost": ParameterValue(
                                    planner_precheck_high_cost, value_type=int
                                ),
                                "planner_precheck_max_high_cost_fraction": ParameterValue(
                                    planner_precheck_max_high_cost_fraction, value_type=float
                                ),
                                "goal_clearance_radius_m": ParameterValue(
                                    goal_clearance_radius_m, value_type=float
                                ),
                                "goal_clearance_cost_min": ParameterValue(
                                    goal_clearance_cost_min, value_type=int
                                ),
                                "breadcrumb_return_enabled": ParameterValue(
                                    breadcrumb_return_enabled, value_type=bool
                                ),
                                "breadcrumb_spacing_m": ParameterValue(breadcrumb_spacing_m, value_type=float),
                                "breadcrumb_max_home_radius_factor": ParameterValue(
                                    breadcrumb_max_home_radius_factor, value_type=float
                                ),
                                "return_breadcrumb_stride_m": ParameterValue(
                                    return_breadcrumb_stride_m, value_type=float
                                ),
                                "return_breadcrumb_min_distance_m": ParameterValue(
                                    return_breadcrumb_min_distance_m, value_type=float
                                ),
                                "mission_cell_size_m": ParameterValue(mission_cell_size_m, value_type=float),
                                "mission_coverage_min_known_fraction": ParameterValue(
                                    mission_coverage_min_known_fraction, value_type=float
                                ),
                                "mission_candidate_weight": ParameterValue(
                                    mission_candidate_weight, value_type=float
                                ),
                                "mission_candidate_limit": ParameterValue(
                                    mission_candidate_limit, value_type=int
                                ),
                            }
                        ],
                    ),
                ],
            ),
        ]
    )
