from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_glim = LaunchConfiguration("start_glim")
    start_bev_map = LaunchConfiguration("start_bev_map")
    glim_config_path = LaunchConfiguration("glim_config_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_glim", default_value="true"),
            DeclareLaunchArgument("start_bev_map", default_value="true"),
            DeclareLaunchArgument("voxel_size", default_value="0.05"),
            DeclareLaunchArgument("bev_resolution", default_value="0.10"),
            DeclareLaunchArgument("crop_min_x", default_value="0.20"),
            DeclareLaunchArgument("crop_max_x", default_value="0.80"),
            DeclareLaunchArgument("crop_min_y", default_value="-0.45"),
            DeclareLaunchArgument("crop_max_y", default_value="0.45"),
            DeclareLaunchArgument("crop_min_z", default_value="-0.35"),
            DeclareLaunchArgument("crop_max_z", default_value="0.05"),
            DeclareLaunchArgument(
                "glim_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("glim_slam_launch"), "config", "glim_astrobot"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [PathJoinSubstitution([FindPackageShare("glim_slam_launch"), "launch", "glim_live.launch.py"])]
                ),
                condition=IfCondition(start_glim),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "config_path": glim_config_path,
                    "crop_min_x": LaunchConfiguration("crop_min_x"),
                    "crop_max_x": LaunchConfiguration("crop_max_x"),
                    "crop_min_y": LaunchConfiguration("crop_min_y"),
                    "crop_max_y": LaunchConfiguration("crop_max_y"),
                    "crop_min_z": LaunchConfiguration("crop_min_z"),
                    "crop_max_z": LaunchConfiguration("crop_max_z"),
                }.items(),
            ),
            Node(
                package="slam_mapping",
                executable="probabilistic_voxel_mapper_node",
                name="probabilistic_voxel_mapper",
                output="screen",
                condition=IfCondition(start_bev_map),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": "/glim_rosnode/aligned_points_corrected",
                        "pose_topic": "/glim_rosnode/pose_corrected",
                        "voxel_output_topic": "/slam/prob_voxel_map",
                        "costmap_output_topic": "/slam/bev_costmap",
                        "frame_id": "glim_map",
                        "voxel_size": ParameterValue(LaunchConfiguration("voxel_size"), value_type=float),
                        "bev_resolution": ParameterValue(LaunchConfiguration("bev_resolution"), value_type=float),
                        "width_m": 40.0,
                        "height_m": 40.0,
                        "origin_x": -20.0,
                        "origin_y": -20.0,
                        "hit_log_odds": 0.85,
                        "miss_log_odds": -0.20,
                        "min_log_odds": -3.0,
                        "max_log_odds": 4.0,
                        "occupied_threshold": 1.0,
                        "free_threshold": -0.8,
                        "max_range": 35.0,
                        "point_stride": 3,
                        "ray_stride": 1,
                        "min_z": -2.0,
                        "max_z": 4.0,
                        "max_voxels": 4000000,
                        "ground_window_radius": 2,
                        "ground_percentile": 0.10,
                        "low_z_percentile": 0.10,
                        "high_z_percentile": 0.90,
                        "min_occupied_voxels_per_cell": 1,
                        "obstacle_height": 0.12,
                        "lethal_obstacle_height": 0.35,
                        "obstacle_cost_min": 88,
                        "obstacle_cost_max": 100,
                        "terrain_risk_height": 0.06,
                        "terrain_risk_cost_max": 8,
                        "obstacle_inflation_radius": 0.85,
                        "obstacle_inflation_cost": 62,
                        "obstacle_inflation_min_cost": 8,
                        "obstacle_inflation_exponent": 1.35,
                        "publish_voxel_map": False,
                        "publish_rate_hz": 2.0,
                    }
                ],
            ),
        ]
    )
