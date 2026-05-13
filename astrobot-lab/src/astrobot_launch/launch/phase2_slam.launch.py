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
    start_voxel_map = LaunchConfiguration("start_voxel_map")
    start_bev_map = LaunchConfiguration("start_bev_map")
    glim_config_path = LaunchConfiguration("glim_config_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_glim", default_value="true"),
            DeclareLaunchArgument("start_voxel_map", default_value="true"),
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
                executable="voxel_map_builder_node",
                name="voxel_map_builder",
                output="screen",
                condition=IfCondition(start_voxel_map),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": "/glim_rosnode/aligned_points_corrected",
                        "output_topic": "/slam/voxel_map",
                        "voxel_size": ParameterValue(LaunchConfiguration("voxel_size"), value_type=float),
                        "publish_rate_hz": 1.0,
                        "max_voxels": 2000000,
                    }
                ],
            ),
            Node(
                package="slam_eval",
                executable="bev_map_builder",
                name="bev_map_builder",
                output="screen",
                condition=IfCondition(start_bev_map),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": "/slam/voxel_map",
                        "output_topic": "/slam/bev_map",
                        "resolution": 0.10,
                        "width_m": 40.0,
                        "height_m": 40.0,
                        "origin_x": -20.0,
                        "origin_y": -20.0,
                        "height_threshold": 0.02,
                        "min_points_per_cell": 2,
                        "max_scans": 1,
                        "publish_rate_hz": 1.0,
                    }
                ],
            ),
            Node(
                package="slam_mapping",
                executable="voxel_to_bev_costmap_node",
                name="voxel_to_bev_costmap",
                output="screen",
                condition=IfCondition(start_bev_map),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": "/slam/voxel_map",
                        "output_topic": "/slam/bev_costmap",
                        "resolution": ParameterValue(LaunchConfiguration("bev_resolution"), value_type=float),
                        "width_m": 40.0,
                        "height_m": 40.0,
                        "origin_x": -20.0,
                        "origin_y": -20.0,
                        "min_points_per_cell": 1,
                        "ground_window_radius": 2,
                        "slope_window_radius": 1,
                        "obstacle_height": 0.02,
                        "lethal_obstacle_height": 0.20,
                        "use_height_span_evidence": True,
                        "mark_obstacles_lethal": True,
                        "obstacle_inflation_seed_height": 0.10,
                        "obstacle_seed_cost": 100,
                        "obstacle_inflation_radius": 0.25,
                        "obstacle_inflation_cost": 60,
                        "slope_warn": 0.45,
                        "slope_lethal": 1.00,
                        "free_fill_radius": 0.20,
                    }
                ],
            ),
        ]
    )
