"""v2 SLAM + terrain for navigation: supervised GLIM, synthetic RTK, live v2 terrain publisher.

Replaces phase2_slam (legacy BEV mapper) under phase3_nav `slam_stack:=v2`. Same inputs as the
validated captures (selected LiDAR, physical IMU, frozen mount-audited GLIM config, RTK home and
antenna), but the supervisor runs in navigation mode: its map readiness reads the v2 planner
window and the v2 publisher serves slam/reset_map. Requires the simulator nav stream
(docker-compose.selected-nav.yml).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Argument names are v2-specific: launch configurations are shared with the including
    # phase3_nav, whose glim_config_path is the legacy default.
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("robot_namespace", default_value="astrobot_0"),
        DeclareLaunchArgument("v2_glim_config_path", default_value="/archive/revision-ed871eabc2e64d1eb657d292e5d9ac77/config",
                              description="Frozen, mount-audited selected-LiDAR GLIM configuration"),
        DeclareLaunchArgument("terrain_library", default_value="/archive/revision-06a82bedebf641418a4d6d420447cef2/liblocal_terrain.so"),
        DeclareLaunchArgument("terrain_request_json", default_value=PathJoinSubstitution(
            [FindPackageShare("astrobot_launch"), "config", "v2_nav_request.json"])),
        DeclareLaunchArgument("terrain_archive_raw", default_value="false",
                              description="Archive native bytes/events for offline audit replay (~28 MB per sim-s)"),
        DeclareLaunchArgument("mosaic_half_extent_m", default_value="30.0"),
        DeclareLaunchArgument("rtk_dropout_prob", default_value="0.0"),
        # T3 fault injection: RTK lost while (sim time % period) < duration; 0 disables
        DeclareLaunchArgument("rtk_occlusion_period_s", default_value="0.0"),
        DeclareLaunchArgument("rtk_occlusion_duration_s", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("glim_slam_launch"), "launch", "glim_live.launch.py"])),
            launch_arguments={"config_path": LaunchConfiguration("v2_glim_config_path"),
                              "robot_namespace": LaunchConfiguration("robot_namespace"),
                              "map_topic": "/terrain_shadow/planner_costmap",
                              "shadow_mode": "false", "use_sim_time": use_sim_time}.items()),
        Node(package="slam_mapping", executable="live_local_terrain.py", name="terrain_local_v2", output="screen",
             parameters=[{"use_sim_time": True,
                          "deployment": "nav",
                          "library": LaunchConfiguration("terrain_library"),
                          "request_json": LaunchConfiguration("terrain_request_json"),
                          "planner_gap_policy": "nav2_bounded_shadow",
                          "archive_raw": ParameterValue(LaunchConfiguration("terrain_archive_raw"), value_type=bool),
                          "mosaic_half_extent_m": ParameterValue(LaunchConfiguration("mosaic_half_extent_m"),
                                                                 value_type=float)}]),
        # Same RTK as the validated captures (terrain_shadow.launch.py): map = world - home.
        Node(package="synthetic_rtk", executable="synthetic_rtk", name="synthetic_rtk", output="screen",
             parameters=[{"use_sim_time": True, "gt_topic": "/gt/base_link_pose", "output_topic": "/rtk/pose",
                          "world_frame": "map", "home_xy": [-20.0, -20.0],
                          "antenna_offset_xyz": [0.26399862939321395, 0.017279625659938347, 0.2624700776230195],
                          "sigma0": 0.02, "sigma_per_m": 0.001, "latency_s": 0.0,
                          "dropout_prob": ParameterValue(LaunchConfiguration("rtk_dropout_prob"), value_type=float),
                          "occlusion_period_s": ParameterValue(LaunchConfiguration("rtk_occlusion_period_s"), value_type=float),
                          "occlusion_duration_s": ParameterValue(LaunchConfiguration("rtk_occlusion_duration_s"),
                                                                 value_type=float)}]),
    ])
