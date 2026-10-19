"""Opt-in calibration localization, optionally advisory terrain. No commands/Nav2.

The home is the declared analytic platform origin, not a fit to observed GT.
Use an audited mount-specific config snapshot, never the untilted default.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mapper = Node(package="slam_mapping", executable=LaunchConfiguration("terrain_executable"), name="terrain_shadow_mapper",
                  condition=IfCondition(LaunchConfiguration("terrain_mapping")), output="screen",
                  parameters=[{"use_sim_time": True,
                               "library": LaunchConfiguration("terrain_library"),
                               "request_json": LaunchConfiguration("terrain_request_json"),
                               "contact_advisory": ParameterValue(LaunchConfiguration("contact_advisory"), value_type=bool),
                               "isolated_gap_costs": ParameterValue(LaunchConfiguration("isolated_gap_costs"), value_type=bool),
                               "contact_deployment_confirmed": ParameterValue(LaunchConfiguration("contact_deployment_confirmed"), value_type=bool),
                               "contact_support_core": ParameterValue(LaunchConfiguration("contact_support_core"), value_type=bool),
                               "contact_startup_turn_area": ParameterValue(LaunchConfiguration("contact_startup_turn_area"), value_type=bool),
                               "contact_current_turn_area": ParameterValue(LaunchConfiguration("contact_current_turn_area"), value_type=bool),
                               "origin_x": ParameterValue(LaunchConfiguration("terrain_origin_x"), value_type=float),
                               "origin_y": ParameterValue(LaunchConfiguration("terrain_origin_y"), value_type=float),
                               "min_range": ParameterValue(LaunchConfiguration("terrain_min_range"), value_type=float),
                               "max_range": ParameterValue(LaunchConfiguration("terrain_max_range"), value_type=float)}])
    return LaunchDescription([
        DeclareLaunchArgument("config_path", description="Frozen, mount-audited GLIM configuration directory"),
        DeclareLaunchArgument("shadow_seed_path", default_value=""),
        DeclareLaunchArgument("terrain_mapping", default_value="false"),
        DeclareLaunchArgument("terrain_executable", default_value="live_terrain_shadow.py"),
        DeclareLaunchArgument("terrain_request_json", default_value=""),
        DeclareLaunchArgument("contact_advisory", default_value="false"),
        DeclareLaunchArgument("isolated_gap_costs", default_value="false"),
        DeclareLaunchArgument("contact_deployment_confirmed", default_value="false"),
        DeclareLaunchArgument("contact_support_core", default_value="false"),
        DeclareLaunchArgument("contact_startup_turn_area", default_value="false"),
        DeclareLaunchArgument("contact_current_turn_area", default_value="false"),
        DeclareLaunchArgument("terrain_library", default_value=""),
        DeclareLaunchArgument("terrain_origin_x", default_value="nan"),
        DeclareLaunchArgument("terrain_origin_y", default_value="nan"),
        DeclareLaunchArgument("terrain_min_range", default_value="nan"),
        DeclareLaunchArgument("terrain_max_range", default_value="nan"),
        RegisterEventHandler(OnProcessExit(target_action=mapper,
            on_exit=[EmitEvent(event=Shutdown(reason="advisory terrain process exited; end guarded experiment"))])),
        mapper,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("glim_slam_launch"), "launch", "glim_live.launch.py"])),
            launch_arguments={"config_path": LaunchConfiguration("config_path"),
                              "shadow_seed_path": LaunchConfiguration("shadow_seed_path"),
                              "shadow_mode": "true", "use_sim_time": "true"}.items()),
        Node(package="synthetic_rtk", executable="synthetic_rtk", name="synthetic_rtk", output="screen",
             parameters=[{"use_sim_time": True, "gt_topic": "/gt/base_link_pose", "output_topic": "/rtk/pose",
                          "world_frame": "map", "home_xy": [-20.0, -20.0],
                          # Measured settled USD/TF joint origin; not a fitted
                          # trajectory alignment (see audit_shadow_inputs.py).
                          "antenna_offset_xyz": [0.26399862939321395, 0.017279625659938347, 0.2624700776230195],
                          "sigma0": 0.05, "sigma_per_m": 0.01, "dropout_prob": 0.0, "latency_s": 0.0}]),
    ])
