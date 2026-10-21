from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_glim = LaunchConfiguration("start_glim")
    start_bev_map = LaunchConfiguration("start_bev_map")
    start_synthetic_rtk = LaunchConfiguration("start_synthetic_rtk")
    glim_config_path = LaunchConfiguration("glim_config_path")
    robot_namespace = LaunchConfiguration("robot_namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("robot_namespace", default_value="astrobot_0"),
            DeclareLaunchArgument("start_glim", default_value="true"),
            DeclareLaunchArgument("start_bev_map", default_value="true"),
            DeclareLaunchArgument("start_synthetic_rtk", default_value="true"),
            DeclareLaunchArgument("rtk_sigma0", default_value="0.02"),
            DeclareLaunchArgument("rtk_sigma_per_m", default_value="0.001"),
            DeclareLaunchArgument("rtk_dropout_prob", default_value="0.0"),
            DeclareLaunchArgument("rtk_latency_s", default_value="0.0"),
            DeclareLaunchArgument("voxel_size", default_value="0.10"),
            DeclareLaunchArgument("bev_resolution", default_value="0.10"),
            # Self-filter box (lidar_0 frame) FLIPPED for the front+lower re-mount (2026-06-13):
            # the LiDAR moved from base (-0.15,0,0.415) to (0.40,0,0.25), so the chassis body is now
            # BEHIND + below the sensor (negative x in lidar frame). Old box removed x[0.2,0.8]
            # (forward, when the lidar was at the rear) -- keeping that would now delete the very
            # forward rocks we re-mounted to see. New box removes the rear/under body; forward
            # (x > 0.05) is preserved. VALIDATE against the live cloud and tighten if it clips rocks.
            DeclareLaunchArgument("crop_min_x", default_value="-0.95"),
            DeclareLaunchArgument("crop_max_x", default_value="0.05"),
            DeclareLaunchArgument("crop_min_y", default_value="-0.45"),
            DeclareLaunchArgument("crop_max_y", default_value="0.45"),
            DeclareLaunchArgument("crop_min_z", default_value="-0.30"),
            DeclareLaunchArgument("crop_max_z", default_value="0.25"),
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
                    "robot_namespace": robot_namespace,
                    "config_path": glim_config_path,
                    "crop_min_x": LaunchConfiguration("crop_min_x"),
                    "crop_max_x": LaunchConfiguration("crop_max_x"),
                    "crop_min_y": LaunchConfiguration("crop_min_y"),
                    "crop_max_y": LaunchConfiguration("crop_max_y"),
                    "crop_min_z": LaunchConfiguration("crop_min_z"),
                    "crop_max_z": LaunchConfiguration("crop_max_z"),
                }.items(),
            ),
            # Synthetic RTK / surface-beacon absolute-position source. Publishes a
            # measurement topic (/rtk/pose) ONLY -- never a TF (honours the
            # OMNILRS_GT_TF_ENABLED=false discipline). Consumed by GLIM's
            # abs_constraint extension module to bound degenerate-terrain jumps.
            # Absolute topics (/gt/base_link_pose, /rtk/pose) -> not namespaced.
            Node(
                package="synthetic_rtk",
                executable="synthetic_rtk",
                name="synthetic_rtk",
                output="screen",
                condition=IfCondition(start_synthetic_rtk),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "gt_topic": "/gt/base_link_pose",
                        "output_topic": "/rtk/pose",
                        "world_frame": "map",
                        "home_xy": [20.0, 18.0],
                        "antenna_offset_xyz": [0.264, 0.017, 0.262],
                        "sigma0": ParameterValue(LaunchConfiguration("rtk_sigma0"), value_type=float),
                        "sigma_per_m": ParameterValue(
                            LaunchConfiguration("rtk_sigma_per_m"), value_type=float
                        ),
                        "dropout_prob": ParameterValue(
                            LaunchConfiguration("rtk_dropout_prob"), value_type=float
                        ),
                        "latency_s": ParameterValue(LaunchConfiguration("rtk_latency_s"), value_type=float),
                    }
                ],
            ),
            GroupAction(
                [
                    PushRosNamespace(robot_namespace),
                    Node(
                        package="slam_mapping",
                        executable="probabilistic_voxel_mapper_node",
                        name="probabilistic_voxel_mapper",
                        output="screen",
                        condition=IfCondition(start_bev_map),
                        parameters=[
                            PathJoinSubstitution([FindPackageShare("astrobot_launch"), "config", "mapper_surface.yaml"]),
                            {
                                "use_sim_time": use_sim_time,
                                "voxel_size": ParameterValue(LaunchConfiguration("voxel_size"), value_type=float),
                                "bev_resolution": ParameterValue(LaunchConfiguration("bev_resolution"), value_type=float),
                            },
                        ],
                    ),
                ]
            ),
        ]
    )
