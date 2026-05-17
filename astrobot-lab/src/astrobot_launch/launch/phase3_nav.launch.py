from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    start_nav2 = LaunchConfiguration("start_nav2")
    start_slam_tf = LaunchConfiguration("start_slam_tf")
    slam_start_delay = LaunchConfiguration("slam_start_delay")
    glim_config_path = LaunchConfiguration("glim_config_path")
    nav2_params = LaunchConfiguration("nav2_params")
    basic_control_params = LaunchConfiguration("basic_control_params")
    slam_base_offset_x = LaunchConfiguration("slam_base_offset_x")
    slam_base_offset_y = LaunchConfiguration("slam_base_offset_y")
    slam_base_yaw_offset = LaunchConfiguration("slam_base_yaw_offset")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_slam_tf", default_value="true"),
            DeclareLaunchArgument("slam_start_delay", default_value="15.0"),
            DeclareLaunchArgument(
                "glim_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("glim_slam_launch"), "config", "glim_astrobot"]
                ),
            ),
            DeclareLaunchArgument("slam_base_offset_x", default_value="0.264"),
            DeclareLaunchArgument("slam_base_offset_y", default_value="0.017"),
            DeclareLaunchArgument("slam_base_yaw_offset", default_value="3.141592653589793"),
            DeclareLaunchArgument("crop_min_x", default_value="0.20"),
            DeclareLaunchArgument("crop_max_x", default_value="0.80"),
            DeclareLaunchArgument("crop_min_y", default_value="-0.45"),
            DeclareLaunchArgument("crop_max_y", default_value="0.45"),
            DeclareLaunchArgument("crop_min_z", default_value="-0.35"),
            DeclareLaunchArgument("crop_max_z", default_value="0.05"),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("astrobot_launch"), "config", "nav2_phase3_bev.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "basic_control_params",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("astrobot_launch"), "config", "basic_control_phase1.yaml"]
                ),
            ),
            LogInfo(
                msg=(
                    "Phase 3 SLAM-frame navigation requires simulator GT map->odom TF disabled "
                    "(run compose with OMNILRS_GT_TF_ENABLED=false). GT topics may still be recorded for scoring."
                )
            ),
            TimerAction(
                period=slam_start_delay,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                PathJoinSubstitution(
                                    [FindPackageShare("astrobot_launch"), "launch", "phase2_slam.launch.py"]
                                )
                            ]
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "glim_config_path": glim_config_path,
                            "voxel_size": "0.10",
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
                        executable="slam_map_to_odom_tf_node",
                        name="slam_map_to_odom_tf",
                        output="screen",
                        condition=IfCondition(start_slam_tf),
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "slam_pose_topic": "/glim_rosnode/pose_corrected",
                                "odom_topic": "/odom",
                                "map_frame": "glim_map",
                                "odom_frame": "odom",
                                "base_frame": "base_link",
                                "base_offset_x": ParameterValue(slam_base_offset_x, value_type=float),
                                "base_offset_y": ParameterValue(slam_base_offset_y, value_type=float),
                                "base_yaw_offset": ParameterValue(slam_base_yaw_offset, value_type=float),
                                "publish_rate_hz": 30.0,
                            }
                        ],
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                PathJoinSubstitution(
                                    [FindPackageShare("astrobot_launch"), "launch", "phase1_nav.launch.py"]
                                )
                            ]
                        ),
                        condition=IfCondition(start_nav2),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "autostart": autostart,
                            "nav2_params": nav2_params,
                            "basic_control_params": basic_control_params,
                        }.items(),
                    ),
                ],
            ),
        ]
    )
