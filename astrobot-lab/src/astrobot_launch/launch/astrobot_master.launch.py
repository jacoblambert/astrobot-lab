from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params = LaunchConfiguration("nav2_params")
    basic_control_params = LaunchConfiguration("basic_control_params")
    robot_namespace = LaunchConfiguration("robot_namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("robot_namespace", default_value="astrobot_0"),
            DeclareLaunchArgument("crop_min_x", default_value="0.20"),
            DeclareLaunchArgument("crop_max_x", default_value="0.80"),
            DeclareLaunchArgument("crop_min_y", default_value="-0.45"),
            DeclareLaunchArgument("crop_max_y", default_value="0.45"),
            DeclareLaunchArgument("crop_min_z", default_value="-0.35"),
            DeclareLaunchArgument("crop_max_z", default_value="0.05"),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("astrobot_launch"), "config", "nav2_phase1.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "basic_control_params",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("astrobot_launch"), "config", "basic_control_phase1.yaml"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [PathJoinSubstitution([FindPackageShare("astrobot_launch"), "launch", "phase1_nav.launch.py"])]
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "robot_namespace": robot_namespace,
                    "nav2_params": nav2_params,
                    "basic_control_params": basic_control_params,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [PathJoinSubstitution([FindPackageShare("astrobot_launch"), "launch", "phase2_slam.launch.py"])]
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "robot_namespace": robot_namespace,
                    "crop_min_x": LaunchConfiguration("crop_min_x"),
                    "crop_max_x": LaunchConfiguration("crop_max_x"),
                    "crop_min_y": LaunchConfiguration("crop_min_y"),
                    "crop_max_y": LaunchConfiguration("crop_max_y"),
                    "crop_min_z": LaunchConfiguration("crop_min_z"),
                    "crop_max_z": LaunchConfiguration("crop_max_z"),
                }.items(),
            ),
        ]
    )
