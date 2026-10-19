from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_path = LaunchConfiguration("config_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_namespace = LaunchConfiguration("robot_namespace")

    default_config_path = PathJoinSubstitution(
        [FindPackageShare("glim_slam_launch"), "config", "glim_astrobot"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("robot_namespace", default_value="astrobot_0"),
            DeclareLaunchArgument("config_path", default_value=default_config_path),
            DeclareLaunchArgument("shadow_mode", default_value="false"),
            DeclareLaunchArgument("shadow_seed_path", default_value=""),
            # Self-filter box FLIPPED for the front+lower LiDAR re-mount (2026-06-13): LiDAR moved
            # to base (0.40,0,0.25) so the chassis is now BEHIND/below it (negative x in lidar frame).
            DeclareLaunchArgument("crop_min_x", default_value="-0.95"),
            DeclareLaunchArgument("crop_max_x", default_value="0.05"),
            DeclareLaunchArgument("crop_min_y", default_value="-0.45"),
            DeclareLaunchArgument("crop_max_y", default_value="0.45"),
            DeclareLaunchArgument("crop_min_z", default_value="-0.30"),
            DeclareLaunchArgument("crop_max_z", default_value="0.25"),
            GroupAction(
                [
                    PushRosNamespace(robot_namespace),
                    Node(
                        package="pointcloud_preprocessor",
                        executable="pointcloud_crop_filter_node",
                        name="pointcloud_crop_filter",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "input_topic": "lidar_0/pointcloud/raw",
                                "output_topic": "lidar_0/pointcloud/filtered",
                                "min_x": ParameterValue(LaunchConfiguration("crop_min_x"), value_type=float),
                                "max_x": ParameterValue(LaunchConfiguration("crop_max_x"), value_type=float),
                                "min_y": ParameterValue(LaunchConfiguration("crop_min_y"), value_type=float),
                                "max_y": ParameterValue(LaunchConfiguration("crop_max_y"), value_type=float),
                                "min_z": ParameterValue(LaunchConfiguration("crop_min_z"), value_type=float),
                                "max_z": ParameterValue(LaunchConfiguration("crop_max_z"), value_type=float),
                            }
                        ],
                    ),
                    Node(
                        package="localization_supervisor",
                        executable="localization_supervisor",
                        name="localization_supervisor",
                        output="screen",
                        parameters=[{"use_sim_time": use_sim_time, "config_path": config_path,
                                     "shadow_seed_path": LaunchConfiguration("shadow_seed_path"),
                                     "shadow_mode": ParameterValue(LaunchConfiguration("shadow_mode"), value_type=bool)}],
                    ),
                ]
            ),
        ]
    )
