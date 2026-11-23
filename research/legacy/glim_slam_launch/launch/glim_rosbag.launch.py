from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bag_path = LaunchConfiguration("bag_path")
    config_path = LaunchConfiguration("config_path")

    default_config_path = PathJoinSubstitution(
        [FindPackageShare("glim_slam_launch"), "config", "glim_astrobot"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("bag_path"),
            DeclareLaunchArgument("config_path", default_value=default_config_path),
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    [
                        "exec ros2 run glim_ros glim_rosbag ",
                        bag_path,
                        " --ros-args -p config_path:=",
                        config_path,
                    ],
                ],
                output="screen",
            ),
        ]
    )
