from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
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
            # Supervisor map-readiness input: the terrain node's local planner window.
            DeclareLaunchArgument("map_topic", default_value="/terrain_shadow/planner_costmap"),
            GroupAction(
                [
                    PushRosNamespace(robot_namespace),
                    Node(
                        package="astrobot_localization",
                        executable="localization_supervisor",
                        name="localization_supervisor",
                        output="screen",
                        parameters=[{"use_sim_time": use_sim_time, "config_path": config_path,
                                     "map_topic": LaunchConfiguration("map_topic")}],
                    ),
                ]
            ),
        ]
    )
