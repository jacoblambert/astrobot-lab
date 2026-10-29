from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    robot_namespace = LaunchConfiguration("robot_namespace")
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="astrobot_0"),
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8080"),
            # v2 stack: /terrain_shadow/global_costmap
            DeclareLaunchArgument("obstacle_map_topic", default_value="slam/bev_costmap"),
            LogInfo(
                msg=(
                    "Starting Phase 5 UI backend only. Run the frontend from "
                    "ui/astrobot-console with npm run dev."
                )
            ),
            PushRosNamespace(robot_namespace),
            Node(
                package="astrobot_ui_backend",
                executable="ui_backend",
                name="ui_backend",
                output="screen",
                parameters=[{"obstacle_map_topic": LaunchConfiguration("obstacle_map_topic")}],
                arguments=[
                    "--host",
                    host,
                    "--port",
                    port,
                    "--robot-namespace",
                    robot_namespace,
                ],
            ),
        ]
    )
