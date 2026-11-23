from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    robot_namespace = LaunchConfiguration("robot_namespace")
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="astrobot_0"),
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8080"),
            # legacy: BEV mapper layers. v2: v2 terrain mosaic/exploration/local window, supervisor
            # and terrain status (pair with a slam_stack:=v2 mission, tools/v2_demo.sh).
            DeclareLaunchArgument("stack", default_value="legacy"),
            DeclareLaunchArgument("obstacle_map_topic", default_value="", description="override the stack's obstacle layer"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gt_scene_path", default_value="",
                                  description="sim evaluator natural_scene.json: GT rock footprints overlay"),
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
                parameters=[{
                    "stack": LaunchConfiguration("stack"),
                    "obstacle_map_topic": LaunchConfiguration("obstacle_map_topic"),
                    "gt_scene_path": LaunchConfiguration("gt_scene_path"),
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                }],
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
