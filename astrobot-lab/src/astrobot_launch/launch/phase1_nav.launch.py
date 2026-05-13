from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    nav2_params = LaunchConfiguration("nav2_params")
    basic_control_params = LaunchConfiguration("basic_control_params")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
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
            Node(
                package="basic_control",
                executable="command_mux",
                name="command_mux",
                output="screen",
                parameters=[basic_control_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_params],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_params],
                remappings=[("/cmd_vel", "/cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[nav2_params],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[nav2_params],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "bond_timeout": 4.0,
                        "node_names": [
                            "planner_server",
                            "controller_server",
                            "bt_navigator",
                            "behavior_server",
                        ],
                    }
                ],
            ),
        ]
    )
