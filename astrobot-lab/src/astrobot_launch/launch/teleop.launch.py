from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_namespace = LaunchConfiguration("robot_namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("robot_namespace", default_value="astrobot_0"),
            GroupAction(
                [
                    PushRosNamespace(robot_namespace),
                    Node(
                        package="teleop_twist_keyboard",
                        executable="teleop_twist_keyboard",
                        name="teleop_twist_keyboard",
                        emulate_tty=True,
                        output="screen",
                        parameters=[{"use_sim_time": use_sim_time}],
                        remappings=[("cmd_vel", "cmd_vel_teleop")],
                    ),
                ]
            ),
        ]
    )
