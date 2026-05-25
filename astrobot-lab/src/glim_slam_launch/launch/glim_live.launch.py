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
            DeclareLaunchArgument("crop_min_x", default_value="0.20"),
            DeclareLaunchArgument("crop_max_x", default_value="0.80"),
            DeclareLaunchArgument("crop_min_y", default_value="-0.45"),
            DeclareLaunchArgument("crop_max_y", default_value="0.45"),
            DeclareLaunchArgument("crop_min_z", default_value="-0.35"),
            DeclareLaunchArgument("crop_max_z", default_value="0.05"),
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
                        package="glim_ros",
                        executable="glim_rosnode",
                        name="slam",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "config_path": config_path,
                            }
                        ],
                        remappings=[
                            ("~/pose", "slam/pose"),
                            ("~/pose_corrected", "slam/pose_corrected"),
                            ("~/odom", "slam/odom"),
                            ("~/odom_corrected", "slam/odom_corrected"),
                            ("~/aligned_points", "slam/aligned_points"),
                            ("~/aligned_points_corrected", "slam/aligned_points_corrected"),
                            ("~/points", "slam/points"),
                            ("~/points_corrected", "slam/points_corrected"),
                            ("~/map", "slam/glim_map"),
                        ],
                    ),
                ]
            ),
        ]
    )
