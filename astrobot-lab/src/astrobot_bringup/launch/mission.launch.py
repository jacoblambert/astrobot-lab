# Copyright 2026 Jacob Lambert
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Astrobot mission: localization, terrain, Nav2, command mux and the mission manager.

  ros2 launch astrobot_bringup mission.launch.py mission:=explore_radius

Requires the simulator nav stream (docker-compose.yml, OMNILRS_GT_TF_ENABLED=false). SLAM,
terrain and Nav2 start after slam_start_delay (the sim publishes clock and sensors first); the
mission manager starts after mission_start_delay. Mission parameters come from
astrobot_exploration/config/<mission>.yaml, or any file passed as mission_params.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

NAV2_SERVERS = [
    'planner_server', 'controller_server', 'velocity_smoother', 'bt_navigator', 'behavior_server']


def share(*path):
    return PathJoinSubstitution([FindPackageShare('astrobot_bringup'), *path])


def as_float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    ns = LaunchConfiguration('robot_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_time = {'use_sim_time': use_sim_time}
    nav2_params = RewrittenYaml(
        source_file=LaunchConfiguration('nav2_params'), root_key=ns, convert_types=True,
        param_rewrites={
            'default_nav_to_pose_bt_xml':
                share('behavior_trees', 'navigate_to_pose_lunar_recovery.xml'),
            'default_nav_through_poses_bt_xml':
                share('behavior_trees', 'navigate_through_poses.xml'),
        })
    mux_params = RewrittenYaml(
        source_file=share('config', 'command_mux.yaml'), root_key=ns, convert_types=True,
        param_rewrites={})

    localization_and_navigation = GroupAction([
        PushRosNamespace(ns),
        # Owns the SLAM (GLIM) child process; motion permission for everything below.
        Node(
            package='astrobot_localization', executable='localization_supervisor',
            name='localization_supervisor', output='screen',
            parameters=[{
                **sim_time, 'config_path': LaunchConfiguration('glim_config'),
                'map_topic': 'terrain/local_costmap'}]),
        Node(
            package='astrobot_localization', executable='map_to_odom_tf',
            name='slam_map_to_odom_tf', output='screen',
            parameters=[{
                **sim_time, 'slam_pose_topic': 'slam/pose_corrected', 'odom_topic': 'odom',
                'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_link',
                'base_offset_x': 0.264, 'base_offset_y': 0.017,
                'base_yaw_offset': 3.141592653589793, 'publish_rate_hz': 30.0}]),
        Node(
            package='astrobot_terrain', executable='terrain', name='terrain', output='screen',
            parameters=[
                LaunchConfiguration('robot_config'),
                {**sim_time, 'mosaic_half_extent_m': as_float('mosaic_half_extent_m')}]),
        Node(
            package='astrobot_control', executable='command_mux', name='command_mux',
            output='screen', parameters=[mux_params, {**sim_time, 'require_localization': True}]),
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen', parameters=[nav2_params]),
        # Controller -> velocity smoother -> command mux; recoveries go through the same chain.
        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen', parameters=[nav2_params], remappings=[('cmd_vel', 'cmd_vel_raw')]),
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', output='screen', parameters=[nav2_params],
            remappings=[('cmd_vel', 'cmd_vel_raw'), ('cmd_vel_smoothed', 'cmd_vel_nav')]),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            output='screen', parameters=[nav2_params]),
        Node(
            package='nav2_behaviors', executable='behavior_server', name='behavior_server',
            output='screen', parameters=[nav2_params], remappings=[('cmd_vel', 'cmd_vel_raw')]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                **sim_time, 'autostart': True, 'bond_timeout': 4.0,
                'node_names': NAV2_SERVERS}]),
    ])
    # Simulation only: noisy RTK fixes from ground truth (map = world - home).
    synthetic_rtk = Node(
        package='astrobot_sim', executable='synthetic_rtk', name='synthetic_rtk', output='screen',
        parameters=[
            share('config', 'sim.yaml'),
            {**sim_time, 'dropout_prob': as_float('rtk_dropout_prob'),
             'occlusion_period_s': as_float('rtk_occlusion_period_s'),
             'occlusion_duration_s': as_float('rtk_occlusion_duration_s')}])
    mission = GroupAction([
        PushRosNamespace(ns),
        Node(
            package='astrobot_sim', executable='gt_diagnostics', name='diagnostics',
            output='screen', condition=IfCondition(LaunchConfiguration('start_diagnostics')),
            parameters=[{
                **sim_time, 'publish_topic': 'diagnostics', 'period_sec': 2.0,
                'path_corridor_radius_m': 0.35, 'goal_radius_m': 0.75,
                'progress_timeout_sec': 35.0}]),
        Node(
            package='astrobot_exploration', executable='exploration_manager',
            name='exploration_manager', output='screen',
            condition=IfCondition(LaunchConfiguration('start_mission')),
            parameters=[
                LaunchConfiguration('mission_params'),
                {**sim_time, 'return_behavior_tree':
                    share('behavior_trees', 'navigate_to_pose_lunar_recovery_precise.xml')}]),
    ])
    mission_params = PathJoinSubstitution([
        FindPackageShare('astrobot_exploration'), 'config',
        [LaunchConfiguration('mission'), '.yaml']])
    return LaunchDescription([
        DeclareLaunchArgument(
            'mission', default_value='explore_radius',
            description='explore_radius | sample_return | teleop_click'),
        DeclareLaunchArgument('mission_params', default_value=mission_params),
        DeclareLaunchArgument('robot_namespace', default_value='astrobot_0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_config', default_value=share('config', 'robot.yaml')),
        DeclareLaunchArgument('nav2_params', default_value=share('config', 'nav2.yaml')),
        DeclareLaunchArgument(
            'glim_config',
            default_value=share('config', 'glim'),
            description='GLIM configuration directory (selected LiDAR, lunar IMU, RTK)'),
        DeclareLaunchArgument('slam_start_delay', default_value='15.0'),
        DeclareLaunchArgument('mission_start_delay', default_value='35.0'),
        DeclareLaunchArgument('start_diagnostics', default_value='true'),
        DeclareLaunchArgument('start_mission', default_value='true'),
        DeclareLaunchArgument('mosaic_half_extent_m', default_value='30.0'),
        # Fault injection (sim): RTK lost while (sim time % period) < duration; 0 disables.
        DeclareLaunchArgument('rtk_dropout_prob', default_value='0.0'),
        DeclareLaunchArgument('rtk_occlusion_period_s', default_value='0.0'),
        DeclareLaunchArgument('rtk_occlusion_duration_s', default_value='0.0'),
        TimerAction(
            period=LaunchConfiguration('slam_start_delay'),
            actions=[localization_and_navigation, synthetic_rtk]),
        TimerAction(period=LaunchConfiguration('mission_start_delay'), actions=[mission]),
    ])
