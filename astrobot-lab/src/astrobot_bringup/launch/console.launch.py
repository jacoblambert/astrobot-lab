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
Ops console backend (FastAPI + WebSocket on host:port).

  ros2 launch astrobot_bringup console.launch.py [gt_scene_path:=<run>/natural_scene.json]

Then run the frontend from ui/astrobot-console (npm run dev).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ns = LaunchConfiguration('robot_namespace')
    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='astrobot_0'),
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'gt_scene_path', default_value='',
            description='sim scene inventory (natural_scene.json): GT rock overlay'),
        PushRosNamespace(ns),
        Node(
            package='astrobot_ui_backend', executable='ui_backend', name='ui_backend',
            output='screen',
            parameters=[{
                'gt_scene_path': LaunchConfiguration('gt_scene_path'),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)}],
            arguments=[
                '--host', LaunchConfiguration('host'), '--port', LaunchConfiguration('port'),
                '--robot-namespace', ns]),
    ])
