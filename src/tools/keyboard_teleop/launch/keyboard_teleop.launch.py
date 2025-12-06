#!/usr/bin/env python3
# Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Get package share directory
    package_share_directory = get_package_share_directory('keyboard_teleop')
    
    # Get config file path from launch argument or use default
    config_file = LaunchConfiguration('config_file').perform(context)
    if not config_file:
        config_file = os.path.join(
            package_share_directory,
            'config',
            'keyboard_teleop_params.yaml'
        )
    
    # Create keyboard teleop node with parameters from yaml file
    # Note: This node requires direct terminal access for keyboard input
    # When launched via launch file, stdin may not be a terminal
    # It's recommended to run this node directly: ros2 run keyboard_teleop keyboard_teleop_node
    keyboard_teleop_node = Node(
        package='keyboard_teleop',
        executable='keyboard_teleop_node',
        name='keyboard_teleop_node',
        output='screen',
        parameters=[config_file],
        emulate_tty=True,  # Emulate TTY to allow terminal interaction
    )

    return [keyboard_teleop_node]


def generate_launch_description():
    declared_arguments = []

    # Declare launch argument for config file
    declared_arguments.append(
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Path to the YAML configuration file. If not specified, uses default config/keyboard_teleop_params.yaml',
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


