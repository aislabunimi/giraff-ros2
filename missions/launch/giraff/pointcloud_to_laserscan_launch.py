# Copyright (c) 2018 Intel Corporation
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution
import xacro

# GIRAFF YELLOW
NAMESPACE = 'giraff_yellow'
camera_up_serial_number = '18072330021'
camera_down_serial_number = '18072430160'
camera_down_tf = ["0.06", "0.0", "0.95", "0", "-0.2", "0"]
camera_up_tf = ["0.1", "-0.02", "1.6", "0.0", "0.8", "0"]

def generate_launch_description():

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value=NAMESPACE),
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
        ),
        PushRosNamespace(NAMESPACE),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', ['/giraff_yellow/camera_up', '/depth', '/points']),
                        ('scan', [NAMESPACE, '/laser_scan_camera_up'])],
            parameters=[{
                'target_frame': f'{NAMESPACE}_laser_link',
                'transform_tolerance': 0.01,
                'min_height': 0.05,
                'max_height': 2.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.33,
                'range_min': 0.45,
                'range_max': 4.0,
                'use_inf': False,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan_pc')

    ])