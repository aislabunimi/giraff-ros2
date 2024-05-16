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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import xacro

def launch_setup(context, *args, **kwargs):
    # Get the launch directory
    namespace = LaunchConfiguration('namespace').perform(context)

     # common variables

    params_yaml_file = ParameterFile(os.path.join(get_package_share_directory('missions_pkg'), 'params', 'giraff_params.yaml'), allow_substs=True)
    

    giraff_driver = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('giraff_ros2_driver'), 'launch', 'giraff_launch.py')
            )
        )
    ]
    odometry = [
        Node(package='rf2o_laser_odometry',
             executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry',
             output='screen',
             parameters=[{
                 'laser_scan_topic' : '/giraff/laser_scan',
                 'odom_topic' : '/giraff/odom',
                 'publish_tf' : True,
                 'base_frame_id' : 'giraff_base_footprint',
                 'odom_frame_id' : 'giraff_odom',
                 'init_pose_from_topic' : '',
                 'freq' : 50.0}],
             ),
    ]

    hokuyo_node = [
        Node(
        package='urg_node',
        executable='urg_node_driver',
        name='hokuyo_front',
        #prefix='xterm -hold -e',
        output='screen',
        parameters=[params_yaml_file]
        ),  
    ]

    astra_cameras = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('astra_camera'), 'launch', 'multi_astra.launch.py')
            ),
            launch_arguments={
                'usb_port_down': '1-1.3',
                'usb_port_up': '2-1.6',  # replace your usb port here
            }.items()
        )
    ]

    #robot description for state_pùblisher
    robot_desc = xacro.process_file(os.path.join(get_package_share_directory('missions_pkg'), 'params', 'giraff.xacro'), mappings={'frame_ns': namespace})
    robot_desc = robot_desc.toprettyxml(indent='  ')

    robot_state_publisher = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {
                    'use_sim_time': True, 
                    'robot_description': robot_desc
                }
            ],
        )
    ]



    actions=[PushRosNamespace(namespace)]
    actions.extend(giraff_driver)
    #actions.extend(robot_state_publisher)
    actions.extend(hokuyo_node)
    actions.extend(astra_cameras)
    actions.extend(odometry)
    #actions.extend(start_async_slam_toolbox_node)
    #actions.extend(keyboard_control)
    return[
        GroupAction
        (
            actions=actions
        ),
    ]

def generate_launch_description():

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
            ),
        DeclareLaunchArgument('namespace', default_value="giraff"),
        OpaqueFunction(function = launch_setup)
    ])
