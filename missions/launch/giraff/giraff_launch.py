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


def launch_setup(context, *args, **kwargs):

    namespace = LaunchConfiguration('namespace').perform(context)
    params_yaml_file = ParameterFile(os.path.join(get_package_share_directory('missions_pkg'), 'params', 'giraff_params.yaml'), allow_substs=True)


    giraff_driver = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('giraff_ros2_driver'), 'launch', 'giraff_launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'publish_odom': 'False',
                'publish_other_tf': 'False'
            }.items()

        )
    ]
    odometry = [
        Node(package='rf2o_laser_odometry',
             executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry',
             output='screen',
             parameters=[{
                 'laser_scan_topic' : f'/{namespace}/laser_scan',
                 'odom_topic' : f'/{namespace}/odom',
                 'publish_tf' : True,
                 'base_frame_id' : f'{namespace}_base_footprint',
                 'odom_frame_id' : f'{namespace}_odom',
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

    astra_camera_up = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('astra_camera'), 'launch', 'astra.launch.py')
            ),
            launch_arguments={
                'serial_number': camera_up_serial_number,
                'camera_name': 'camera_up',
                'device_num': '2',
                'namespace': namespace
            }.items()
        ),
        Node(
            package='topic_tools',
            executable='throttle',
            arguments=['messages', f'/{namespace}/camera_up/depth/points', '15.0'],
        ),
        ]
    astra_camera_down = [TimerAction(
        period=2.0,
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('astra_camera'), 'launch', 'astra.launch.py')
                ),
                launch_arguments={
                    'serial_number': camera_down_serial_number,
                    'camera_name': 'camera_down',
                    'device_num': '2',
                    'namespace': namespace,
                }.items()
            ),
            Node(
                package='topic_tools',
                executable='throttle',
                arguments=['messages', f'/{namespace}/camera_down/depth/points', '1.0' ],
            )
        ]
    )]

    astra_cameras_tf = [TimerAction(period=2.0,
                                    actions=[

                                        Node(
                                            package="tf2_ros",
                                            executable="static_transform_publisher",
                                            arguments=camera_down_tf + [f"{namespace}_base_link", f"{namespace}_camera_down_link"],
                                        ),
                                        Node(
                                            package="tf2_ros",
                                            executable="static_transform_publisher",
                                            arguments=camera_up_tf + [f"{namespace}_base_link", f"{namespace}_camera_up_link"],
                                        )
                                    ])]

    #robot description for state_pùblisher
    robot_desc = xacro.process_file(os.path.join(get_package_share_directory('missions_pkg'), 'params', 'giraff.xacro'), mappings={'frame_ns': namespace})
    robot_desc = robot_desc.toprettyxml(indent='  ')

    robot_state_publisher = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {
                    'use_sim_time': False,
                    'robot_description': robot_desc,
                }
            ],
        )
    ]

    nav2 = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('missions_pkg'), 'launch', 'giraff', 'nav2_launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
            }.items()

        )
    ]

    point_cloud_to_laser_scan = [TimerAction(
        period=3.0,
        actions=[PushRosNamespace(namespace),
                 Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', [f'/{namespace}/camera_up', '/depth', '/points_throttle']),
                        ('scan', [f'/{namespace}/laser_scan_camera_up'])],
            parameters=[{
                'target_frame': f'{namespace}_laser_link',
                'transform_tolerance': 0.01,
                'min_height': 0.05,
                'max_height': 2.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.33,
                'range_min': 0.45,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan')
        ]
    )]
    actions=[PushRosNamespace(namespace)]
    actions.extend(giraff_driver)
    actions.extend(robot_state_publisher)
    actions.extend(hokuyo_node)
    actions.extend(astra_camera_up)
    actions.extend(astra_camera_down)
    actions.extend(odometry)
    #actions.extend(nav2)
    actions.extend(astra_cameras_tf)
    actions.extend(point_cloud_to_laser_scan)
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
        DeclareLaunchArgument('namespace', default_value=NAMESPACE),
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],  #debug, info
            description="Logging level",
        ),

        OpaqueFunction(function = launch_setup)
    ])