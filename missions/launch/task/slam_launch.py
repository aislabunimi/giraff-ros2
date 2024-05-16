# LAUNCH this file to run a SLAM node for generating an occupancy gridmap from laser scans.

# USE THIS COMMAND TO SAVE TO FILE
# ros2 run nav2_map_server map_saver_cli -f map_name

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

NAMESPACE = 'giraff_yellow'


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    slam_params_file = LaunchConfiguration('slam_params_file').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    print('XXXXXXX', namespace)



    start_async_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )



    start_lifecicle_mamanager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},

                    ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d' +  os.path.join(get_package_share_directory('missions_pkg'), 'rviz', 'giraff.rviz')],
        #remappings=[
            #("/initialpose", f"/{namespace}/initialpose"),
            #("/goal_pose", f"/{namespace}/goal_pose")
        #]
    )

    return [GroupAction([
        #PushRosNamespace(namespace),
        start_async_slam_toolbox_node,
        start_lifecicle_mamanager,
        rviz
    ])]

def generate_launch_description():
    namespace_argument = DeclareLaunchArgument(
        'namespace',
        default_value=NAMESPACE)
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("missions_pkg"),
                                   'params', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    # Add actions
    return LaunchDescription([
        namespace_argument,
        declare_use_sim_time_argument,
        declare_slam_params_file_cmd,
        OpaqueFunction(function=launch_setup)
    ])
