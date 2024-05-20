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

    namespace = LaunchConfiguration('namespace').perform(context)


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d' +  os.path.join(get_package_share_directory('missions_pkg'), 'rviz', 'giraff.rviz')],
        remappings=[
            ("/initialpose", f"/{namespace}/initialpose"),
            ("/goal_pose", f"/{namespace}/goal_pose")
        ]
    )

    return [GroupAction([
        #PushRosNamespace(namespace),
        rviz
    ])]

def generate_launch_description():
    namespace_argument = DeclareLaunchArgument(
        'namespace',
        default_value=NAMESPACE)

    # Add actions
    return LaunchDescription([
        namespace_argument,
        OpaqueFunction(function=launch_setup)
    ])
