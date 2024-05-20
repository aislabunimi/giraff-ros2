from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.frontend.parse_substitution import parse_substitution

namespace = ''

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    publish_odom = LaunchConfiguration('publish_odom').perform(context)
    publish_other_tf = LaunchConfiguration('publish_other_tf').perform(context)

    print('XXXXXXXXX', publish_other_tf, type(publish_other_tf))
    giraff = Node (
        package='giraff_ros2_driver',
        executable='giraff_node',
        name='giraff_node',
        output='screen',
        parameters=[
            {'giraff_avr_port':'/dev/ttyS1'},
            {'publish_odometry_over_tf': publish_odom if type(publish_odom == bool) else (True if str(publish_odom) == 'True' else False)},
            {'publish_other_tf': publish_other_tf if type(publish_other_tf == bool) else (True if str(publish_other_tf) == 'True' else False)},
            {'odom_topic':'odom'},
            {'freq': 100.0},
            {'verbose':False},

            {'base_frame_id': f'{namespace}_base_link'},
            {'odom_frame_id': f'{namespace}_odom'},
            {'stalk_frame_id': f'{namespace}_neck'},
            {'head_frame_id': f'{namespace}_head'},
            {'screen_frame_id': f'{namespace}_screen'},
            {'base_footprint_frame_id': f'{namespace}_base_footprint'},
            {'camera_frame_id': f'{namespace}_camera'},
            {'laser_frame_id': f'{namespace}_laser_frame'},

            {'controller_mode':0},
            {'max_linear_vel':0.7},
            {'max_angular_vel':0.7},
            {'linear_acceleration':0.2},
            {'angular_acceleration':0.4},
            {'virtual_gear_ratio':20.0},
            {'tilt_bias':0.6},
            {'battery_technology':'NIMH'},
            {'battery_design_capacity':4.4},
            {'battery_serial_number':'Giraff_battery'}
        ]
    )

    return [GroupAction([giraff])]

def generate_launch_description():


    return LaunchDescription([
    DeclareLaunchArgument('publish_odom', default_value='False'),
    DeclareLaunchArgument('publish_other_tf', default_value='False'),
        DeclareLaunchArgument('namespace', default_value='giraff'),
        OpaqueFunction(function=launch_setup)
    ])