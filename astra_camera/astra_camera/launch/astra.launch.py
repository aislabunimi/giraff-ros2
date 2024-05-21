from launch import LaunchDescription, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
import yaml



def launch_setup(context, *args, **kwargs):

    namespace = LaunchConfiguration('namespace').perform(context)
    serial_number = LaunchConfiguration('serial_number').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)

    params_file = get_package_share_directory(
        "astra_camera") + "/params/astra_params.yaml"
    with open(params_file, 'r') as file:
        config_params = yaml.safe_load(file)

    config_params['serial_number'] = serial_number
    config_params['camera_name'] = camera_name

    container = ComposableNodeContainer(
        name='astra_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        #prefix='xterm -e gdb --args',
        composable_node_descriptions=[
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::OBCameraNodeFactory',
                           name='camera',
                           namespace=namespace,
                           parameters=[config_params]),
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::PointCloudXyzNode',
                           namespace='camera',
                           name='point_cloud_xyz',
                           parameters=[config_params]),
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::PointCloudXyzrgbNode',
                           namespace=namespace,
                           name='point_cloud_xyzrgb',
                           parameters=[config_params])
        ],
        output='screen')

    return [GroupAction([
        PushRosNamespace(namespace),
        container,
    ])]


def generate_launch_description():
    camera_name = DeclareLaunchArgument('camera_name', default_value='astra_camera')
    serial_number = DeclareLaunchArgument('serial_number', default_value='18072430160')
    return LaunchDescription([camera_name,
                              serial_number,
                              DeclareLaunchArgument('namespace', default_value='giraff'),
                              OpaqueFunction(function = launch_setup)])
