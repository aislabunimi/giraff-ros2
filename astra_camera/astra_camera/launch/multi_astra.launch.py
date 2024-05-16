import sys
from launch import LaunchDescription
import launch_ros.actions
from ament_index_python import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import copy
from os import path
import yaml
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.frontend.parse_substitution import parse_substitution
from launch.substitutions import LaunchConfiguration




def generate_container_node(camera_name, params):
    return ComposableNodeContainer(
        name='astra_camera_container',
        namespace=camera_name,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::OBCameraNodeFactory',
                           name='camera',
                           parameters=[params],
                           namespace=camera_name),
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::PointCloudXyzNode',
                           namespace=camera_name,
                           name='point_cloud_xyz'),
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::PointCloudXyzrgbNode',
                           namespace=camera_name,
                           name='point_cloud_xyzrgb',)
        ],
        output='screen')


def duplicate_params(general_params, posix, serial_number):
    local_params = copy.deepcopy(general_params)
    local_params["camera_name"] += f'_{posix}'
    local_params["serial_number"] = str(serial_number)
    local_params['device_num'] = 2
    return local_params


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    serial_number_up = LaunchConfiguration('serial_number_up').perform(context)
    serial_number_down = LaunchConfiguration('serial_number_down').perform(context)

    params_file = get_package_share_directory("astra_camera") + "/params/astra_params.yaml"
    if not path.exists(params_file):
        print("path %s is not exists" % params_file)
        sys.exit(-1)
    with open(params_file, 'r') as file:
        default_params = yaml.safe_load(file)

    # leave serial numbers empty to autoselect
    params1 = duplicate_params(default_params, "up", serial_number_up)
    params2 = duplicate_params(default_params, "down", serial_number_down)
    container1 = generate_container_node("camera_up", params1)
    container2 = generate_container_node("camera_down", params2)
    # dummy static transformation from camera1 to camera2
    dummy_tf_node = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "camera_up_link",
            "camera_down_link",
        ],
    )

    return [GroupAction([

        container1, container2, dummy_tf_node
    ])]



def generate_launch_description():
    serial_number_down_parameter = DeclareLaunchArgument('serial_number_down', default_value='18072430160')
    serial_number_up_parameter = DeclareLaunchArgument('serial_number_up', default_value='18072330021')

    return LaunchDescription([
        serial_number_up_parameter,
        serial_number_down_parameter,
        DeclareLaunchArgument('namespace', default_value='giraff'),
        OpaqueFunction(function = launch_setup)
    ])
