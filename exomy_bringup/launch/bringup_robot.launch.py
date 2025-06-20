import os, yaml
from tempfile import NamedTemporaryFile
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    params_file = os.path.join(get_package_share_directory('exomy_bringup'), 'params', 'exomy_bringup_params.yaml')
    namespace = LaunchConfiguration('namespace').perform(context)
    configured_params = prepend_namespace_to_yaml(params_file, namespace)
    robot_lowlevel_dir = get_package_share_directory('exomy')

    robot_lowlevel_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_lowlevel_dir, 'exomy.launch.py')),
            launch_arguments={'namespace': namespace}.items())

    arducam_node_cmd = Node(
        package = 'arducam_rclpy_tof_pointcloud',
        executable = 'tof_pointcloud',
        name = 'tof_pointcloud',
        namespace = namespace,
        output = 'screen'
    )

    laserscan_node_cmd = Node(
        package = 'pointcloud_to_laserscan',
        executable = 'pointcloud_to_laserscan_node',
        name = 'pointcloud_to_laserscan',
        namespace = namespace,
        output = 'screen',
        remappings=[
            ('cloud_in', 'point_cloud'),
            ('scan', 'scan'),
        ],
        parameters=[configured_params]
    )

    return [robot_lowlevel_cmd, arducam_node_cmd, laserscan_node_cmd]

def generate_launch_description():
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    return LaunchDescription([
        declare_namespace_cmd,
        OpaqueFunction(function=launch_setup)
    ])

def prepend_namespace_to_yaml(input_file, namespace):
    with open(input_file, 'r') as f:
        data = yaml.safe_load(f)

    namespaced_data = {}
    for node_name, node_config in data.items():
        if namespace:
            namespaced_key = f'/{namespace}/{node_name}'
        else:
            namespaced_key = f'/{node_name}'
        namespaced_data[namespaced_key] = node_config

    tmp_file = NamedTemporaryFile(delete=False, mode='w', suffix='.yaml')
    yaml.dump(namespaced_data, tmp_file)
    tmp_file.close()

    return tmp_file.name
