import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    package_dir = get_package_share_directory('exomy_bringup')
    robot_desc_dir = get_package_share_directory('exomy_description')
    robot_lowlevel_dir = get_package_share_directory('exomy')
    params_file = LaunchConfiguration('params_file')

    declare_bringup_params_cmd = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(
            package_dir,
            'params',
            'exomy_bringup_params.yaml')
    )


    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    robot_desc_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_desc_dir, 'launch', 'robot.launch.py')),
            launch_arguments={'namespace': namespace}.items())

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
        parameters=[LaunchConfiguration('params_file')]
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_bringup_params_cmd)
    ld.add_action(robot_desc_cmd)
    ld.add_action(robot_lowlevel_cmd)
    ld.add_action(arducam_node_cmd)
    ld.add_action(laserscan_node_cmd)
    return ld
