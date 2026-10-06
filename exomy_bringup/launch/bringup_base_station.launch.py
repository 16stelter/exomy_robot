import os, yaml
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from tempfile import NamedTemporaryFile

def launch_setup(context, *args, **kwargs):
  params_file = os.path.join(get_package_share_directory('exomy_bringup'), 'params', 'exomy_bringup_params.yaml')
  namespace = LaunchConfiguration('namespace').perform(context)
  use_udp_bridge = LaunchConfiguration('udp_bridge').perform(context).lower()
  use_openvins = LaunchConfiguration('use_openvins').perform(context).lower()
  configured_params = prepend_namespace_to_yaml(params_file, namespace)
  robot_desc_dir = get_package_share_directory('exomy_description')

  udp_bridge_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(get_package_share_directory('udp_bridge'), 'launch', 'udp_bridge.launch.py')),
          launch_arguments={
              'namespace': namespace,
              'config_file': os.path.join(get_package_share_directory('udp_bridge'), 'config', 'base.yaml')
          }.items(),
          condition=IfCondition(use_udp_bridge),
  )

  robot_desc_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(robot_desc_dir, 'launch', 'robot.launch.py')),
        launch_arguments={'namespace': namespace}.items(),
        condition=IfCondition(use_udp_bridge), # we only need to create a robot description if we are using the UDP bridge, otherwise we directly get it from the robot
  )

  openvins_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(get_package_share_directory('ov_msckf'), 'launch', 'subscribe.launch.py')),
          launch_arguments={
              'namespace': namespace,
              'max_cameras': '1',
              'rviz_enable': 'false',
              'config': 'exomy'
          }.items(),
          condition=IfCondition(use_openvins),
    )

  return [robot_desc_cmd, udp_bridge_cmd, openvins_cmd]

def generate_launch_description():
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_udp_bridge_cmd = DeclareLaunchArgument(
        'udp_bridge', default_value='true', description='Whether to launch the UDP bridge node'
    )

    declare_use_openvins_cmd = DeclareLaunchArgument(
        'use_openvins', default_value='true', description='Whether to launch the OpenVINS node'
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_udp_bridge_cmd,
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
