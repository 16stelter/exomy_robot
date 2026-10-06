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
  namespace = LaunchConfiguration('namespace').perform(context)
  use_udp_bridge = LaunchConfiguration('udp_bridge').perform(context).lower()
  use_openvins = LaunchConfiguration('use_openvins').perform(context).lower()
  use_rviz = LaunchConfiguration('rviz').perform(context).lower()
  robot_desc_dir = get_package_share_directory('exomy_description')

  udp_bridge_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(get_package_share_directory('udp_bridge'), 'launch', 'udp_bridge.launch.py')),
          launch_arguments={
              'namespace': namespace,
              'config_file': 'base.yaml',
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
    'rviz_enable': use_rviz,
    'config_path': os.path.join(get_package_share_directory('exomy_description'), 'config', 'open_vins', 'estimator_config.yaml'),
    }.items(),
    condition=IfCondition(use_openvins),
  )

  image_transport_cmd = Node(
    package='image_transport',
    executable='republish',
    name='image_republisher',
    namespace=namespace,
    parameters=[{'in_transport': 'compressed', 'out_transport': 'raw'}],
    remappings=[
      ('in/compressed', '/exomy/camera/color/image_raw/compressed'),
      ('out', '/exomy/camera/color/image_raw')
    ],
    condition=IfCondition(use_udp_bridge),
  )

  return [robot_desc_cmd, udp_bridge_cmd, openvins_cmd, image_transport_cmd]

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

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false', description='Whether to launch RViz'
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_udp_bridge_cmd,
        declare_use_openvins_cmd,
        declare_rviz_cmd,
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
