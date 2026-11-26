"""Launch the BMS node.

  ros2 launch bms_ros bms.launch.py port:=/dev/ttyUSB0 protocol:=jbd
  ros2 launch bms_ros bms.launch.py namespace:=pack2 port:=/dev/ttyUSB1 protocol:=daly
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('bms_ros'), 'config', 'bms.yaml')
    args = [
        DeclareLaunchArgument('namespace', default_value='bms', description='Topic namespace'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='9600'),
        DeclareLaunchArgument('protocol', default_value='jbd', description='jbd or daly'),
        DeclareLaunchArgument('rate', default_value='1.0', description='Poll rate in Hz'),
    ]
    node = Node(
        package='bms_ros',
        executable='bms_node',
        name='bms',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[cfg, {
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'protocol': LaunchConfiguration('protocol'),
            'rate': LaunchConfiguration('rate'),
        }],
    )
    return LaunchDescription(args + [node])
