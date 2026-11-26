"""Launch the BMS node.

  ros2 launch bms_ros bms.launch.py                          # port/protocol from .env, else auto-detect
  ros2 launch bms_ros bms.launch.py port:=/dev/ttyUSB0 protocol:=daly
  ros2 launch bms_ros bms.launch.py namespace:=pack2 env_file:=/home/me/pack2.env
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
        DeclareLaunchArgument('port', default_value='auto', description="Serial port, or 'auto' (.env, then detect)"),
        DeclareLaunchArgument('protocol', default_value='auto', description="daly, jbd or 'auto'"),
        DeclareLaunchArgument('baud', default_value='0', description='0 = .env value or 9600'),
        DeclareLaunchArgument('rate', default_value='0.0', description='Poll rate in Hz, 0 = .env value or 1 Hz'),
        DeclareLaunchArgument('env_file', default_value='', description='Path to a .env file'),
    ]
    node = Node(
        package='bms_ros',
        executable='bms_node',
        name='bms',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[cfg, {
            'port': LaunchConfiguration('port'),
            'protocol': LaunchConfiguration('protocol'),
            'baud': LaunchConfiguration('baud'),
            'rate': LaunchConfiguration('rate'),
            'env_file': LaunchConfiguration('env_file'),
        }],
    )
    return LaunchDescription(args + [node])
