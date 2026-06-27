"""color_detect 启动：双摄检测 + 串口桥接（不含摄像头节点）。

摄像头已分离到 camera_stream 包，单独启动。

用法:
  ros2 launch color_detect color_detect_all.launch.py serial_port:=/dev/ttyACM0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('color_detect'),
        'config',
        'color_detect_config.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('debug',       default_value='false'),

        Node(
            package='color_detect',
            executable='color_detect_node',
            name='color_detect_node',
            output='screen',
            parameters=[config_file, {
                'debug': LaunchConfiguration('debug'),
            }],
        ),

        Node(
            package='color_detect',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            output='screen',
            parameters=[{
                'port':     LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('serial_baud'),
            }],
        ),
    ])
