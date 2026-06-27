"""color_detect 双摄像头视觉信标启动。

用法:
  # 双摄像头（默认话题 /cam_left/image_raw, /cam_right/image_raw）
  ros2 launch color_detect color_detect.launch.py

  # 指定话题
  ros2 launch color_detect color_detect.launch.py \
    camera_topic_left:=/usb_cam_1/image_raw \
    camera_topic_right:=/usb_cam_2/image_raw

  # 启用调试
  ros2 launch color_detect color_detect.launch.py debug:=true
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
        DeclareLaunchArgument('camera_topic_left',
                              default_value='/cam_left/image_raw'),
        DeclareLaunchArgument('camera_topic_right',
                              default_value='/cam_right/image_raw'),
        DeclareLaunchArgument('debug', default_value='false'),

        Node(
            package='color_detect',
            executable='color_detect_node',
            name='color_detect_node',
            output='screen',
            parameters=[config_file, {
                'camera_topic_left':  LaunchConfiguration('camera_topic_left'),
                'camera_topic_right': LaunchConfiguration('camera_topic_right'),
                'debug':              LaunchConfiguration('debug'),
            }],
            arguments=['--ros-args', '--log-level', 'info']
        ),

        # 如果 debug=true，可用 image_view 看调试画面：
        # ros2 run image_view image_view /color_detect/debug_left
        # ros2 run image_view image_view /color_detect/debug_right
    ])
