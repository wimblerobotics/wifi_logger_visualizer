import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'wifi_logger_visualizer'
    package_dir = get_package_share_directory(package_name)
    config_filepath = os.path.join(package_dir, 'config', 'wifi_logger_config.yaml')

    return LaunchDescription([
        Node(
            package=package_name,
            executable='wifi_logger_node.py',
            name='wifi_logger',
            parameters=[config_filepath],
            output='screen'
        )
    ])