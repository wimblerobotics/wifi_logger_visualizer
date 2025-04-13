from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file_path = os.path.join(os.getcwd(), 'config', 'wifi_logger_config.yaml')

    wifi_logger_node = Node(
        package='wifi_logger_visualizer',
        executable='wifi_logger_node.py',
        name='wifi_logger',
        output='screen',
        parameters=[config_file_path]
    )

    return LaunchDescription([
        wifi_logger_node
    ])