from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wifi_logger_visualizer',
            executable='heat_mapper_node.py',  # Remove the .py extension
            name='heat_mapper_node',
            output='screen'
        )
    ])
