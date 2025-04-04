from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Declare launch arguments
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value=os.path.join(os.getcwd(), 'wifi_data.db'),
        description='Path to the SQLite database file'
    )
    
    standalone_arg = DeclareLaunchArgument(
        'standalone',
        default_value='false',
        description='Whether to publish a costmap (true) or display a matplotlib image (false)'
    )
    
    costmap_topic_arg = DeclareLaunchArgument(
        'costmap_topic',
        default_value='/global_costmap/costmap',
        description='Topic to read costmap dimensions from'
    )
    
    scale_factor_arg = DeclareLaunchArgument(
        'scale_factor',
        default_value='1.0',
        description='Scale factor for the heatmap (larger values create finer grids)'
    )
    
    # Create the heat mapper node
    heat_mapper_node = Node(
        package='wifi_logger_visualizer',
        executable='heat_mapper_node.py',
        name='heat_mapper',
        output='screen',
        parameters=[{
            'db_path': LaunchConfiguration('db_path'),
            'standalone': LaunchConfiguration('standalone'),
            'costmap_topic': LaunchConfiguration('costmap_topic'),
            'scale_factor': LaunchConfiguration('scale_factor')
        }]
    )
    
    return LaunchDescription([
        db_path_arg,
        standalone_arg,
        costmap_topic_arg,
        scale_factor_arg,
        heat_mapper_node
    ]) 