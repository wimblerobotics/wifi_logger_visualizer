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
    
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='1.0',
        description='Frequency at which to publish costmaps (Hz)'
    )
    
    db_check_frequency_arg = DeclareLaunchArgument(
        'db_check_frequency',
        default_value='2.0',
        description='Frequency at which to check for database updates (Hz)'
    )
    
    max_interpolation_distance_arg = DeclareLaunchArgument(
        'max_interpolation_distance',
        default_value='1.0',
        description='Maximum distance for interpolation in meters'
    )
    
    enable_link_quality_arg = DeclareLaunchArgument(
        'enable_link_quality',
        default_value='true',
        description='Enable link quality costmap'
    )
    
    enable_signal_level_arg = DeclareLaunchArgument(
        'enable_signal_level',
        default_value='true',
        description='Enable signal level costmap'
    )
    
    enable_bit_rate_arg = DeclareLaunchArgument(
        'enable_bit_rate',
        default_value='true',
        description='Enable bit rate costmap'
    )
    
    costmap_topic_arg = DeclareLaunchArgument(
        'costmap_topic',
        default_value='/global_costmap/costmap',
        description='Topic to read costmap dimensions from'
    )
    
    # Create the visualization node
    wifi_visualizer_node = Node(
        package='wifi_logger_visualizer',
        executable='wifi_visualizer_node.py',
        name='wifi_visualizer',
        output='screen',
        parameters=[{
            'db_path': LaunchConfiguration('db_path'),
            'publish_frequency': LaunchConfiguration('publish_frequency'),
            'db_check_frequency': LaunchConfiguration('db_check_frequency'),
            'max_interpolation_distance': LaunchConfiguration('max_interpolation_distance'),
            'enable_link_quality': LaunchConfiguration('enable_link_quality'),
            'enable_signal_level': LaunchConfiguration('enable_signal_level'),
            'enable_bit_rate': LaunchConfiguration('enable_bit_rate'),
            'costmap_topic': LaunchConfiguration('costmap_topic')
        }]
    )
    
    return LaunchDescription([
        db_path_arg,
        publish_frequency_arg,
        db_check_frequency_arg,
        max_interpolation_distance_arg,
        enable_link_quality_arg,
        enable_signal_level_arg,
        enable_bit_rate_arg,
        costmap_topic_arg,
        wifi_visualizer_node
    ]) 