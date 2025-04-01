from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # Declare launch arguments
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value=os.path.join(os.getcwd(), 'wifi_data.db'),
        description='Path to the SQLite database file'
    )
    
    wifi_interface_arg = DeclareLaunchArgument(
        'wifi_interface',
        default_value='',
        description='WiFi interface name (empty for auto-detect)'
    )
    
    update_interval_arg = DeclareLaunchArgument(
        'update_interval',
        default_value='1.0',
        description='Update interval in seconds'
    )
    
    max_signal_strength_arg = DeclareLaunchArgument(
        'max_signal_strength',
        default_value='-30.0',
        description='Maximum expected signal strength in dBm'
    )
    
    min_signal_strength_arg = DeclareLaunchArgument(
        'min_signal_strength',
        default_value='-90.0',
        description='Minimum expected signal strength in dBm'
    )
    
    cleanup_days_arg = DeclareLaunchArgument(
        'cleanup_days',
        default_value='30',
        description='Number of days to keep data before cleanup'
    )
    
    # Create the WiFi logger node
    wifi_logger_node = Node(
        package='wifi_logger_visualizer',
        executable='wifi_logger_node.py',
        name='wifi_logger',
        output='screen',
        parameters=[{
            'db_path': LaunchConfiguration('db_path'),
            'wifi_interface': LaunchConfiguration('wifi_interface'),
            'update_interval': LaunchConfiguration('update_interval'),
            'max_signal_strength': LaunchConfiguration('max_signal_strength'),
            'min_signal_strength': LaunchConfiguration('min_signal_strength'),
            'cleanup_days': LaunchConfiguration('cleanup_days')
        }]
    )
    
    return LaunchDescription([
        db_path_arg,
        wifi_interface_arg,
        update_interval_arg,
        max_signal_strength_arg,
        min_signal_strength_arg,
        cleanup_days_arg,
        wifi_logger_node
    ]) 