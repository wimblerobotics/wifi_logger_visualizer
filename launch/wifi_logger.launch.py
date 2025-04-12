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
    
    ov_do_short_arg = DeclareLaunchArgument(
        'ov_do_short',
        default_value='True',
        description='Whether to display WiFi summary in RViz2 overlay'
    )
    
    ov_do_full_arg = DeclareLaunchArgument(
        'ov_do_full',
        default_value='True',
        description='Whether to display full WiFi info in RViz2 overlay'
    )
    
    decimals_to_round_coordinates_arg = DeclareLaunchArgument(
        'decimals_to_round_coordinates',
        default_value='3',
        description='Number of decimal places to round x and y coordinates'
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
            'cleanup_days': LaunchConfiguration('cleanup_days'),
            'decimals_to_round_coordinates': LaunchConfiguration('decimals_to_round_coordinates'),
            # Text overlay parameters:
            'ov_horizontal_alignment': 0, # LEFT:0 RIGHT:1 CENTER:2
            'ov_vertical_alignment': 3, # CENTER:2 TOP:3 Bottom:4
            'ov_horizontal_distance': 10,
            'ov_vertical_distance': 10,
            'ov_width_factor': 1.0,  # adjust overlay canvas width
            'ov_height_factor': 1.0,  # adjust overlay canvas height
            'ov_font': 'DejaVu Sans Mono',
            'ov_font_size': 12.0,
            'ov_font_color': "0.8 0.8 0.3 0.8", # RGBA
            'ov_bg_color': "0.0 0.0 0.0 0.05",
            'ov_do_short': LaunchConfiguration('ov_do_short'),
            'ov_do_full': LaunchConfiguration('ov_do_full')
        }]
    )
    
    return LaunchDescription([
        db_path_arg,
        wifi_interface_arg,
        update_interval_arg,
        max_signal_strength_arg,
        min_signal_strength_arg,
        cleanup_days_arg,
        ov_do_short_arg,
        ov_do_full_arg,
        decimals_to_round_coordinates_arg,
        wifi_logger_node
    ])