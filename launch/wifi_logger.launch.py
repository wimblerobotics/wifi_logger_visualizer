import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'wifi_logger_visualizer'
    package_dir = get_package_share_directory(package_name)
    config_filepath = os.path.join(package_dir, 'config', 'wifi_logger_config.yaml')

    # Declare launch arguments for command-line overrides
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value=os.path.join(package_dir, 'wifi_data.db'),
        description='Path to the SQLite database file.'
    )

    wifi_interface_arg = DeclareLaunchArgument(
        'wifi_interface',
        default_value='',
        description='Name of the WiFi interface to monitor.'
    )

    update_interval_arg = DeclareLaunchArgument(
        'update_interval',
        default_value='1.0',
        description='Interval for periodic updates in seconds.'
    )

    # Declare additional launch arguments for all parameters in wifi_logger_node.py
    min_signal_level_arg = DeclareLaunchArgument(
        'min_signal_level',
        default_value='-100.0',
        description='Minimum expected signal level in dBm.'
    )

    max_signal_level_arg = DeclareLaunchArgument(
        'max_signal_level',
        default_value='-10.0',
        description='Maximum expected signal level in dBm.'
    )

    decimals_to_round_coordinates_arg = DeclareLaunchArgument(
        'decimals_to_round_coordinates',
        default_value='3',
        description='Number of decimal places to round coordinates.'
    )

    publish_metrics_arg = DeclareLaunchArgument(
        'publish_metrics',
        default_value='true',
        description='Whether to publish WiFi metrics.'
    )

    publish_overlay_arg = DeclareLaunchArgument(
        'publish_overlay',
        default_value='true',
        description='Whether to publish WiFi overlay messages.'
    )

    iperf3_host_arg = DeclareLaunchArgument(
        'iperf3_host',
        default_value='',
        description='Host for iperf3 testing.'
    )

    iperf3_interval_arg = DeclareLaunchArgument(
        'iperf3_interval',
        default_value='1.0',
        description='Interval for iperf3 testing in seconds.'
    )

    do_iperf3_arg = DeclareLaunchArgument(
        'do_iperf3',
        default_value='true',
        description='Whether to enable iperf3 testing.'
    )

    return LaunchDescription([
        db_path_arg,
        wifi_interface_arg,
        update_interval_arg,
        min_signal_level_arg,
        max_signal_level_arg,
        decimals_to_round_coordinates_arg,
        publish_metrics_arg,
        publish_overlay_arg,
        iperf3_host_arg,
        iperf3_interval_arg,
        do_iperf3_arg,
        Node(
            package=package_name,
            executable='wifi_logger_node.py',
            name='wifi_logger',
            parameters=[{
                'db_path': LaunchConfiguration('db_path'),
                'wifi_interface': LaunchConfiguration('wifi_interface'),
                'update_interval': LaunchConfiguration('update_interval'),
                'min_signal_level': LaunchConfiguration('min_signal_level'),
                'max_signal_level': LaunchConfiguration('max_signal_level'),
                'decimals_to_round_coordinates': LaunchConfiguration('decimals_to_round_coordinates'),
                'publish_metrics': LaunchConfiguration('publish_metrics'),
                'publish_overlay': LaunchConfiguration('publish_overlay'),
                'iperf3_host': LaunchConfiguration('iperf3_host'),
                'iperf3_interval': LaunchConfiguration('iperf3_interval'),
                'do_iperf3': LaunchConfiguration('do_iperf3'),
            }],
            output='screen'
        )
    ])