import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'wifi_logger_visualizer'
    package_dir = get_package_share_directory(package_name)

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
        default_value='amdc',
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

    # Overlay (ov_) parameter declarations
    ov_horizontal_alignment_arg = DeclareLaunchArgument(
        'ov_horizontal_alignment', default_value='0', description='Overlay horizontal alignment (LEFT:0 RIGHT:1 CENTER:2)')
    ov_vertical_alignment_arg = DeclareLaunchArgument(
        'ov_vertical_alignment', default_value='3', description='Overlay vertical alignment (CENTER:2 TOP:3 Bottom:4)')
    ov_horizontal_distance_arg = DeclareLaunchArgument(
        'ov_horizontal_distance', default_value='10', description='Overlay horizontal distance')
    ov_vertical_distance_arg = DeclareLaunchArgument(
        'ov_vertical_distance', default_value='10', description='Overlay vertical distance')
    ov_width_factor_arg = DeclareLaunchArgument(
        'ov_width_factor', default_value='1.0', description='Overlay width factor')
    ov_height_factor_arg = DeclareLaunchArgument(
        'ov_height_factor', default_value='1.0', description='Overlay height factor')
    ov_font_arg = DeclareLaunchArgument(
        'ov_font', default_value='DejaVu Sans Mono', description='Overlay font')
    ov_font_color_arg = DeclareLaunchArgument(
        'ov_font_color', default_value='0.1 0.1 0.1 1.0', description='Overlay font color (RGBA)')
    ov_font_size_arg = DeclareLaunchArgument(
        'ov_font_size', default_value='12.0', description='Overlay font size')
    ov_bg_color_arg = DeclareLaunchArgument(
        'ov_bg_color', default_value='1.0 1.0 1.0 0.5', description='Overlay background color (RGBA)')
    ov_do_short_arg = DeclareLaunchArgument(
        'ov_do_short', default_value='true', description='Overlay short text enabled')
    ov_do_full_arg = DeclareLaunchArgument(
        'ov_do_full', default_value='true', description='Overlay full text enabled')

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
        ov_horizontal_alignment_arg,
        ov_vertical_alignment_arg,
        ov_horizontal_distance_arg,
        ov_vertical_distance_arg,
        ov_width_factor_arg,
        ov_height_factor_arg,
        ov_font_arg,
        ov_font_color_arg,
        ov_font_size_arg,
        ov_bg_color_arg,
        ov_do_short_arg,
        ov_do_full_arg,
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
                'ov_horizontal_alignment': LaunchConfiguration('ov_horizontal_alignment'),
                'ov_vertical_alignment': LaunchConfiguration('ov_vertical_alignment'),
                'ov_horizontal_distance': LaunchConfiguration('ov_horizontal_distance'),
                'ov_vertical_distance': LaunchConfiguration('ov_vertical_distance'),
                'ov_width_factor': LaunchConfiguration('ov_width_factor'),
                'ov_height_factor': LaunchConfiguration('ov_height_factor'),
                'ov_font': LaunchConfiguration('ov_font'),
                'ov_font_color': LaunchConfiguration('ov_font_color'),
                'ov_font_size': LaunchConfiguration('ov_font_size'),
                'ov_bg_color': LaunchConfiguration('ov_bg_color'),
                'ov_do_short': LaunchConfiguration('ov_do_short'),
                'ov_do_full': LaunchConfiguration('ov_do_full'),
            }],
            output='screen'
        )
    ])