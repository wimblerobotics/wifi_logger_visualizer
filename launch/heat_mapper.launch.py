from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare the standalone argument
    standalone_arg = DeclareLaunchArgument(
        'standalone',
        default_value='false',
        description='Whether to publish a costmap (true) or display matplotlib visualization (false)'
    )

    package_name = 'wifi_logger_visualizer'
    package_dir = get_package_share_directory(package_name)

    # Declare launch arguments for command-line overrides
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value=os.path.join(package_dir, 'wifi_data.db'),
        description='Path to the SQLite database file.'
    )


    # Declare the scale_factor argument
    scale_factor_arg = DeclareLaunchArgument(
        'scale_factor',
        default_value='1.0',
        description='Scale factor for the heatmap visualization'
    )
    
    # Declare the text_size argument - match the default in the node (0.25)
    text_size_arg = DeclareLaunchArgument(
        'text_size',
        default_value='0.08',
        description='Size of the text markers in meters'
    )

    # Declare the heatmap_field argument
    heatmap_field_arg = DeclareLaunchArgument(
        'heatmap_field',
        default_value='iperf3_sender_bitrate',
        description='Field to visualize as a heatmap (e.g., signal_level, bit_rate, link_quality, iperf3_sender_bitrate, iperf3_receiver_bitrate)'
    )

    # Declare the costmap_topic argument
    costmap_topic_arg = DeclareLaunchArgument(
        'costmap_topic',
        default_value='/global_costmap/costmap',
        description='Topic to subscribe to for costmap dimensions'
    )

    # Declare the do_publish_markers argument
    do_publish_markers_arg = DeclareLaunchArgument(
        'do_publish_markers',
        default_value='true',
        description='Whether to publish value markers'
    )

    # Declare the do_publish_text_markers argument
    do_publish_text_markers_arg = DeclareLaunchArgument(
        'do_publish_text_markers',
        default_value='true',
        description='Whether to publish text markers'
    )

    # Declare the aggregation_type argument
    aggregation_type_arg = DeclareLaunchArgument(
        'aggregation_type',
        default_value='average',
        description='Aggregation type for the heatmap (min, max, average)'
    )

    # Create the heat mapper node with explicit parameter passing
    heat_mapper_node = Node(
        package='wifi_logger_visualizer',
        executable='heat_mapper_node.py',
        name='heat_mapper_node',
        parameters=[{
            'standalone': LaunchConfiguration('standalone'),
            'db_path': LaunchConfiguration('db_path'),
            'scale_factor': LaunchConfiguration('scale_factor'),
            'text_size': LaunchConfiguration('text_size'),
            'heatmap_field': LaunchConfiguration('heatmap_field'),
            'costmap_topic': LaunchConfiguration('costmap_topic'),
            'do_publish_markers': LaunchConfiguration('do_publish_markers'),
            'do_publish_text_markers': LaunchConfiguration('do_publish_text_markers'),
            'aggregation_type': LaunchConfiguration('aggregation_type'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        standalone_arg,
        db_path_arg,
        scale_factor_arg,
        text_size_arg,
        heatmap_field_arg,
        costmap_topic_arg,
        do_publish_markers_arg,
        do_publish_text_markers_arg,
        aggregation_type_arg,
        heat_mapper_node,
    ])