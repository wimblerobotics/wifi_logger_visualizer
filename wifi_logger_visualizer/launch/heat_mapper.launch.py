from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Declare launch arguments
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value=os.path.join(os.getcwd(), 'wifi_data.db'),
        description='Path to the SQLite database file'
    )
    
    publish_costmap_arg = DeclareLaunchArgument(
        'publish_costmap',
        default_value='true',
        description='Whether to publish a costmap or display a matplotlib image'
    )
    
    costmap_topic_arg = DeclareLaunchArgument(
        'costmap_topic',
        default_value='/global_costmap/costmap',
        description='Topic to subscribe to for costmap dimensions'
    )
    
    scale_factor_arg = DeclareLaunchArgument(
        'scale_factor',
        default_value='1.0',
        description='Scale factor for the heatmap values'
    )
    
    text_size_arg = DeclareLaunchArgument(
        'text_size',
        default_value='0.25',
        description='Size of the text markers in meters'
    )
    
    do_publish_markers_arg = DeclareLaunchArgument(
        'do_publish_markers',
        default_value='true',
        description='Whether to publish value markers'
    )
    
    do_publish_text_markers_arg = DeclareLaunchArgument(
        'do_publish_text_markers',
        default_value='true',
        description='Whether to publish text markers'
    )
    
    heatmap_field_arg = DeclareLaunchArgument(
        'heatmap_field',
        default_value='signal_level',
        description='Field to visualize in the heatmap (e.g., signal_level, bit_rate)'
    )

    aggregation_type_arg = DeclareLaunchArgument(
        'aggregation_type',
        default_value='average',
        description='Aggregation type for the heatmap (min, max, average)'
    )

    # Create the heat mapper node
    heat_mapper_node = Node(
        package='wifi_logger_visualizer',
        executable='heat_mapper_node',
        name='heat_mapper_node',
        parameters=[{
            'db_path': LaunchConfiguration('db_path'),
            'publish_costmap': LaunchConfiguration('publish_costmap'),
            'costmap_topic': LaunchConfiguration('costmap_topic'),
            'scale_factor': LaunchConfiguration('scale_factor'),
            'text_size': LaunchConfiguration('text_size'),
            'do_publish_markers': LaunchConfiguration('do_publish_markers'),
            'do_publish_text_markers': LaunchConfiguration('do_publish_text_markers'),
            'heatmap_field': LaunchConfiguration('heatmap_field'),
            'aggregation_type': LaunchConfiguration('aggregation_type')
        }]
    )
    
    return LaunchDescription([
        db_path_arg,
        publish_costmap_arg,
        costmap_topic_arg,
        scale_factor_arg,
        text_size_arg,
        do_publish_markers_arg,
        do_publish_text_markers_arg,
        heatmap_field_arg,
        aggregation_type_arg,
        heat_mapper_node
    ])