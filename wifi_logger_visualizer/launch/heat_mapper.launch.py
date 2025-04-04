from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare the standalone argument
    standalone_arg = DeclareLaunchArgument(
        'standalone',
        default_value='false',
        description='Whether to publish a costmap (true) or display matplotlib visualization (false)'
    )

    # Get the current working directory for the default db_path
    current_dir = os.getcwd()
    default_db_path = os.path.join(current_dir, 'wifi_data.db')

    # Declare the db_path argument
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value=default_db_path,
        description='Path to the SQLite database file'
    )

    # Declare the scale_factor argument
    scale_factor_arg = DeclareLaunchArgument(
        'scale_factor',
        default_value='1.0',
        description='Scale factor for the heatmap visualization'
    )

    # Create the heat mapper node
    heat_mapper_node = Node(
        package='wifi_logger_visualizer',
        executable='heat_mapper_node',
        name='heat_mapper',
        parameters=[{
            'standalone': LaunchConfiguration('standalone'),
            'db_path': LaunchConfiguration('db_path'),
            'scale_factor': LaunchConfiguration('scale_factor'),
        }],
        output='screen'
    )

    return LaunchDescription([
        standalone_arg,
        db_path_arg,
        scale_factor_arg,
        heat_mapper_node
    ]) 