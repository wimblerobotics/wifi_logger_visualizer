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
    
    # Declare the text_size argument - match the default in the node (0.25)
    text_size_arg = DeclareLaunchArgument(
        'text_size',
        default_value='0.08',
        description='Size of the text markers in meters'
    )

    # Create the heat mapper node with explicit parameter passing
    heat_mapper_node = Node(
        package='wifi_logger_visualizer',
        executable='heat_mapper_node.py',
        name='heat_mapper',
        parameters=[{
            'standalone': LaunchConfiguration('standalone'),
            'db_path': LaunchConfiguration('db_path'),
            'scale_factor': LaunchConfiguration('scale_factor'),
            'text_size': LaunchConfiguration('text_size'),
        }],
        output='screen'
    )
    
    # Create a command to set the text_size parameter after the node starts
    # This is a workaround for Jazzy parameter handling
    set_text_size_cmd = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'param',
            'set',
            '/heat_mapper',
            'text_size',
            LaunchConfiguration('text_size')
        ],
        output='screen'
    )

    return LaunchDescription([
        standalone_arg,
        db_path_arg,
        scale_factor_arg,
        text_size_arg,
        heat_mapper_node,
        set_text_size_cmd
    ]) 