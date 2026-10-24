import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('wifi_logger_visualizer'),
        'config',
        'wifi_logger_config.yaml',
    )

    def build_node(context):
        overrides = {
            'standalone':
                LaunchConfiguration('standalone').perform(context).lower()
                == 'true'
        }
        db_path = LaunchConfiguration('db_path').perform(context)
        if db_path:
            overrides['db_path'] = db_path
        return [Node(
            package='wifi_logger_visualizer',
            executable='heat_mapper_node.py',
            parameters=[LaunchConfiguration('params_file'), overrides],
            output='screen',
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to a ROS 2 parameters YAML file',
        ),
        DeclareLaunchArgument(
            'standalone',
            default_value='false',
            description='Display matplotlib instead of publishing RViz markers',
        ),
        DeclareLaunchArgument(
            'db_path',
            default_value='',
            description='Optional database path override',
        ),
        OpaqueFunction(function=build_node),
    ])
