"""Launch saved/live collection with hardware or synthetic Wi-Fi inputs.

Example: ros2 launch wifi_logger_visualizer wifi_logger_visualizer.launch.py
map_source:=live wifi_source:=simulated use_sim_time:=true
The source selectors are launch arguments and override no YAML source setting.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from wifi_logger_visualizer.suite_launch import ARGUMENTS, launch_suite


def generate_launch_description():
    config = os.path.join(get_package_share_directory('wifi_logger_visualizer'),
                          'config', 'wifi_logger_config.yaml')
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=config if key == 'config_filepath' else value)
          for key, value in ARGUMENTS.items()],
        OpaqueFunction(function=launch_suite),
    ])
