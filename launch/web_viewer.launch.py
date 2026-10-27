"""
Launch the standalone Wi-Fi database web viewer.

Usage:
    ros2 launch wifi_logger_visualizer web_viewer.launch.py
    ros2 launch wifi_logger_visualizer web_viewer.launch.py port:=8089
    ros2 launch wifi_logger_visualizer web_viewer.launch.py db:=/tmp/my_wifi.db
"""
import os

import math

from ament_index_python.packages import get_package_share_directory
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import ExecutableInPackage


def _launch_viewer(context):
    config = os.path.expanduser(LaunchConfiguration("config_filepath").perform(context))
    with open(config) as stream:
        settings = yaml.safe_load(stream) or {}
    common = settings.get("/**", {}).get("ros__parameters", {})
    viewer = settings.get("/**/wifi_web_viewer", {}).get("ros__parameters", {})

    def value(argument, default):
        override = LaunchConfiguration(argument).perform(context)
        return override if override else default

    db = value("db", common.get("db_path", "~/.ros/titania_wifi.db"))
    host = value("host", viewer.get("host", "0.0.0.0"))
    port = str(value("port", viewer.get("port", 8088)))
    if not port.isdecimal() or not 1 <= int(port) <= 65535:
        raise ValueError("port must be an integer between 1 and 65535")
    cell_size = float(value("cell_size", common.get("cell_size", 0.25)))
    if not math.isfinite(cell_size) or cell_size <= 0:
        raise ValueError("cell_size must be positive and finite")

    command = [
        ExecutableInPackage(package="wifi_logger_visualizer", executable="web_viewer.py"),
        "--db", os.path.expanduser(str(db)),
        "--host", str(host),
        "--port", port,
        "--cell-size", str(cell_size),
    ]

    node_names = value("node_names", viewer.get("node_names", ""))
    if node_names:
        command.extend(["--node-names", os.path.expanduser(str(node_names))])
    # The viewer uses argparse rather than ROS parameters.
    return [ExecuteProcess(
        cmd=command, output="screen", additional_env={"PYTHONUNBUFFERED": "1"},
    )]


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("wifi_logger_visualizer"),
        "config", "wifi_logger_config.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("config_filepath", default_value=config,
                              description="Wi-Fi YAML config; explicit launch arguments override it"),
        DeclareLaunchArgument("db", default_value="",
                              description="Path to the V2 Wi-Fi SQLite database"),
        DeclareLaunchArgument("host", default_value="",
                              description="HTTP bind address"),
        DeclareLaunchArgument("port", default_value="",
                              description="HTTP listening port (1..65535)"),
        DeclareLaunchArgument("cell_size", default_value="",
                              description="Wi-Fi cell size in meters"),
        DeclareLaunchArgument("node_names", default_value="",
                              description="Optional access-point names YAML file"),
        OpaqueFunction(function=_launch_viewer),
    ])
