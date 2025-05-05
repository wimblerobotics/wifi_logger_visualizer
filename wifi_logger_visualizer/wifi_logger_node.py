#!/usr/bin/env python3
import sys
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
import xml.etree.ElementTree as ET

# Get the directory of this script
script_dir = Path(__file__).resolve().parent

# Add the parent directory to sys.path if not already there
parent_dir = script_dir.parent
if str(parent_dir) not in sys.path:
    sys.path.insert(0, str(parent_dir))

# Now use absolute imports
from wifi_logger_visualizer.database_manager import DatabaseManager
from wifi_logger_visualizer.wifi_data_fetcher import WiFiDataFetcher

import numpy as np
import rclpy
from rclpy.node import Node
import subprocess
import re
import sqlite3
from typing import Optional, Tuple
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from example_interfaces.msg import Float32MultiArray
from rviz_2d_overlay_msgs.msg import OverlayText
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import do_transform_pose
import pprint
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

MAX_SIGNAL_LEVEL = -20.0
MIN_SIGNAL_LEVEL = -100.0
DEFAULT_DECIMALS_TO_ROUND = 3

class WifiDataCollector(Node):
    """
    A ROS2 node for collecting and logging WiFi data, including bit rate, link quality, and signal level.
    
    This node integrates with GPS and odometry data to associate WiFi metrics with spatial coordinates.
    It also publishes WiFi metrics and overlay messages for visualization in RViz2.

    Features:
    - Collects WiFi data using `iwconfig`.
    - Associates WiFi data with GPS and odometry coordinates.
    - Publishes WiFi metrics and overlay messages.
    - Dynamically updates parameters at runtime.
    - Stores WiFi data in an SQLite database.

    Attributes:
        db_path (str): Path to the SQLite database file.
        wifi_interface (str): Name of the WiFi interface.
        min_signal_level (float): Minimum expected signal level in dBm.
        max_signal_level (float): Maximum expected signal level in dBm.
        update_interval (float): Interval for periodic updates in seconds.
        decimals_to_round_coordinates (int): Number of decimal places to round coordinates.
        do_publish_metrics (bool): Whether to publish WiFi metrics.
        do_publish_overlay (bool): Whether to publish WiFi overlay messages.
    """

    def __init__(self):
        """
        Initialize the WiFi Data Collector node.

        Loads configuration from a YAML file, initializes parameters, sets up publishers and subscribers,
        and starts a timer for periodic updates.
        """
        super().__init__('wifi_logger_node')

        self.current_pose = None  # <-- Add this line

        # Log source file name, version, and compile time
        source_file = os.path.basename(__file__)
        package_xml_path = os.path.join(get_package_share_directory('wifi_logger_visualizer'), 'package.xml')
        try:
            tree = ET.parse(package_xml_path)
            root = tree.getroot()
            version = root.find('version').text
        except Exception as e:
            version = "unknown"
            self.get_logger().warn(f"Could not retrieve version from package.xml: {e}")

        compile_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.get_logger().info(f"Source File: {source_file}, Version: {version}, Compile Time: {compile_time}")

        try:
            # Load configuration from YAML file
            package_name = 'wifi_logger_visualizer'
            package_dir = get_package_share_directory(package_name)
            config_file_path = os.path.join(package_dir, 'config', 'wifi_logger_config.yaml')
            self.get_logger().info(f"Loading configuration from: {config_file_path}")
            
            with open(config_file_path, 'r') as file:
                config = yaml.safe_load(file)

        except FileNotFoundError:
            self.get_logger().error(f"Configuration file not found at {config_file_path}. Using default values.")
            config = {}

        def to_float(val, default):
            try:
                return float(val)
            except (TypeError, ValueError):
                return float(default)

        # Declare parameters and load from configuration file
        self.db_path = self.declare_and_get_param('db_path', config.get('db_path', 'wifi_data.db'))
        self.wifi_interface = self.declare_and_get_param('wifi_interface', config.get('wifi_interface', ''))
        self.min_signal_level = to_float(self.declare_and_get_param('min_signal_level', config.get('min_signal_level', -100.0)), -100.0)
        self.max_signal_level = to_float(self.declare_and_get_param('max_signal_level', config.get('max_signal_level', -10.0)), -10.0)
        self.update_interval = to_float(self.declare_and_get_param('update_interval', config.get('update_interval', 1.0)), 1.0)
        self.decimals_to_round_coordinates = self.declare_and_get_param('decimals_to_round_coordinates', config.get('decimals_to_round_coordinates', 3))
        self.do_publish_metrics = self.declare_and_get_param('publish_metrics', config.get('publish_metrics', True))
        self.do_publish_overlay = self.declare_and_get_param('publish_overlay', config.get('publish_overlay', True))

        # Declare iperf3-related parameters
        self.iperf3_host = self.declare_and_get_param('iperf3_host', config.get('iperf3_host', ''))
        self.iperf3_interval = self.declare_and_get_param('iperf3_interval', config.get('iperf3_interval', 1.0))
        self.do_iperf3 = self.declare_and_get_param('do_iperf3', config.get('do_iperf3', True))

        # Initialize iperf3 results
        self.iperf3_sender_bitrate = None
        self.iperf3_receiver_bitrate = None
        self.iperf3_ip = None
        self.iperf3_running = False  # Flag to prevent overlapping executions

        # If iperf3 is enabled, test and start the timer
        if self.do_iperf3 and self.iperf3_host:
            self.test_iperf3()
            self.create_timer(self.iperf3_interval, self.run_iperf3)

        # Ensure db_path is an absolute path
        if not os.path.isabs(self.db_path):
            self.db_path = os.path.abspath(self.db_path)
        self.get_logger().info(f"Database path resolved to: {self.db_path}")

        # Log all parameters
        self.get_logger().info("Loaded parameters:")
        self.get_logger().info(f"  db_path: {self.db_path}")
        self.get_logger().info(f"  wifi_interface: {self.wifi_interface}")
        self.get_logger().info(f"  min_signal_level: {self.min_signal_level}")
        self.get_logger().info(f"  max_signal_level: {self.max_signal_level}")
        self.get_logger().info(f"  update_interval: {self.update_interval}")
        self.get_logger().info(f"  decimals_to_round_coordinates: {self.decimals_to_round_coordinates}")
        self.get_logger().info(f"  publish_metrics: {self.do_publish_metrics}")
        self.get_logger().info(f"  publish_overlay: {self.do_publish_overlay}")
        self.get_logger().info(f"  iperf3_host: {self.iperf3_host}")
        self.get_logger().info(f"  iperf3_interval: {self.iperf3_interval}")
        self.get_logger().info(f"  do_iperf3: {self.do_iperf3}")

        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # RViz2 overlay parameters
        self.ov_horizontal_alignment = self.declare_and_get_param('ov_horizontal_alignment', config.get('ov_horizontal_alignment', 0))
        self.ov_vertical_alignment = self.declare_and_get_param('ov_vertical_alignment', config.get('ov_vertical_alignment', 3))
        self.ov_horizontal_distance = self.declare_and_get_param('ov_horizontal_distance', config.get('ov_horizontal_distance', 10))
        self.ov_vertical_distance = self.declare_and_get_param('ov_vertical_distance', config.get('ov_vertical_distance', 10))
        self.ov_width_factor = to_float(self.declare_and_get_param('ov_width_factor', config.get('ov_width_factor', 1.0)), 1.0)
        self.ov_height_factor = to_float(self.declare_and_get_param('ov_height_factor', config.get('ov_height_factor', 1.0)), 1.0)
        self.ov_font = self.declare_and_get_param('ov_font', config.get('ov_font', "DejaVu Sans Mono"))
        self.ov_font_color = self.declare_and_get_param('ov_font_color', config.get('ov_font_color', "0.8 0.8 0.3 0.8"))
        self.ov_font_size = to_float(self.declare_and_get_param('ov_font_size', config.get('ov_font_size', 12.0)), 12.0)
        self.ov_bg_color = self.declare_and_get_param('ov_bg_color', config.get('ov_bg_color', "0.0 0.0 0.0 0.05"))
        self.ov_do_short = self.declare_and_get_param('ov_do_short', config.get('ov_do_short', True))
        self.ov_do_full = self.declare_and_get_param('ov_do_full', config.get('ov_do_full', True))

        # Log RViz2 overlay parameters
        self.get_logger().info("RViz2 overlay parameters:")
        self.get_logger().info(f"  ov_horizontal_alignment: {self.ov_horizontal_alignment}")
        self.get_logger().info(f"  ov_vertical_alignment: {self.ov_vertical_alignment}")
        self.get_logger().info(f"  ov_horizontal_distance: {self.ov_horizontal_distance}")
        self.get_logger().info(f"  ov_vertical_distance: {self.ov_vertical_distance}")
        self.get_logger().info(f"  ov_width_factor: {self.ov_width_factor}")
        self.get_logger().info(f"  ov_height_factor: {self.ov_height_factor}")
        self.get_logger().info(f"  ov_font: {self.ov_font}")
        self.get_logger().info(f"  ov_font_size: {self.ov_font_size}")
        self.get_logger().info(f"  ov_font_color: {self.ov_font_color}")
        self.get_logger().info(f"  ov_bg_color: {self.ov_bg_color}")
        self.get_logger().info(f"  ov_do_short: {self.ov_do_short}")
        self.get_logger().info(f"  ov_do_full: {self.ov_do_full}")

        # Initialize helpers
        self.database_manager = DatabaseManager(self.db_path)
        self.wifi_data_fetcher = WiFiDataFetcher(self.wifi_interface, self.min_signal_level, self.max_signal_level)

        # --- Restore publishers and subscribers here ---
        # Odometry subscriber
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # GPS subscriber
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )

        # WiFi metrics publisher
        if self.do_publish_metrics:
            self.metrics_publisher = self.create_publisher(
                Float32MultiArray,
                '/wifi_logger/metrics',
                10
            )

        # Overlay publisher
        if self.do_publish_overlay:
            self.overlay_publisher = self.create_publisher(
                OverlayText,
                '/wifi_logger/overlay',
                10
            )
            
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Track if transform is available
        self.transform_available = False

        # Other initialization...

        # Initialize globals which can be used before being filled:
        self.gps_sample_time = None
        self.latitude = None
        self.longitude = None
        self.gps_status = -2  # STATUS_UNKNOWN
        self.gps_service = 0  # SERVICE_UNKNOWN

        # Initialize WiFi interface
        if not self.wifi_interface:
            self.wifi_interface = self.get_wifi_interface()
            if not self.wifi_interface:
                self.get_logger().error("Could not determine WiFi interface. Exiting.")
                rclpy.shutdown()
                exit()
            self.get_logger().info(f"Using WiFi interface: {self.wifi_interface}")
        else:
            self.get_logger().info(f"Using WiFi interface: {self.wifi_interface}")

        self.wait_for_transform()

        # Create timer for periodic updates
        self.timer = self.create_timer(self.update_interval, self.timer_callback)

    def parameter_callback(self, params):
        """
        Handle dynamic parameter updates.

        Parameters:
            params (list): List of parameters being updated.

        Returns:
            SetParametersResult: Indicates whether the parameter update was successful.
        """
        for param in params:
            if param.name == 'update_interval' and param.type_ == Parameter.Type.DOUBLE:
                self.update_interval = param.value
                self.get_logger().info(f"Updated 'update_interval' to {self.update_interval}")
            elif param.name == 'min_signal_level' and param.type_ == Parameter.Type.DOUBLE:
                self.min_signal_level = param.value
                self.get_logger().info(f"Updated 'min_signal_level' to {self.min_signal_level}")
            elif param.name == 'max_signal_level' and param.type_ == Parameter.Type.DOUBLE:
                self.max_signal_level = param.value
                self.get_logger().info(f"Updated 'max_signal_level' to {self.max_signal_level}")
            elif param.name == 'decimals_to_round_coordinates' and param.type_ == Parameter.Type.INTEGER:
                self.decimals_to_round_coordinates = param.value
                self.get_logger().info(f"Updated 'decimals_to_round_coordinates' to {self.decimals_to_round_coordinates}")
        return SetParametersResult(successful=True)

    def declare_and_get_param(self, name, default_value):
        """
        Declare a parameter and retrieve its value.

        Parameters:
            name (str): Name of the parameter.
            default_value: Default value of the parameter.

        Returns:
            The value of the parameter.
        """
        self.declare_parameter(name, default_value)
        return self.get_parameter(name).value

    def get_wifi_interface(self) -> Optional[str]:
        """
        Detect the WiFi interface using multiple methods.

        Returns:
            str: The name of the WiFi interface, or None if not found.
        """
        try:
            # Method 1: Check iwconfig
            output = subprocess.check_output(["iwconfig"]).decode("utf-8")
            interfaces = re.findall(r"^(?P<interface>wlp\d+s\d*|wlan\d+)", output, re.MULTILINE)
            if interfaces:
                self.get_logger().info(f"Found WiFi interface: {interfaces[0]}")
                return interfaces[0]

            # Method 2: Check /sys/class/net
            net_interfaces = os.listdir('/sys/class/net')
            for interface in net_interfaces:
                if interface.startswith(('wlan', 'wlp')):
                    self.get_logger().info(f"Found WiFi interface: {interface}")
                    return interface

            # Method 3: Check ip link show
            output = subprocess.check_output(["ip", "link", "show"]).decode("utf-8")
            interfaces = re.findall(r"\d+:\s+(?P<interface>wlan\d+|wlp\d+s\d*):", output)
            if interfaces:
                self.get_logger().info(f"Found WiFi interface: {interfaces[0]}")
                return interfaces[0]

            self.get_logger().warn("Could not find WiFi interface using any method")
            return None

        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            self.get_logger().warn(f"Error detecting WiFi interface: {e}")
            return None

    def gps_unavailable(self):
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.gps_status = -2  # STATUS_UNKNOWN
        self.gps_service = 0  # SERVICE_UNKNOWN
        #self.gps_sample_time = None

    def gps_status_str(self) -> str:
        str = "Unknown"

        match self.gps_status:
            case -1:
                str = "No Fix"
            case 0:
                str = "Fix"
            case 1:
                str = "Fix + Sat Augm"
            case 2:
                str = "Fix + Ground Augm"

        return str

    def gps_service_str(self):
        str = ""

        if self.gps_service & 1:
            str += "GPS "

        if self.gps_service & 2:
            str += "GLONASS "

        if self.gps_service & 4:
            str += "BeiDou "

        if self.gps_service & 8:
            str += "Galileo "

        return str.rstrip()

    def gps_callback(self, msg):
        try:
            # https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/NavSatFix.msg
            # https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/NavSatStatus.msg

            self.gps_status = msg.status.status
            self.gps_service = msg.status.service

            if self.gps_status > 0:
                # Extract latitude and longitude from the gps data
                self.latitude = msg.latitude
                self.longitude = msg.longitude
                self.altitude = round(msg.altitude, 1)
                self.gps_sample_time = self.get_clock().now()  # msg.header.stamp

                # self.get_logger().info(f"GPS OK: status: {self.gps_status_str()}  service: {self.gps_service_str()}  ({self.latitude}, {self.longitude}, {self.altitude})")
            elif self.gps_sample_time is not None:
                self.get_logger().info(f"GPS: no data, status: {self.gps_status_str()}  service: {self.gps_service_str()}")
                self.gps_unavailable()

        except Exception as e:
            self.get_logger().error(f"Unexpected error in gps_callback: {e}")
            self.gps_unavailable()

    def test_iperf3(self):
        """Test if iperf3 can connect to the specified host."""
        try:
            result = subprocess.run(['iperf3', '-c', self.iperf3_host, '--bidir', '--time', '1', '-O', '0', '-P', '1'],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
            self.get_logger().info(f"iperf3 test successful: {result.stdout}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"iperf3 test failed: {e.stderr}")
            self.do_iperf3 = False

    def run_iperf3(self):
        """Run iperf3 and capture the results."""
        if self.iperf3_running:
            self.get_logger().warn("iperf3 is already running. Skipping this execution.")
            return

        self.iperf3_running = True  # Set the flag to indicate the function is running
        try:
            result = subprocess.run(['iperf3', '-c', self.iperf3_host, '--bidir', '--time', '1', '-O', '0', '-P', '1'],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
            output = result.stdout

            # Always log the full output at INFO level
            self.get_logger().info(f"Full iperf3 output:\n{output}")

            # Updated regex patterns to capture the first TX-C and RX-C bitrates
            tx_match = re.search(r"\[\s*\d+\]\[TX-C\][^\n]*?(\d+(?:\.\d+)?)\s+(K|M|G)bits/sec", output)
            rx_match = re.search(r"\[\s*\d+\]\[RX-C\][^\n]*?(\d+(?:\.\d+)?)\s+(K|M|G)bits/sec", output)

            if tx_match:
                tx_value, tx_unit = tx_match.group(1), tx_match.group(2)
                self.get_logger().info(f"TX-C match succeeded. Matched value: {tx_value} {tx_unit}.")
                self.iperf3_sender_bitrate = convert_to_mbps(tx_value, tx_unit)
            else:
                self.get_logger().warn("TX-C match failed.")

            if rx_match:
                rx_value, rx_unit = rx_match.group(1), rx_match.group(2)
                self.get_logger().info(f"RX-C match succeeded. Matched value: {rx_value} {rx_unit}.")
                self.iperf3_receiver_bitrate = convert_to_mbps(rx_value, rx_unit)
            else:
                self.get_logger().warn("RX-C match failed.")

            # Log results or warn if parsing failed
            if not (tx_match or rx_match):
                self.get_logger().warn("iperf3 output parsing failed. Output:")
                self.get_logger().warn(output)
            else:
                self.iperf3_ip = self.iperf3_host
                self.get_logger().info(f"iperf3 results: Sender: {self.iperf3_sender_bitrate} Mbps, Receiver: {self.iperf3_receiver_bitrate} Mbps")

            # Debugging logs to ensure values are assigned correctly
            self.get_logger().info(f"Parsed iperf3_sender_bitrate: {self.iperf3_sender_bitrate}")
            self.get_logger().info(f"Parsed iperf3_receiver_bitrate: {self.iperf3_receiver_bitrate}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"iperf3 execution failed: {e.stderr}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error during iperf3 execution: {e}")
        finally:
            self.iperf3_running = False  # Reset the flag

    def timer_callback(self):
        """
        Periodic callback to collect and store WiFi data.

        This method collects WiFi metrics, associates them with GPS and odometry data,
        publishes the metrics and overlay messages, and stores the data in the database.
        """
        if self.current_pose is None:
            self.get_logger().warn("Current pose not available, skipping data insertion")
            return

        if self.gps_sample_time is None:
            self.gps_unavailable()  # Mark GPS as unavailable without printing an error
        elif (self.get_clock().now() - self.gps_sample_time).nanoseconds / 1e9 > 2.0:
            self.get_logger().warning(f"GPS: data too old, status: {self.gps_status_str()}  service: {self.gps_service_str()}")
            self.gps_unavailable()

        self.x, self.y = tuple(round(x, self.decimals_to_round_coordinates) for x in self.current_pose)  # Round coordinates
        bit_rate, link_quality, signal_level = self.wifi_data_fetcher.get_wifi_data()
        bit_rate = bit_rate if bit_rate is not None else float('nan')
        link_quality = link_quality if link_quality is not None else float('nan')
        signal_level = signal_level if signal_level is not None else float('nan')

        sender_bitrate = self.iperf3_sender_bitrate if self.iperf3_sender_bitrate is not None else float('nan')
        receiver_bitrate = self.iperf3_receiver_bitrate if self.iperf3_receiver_bitrate is not None else float('nan')
        self.publish_wifi_data(bit_rate, link_quality, signal_level, sender_bitrate, receiver_bitrate)

        if self.do_publish_overlay:
            self.publish_wifi_overlay(bit_rate, link_quality, signal_level, self.wifi_data_fetcher.iwconfig_output)

        if all(v is not None for v in [bit_rate, link_quality, signal_level]):
            if self.iperf3_sender_bitrate is not None and self.iperf3_receiver_bitrate is not None:
                self.database_manager.insert_data(
                    self.x, self.y, self.latitude, self.longitude, self.gps_status, self.gps_service,
                    bit_rate, link_quality, signal_level,
                    self.iperf3_sender_bitrate, self.iperf3_receiver_bitrate, self.iperf3_ip
                )
            else:
                self.database_manager.insert_data(
                    self.x, self.y, self.latitude, self.longitude, self.gps_status, self.gps_service,
                    bit_rate, link_quality, signal_level
                )
            self.get_logger().info(
                f"X: {self.x}, Y: {self.y}, "
                # f"Bit Rate: {bit_rate} Mb/s, "
                # f"Link Quality: {link_quality:.2f}, "
                # f"Signal Level: {signal_level} dBm",
                f"Sender Bitrate: {sender_bitrate} Mbps, "
                f"Receiver Bitrate: {receiver_bitrate} Mbps"
            )
        else:
            self.get_logger().warn("Could not retrieve all WiFi data, skipping insertion")
        
    def publish_wifi_data(self, bit_rate, link_quality, signal_level, sender_bitrate, receiver_bitrate):
        """Publish WiFi data including iperf3 results."""
        try:
            msg = Float32MultiArray()
            # Populate the data array with 5 values: bit_rate, link_quality, signal_level, sender_bitrate, receiver_bitrate
            msg.data = [bit_rate, link_quality, signal_level, sender_bitrate, receiver_bitrate]

            if self.do_publish_metrics:
                self.metrics_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Unexpected error publishing WiFi data: {e}")

    def publish_wifi_overlay(self, bit_rate, link_quality, signal_level, iwconfig_output):
        try:

            #self.get_logger().info(iwconfig_output)

            msg = OverlayText() # https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins/blob/main/rviz_2d_overlay_msgs/msg/OverlayText.msg

            msg.action = OverlayText.ADD

            #msg.line_width = 25 # int, pixels - Lines height in the overlay - doesn't seem to affect anything
            msg.text_size = self.ov_font_size # font size
            msg.font = self.ov_font

            nlines = 0

            msg.text = "<pre>"
            if self.ov_do_short:
                iperf3_ip = self.iperf3_ip if self.iperf3_ip is not None else 'NO iperf3'
                sender_bitrate = self.iperf3_sender_bitrate if self.iperf3_sender_bitrate is not None else float('nan')
                receiver_bitrate = self.iperf3_receiver_bitrate if self.iperf3_receiver_bitrate is not None else float('nan')
                msg.text += f"iperf3 to: {iperf3_ip}, sender: {sender_bitrate:4.1f} Mbps, receiver: {receiver_bitrate:4.1f} Mbps\n" 
                msg.text += f"({self.x:4.3f},{self.y:4.3f}), Bit Rate: {bit_rate:4.1f}, Quality: {link_quality:2.1f}, db: {signal_level:2.1f}\n"
                msg.text += f"(lat: {self.latitude}, lon: {self.longitude}, alt: {self.altitude})  stat: {self.gps_status_str()}  {self.gps_service_str()}\n"
                nlines += 3
            if self.ov_do_full:
                msg.text += iwconfig_output.rstrip()
                nlines += 8
            msg.text += "</pre>"

            #self.get_logger().info(msg.text)

            canvas_height = int(self.ov_font_size * nlines * 2.0) # adjust with ov_height_factor

            # text color:
            text_color = np.fromstring(self.ov_font_color, dtype=float, sep=" ")
            #print(f"Text Color: {text_color}")
            msg.fg_color.r = text_color[0]
            msg.fg_color.g = text_color[1]
            msg.fg_color.b = text_color[2]
            msg.fg_color.a = text_color[3]

            # overlay canvas color:
            bg_color = np.fromstring(self.ov_bg_color, dtype=float, sep=" ")
            #print(f"Background Color: {bg_color}")
            msg.bg_color.r = bg_color[0]
            msg.bg_color.g = bg_color[1]
            msg.bg_color.b = bg_color[2]
            msg.bg_color.a = bg_color[3]

            msg.horizontal_alignment = self.ov_horizontal_alignment # OverlayText.LEFT # one of LEFT, CENTER, RIGHT
            msg.vertical_alignment = self.ov_vertical_alignment # OverlayText.TOP # one of TOP, CENTER, BOTTOM
            msg.horizontal_distance = self.ov_horizontal_distance # int, pixels - Horizontal distance from left/right border or center, depending on alignment
            msg.vertical_distance = self.ov_vertical_distance # int, pixels - Vertical distance between from top/bottom border or center, depending on alignment
            msg.width = int(1200.0 * self.ov_width_factor) # int, pixels - Width of the overlay canvas
            msg.height = int(canvas_height * self.ov_height_factor) # int, pixels - Height of the overlay canvas

            #self.get_logger().info(f"Publishing WiFi overlay: interface: {self.wifi_interface}  {iwconfig_output}")
            self.overlay_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Unexpected error publishing WiFi overlay: {e}")

    def odom_callback(self, msg):
        try:
            # Create a PoseStamped message from the odometry data
            odom_pose = PoseStamped()
            odom_pose.header = msg.header
            odom_pose.pose = msg.pose.pose

            # Try to get the transform with a very short timeout
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',  # Target frame
                    'odom',  # Source frame
                    Time(),  # Get the latest available transform
                    timeout=Duration(seconds=0.01)  # Very short timeout
                )

                # Transform the pose to the map frame
                transformed_pose = do_transform_pose(odom_pose.pose, transform)
                self.current_pose = (transformed_pose.position.x, transformed_pose.position.y)
                
                if not self.transform_available:
                    self.transform_available = True
                    self.get_logger().info("Transform from odom to map is now available")
                
            except (LookupException, ConnectivityException, ExtrapolationException):
                # Transform not available yet, use odometry frame directly
                if not self.transform_available:
                    self.current_pose = None
                    # self.current_pose = (odom_pose.pose.position.x, odom_pose.pose.position.y)
                    # self.get_logger().warn("Using odometry frame directly, transform not available yet")
                else:
                    pass
                    # self.get_logger().warn("Transform temporarily unavailable, using last known transform")
            
        except Exception as e:
            self.get_logger().error(f"Unexpected error in odom_callback: {e}")
            self.current_pose = None

    def wait_for_transform(self, timeout_sec: float = 60.0):
        """Wait for the transform from odom to map to become available."""
        self.get_logger().info("Waiting for transform from odom to map...")
        start_time = self.get_clock().now()
        
        while rclpy.ok():
            try:
                # Try to get the transform
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    'odom',
                    Time(),
                    timeout=Duration(seconds=0.1)
                )
                self.get_logger().info("Transform from odom to map is available")
                return
            except (LookupException, ConnectivityException, ExtrapolationException):
                # Check if we've exceeded the timeout
                if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                    self.get_logger().error(f"Timeout waiting for transform after {timeout_sec} seconds")
                    raise RuntimeError("Transform not available")
            # Sleep briefly before trying again
            rclpy.spin_once(self, timeout_sec=0.1)

def convert_to_mbps(value: str, unit: str) -> float:
    """
    Convert a numeric value and unit (K, M, G) to Mbps.
    """
    value = float(value)
    if unit == 'G':
        return value * 1000
    elif unit == 'M':
        return value
    elif unit == 'K':
        return value / 1000
    else:
        return float('nan')

def main(args=None):
    # import ipdb; ipdb.set_trace()  # Add this line to start the debugger
    rclpy.init(args=args)
    print("Starting WiFi logger node")
    wifi_data_collector = WifiDataCollector()
    rclpy.spin(wifi_data_collector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
