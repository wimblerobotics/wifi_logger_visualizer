#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
import subprocess
import re
import sqlite3
from typing import Optional, Tuple
import os
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

class WifiDataCollector(Node):
    def __init__(self):
        super().__init__('wifi_logger_node')
        
        # Initialize TF buffer and listener
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Declare and get parameters
        self.declare_parameter('db_path', os.path.join(os.getcwd(), 'wifi_data.db'))
        self.declare_parameter('wifi_interface', '')  # Empty string means auto-detect
        self.declare_parameter('update_interval', 1.0)
        self.declare_parameter('max_signal_strength', -30.0)  # dBm
        self.declare_parameter('min_signal_strength', -90.0)  # dBm
        self.declare_parameter('decimals_to_round_coordinates', 3)  # Default to 3 decimal places

        # What to publish:
        self.declare_parameter('publish_metrics', True)
        self.declare_parameter('publish_overlay', True)
        
        # RViz2 overlay parameters:
        self.declare_parameter('ov_horizontal_alignment', 0) # LEFT:0 RIGHT:1 CENTER:2
        self.declare_parameter('ov_vertical_alignment', 3) # CENTER:2 TOP:3 Bottom:4
        self.declare_parameter('ov_horizontal_distance', 10)
        self.declare_parameter('ov_vertical_distance', 10)
        self.declare_parameter('ov_width_factor', 1.0)  # adjust overlay canvas width
        self.declare_parameter('ov_height_factor', 1.0)  # adjust overlay canvas height
        self.declare_parameter('ov_font', "DejaVu Sans Mono")
        self.declare_parameter('ov_font_size', 12.0)
        self.declare_parameter('ov_font_color', "0.8 0.8 0.3 0.8") # RGBA
        self.declare_parameter('ov_bg_color', "0.0 0.0 0.0 0.05")
        self.declare_parameter('ov_do_short', True)
        self.declare_parameter('ov_do_full', True)

        # Get parameter values
        self.db_path = self.get_parameter('db_path').value
        self.wifi_interface = self.get_parameter('wifi_interface').value
        self.update_interval = self.get_parameter('update_interval').value
        self.max_signal_strength = self.get_parameter('max_signal_strength').value
        self.min_signal_strength = self.get_parameter('min_signal_strength').value
        self.decimals_to_round_coordinates = self.get_parameter('decimals_to_round_coordinates').value

        # What to publish:
        self.do_publish_metrics = self.get_parameter('publish_metrics').value
        self.do_publish_overlay = self.get_parameter('publish_overlay').value

        # RViz2 overlay parameters:
        self.ov_horizontal_alignment = self.get_parameter('ov_horizontal_alignment').value
        self.ov_vertical_alignment = self.get_parameter('ov_vertical_alignment').value
        self.ov_horizontal_distance = self.get_parameter('ov_horizontal_distance').value
        self.ov_vertical_distance = self.get_parameter('ov_vertical_distance').value
        self.ov_width_factor = self.get_parameter('ov_width_factor').value
        self.ov_height_factor = self.get_parameter('ov_height_factor').value
        self.ov_font = self.get_parameter('ov_font').value
        self.ov_font_size = self.get_parameter('ov_font_size').value
        self.ov_font_color = self.get_parameter('ov_font_color').value
        self.ov_bg_color = self.get_parameter('ov_bg_color').value
        self.ov_do_short = self.get_parameter('ov_do_short').value
        self.ov_do_full = self.get_parameter('ov_do_full').value

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

        # Initialize database
        self.create_table()
        
        # Create odometry subscription with optimized QoS settings
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Use best effort for real-time performance
            history=QoSHistoryPolicy.KEEP_LAST,  # Keep only the last message
            depth=100,  # Keep last 100 messages
            durability=QoSDurabilityPolicy.VOLATILE  # Don't persist messages
        )
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos
        )

        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            '/gps/filtered',
            self.gps_callback,
            qos
        )

        if self.do_publish_metrics:
            self.metrics_publisher = self.create_publisher(
                Float32MultiArray,
                '/wifi/metrics',
                10
            )

        if self.do_publish_overlay:
            self.overlay_publisher = self.create_publisher(
                OverlayText,
                '/wifi/overlay',
                10
            )

        self.current_pose = None
        self.transform_available = False

        self.wait_for_transform()
        
        # Create timer for periodic updates
        self.timer = self.create_timer(self.update_interval, self.timer_callback)
        
        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """Handle parameter updates."""
        for param in params:
            if param.name == 'update_interval':
                self.update_interval = param.value
                self.timer.timer_period_ns = int(self.update_interval * 1e9)
            elif param.name == 'max_signal_strength':
                self.max_signal_strength = param.value
            elif param.name == 'min_signal_strength':
                self.min_signal_strength = param.value
        return True

    def get_wifi_interface(self) -> Optional[str]:
        """Detect the WiFi interface using multiple methods."""
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

    def create_table(self):
        """Create the database table with proper constraints and indices."""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # Create table with proper constraints
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS wifi_data (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    lat REAL NULL,
                    lon REAL NULL,
                    gps_status INT NULL,
                    gps_service INT NULL,
                    bit_rate REAL CHECK (bit_rate >= 0),
                    link_quality REAL CHECK (link_quality >= 0 AND link_quality <= 1),
                    signal_level REAL CHECK (signal_level >= -90.0 AND signal_level <= -30.0),
                    UNIQUE(x, y, timestamp)
                )
            """)
            
            # Create indices for better query performance
            cursor.execute("CREATE INDEX IF NOT EXISTS idx_timestamp ON wifi_data(timestamp)")
            cursor.execute("CREATE INDEX IF NOT EXISTS idx_coordinates ON wifi_data(x, y)")
            
            conn.commit()
            conn.close()
            self.get_logger().info(f"Database table created/verified successfully at {self.db_path}")
        except sqlite3.Error as e:
            self.get_logger().error(f"Error creating table: {e}")
            raise

    def validate_data(self, bit_rate: float, link_quality: float, signal_level: float) -> bool:
        """Validate WiFi data before insertion."""
        if bit_rate is not None and bit_rate < 0:
            self.get_logger().warn(f"Invalid bit rate: {bit_rate}")
            return False
            
        if link_quality is not None and not (0 <= link_quality <= 1):
            self.get_logger().warn(f"Invalid link quality: {link_quality}")
            return False
            
        if signal_level is not None and not (self.min_signal_strength <= signal_level <= self.max_signal_strength):
            self.get_logger().warn(f"Signal level {signal_level} dBm outside expected range")
            return False
            
        return True

    def insert_data(self, bit_rate: float, link_quality: float, signal_level: float):
        """Insert WiFi data with validation and error handling."""
        if not self.validate_data(bit_rate, link_quality, signal_level):
            return

        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # Check if we already have data for this location
            cursor.execute("""
                SELECT id FROM wifi_data 
                WHERE x = ? AND y = ? 
                ORDER BY timestamp DESC LIMIT 1
            """, (self.x, self.y))
            
            existing_record = cursor.fetchone()
            
            if existing_record:
                # Update existing record if it's within the last minute
                cursor.execute("""
                    UPDATE wifi_data 
                    SET bit_rate = ?, link_quality = ?, signal_level = ?, timestamp = CURRENT_TIMESTAMP
                    WHERE id = ?
                """, (bit_rate, link_quality, signal_level, existing_record[0]))
                # self.get_logger().info(f"Updated existing record: {existing_record[0]}")
            else:
                # Insert new record
                cursor.execute("""
                    INSERT INTO wifi_data (x, y, lat, lon, gps_status, gps_service, bit_rate, link_quality, signal_level)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (self.x, self.y, self.latitude, self.longitude, self.gps_status, self.gps_service, bit_rate, link_quality, signal_level))
                #self.get_logger().info(f"Inserted new record: {self.x}, {self.y}, {self.latitude}, {self.longitude}, {self.gps_status}, {self.gps_service}, {bit_rate}, {link_quality}, {signal_level}")
            conn.commit()
            conn.close()
            
        except sqlite3.Error as e:
            self.get_logger().error(f"Error inserting data: {e}")
            # Try to recover by recreating the table
            try:
                self.create_table()
            except sqlite3.Error as e2:
                self.get_logger().error(f"Failed to recover from database error: {e2}")

    def cleanup_old_data(self, max_age_days: int = 30):
        """Remove data older than specified days."""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute("""
                DELETE FROM wifi_data 
                WHERE timestamp < datetime('now', '-' || ? || ' days')
            """, (max_age_days,))
            
            deleted_count = cursor.rowcount
            conn.commit()
            conn.close()
            
            if deleted_count > 0:
                self.get_logger().info(f"Cleaned up {deleted_count} old records")
                
        except sqlite3.Error as e:
            self.get_logger().error(f"Error cleaning up old data: {e}")

    def get_wifi_data(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        """Get WiFi data with improved error handling and validation."""
        try:
            self.iwconfig_output = subprocess.check_output(["iwconfig", self.wifi_interface]).decode("utf-8")

            #self.get_logger().info(self.iwconfig_output)

            # Extract bit rate
            bit_rate_match = re.search(r"Bit Rate[:=](?P<bit_rate>\d+\.?\d*) (Mb/s|Gb/s)", self.iwconfig_output)
            if bit_rate_match:
                bit_rate = float(bit_rate_match.group("bit_rate"))
                if bit_rate_match.group(2) == "Gb/s":
                    bit_rate *= 1000  # Convert Gb/s to Mb/s
            else:
                bit_rate = None

            # Extract link quality
            link_quality_match = re.search(r"Link Quality=(?P<link_quality>\d+/\d+)", self.iwconfig_output)
            if link_quality_match:
                link_quality_str = link_quality_match.group("link_quality")
                link_quality = float(link_quality_str.split('/')[0]) / float(link_quality_str.split('/')[1])
            else:
                link_quality = None

            # Extract and validate signal level
            signal_level_match = re.search(r"Signal level[:=](?P<signal_level>-?\d+) dBm", self.iwconfig_output)
            if signal_level_match:
                signal_level = float(signal_level_match.group("signal_level"))
                # Validate signal level is within expected range
                if not (self.min_signal_strength <= signal_level <= self.max_signal_strength):
                    self.get_logger().warn(f"Signal level {signal_level} dBm outside expected range")
                    signal_level = None
            else:
                signal_level = None

            return bit_rate, link_quality, signal_level

        except subprocess.CalledProcessError as e:
            self.get_logger().warn(f"Could not retrieve WiFi data: {e}")
            return None, None, None
        except Exception as e:
            self.get_logger().error(f"Unexpected error getting WiFi data: {e}")
            return None, None, None

    def publish_wifi_data(self, bit_rate, link_quality, signal_level):
        try:
            msg = Float32MultiArray()
            # Populate the data array (a 3x1 array), leave layout empty:
            data = [bit_rate, link_quality, signal_level]
            msg.data = data

            #self.get_logger().info(f"Publishing WiFi data: interface: {self.wifi_interface}  bit_rate: {bit_rate}  link_quality: {link_quality}  signal_level: {signal_level}")
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
                msg.text += f"({self.x},{self.y})  Bit Rate: {bit_rate}  Quality: {link_quality}  db: {signal_level}\n"
                msg.text += f"({self.latitude}, {self.longitude}, {self.altitude})  {self.gps_status_str()}  {self.gps_service_str()}\n"
                nlines += 2
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

    def gps_unavailable(self):
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.gps_status = -2  # STATUS_UNKNOWN
        self.gps_service = 0  # SERVICE_UNKNOWN
        #self.gps_sample_time = None

    def gps_status_str(self):
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
                self.altitude = round(msg.altitude,1)
                self.gps_sample_time = self.get_clock().now() # msg.header.stamp

                #self.get_logger().info(f"GPS OK: status: {self.gps_status_str()}  service: {self.gps_service_str()}  ({self.latitude}, {self.longitude}, {self.altitude})")
            else:
                self.get_logger().info(f"GPS: no data, status: {self.gps_status_str()}  service: {self.gps_service_str()}")
                self.gps_unavailable()

        except Exception as e:
            self.get_logger().error(f"Unexpected error in gps_callback: {e}")
            self.gps_unavailable()

    def timer_callback(self):
        """Periodic callback to collect and store WiFi data."""
        if self.current_pose is None:
            self.get_logger().warn("Current pose not available, skipping data insertion")
            return

        if self.gps_sample_time is None or (self.get_clock().now() - self.gps_sample_time).nanoseconds / 1e9 > 2.0:
            self.get_logger().warning(f"GPS: data too old, status: {self.gps_status_str()}  service: {self.gps_service_str()}")
            self.gps_unavailable() # last GPS was more than 2 seconds ago, mark it invalid

        self.x, self.y = tuple(round(x, self.decimals_to_round_coordinates) for x in self.current_pose) # let's work on a grid defined by decimals_to_round_coordinates
        bit_rate, link_quality, signal_level = self.get_wifi_data()

        if self.do_publish_metrics:
            self.publish_wifi_data(bit_rate, link_quality, signal_level)
        
        if self.do_publish_overlay:
            self.publish_wifi_overlay(bit_rate, link_quality, signal_level, self.iwconfig_output)
        
        if all(v is not None for v in [bit_rate, link_quality, signal_level]):
            self.insert_data(bit_rate, link_quality, signal_level)
            self.get_logger().debug(
                f"X: {self.x}, Y: {self.y}, "
                f"Bit Rate: {bit_rate} Mb/s, "
                f"Link Quality: {link_quality:.2f}, "
                f"Signal Level: {signal_level} dBm"
            )
        else:
            self.get_logger().warn("Could not retrieve all WiFi data, skipping insertion")
        
        # Clean up old data once per day
        # if self.get_clock().now().nanoseconds % (24 * 60 * 60 * 1e9) < self.update_interval * 1e9:
        #     self.cleanup_old_data()

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

def main(args=None):
    # import ipdb; ipdb.set_trace()  # Add this line to start the debugger
    rclpy.init(args=args)
    print("Starting WiFi logger node")
    wifi_data_collector = WifiDataCollector()
    rclpy.spin(wifi_data_collector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
