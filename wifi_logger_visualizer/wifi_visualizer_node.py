#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import sqlite3
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter
from scipy.interpolate import Rbf  # Import Radial Basis Function interpolator
from rclpy.time import Time
from rclpy.duration import Duration
import os
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
from datetime import datetime

class WifiVisualizerNode(Node):
    def __init__(self):
        super().__init__('wifi_visualizer_node')
        
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
        
        # Declare parameters
        self.declare_parameter('db_path', 'wifi_data.db')
        self.declare_parameter('publish_frequency', 1.0)  # Hz
        self.declare_parameter('db_check_frequency', 2.0)  # Hz
        self.declare_parameter('max_interpolation_distance', 1.0)  # meters
        self.declare_parameter('enable_link_quality', True)
        self.declare_parameter('enable_signal_level', True)
        self.declare_parameter('enable_bit_rate', True)
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        
        # Get parameter values
        self.db_path = self.get_parameter('db_path').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.db_check_frequency = self.get_parameter('db_check_frequency').value
        self.max_interpolation_distance = self.get_parameter('max_interpolation_distance').value
        self.enable_link_quality = self.get_parameter('enable_link_quality').value
        self.enable_signal_level = self.get_parameter('enable_signal_level').value
        self.enable_bit_rate = self.get_parameter('enable_bit_rate').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        
        # Log all parameter values
        self.get_logger().info("Parameter values:")
        self.get_logger().info(f"  costmap_topic: {self.costmap_topic}")
        self.get_logger().info(f"  db_check_frequency: {self.db_check_frequency} Hz")
        self.get_logger().info(f"  db_path: {self.db_path}")
        self.get_logger().info(f"  enable_bit_rate: {self.enable_bit_rate}")
        self.get_logger().info(f"  enable_link_quality: {self.enable_link_quality}")
        self.get_logger().info(f"  enable_signal_level: {self.enable_signal_level}")
        self.get_logger().info(f"  max_interpolation_distance: {self.max_interpolation_distance} meters")
        self.get_logger().info(f"  publish_frequency: {self.publish_frequency} Hz")
        
        # Initialize costmap dimensions
        self.costmap_resolution = None
        self.costmap_width = None
        self.costmap_height = None
        self.costmap_origin = None
        
        # Wait for costmap topic
        self.get_logger().info(f'Waiting for costmap topic {self.costmap_topic}...')
        self.wait_for_costmap()
        
        # Initialize publishers
        if self.enable_link_quality:
            self.link_quality_pub = self.create_publisher(
                OccupancyGrid, 
                'wifi_link_quality_costmap', 
                10
            )
        if self.enable_signal_level:
            self.signal_level_pub = self.create_publisher(
                OccupancyGrid, 
                'wifi_signal_level_costmap', 
                10
            )
        if self.enable_bit_rate:
            self.bit_rate_pub = self.create_publisher(
                OccupancyGrid, 
                'wifi_bit_rate_costmap', 
                10
            )

            
        
        # Initialize database connection and cache
        self.conn = sqlite3.connect(self.db_path)
        self.cursor = self.conn.cursor()
        self.last_timestamp = self.get_latest_timestamp()
        self.last_publish_time = self.get_clock().now()
        self.last_db_check_time = self.get_clock().now()
        
        # Create timers
        self.publish_timer = self.create_timer(1.0 / self.publish_frequency, self.publish_timer_callback)
        self.db_check_timer = self.create_timer(1.0 / self.db_check_frequency, self.db_check_timer_callback)
        
        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info("WifiVisualizerNode initialized")

    def wait_for_costmap(self):
        """Wait for the first costmap message to get dimensions."""
        msg = None
        received = False
        
        def callback(msg_data):
            nonlocal msg, received
            msg = msg_data
            received = True
            # Log the received costmap info immediately
            self.get_logger().info("Received costmap info:")
            self.get_logger().info(f"  - Resolution: {msg_data.info.resolution}")
            self.get_logger().info(f"  - Width: {msg_data.info.width}")
            self.get_logger().info(f"  - Height: {msg_data.info.height}")
            self.get_logger().info(f"  - Origin: ({msg_data.info.origin.position.x}, {msg_data.info.origin.position.y})")
        
        sub = self.create_subscription(OccupancyGrid, self.costmap_topic, callback, 1)
        
        start_time = self.get_clock().now()
        while not received and (self.get_clock().now() - start_time).nanoseconds / 1e9 < 20.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.destroy_subscription(sub)
        
        if received:
            self.costmap_resolution = msg.info.resolution
            self.costmap_width = msg.info.width
            self.costmap_height = msg.info.height
            self.costmap_origin = msg.info.origin  # Store the origin in the class member
            # self.get_logger().info(f'Stored costmap with resolution {self.costmap_resolution}, '
            #                      f'width {self.costmap_width}, height {self.costmap_height}, '
            #                      f'origin: x={self.costmap_origin.position.x}, '
            #                      f'y={self.costmap_origin.position.y}')
        else:
            raise RuntimeError(f'Failed to receive costmap from topic {self.costmap_topic}')

    def parameter_callback(self, params):
        """Handle parameter updates."""
        for param in params:
            if param.name == 'publish_frequency':
                self.publish_frequency = param.value
                self.publish_timer.timer_period_ns = int(1e9 / self.publish_frequency)
            elif param.name == 'db_check_frequency':
                self.db_check_frequency = param.value
                self.db_check_timer.timer_period_ns = int(1e9 / self.db_check_frequency)
            elif param.name == 'max_interpolation_distance':
                self.max_interpolation_distance = param.value
            elif param.name == 'enable_link_quality':
                self.enable_link_quality = param.value
            elif param.name == 'enable_signal_level':
                self.enable_signal_level = param.value
            elif param.name == 'enable_bit_rate':
                self.enable_bit_rate = param.value
            elif param.name == 'costmap_topic':
                self.costmap_topic = param.value
                self.wait_for_costmap()  # Wait for new costmap dimensions
        return True

    def get_latest_timestamp(self):
        """Get the latest timestamp from the database."""
        try:
            self.cursor.execute("SELECT MAX(timestamp) FROM wifi_data")
            return self.cursor.fetchone()[0]
        except sqlite3.Error as e:
            self.get_logger().error(f"Error getting latest timestamp: {e}")
            return None

    def get_data(self):
        """Get all data from the database."""
        try:
            self.cursor.execute("""
                SELECT x, y, link_quality, signal_level, bit_rate 
                FROM wifi_data
            """)
            rows = self.cursor.fetchall()
            # self.get_logger().info(f"Fetched {len(rows)} rows from database")
            return rows
        except sqlite3.Error as e:
            self.get_logger().error(f"Error getting data: {e}")
            return []

    def create_costmap(self, data, field_name, min_val, max_val):
        """Create a signal field map using interpolation only between points within max_interpolation_distance."""
        if not data:
            return None

        x = np.array([row[0] for row in data])
        y = np.array([row[1] for row in data])
        values = np.array([row[field_name] for row in data])

        grid_x, grid_y = np.meshgrid(
            np.arange(self.costmap_width),
            np.arange(self.costmap_height)
        )
        world_x = grid_x * self.costmap_resolution + self.costmap_origin.position.x
        world_y = grid_y * self.costmap_resolution + self.costmap_origin.position.y

        points = np.column_stack((x, y))
        grid_points = np.column_stack((world_x.ravel(), world_y.ravel()))

        # Interpolate as before
        zi = griddata(
            points,
            values,
            grid_points,
            method='linear',
            fill_value=np.nan
        )

        # Mask out grid points farther than max_interpolation_distance from any data point
        from scipy.spatial import cKDTree
        tree = cKDTree(points)
        dists, _ = tree.query(grid_points, k=1)
        mask = dists <= self.max_interpolation_distance

        zi_masked = np.full_like(zi, np.nan)
        zi_masked[mask] = zi[mask]
        zi_masked = zi_masked.reshape(self.costmap_height, self.costmap_width)

        # Normalize and mask out extrapolated regions (NaN)
        zi_norm = np.full_like(zi_masked, -1, dtype=np.int8)
        valid = ~np.isnan(zi_masked)
        if np.any(valid):
            zi_norm[valid] = np.clip(
                ((zi_masked[valid] - min_val) / (max_val - min_val) * 100).astype(np.int8),
                0, 100
            )
        return zi_norm

    def publish_costmap(self, data, field_name, publisher, min_val, max_val):
        """Create and publish a costmap aligned with the global costmap."""
        costmap = self.create_costmap(data, field_name, min_val, max_val)
        if costmap is None:
            return
            
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Use exactly the same metadata as the global costmap
        msg.info.resolution = self.costmap_resolution
        msg.info.width = self.costmap_width
        msg.info.height = self.costmap_height
        msg.info.origin = self.costmap_origin
        
        # Log costmap metadata for debugging
        # self.get_logger().info("Publishing costmap with:")
        # self.get_logger().info(f"  - Resolution: {msg.info.resolution}")
        # self.get_logger().info(f"  - Width: {msg.info.width}")
        # self.get_logger().info(f"  - Height: {msg.info.height}")
        # self.get_logger().info(f"  - Origin: ({msg.info.origin.position.x}, {msg.info.origin.position.y})")
        
        msg.data = costmap.flatten().tolist()
        
        publisher.publish(msg)

    def db_check_timer_callback(self):
        """Check if the database has been updated."""
        current_timestamp = self.get_latest_timestamp()
        if current_timestamp and current_timestamp != self.last_timestamp:
            self.last_timestamp = current_timestamp
            # self.get_logger().info("Database updated, will publish new costmaps")

    def publish_timer_callback(self):
        """Publish costmaps if needed."""
        current_time = self.get_clock().now()
        
        # Check if we should publish based on frequency
        if (current_time - self.last_publish_time).nanoseconds / 1e9 < 1.0 / self.publish_frequency:
            return
            
        # Get data from database
        data = self.get_data()
        if not data:
            return
            
        # Publish enabled costmaps
        if self.enable_link_quality:
            self.publish_costmap(data, 2, self.link_quality_pub, 0.0, 1.0)
        if self.enable_signal_level:
            self.publish_costmap(data, 3, self.signal_level_pub, -90.0, -30.0)
        if self.enable_bit_rate:
            self.publish_costmap(data, 4, self.bit_rate_pub, 0.0, 1000.0)  # Assuming max bit rate of 1000 Mb/s
            
        self.last_publish_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = WifiVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
