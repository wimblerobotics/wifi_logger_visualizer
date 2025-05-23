#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import math
import sqlite3
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from rclpy.time import Time
from rclpy.duration import Duration
import copy
import os
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
from datetime import datetime

class HeatMapperNode(Node):
    def __init__(self):
        super().__init__('heat_mapper_node')
        self.get_logger().info('Heat Mapper Node has been started.')

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

        # Declare parameters and retrieve values
        self.standalone = self.declare_and_get_param('standalone', True)
        self.db_path = self.declare_and_get_param('db_path', 'XXwifi_data.db')
        self.costmap_topic = self.declare_and_get_param('costmap_topic', '/global_costmap/costmap')
        self.scale_factor = self.declare_and_get_param('scale_factor', 1.0)
        self.text_size = self.declare_and_get_param('text_size', 0.08)  # Text size in meters
        self.do_publish_markers = self.declare_and_get_param('do_publish_markers', True)  # Whether to publish value markers
        self.do_publish_text_markers = self.declare_and_get_param('do_publish_text_markers', True)  # Whether to publish text markers
        self.heatmap_field = self.declare_and_get_param('heatmap_field', 'iperf3_sender_bitrate')  # Field to visualize
        self.aggregation_type = self.declare_and_get_param('aggregation_type', 'max')  # Aggregation type (min, max, average)

        # Log parameter values
        self.get_logger().info(f"Parameter values:")
        self.get_logger().info(f"  aggregation_type: {self.aggregation_type}")
        self.get_logger().info(f"  costmap_topic: {self.costmap_topic}")
        self.get_logger().info(f"  db_path: {self.db_path}")
        self.get_logger().info(f"  do_publish_markers: {self.do_publish_markers}")
        self.get_logger().info(f"  do_publish_text_markers: {self.do_publish_text_markers}")
        self.get_logger().info(f"  heatmap_field: {self.heatmap_field}")
        self.get_logger().info(f"  scale_factor: {self.scale_factor}")
        self.get_logger().info(f"  standalone: {self.standalone}")
        self.get_logger().info(f"  text_size: {self.text_size} (type: {type(self.text_size)})")
        self.get_logger().info(f"Heatmap field: {self.heatmap_field}, Aggregation type: {self.aggregation_type}")
        
        # Initialize costmap dimensions
        self.costmap_resolution = None
        self.costmap_width = None
        self.costmap_height = None
        
        if not self.standalone:
            # Wait for costmap topic
            self.get_logger().info(f'Waiting for costmap topic {self.costmap_topic}...')
            self.wait_for_costmap()
            
            # Create publishers for heatmap markers and text markers
            if self.do_publish_markers:
                self.heatmap_pub = self.create_publisher(
                    MarkerArray, 
                    'wifi_heat_markers', 
                    10
                )
                
            if self.do_publish_text_markers:
                self.text_markers_pub = self.create_publisher(
                    MarkerArray, 
                    'wifi_heat_text_markers', 
                    10
                )
            
            # Create timer for periodic updates
            self.timer = self.create_timer(1.0, self.timer_callback)
        else:
            # Create matplotlib heatmap
            self.create_heatmap()
        
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

    def wait_for_costmap(self):
        """Wait for the first costmap message to get dimensions."""
        msg = None
        received = False
        
        def callback(msg_data):
            nonlocal msg, received
            msg = msg_data
            received = True
        
        sub = self.create_subscription(OccupancyGrid, self.costmap_topic, callback, 1)
        
        start_time = self.get_clock().now()
        while not received and (self.get_clock().now() - start_time).nanoseconds / 1e9 < 20.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.destroy_subscription(sub)
        
        if received:
            self.costmap_resolution = msg.info.resolution
            self.costmap_width = msg.info.width
            self.costmap_height = msg.info.height
            self.get_logger().info(f'Received costmap with resolution {self.costmap_resolution}, '
                                 f'width {self.costmap_width}, height {self.costmap_height}')
        else:
            raise RuntimeError(f'Failed to receive costmap from topic {self.costmap_topic}')
    
    def timer_callback(self):
        """Periodic callback to publish heatmap markers."""
        # Get data from database
        data = self.get_data()
        if data is None or len(data) == 0:
            return
            
        # Create and publish markers
        self.publish_heatmap_costmap(data)
    
    def get_data(self):
        """Get all data from the database for the selected heatmap field."""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()


            # Dynamically select the field to query based on heatmap_field
            valid_fields = ['bit_rate', 'link_quality', 'signal_level', 'iperf3_sender_bitrate', 'iperf3_receiver_bitrate']
            if self.heatmap_field not in valid_fields:
                self.get_logger().error(f"Invalid heatmap field: {self.heatmap_field}. Defaulting to 'signal_level'.")
                self.heatmap_field = 'signal_level'

            # Allow user to select aggregation type (min, max, average)
            aggregation_type = self.get_parameter('aggregation_type').value
            if aggregation_type not in ['min', 'max', 'average']:
                self.get_logger().error(f"Invalid aggregation type: {aggregation_type}. Defaulting to 'average'.")
                aggregation_type = 'average'

            if aggregation_type == 'average':
                query = f"SELECT x, y, {self.heatmap_field}_sum / {self.heatmap_field}_count AS {self.heatmap_field}_average FROM wifi_data"
            else:
                query = f"SELECT x, y, {self.heatmap_field}_{aggregation_type} FROM wifi_data"

            cursor.execute(query)
            raw_rows = cursor.fetchall()
            # Filter out rows where any value is None
            rows = [row for row in raw_rows if None not in row]
            if not rows:
                self.get_logger().warn("No valid data found in database")
                conn.close()
                return

            rows = np.array(rows)
            rows = np.round(rows, decimals=1) * self.scale_factor

            conn.close()
            row_count = len(rows)
            # self.get_logger().info(f"Number of rows: {row_count}")
            return rows
        except sqlite3.Error as e:
            self.get_logger().error(f"Error retrieving wifi data: {e}")
            return []

    def publish_heatmap_costmap(self, data):
        """Publish heatmap as a MarkerArray with value markers and text annotations."""
        if data is None or data.size == 0:
            self.get_logger().warn('No data to publish')
            return

        # Create marker array for value markers
        if self.do_publish_markers:
            value_markers = MarkerArray()
            value_marker = Marker()
            value_marker.header.frame_id = "map"
            value_marker.header.stamp = self.get_clock().now().to_msg()
            value_marker.ns = "wifi_heatmap"
            value_marker.id = 0
            value_marker.type = Marker.CUBE_LIST
            value_marker.action = Marker.ADD
            value_marker.scale.x = self.costmap_resolution * 2.0
            value_marker.scale.y = self.costmap_resolution * 2.0
            value_marker.scale.z = 0.1  # Height of the cubes
            value_marker.pose.orientation.w = 1.0

            # Create marker array for text annotations
            if self.do_publish_text_markers:
                text_markers = MarkerArray()
                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.ns = "wifi_heatmap_text"
                text_marker.id = 0
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.scale.z = self.text_size  # Text size

            # Process each data point
            for x, y, value in data:
                # Normalize the value for color mapping
                normalized_value = (value - 0) / 100.0  # Example normalization, adjust as needed
                normalized_value = max(0.0, min(1.0, normalized_value))  # Clamp to 0-1

                # Create color gradient from red (low) to green (high)
                color = ColorRGBA()
                color.r = 1.0 - normalized_value
                color.g = normalized_value
                color.b = 0.0
                color.a = 0.8  # Slightly transparent

                # Add point to value marker
                if self.do_publish_markers:
                    point = Point()
                    point.x = x
                    point.y = y
                    point.z = 0.0
                    value_marker.points.append(point)
                    value_marker.colors.append(color)

                # Add text annotation
                if self.do_publish_text_markers:
                    text_point = Point()
                    text_point.x = x
                    text_point.y = y
                    text_point.z = 0.2  # Position text above the cube
                    text_marker.pose.position = text_point
                    text_marker.text = f"{value:.1f}"
                    text_markers.markers.append(copy.deepcopy(text_marker))
                    text_marker.id += 1
                    text_marker.color.r = 1.0 - normalized_value
                    text_marker.color.g = normalized_value
                    text_marker.color.b = 0.0
                    text_marker.color.a = 1.0

            # Publish value markers
            if self.do_publish_markers:
                value_markers.markers.append(value_marker)
                self.heatmap_pub.publish(value_markers)

            # Publish text markers
            if self.do_publish_text_markers:
                self.text_markers_pub.publish(text_markers)
    
    def create_heatmap(self):
        """Create a matplotlib heatmap from the database data."""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            # Dynamically select the field to query based on heatmap_field
            valid_fields = ['bit_rate', 'link_quality', 'signal_level', 'iperf3_sender_bitrate', 'iperf3_receiver_bitrate']
            if self.heatmap_field not in valid_fields:
                self.get_logger().error(f"Invalid heatmap field: {self.heatmap_field}. Defaulting to 'signal_level'.")
                self.heatmap_field = 'signal_level'

            aggregation_type = self.aggregation_type
            if aggregation_type not in ['min', 'max', 'average']:
                self.get_logger().error(f"Invalid aggregation type: {aggregation_type}. Defaulting to 'average'.")
                aggregation_type = 'average'

            if aggregation_type == 'average':
                query = f"SELECT x, y, {self.heatmap_field}_sum / {self.heatmap_field}_count AS {self.heatmap_field}_average FROM wifi_data"
            else:
                query = f"SELECT x, y, {self.heatmap_field}_{aggregation_type} FROM wifi_data"
            cursor.execute(query)
            raw_rows = cursor.fetchall()
            # Filter out rows where any value is None
            rows = [row for row in raw_rows if None not in row]
            if not rows:
                self.get_logger().warn("No valid data found in database")
                conn.close()
                return

            rows = np.array(rows)
            rows = np.round(rows, decimals=1) * self.scale_factor

            conn.close()
            row_count = len(rows)
            self.get_logger().info(f"Number of rows: {row_count}")
            
            if row_count == 0:
                self.get_logger().warn("No data found in database")
                return

            min_arr = np.min(rows, axis=0)
            min_x = math.floor(min_arr[0])
            min_y = math.floor(min_arr[1])

            max_arr = np.max(rows, axis=0)
            max_x = math.ceil(max_arr[0])
            max_y = math.ceil(max_arr[1])

            self.get_logger().info(f"min_x: {min_x}  max_x: {max_x}")
            self.get_logger().info(f"min_y: {min_y}  max_y: {max_y}")

            # dimensions of the drawing space:
            dim_x = int(math.ceil(max_x)) - int(math.floor(min_x)) + 3
            dim_y = int(math.ceil(max_y)) - int(math.floor(min_y)) + 3
            self.get_logger().info(f"drawing space: dim_x: {dim_x}  dim_y: {dim_y}")

            heat_data = np.full((dim_y, dim_x), np.nan)  # seed with nan

            hd_min = +1000.0
            hd_max = -1000.0

            for y in range(dim_y):
                for x in range(dim_x):
                    for row in rows:
                        # find data_x and data_y in rows matching our position (x,y) in drawing space:
                        data_x = int(round(row[0]-min_x))+1
                        data_y = int(round(row[1]-min_y))+1
                        if data_x==x and data_y==y:
                            hd_val = math.floor(row[2] / self.scale_factor) # Use the selected heatmap_field
                            if aggregation_type == 'min':
                                if np.isnan(heat_data[y][x]):
                                    heat_data[y][x] = hd_val
                                else:
                                    heat_data[y][x] = min(heat_data[y][x], hd_val)
                            elif aggregation_type == 'max':
                                if np.isnan(heat_data[y][x]):
                                    heat_data[y][x] = hd_val
                                else:
                                    heat_data[y][x] = max(heat_data[y][x], hd_val)
                            else:  # average or other
                                heat_data[y][x] = hd_val

                    if not np.isnan(heat_data[y][x]):
                        hd_min = min(hd_min, heat_data[y][x])
                        hd_max = max(hd_max, heat_data[y][x])

            # Before plotting:
            cmap = plt.get_cmap('coolwarm').copy()
            cmap.set_bad(color='white')  # Set background color for nan values

            # Create the heatmap
            plt.figure(figsize=(10, 8))
            ax = sns.heatmap(
                heat_data,
                annot=True,
                cmap=cmap,
                fmt=".0f",
                linewidths=.2,
                vmin=hd_min,
                vmax=hd_max
            )
            ax.invert_yaxis()

            # Customize the plot (optional)
            plt.title(f'WiFi {self.aggregation_type.capitalize()} {self.heatmap_field.replace("_", " ").title()} Heatmap')
            plt.xlabel('X-axis Travel')
            plt.ylabel('Y-axis Travel')

            # Display the heatmap
            plt.savefig(f"{self.heatmap_field}_{self.aggregation_type}_heatmap.png")
            plt.show()

        except sqlite3.Error as e:
            self.get_logger().error(f"Error: retrieving wifi data: {e}")
            return

def main(args=None):
    rclpy.init(args=args)
    print("Starting WiFi logger node")
    node = HeatMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
