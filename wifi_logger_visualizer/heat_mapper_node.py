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

class HeatMapperNode(Node):
    def __init__(self):
        super().__init__('heat_mapper_node')
        self.get_logger().info('Heat Mapper Node has been started.')

        # Declare parameters
        self.declare_parameter('standalone', False)
        self.declare_parameter('db_path', 'wifi_data.db')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('scale_factor', 1.0)
        self.declare_parameter('text_size', 0.08)  # Text size in meters
        self.declare_parameter('do_publish_markers', True)  # Whether to publish value markers
        self.declare_parameter('do_publish_text_markers', True)  # Whether to publish text markers
        
        # Get parameter values
        self.standalone = self.get_parameter('standalone').value
        self.db_path = self.get_parameter('db_path').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.scale_factor = self.get_parameter('scale_factor').value
        self.text_size = self.get_parameter('text_size').value
        self.do_publish_markers = self.get_parameter('do_publish_markers').value
        self.do_publish_text_markers = self.get_parameter('do_publish_text_markers').value
        
        # Log parameter values
        self.get_logger().info(f"Parameter values:")
        self.get_logger().info(f"  standalone: {self.standalone}")
        self.get_logger().info(f"  db_path: {self.db_path}")
        self.get_logger().info(f"  costmap_topic: {self.costmap_topic}")
        self.get_logger().info(f"  scale_factor: {self.scale_factor}")
        self.get_logger().info(f"  text_size: {self.text_size} (type: {type(self.text_size)})")
        self.get_logger().info(f"  do_publish_markers: {self.do_publish_markers}")
        self.get_logger().info(f"  do_publish_text_markers: {self.do_publish_text_markers}")
        
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
        if not data:
            return
            
        # Create and publish markers
        self.publish_heatmap_costmap(data)
    
    def get_data(self):
        """Get all data from the database."""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            cursor.execute("SELECT x,y,bit_rate,link_quality,signal_level FROM wifi_data")
            rows = cursor.fetchall()
            conn.close()
            # self.get_logger().info(f"Fetched {len(rows)} rows from database")
            return rows
        except sqlite3.Error as e:
            self.get_logger().error(f"Error: retrieving wifi data: {e}")
            return []
    
    def publish_heatmap_costmap(self, data):
        """Publish heatmap as a MarkerArray with value markers and text annotations."""
        if not data:
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
            for x, y, timestamp, link_quality, signal_level in data:
                # Convert signal strength to color (red for weak, green for strong)
                # Assuming signal_level is in dBm, typically ranging from -100 to 0
                normalized_strength = (signal_level + 100) / 100.0  # Normalize to 0-1
                normalized_strength = max(0.0, min(1.0, normalized_strength))  # Clamp to 0-1
                
                # Create color gradient from red (weak) to green (strong)
                color = ColorRGBA()
                color.r = 1.0 - normalized_strength
                color.g = normalized_strength
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
                    text_marker.text = f"{signal_level:.1f}"
                    text_markers.markers.append(copy.deepcopy(text_marker))
                    text_marker.id += 1
                    text_marker.color.r = 1.0 - normalized_strength 
                    text_marker.color.g = normalized_strength
                    text_marker.color.b = 0.0
                    text_marker.color.a = 1.0
            
            # Publish value markers
            if self.do_publish_markers:
                value_markers.markers.append(value_marker)
                self.heatmap_pub.publish(value_markers)
                # self.get_logger().info(f'Published {len(value_marker.points)} value markers')
            
            # Publish text markers
            if self.do_publish_text_markers:
                self.text_markers_pub.publish(text_markers)
                # self.get_logger().info(f'Published {len(text_markers.markers)} text markers')
    
    def create_heatmap(self):
        """Create a matplotlib heatmap from the database data."""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            cursor.execute("SELECT x,y,bit_rate,link_quality,signal_level FROM wifi_data")
            rows = np.array(cursor.fetchall())
            rows = np.round(rows, decimals=1) * self.scale_factor

            conn.close()
            row_count = len(rows)
            self.get_logger().info(f"Number of rows: {row_count}")
            
            if row_count == 0:
                self.get_logger().warn("No data found in database")
                return
                
            # self.get_logger().info(f"Row 0: {rows[0]}")
            # self.get_logger().info(f"Row 0 [0] = x: {rows[0][0]}")
            # self.get_logger().info(f"Row 0 [1] = y: {rows[0][1]}")
            # self.get_logger().info(f"Row 0 [2] = bit_rate: {rows[0][2]}")
            # self.get_logger().info(f"Row 0 [3] = link_quality: {rows[0][3]}")
            # self.get_logger().info(f"Row 0 [4] = signal_level: {rows[0][4]}")

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

            heat_data = np.zeros((dim_y, dim_x)) # somehow x and y are inverted when drawing heatmap

            hd_min = +1000.0
            hd_max = -1000.0

            for y in range(dim_y):
                for x in range(dim_x):
                    heat_data[y][x] = None # comment this out to see zeroes instead of white space
                    for row in rows:
                        # find data_x and data_y in rows matching our position (x,y) in drawing space:
                        data_x = int(round(row[0]-min_x))+1
                        data_y = int(round(row[1]-min_y))+1
                        if data_x==x and data_y==y:
                            hd_val = math.floor(row[4] / self.scale_factor) # signal_level=4; scale it back, round down
                            hd_min = min(hd_min, hd_val)
                            hd_max = max(hd_max, hd_val)
                            heat_data[y][x] = hd_val
                            # self.get_logger().debug(f"{data_x} {data_y}   {x} {y} {heat_data[y][x]}")
                self.get_logger().debug(f"---- end Y {y}")

        except sqlite3.Error as e:
            self.get_logger().error(f"Error: retrieving wifi data: {e}")
            return

        # Create the heatmap
        plt.figure(figsize=(10, 8))
        ax = sns.heatmap(heat_data, annot=True, cmap='coolwarm', fmt=".0f", linewidths=.2, vmin=hd_min, vmax=hd_max)
        # https://seaborn.pydata.org/tutorial/color_palettes.html
        ax.invert_yaxis()

        # Customize the plot (optional)
        plt.title('WiFi Signal Strength Heatmap')
        plt.xlabel('X-axis Travel')
        plt.ylabel('Y-axis Travel')

        # Display the heatmap
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = HeatMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
