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

class HeatMapperNode(Node):
    def __init__(self):
        super().__init__('heat_mapper_node')
        self.get_logger().info('Heat Mapper Node has been started.')

        # Declare parameters
        self.declare_parameter('standalone', False)
        self.declare_parameter('db_path', 'wifi_data.db')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('scale_factor', 1.0)
        self.declare_parameter('text_size', 0.25)  # Text size in meters
        
        # Get parameter values
        self.standalone = self.get_parameter('standalone').value
        self.db_path = self.get_parameter('db_path').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.scale_factor = self.get_parameter('scale_factor').value
        self.text_size = self.get_parameter('text_size').value
        
        # Log parameter values
        self.get_logger().info(f"Parameter values:")
        self.get_logger().info(f"  standalone: {self.standalone}")
        self.get_logger().info(f"  db_path: {self.db_path}")
        self.get_logger().info(f"  costmap_topic: {self.costmap_topic}")
        self.get_logger().info(f"  scale_factor: {self.scale_factor}")
        self.get_logger().info(f"  text_size: {self.text_size} (type: {type(self.text_size)})")
        
        # Initialize costmap dimensions
        self.costmap_resolution = None
        self.costmap_width = None
        self.costmap_height = None
        
        if not self.standalone:
            # Wait for costmap topic
            self.get_logger().info(f'Waiting for costmap topic {self.costmap_topic}...')
            self.wait_for_costmap()
            
            # Create publisher for heatmap markers
            self.heatmap_pub = self.create_publisher(
                MarkerArray, 
                'wifi_heatmap_markers', 
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
        self.publish_heatmap_markers(data)
    
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
    
    def publish_heatmap_markers(self, data):
        """Create and publish a heatmap as MarkerArray with transparency support."""
        if not data:
            return
            
        # Extract coordinates and values
        x = np.array([row[0] for row in data])
        y = np.array([row[1] for row in data])
        signal_levels = np.array([row[4] for row in data])  # signal_level is at index 4
        
        # Find min and max coordinates
        x_min, x_max = np.min(x), np.max(x)
        y_min, y_max = np.min(y), np.max(y)
        
        # Create a discrete grid similar to create_heatmap
        # Calculate grid dimensions
        dim_x = int(math.ceil((x_max - x_min) / self.costmap_resolution)) + 1
        dim_y = int(math.ceil((y_max - y_min) / self.costmap_resolution)) + 1
        
        # Initialize heatmap data with -1 (unknown)
        heat_data = np.full((dim_y, dim_x), -1, dtype=np.int8)
        
        # Find min and max signal levels for normalization
        hd_min = +1000.0
        hd_max = -1000.0
        
        # Fill in the heatmap data
        for i, (data_x, data_y, signal_level) in enumerate(zip(x, y, signal_levels)):
            # Convert real-world coordinates to grid indices
            grid_x = int(round((data_x - x_min) / self.costmap_resolution))
            grid_y = int(round((data_y - y_min) / self.costmap_resolution))
            
            # Ensure indices are within bounds
            if 0 <= grid_x < dim_x and 0 <= grid_y < dim_y:
                # Scale the signal level back if needed
                hd_val = math.floor(signal_level / self.scale_factor)
                hd_min = min(hd_min, hd_val)
                hd_max = max(hd_max, hd_val)
                heat_data[grid_y, grid_x] = hd_val
        
        # Create a MarkerArray message
        marker_array = MarkerArray()
        marker_array.markers = []
        
        # Create a marker for each cell with data
        marker_id = 0
        for i in range(dim_y):
            for j in range(dim_x):
                if heat_data[i, j] != -1:
                    # Create a marker for this cell
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.id = marker_id
                    marker_id += 1
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    
                    # Set position (center of the cell)
                    marker.pose.position.x = x_min + j * self.costmap_resolution + self.costmap_resolution / 2
                    marker.pose.position.y = y_min + i * self.costmap_resolution + self.costmap_resolution / 2
                    marker.pose.position.z = 0.0
                    
                    # Set orientation (identity)
                    marker.pose.orientation.w = 1.0
                    
                    # Set scale (size of the cell)
                    marker.scale.x = self.costmap_resolution
                    marker.scale.y = self.costmap_resolution
                    marker.scale.z = 0.1  # Thin cube
                    
                    # Normalize value to 0-1 range for color
                    if hd_max > hd_min:
                        normalized_value = (heat_data[i, j] - hd_min) / (hd_max - hd_min)
                    else:
                        normalized_value = 0.5  # Default to middle value if all values are the same
                    
                    # Set color based on normalized value (red to blue gradient)
                    marker.color = ColorRGBA()
                    marker.color.r = 1.0 - normalized_value  # Red component (1.0 to 0.0)
                    marker.color.g = 0.0  # Green component (constant)
                    marker.color.b = normalized_value  # Blue component (0.0 to 1.0)
                    marker.color.a = 0.7  # Alpha (transparency) - 0.7 for semi-transparent
                    
                    # Add marker to array
                    marker_array.markers.append(marker)
                    
                    # Create a text marker to show the signal value
                    text_marker = Marker()
                    text_marker.header.frame_id = "map"
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.id = marker_id
                    marker_id += 1
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    
                    # Set position (much higher above the cube for better visibility)
                    text_marker.pose.position.x = marker.pose.position.x
                    text_marker.pose.position.y = marker.pose.position.y
                    text_marker.pose.position.z = 1.0  # Much higher above the cube
                    
                    # Set orientation (identity)
                    text_marker.pose.orientation.w = 1.0
                    
                    # Set text with dBm unit for clarity
                    text_marker.text = f"{heat_data[i, j]} dBm"
                    
                    # Set scale (much larger text size using the parameter)
                    text_marker.scale.z = float(self.text_size)  # Ensure it's a float
                    self.get_logger().debug(f"Setting text size to {text_marker.scale.z} for marker {marker_id}")
                    
                    # Set color (bright yellow text for better visibility)
                    text_marker.color = ColorRGBA()
                    text_marker.color.r = 1.0 - normalized_value   # Red
                    text_marker.color.g = 0.0  # Green
                    text_marker.color.b = normalized_value # No blue
                    text_marker.color.a = 1.0  # Fully opaque
                    
                    # Add text marker to array
                    marker_array.markers.append(text_marker)
                    
                    # Create a background sphere behind the text for better contrast
                    bg_marker = Marker()
                    bg_marker.header.frame_id = "map"
                    bg_marker.header.stamp = self.get_clock().now().to_msg()
                    bg_marker.id = marker_id
                    marker_id += 1
                    bg_marker.type = Marker.SPHERE
                    bg_marker.action = Marker.ADD
                    
                    # Set position (same as text)
                    bg_marker.pose.position.x = text_marker.pose.position.x
                    bg_marker.pose.position.y = text_marker.pose.position.y
                    bg_marker.pose.position.z = text_marker.pose.position.z
                    
                    # Set orientation (identity)
                    bg_marker.pose.orientation.w = 1.0
                    
                    # Set scale (slightly larger than text)
                    bg_size = float(self.text_size) * 1.5  # Ensure it's a float
                    bg_marker.scale.x = bg_size
                    bg_marker.scale.y = bg_size
                    bg_marker.scale.z = bg_size
                    
                    # Set color (black background)
                    bg_marker.color = ColorRGBA()
                    bg_marker.color.r = 0.0
                    bg_marker.color.g = 0.0
                    bg_marker.color.b = 0.0
                    bg_marker.color.a = 0.7  # Semi-transparent
                    
                    # Add background marker to array (before text so it appears behind)
                    marker_array.markers.append(bg_marker)
        
        # Publish the marker array
        self.heatmap_pub.publish(marker_array)
        self.get_logger().debug(f"Published heatmap markers with {len(marker_array.markers)//2} cells")
        
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
