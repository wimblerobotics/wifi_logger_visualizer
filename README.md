# WiFi Logger and Visualizer for ROS2

This package provides a ROS2 node for collecting, logging, and visualizing WiFi metrics such as bit rate, link quality, and signal strength. It integrates with GPS and odometry data to associate WiFi metrics with spatial coordinates and provides visualization capabilities in RViz2.

---

## **Features**
- **WiFi Data Collection**: Uses `iwconfig` to collect WiFi metrics (bit rate, link quality, signal level).
- **GPS and Odometry Integration**: Associates WiFi metrics with GPS and odometry data for spatial context.
- **Data Logging**: Stores WiFi data in an SQLite database for later analysis.
- **RViz2 Visualization**: Publishes overlay messages for real-time visualization of WiFi metrics in RViz2.
- **Dynamic Parameter Updates**: Allows runtime updates to parameters such as update intervals and signal strength thresholds.
- **Configurable via YAML**: All parameters are configurable through a YAML file for ease of use.

---

## **Installation**

### **Prerequisites**
1. **ROS2 (Jazzy or Later)**: Ensure ROS2 Jazzy or a later version is installed on your system. Follow the [official ROS2 installation guide](https://docs.ros.org/en/jazzy/Installation.html).
2. **Python Dependencies**: The package uses Python 3 and requires the following libraries:
   - `numpy`
   - `pyyaml`
   - `sqlite3`
   - ROS2 Python libraries (e.g., `rclpy`, `nav_msgs`, `sensor_msgs`).

   Install missing dependencies using:
   ```bash
   pip install numpy pyyaml
   ```

3. **RViz2 Plugins**: The package uses the `rviz_2d_overlay_plugins` for visualization. Install it using:
   ```bash
   sudo apt install ros-jazzy-rviz-2d-overlay-plugins
   ```

4. **WiFi Tools**: Ensure `iwconfig` is installed on your system:
   ```bash
   sudo apt install wireless-tools
   ```

---

### **Installation Steps**
1. Clone the repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> wimble_wifi_logger_visualizer
   ```

2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

---

## **Usage**

### **Launching the Node**
To start the WiFi logger node, use the provided launch file:
```bash
ros2 launch wifi_logger_visualizer wifi_logger.launch.py
```

### **Configuration**
The node is configured using a YAML file located in the `config` directory:
```yaml
db_path: "/path/to/wifi_data.db"
wifi_interface: ""  # Leave empty for auto-detection
min_signal_strength: -90.0
max_signal_strength: -30.0
update_interval: 1.0
decimals_to_round_coordinates: 3
publish_metrics: true
publish_overlay: true
```

You can modify this file to adjust parameters such as the database path, WiFi interface, and visualization settings.

### **Topics**
The node publishes the following topics:
- **`/wifi/metrics`**: Publishes WiFi metrics as a `Float32MultiArray` message.
- **`/wifi/overlay`**: Publishes overlay messages for RViz2 visualization.

### **Database**
WiFi data is stored in an SQLite database. The default database file is `wifi_data.db` in the package directory. You can change the path in the YAML configuration file.

---

## **Modules**

### **1. `wifi_logger_visualizer/wifi_logger_node.py`**
The main ROS2 node that:
- Collects WiFi metrics using `iwconfig`.
- Associates WiFi data with GPS and odometry coordinates.
- Publishes metrics and overlay messages.
- Stores data in an SQLite database.

### **2. `database_manager.py`**
Handles all database operations, including:
- Creating the database schema.
- Inserting WiFi data.
- Cleaning up old records.

### **3. `wifi_data_fetcher.py`**
Fetches WiFi metrics using `iwconfig` and validates the data.

---

## **Visualization in RViz2**
To visualize WiFi metrics in RViz2:
1. Start RViz2:
   ```bash
   rviz2
   ```
2. Add the `OverlayText` display plugin.
3. Subscribe to the `/wifi/overlay` topic to see real-time WiFi metrics.

---

## **Dynamic Parameter Updates**
You can update parameters at runtime using the `ros2 param` command. For example:
```bash
ros2 param set /wifi_logger update_interval 2.0
```

---

## **Troubleshooting**

### **1. WiFi Interface Not Detected**
Ensure your WiFi interface is active and visible to `iwconfig`. You can manually specify the interface in the YAML configuration file.

### **2. RViz2 Overlay Not Displayed**
Ensure the `rviz_2d_overlay_plugins` package is installed and the `/wifi/overlay` topic is subscribed in RViz2.

### **3. Database Errors**
Check the database path in the YAML configuration file and ensure the directory is writable.

---

## **Documentation**
For detailed API documentation, install and use [rosdoc2](https://github.com/ros2/rosdoc2):
```bash
sudo apt install ros-jazzy-rosdoc2
```
Generate the documentation:
```bash
rosdoc2 build .
```

---

## **License**
This package is licensed under the MIT License. See the `LICENSE` file for details.

---

## **Contributing**
Contributions are welcome! Please submit issues or pull requests on the GitHub repository.

---

## **Acknowledgments**
This package uses:
- ROS2 Jazzy for robotics middleware.
- `rviz_2d_overlay_plugins` for visualization.
- `iwconfig` for WiFi data collection.