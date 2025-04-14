# WiFi Logger and Visualizer for ROS2

This package provides a ROS2 node for collecting, logging, and visualizing WiFi metrics such as bit rate, link quality, and signal strength. It integrates with GPS and odometry data to associate WiFi metrics with spatial coordinates and provides visualization capabilities both in RViz2 and as a standalone chart.

**NOTE for user os previous versions of this package. The database schema has changed for
version 1.2 of this package. Please delete old databases in order to ensure compatability
this this version.**

---

## **Features**
- **WiFi Data Collection**: Uses `iwconfig` to collect WiFi metrics (bit rate, link quality, signal level).
- **GPS and Odometry Integration**: Associates WiFi metrics with GPS and odometry data for spatial context.
- **Data Logging**: Stores WiFi data in an SQLite database for later analysis.
- **RViz2 Visualization**: Publishes overlay messages for real-time visualization of WiFi metrics in RViz2.
- **Standalone Chart**: Generates a standalone chart for visualizing WiFi metrics.
- **Dynamic Parameter Updates**: Allows runtime updates to parameters such as update intervals and signal strength thresholds.
- **Configurable via YAML**: All parameters are configurable through a YAML file for ease of use.

---

## **Installation**

### **Prerequisites**
1. **ROS2 (Jazzy or Later)**: Ensure ROS2 Jazzy or a later version is installed on your system. Follow the [official ROS2 installation guide](https://docs.ros.org/en/jazzy/Installation.html).

2. **SQLite3**: Ensure SQLite3 is installed on your system:
   ```bash
   sudo apt install sqlite3 libsqlite3-dev
   ```

   You can check if SQLite3 is installed by running:
   ```bash
   sqlite3 --version
   ```

   If not installed, you can install it using the command above.
3. **Python Dependencies**: The package uses Python 3 and requires the following libraries:
   - `matplotlib`
   - `numpy`
   - `pyyaml`
   - `seaborn`
   - `scipi`
   - `sqlite3`
   
   Install missing Python dependencies using:
   ```bash
   pip install numpy pyyaml matplotlib seaborn scipy
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
   cd ..
   rosdep install --from-paths src --ignore-src -r -y 

   ```

2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

---

## **Usage**

### **Capturing Wifi Data**
The Wifi Logger node uses the **iwconfig** application to capture
The bit rante, link quality and signal strength from the onboard wifi device.
It requires odometry data to be published so that it can associate a location with
the captured wifi data.
If GPS location data is also being published, it will capture the GPS location as well.

The data is always written to an SQLite database. You can also specify that the data
should be published as one or two ROS topics as well.

#### **Configuration**
Usually, you edit the **config.yaml** file, located in the **config/wifi_logger_config.yaml**
file before building this package to set the default parameters for this package.
But, you can override the configuration parameters as needed on the command line.
This module specifically uses the following parameters:

* **db_path** (default: 'wifi_data.db')  
  The location of the SQLite datbase.
  If it does not exist, it will be created.
  If an abolute pathname is not provided, it will be used as a relative pathname.
* **decimals_round_coordinates** (defaule: 3)  
  When entries are about to be put into the database, the database is checked to see if
  there is already any entry for the current odometry pose. If so, the current entry is
  replaced. Otherwise a new entry is created. As odometry can create poses with very
  small differences in locations, such as **x=0.1234567** vs **x=0.1234568**, sometimes induced
  by sensor noise, this parameter allows you to put pose values into more useful buckets.
  A value of **3**, for instance, would round both of those **x** coordinates to a value
  of 0.123.
* **max_signal_level** (default: -20.0)  
  Maximum expected signal level in dBm. Values higher than this will be rejected.
* **min_signal_level** (default -100.0)  
  Minimum expected signal level in dBm. Values lower than this will be rejected.
* **publish_metrics** (default: 'true')  
  If **true**, publish 
* publish_overlay
* **update_interval**
  How often to publish the wifi metrics and wifi  overlay topics and insert new data into the database. Pay attention to this as the speed of your robot movement and this parameter jointly combine to determine the possible resolution of your data in the database.
  For instance, if you robot is moving 1 meter persecond and the update interval is 1 second,
  then the entries in the database are likely to be about a meter apart. 
  If you wanted wifi readings in the database to be about 10 centimeters aparts,
  you'd have to change this **update_interval** to be 10 times a second.
* **wifi_interface**  
  Specify the WiFi device name. E.g., **wlp9s0**. If left blank, the program will
  attempt to find device name automatically.

The following parameters can be changed dynamically using, e.g., the **rqt** program:
* decimals_to_round_coordinates
* max_signal_level
* min_signal_level
* update_interval

#### **Additional Configuration for Publishing the Overlay Topic**
        self.ov_horizontal_alignment = config.get('ov_horizontal_alignment', 0)
        self.ov_vertical_alignment = config.get('ov_vertical_alignment', 3)
        self.ov_horizontal_distance = config.get('ov_horizontal_distance', 10)
        self.ov_vertical_distance = config.get('ov_vertical_distance', 10)
        self.ov_width_factor = config.get('ov_width_factor', 1.0)
        self.ov_height_factor = config.get('ov_height_factor', 1.0)
        self.ov_font = config.get('ov_font', "DejaVu Sans Mono")
        self.ov_font_size = config.get('ov_font_size', 12.0)
        self.ov_font_color = config.get('ov_font_color', "0.8 0.8 0.3 0.8")
        self.ov_bg_color = config.get('ov_bg_color', "0.0 0.0 0.0 0.05")
        self.ov_do_short = config.get('ov_do_short', True)
        self.ov_do_full = config.get('ov_do_full', True)

  

### **Displaying Wifi Data as a Topological Map**

### **Displaying Wifi Data as a Heat map**

### **Launching the Wifi Loggin Node**
To start the WiFi logger node, use the provided launch file:
```bash
ros2 launch wifi_logger_visualizer wifi_logger.launch.py
```

### **Configuration**
The node is configured using a YAML file located in the `config` directory.

You can modify this file to adjust parameters such as the database path, WiFi interface, and visualization settings.

See the [Configuration Guide](CONFIG_README.md) for a complete description of the file.

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