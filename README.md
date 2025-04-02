## WiFi Signal Strength Logger and Visualizer

This repository contains slightly modified code by Michael Wimble, the original is here:

https://github.com/wimblerobotics/wifi_logger_visualizer

All credits go to Michael Wimble and his work on [Sigyn](https://github.com/wimblerobotics/Sigyn) robot, which has been my inspiration and the source of shameless borrowing.

**Note:** this software runs under **Ubuntu 24.04** and **ROS2 Jazzy**. Familiarity with these environments is required.

## Use cases

My version of WiFi Signal Strength Logger can be used as part of a robot, or as an independent device. 

When the package is compiled and deployed as a robot's component (_Node_), it subscribes to robot's source of coordinates (odometry or GPS topic) and logs WiFi signal data in a _sqlite3_ database.

When used on a dedicated device, it does the same - but that device should have ROS2 GPS Node running. A Raspberry Pi can host this software.

A companion program (_HeatMapper_) is used to overlay accumulated data from _sqlite3_ on a Google map, creating a _WiFi coverage map_ (a.k.a. "_heat map_").
It can be used after the logger completed the survey, or while doing the survey.

## Build and Deployment

Install Python prerequisites:
```
sudo apt install python3-scipy wireless-tools
```
This is how to build the package:
```
mkdir -p ~/wifi_ws/src
cd ~/wifi_ws/src
git clone https://github.com/slgrobotics/wifi_strength_logger.git
cd ~/wifi_ws

sudo rosdep init    # do it once, if you haven't done it before
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -r -y

colcon build
```

## Running the Nodes

To run either node separately or together use the following commands:
```
ros@plucky:~/robot_ws$ ros2 launch wifi_logger_visualizer wifi_logger.launch.py

ros@plucky:~/robot_ws$ ros2 launch wifi_logger_visualizer wifi_visualizer.launch.py

ros@plucky:~/robot_ws$ ros2 launch wifi_logger_visualizer wifi_logger.launch.py
```
**Note:** _sqlite3_ database file will be created in the directory where you run Logger node: 
```
-rw-r--r--  1 ros ros 102400 Apr  2 12:09 wifi_data.db
```
Refer to [database](https://github.com/slgrobotics/wifi_strength_logger/tree/main/database) folder for viewing the data.

## Useful Links

Similar software for iOS and Android (just for reference, I haven't used any of these):
- [WiFi Heatmap](https://play.google.com/store/apps/details?id=ua.com.wifisolutions.wifiheatmap&hl=en_US&pli=1)
- [ekahau WiFi Heatmaps](https://www.ekahau.com/solutions/wi-fi-heatmaps/)
- [NetSpot](https://www.netspotapp.com/wifi-heat-map/best-wifi-heatmap-software.html)  (available for [iOS](https://apps.apple.com/us/app/netspot-wifi-analyzer/id1490247223))

ROS2 Jazzy setup:
- Intel Desktop: https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy
- Raspberry Pi: https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi

For _robot software_ see:
- Michael Wimble - [Sigyn](https://github.com/wimblerobotics/Sigyn) - don't forget to _star_ it on GitHub
- My Robots - [Plucky, Dragger, Turtle](https://github.com/slgrobotics/robots_bringup)

Please see [MIT License](https://github.com/slgrobotics/wifi_strength_logger/blob/main/LICENSE) for this repository
