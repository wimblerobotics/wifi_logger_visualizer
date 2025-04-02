## sqlite3 Database installation

https://linuxcapable.com/how-to-install-sqlite-on-ubuntu-linux/

I am using Ubuntu 24.04 on Intel Desktops and Raspberry Pi 4 and 5
```
sudo apt install sqlite3
sqlite3 --version
```

Usage and general info:
- https://www.sqlite.org/quickstart.html
- https://github.com/sqlite/sqlite
- https://www.sqlite.org/
- https://github.com/wimblerobotics/Sigyn/blob/main/Documentation/Notes/wifi_signal.md

The ```wifi_data.db``` database file is created in the current directory when you run Logger node:
```
ros2 launch wifi_logger_visualizer wifi_logger.launch.py
```

Simple commands (note the semicolon at the end):
```
~/wimble_ws/src/Sigyn/wifi_signal_visualizer$ sqlite3 ../wifi_data.db
   SQLite version 3.45.1 2024-01-30 16:01:20
Enter ".help" for usage hints.

sqlite> select count(*) from wifi_data;
476

sqlite> .headers ON
sqlite> SELECT * from wifi_data LIMIT 2;
timestamp|x|y|bit_rate|link_quality|signal_level
2025-03-29 05:00:19|8.61574735023807|2.26927850638535|960.7|0.828571428571429|-52.0
2025-03-29 05:00:20|8.61576060339672|2.26931183264393|960.7|0.814285714285714|-53.0

sqlite> .tables
wifi_data
sqlite> .schema wifi_data
CREATE TABLE wifi_data (
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    x REAL,
                    y REAL,
                    bit_rate REAL,
                    link_quality REAL,
                    signal_level REAL,
                    PRIMARY KEY (x, y)
                );
sqlite> 
```
Refer also to https://github.com/wimblerobotics/Sigyn/blob/main/Documentation/Notes/wifi_signal.md
