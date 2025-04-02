## WiFi Heat Mapper

This Python script visualizes signal strength data, collected by *[wifi_signal_visualizer](https://github.com/slgrobotics/wifi_strength_logger)* (actually, a logger)

The data must reside in a [sqlite3](https://www.sqlite.org/) database.

It uses [seaborn](https://seaborn.pydata.org/) library to bring up a GUI.

## Running it:

Assuming that you already cloned the repository and have the database file under ```~/wifi_ws/src/wifi_strength_logger/database```:

```
cd ~/wifi_ws/src/wifi_strength_logger/wifi_heat_mapper
python3 heat_mapper.py
```

You will see a heatmap in a pop-up window. 

**Hint:** you can use https://github.com/wimblerobotics/Sigyn/blob/main/wifi_data.db for experiments with this code
