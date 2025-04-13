# Configuration Parameters for WiFi Logger

This document describes the parameters available in the `wifi_logger_config.yaml` file.

## General Parameters

- **`db_path`**: Path to the SQLite database file. Default: `wifi_data.db`.
- **`wifi_interface`**: Name of the WiFi interface. Leave empty for auto-detection.
- **`min_signal_strength`**: Minimum expected signal strength in dBm. Default: `-90.0`. Values lower than this will be rejected.
- **`max_signal_strength`**: Maximum expected signal strength in dBm. Default: `-30.0`. Values higher than this will be rejected.
- **`update_interval`**: Update interval in seconds. Default: `1.0`.
- **`decimals_to_round_coordinates`**: Number of decimal places to round x and y coordinates. Default: `3`. E.g., a coordinate value of *123.456789* will be rounded to *123.456*.

## Publishing Parameters

The database will always be written to, but you can ask for additional data to be published as topics.
- **`publish_metrics`**: Whether to publish WiFi metrics on the /wifi/metrics topic. Default: `true`.
- **`publish_overlay`**: Whether to publish WiFi overlay messages on the topic /wifi/overlay for display as a text overlay in RViz2. Default: `true`.

## RViz2 Overlay Parameters

- **`ov_horizontal_alignment`**: Horizontal alignment of the overlay. Options: `0` (LEFT), `1` (RIGHT), `2` (CENTER). Default: `0`.
- **`ov_vertical_alignment`**: Vertical alignment of the overlay. Options: `3` (TOP), `2` (CENTER), `4` (BOTTOM). Default: `3`.
- **`ov_horizontal_distance`**: Horizontal distance from the border or center in pixels. Default: `10`.
- **`ov_vertical_distance`**: Vertical distance from the border or center in pixels. Default: `10`.
- **`ov_width_factor`**: Factor to adjust the overlay canvas width. Default: `1.0`.
- **`ov_height_factor`**: Factor to adjust the overlay canvas height. Default: `1.0`.
- **`ov_font`**: Font used in the overlay. Default: `"DejaVu Sans Mono"`.
- **`ov_font_size`**: Font size in the overlay. Default: `12.0`.
- **`ov_font_color`**: Font color in RGBA format. Default: `"0.8 0.8 0.3 0.8"`.
- **`ov_bg_color`**: Background color in RGBA format. Default: `"0.0 0.0 0.0 0.05"`.
- **`ov_do_short`**: Whether to display a short WiFi summary in the overlay. Default: `true`.
- **`ov_do_full`**: Whether to display full WiFi information in the overlay. Default: `true`.
