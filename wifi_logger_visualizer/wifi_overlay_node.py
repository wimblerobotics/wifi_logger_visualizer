#!/usr/bin/env python3
"""Bridge passive Wi-Fi JSON status to an RViz text overlay."""
import html
import json
import math

from rclpy.node import Node
from wifi_logger_visualizer.ros_runtime import run_node
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA, String


def format_status(data):
    """Format a status object without interpreting SSIDs as rich text."""
    if not isinstance(data, dict):
        raise ValueError("Wi-Fi status must be a JSON object")

    def text(key):
        value = data.get(key)
        return html.escape(str(value)) if value is not None else "N/A"

    def number(key, unit):
        value = data.get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
            return f"{value:.1f} {unit}"
        return "N/A"

    connected = data.get("connected")
    state = "connected" if connected is True else "disconnected" if connected is False else "unknown"
    return "\n".join([
        f"Wi-Fi: {state}  Interface: {text('interface')}",
        f"SSID: {text('ssid')}  BSSID: {text('bssid')}",
        f"Signal: {number('signal_dbm', 'dBm')}  Frequency: {number('frequency_mhz', 'MHz')}",
        f"TX: {number('tx_bitrate_mbps', 'Mb/s')}  RX: {number('rx_bitrate_mbps', 'Mb/s')}",
        f"Map: {text('map_id')}  Cell: {text('cell')}",
    ])


class WifiOverlayNode(Node):
    """Publish an overlay for each valid passive status message."""

    def __init__(self):
        super().__init__("wifi_overlay_node")
        defaults = {
            "input_topic": "wifi_monitor/status", "output_topic": "wifi/overlay",
            "horizontal_alignment": 2, "vertical_alignment": 3,
            "horizontal_distance": 10, "vertical_distance": 10,
            "width": 1000, "height": 180,
            "font": "DejaVu Sans Mono", "text_size": 12.0,
            "foreground_color": [0.1, 0.1, 0.1, 1.0],
            "background_color": [1.0, 1.0, 1.0, 0.5],
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.publisher = self.create_publisher(
            OverlayText, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            String, self.get_parameter("input_topic").value, self._status, 10
        )

    def _status(self, message):
        try:
            text = format_status(json.loads(message.data))
        except (ValueError, TypeError) as error:
            self.get_logger().warning(f"Ignoring invalid Wi-Fi status: {error}", throttle_duration_sec=10.0)
            return
        overlay = OverlayText()
        overlay.action = OverlayText.ADD
        overlay.text = text
        for key in (
            "horizontal_alignment", "vertical_alignment", "horizontal_distance",
            "vertical_distance", "width", "height", "font", "text_size",
        ):
            setattr(overlay, key, self.get_parameter(key).value)
        for parameter, field in (("foreground_color", "fg_color"), ("background_color", "bg_color")):
            values = self.get_parameter(parameter).value
            if len(values) != 4:
                self.get_logger().error(f"{parameter} must contain four RGBA values")
                return
            setattr(overlay, field, ColorRGBA(**dict(zip(("r", "g", "b", "a"), map(float, values)))))
        self.publisher.publish(overlay)


def main(args=None):
    run_node(WifiOverlayNode, args=args)


if __name__ == "__main__":
    main()
