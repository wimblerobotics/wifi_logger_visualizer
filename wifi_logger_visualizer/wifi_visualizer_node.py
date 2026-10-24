#!/usr/bin/env python3
import colorsys
from datetime import datetime, timezone
import hashlib
from typing import Any, Dict, Optional, Tuple

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from wifi_logger_visualizer.database_manager import DatabaseManager
from wifi_logger_visualizer.map_data import decode_pgm, load_map


METRICS = {
    "signal": ("cell", "signal_dbm_last", -90.0, -35.0, False),
    "tx_link_rate": ("cell", "tx_bitrate_mbps_last", 0.0, 1200.0, False),
    "rx_link_rate": ("cell", "rx_bitrate_mbps_last", 0.0, 1200.0, False),
    "upload": ("survey", "upload_mbps", 0.0, 500.0, False),
    "download": ("survey", "download_mbps", 0.0, 500.0, False),
    "idle_latency": ("survey", "idle_latency_ms", 0.0, 100.0, True),
    "loaded_latency": ("survey", "loaded_latency_ms", 0.0, 200.0, True),
    "age": ("cell", "last_observed_at", 0.0, 30.0, True),
    "access_point": ("cell", "bssid", 0.0, 1.0, False),
}


class WifiVisualizerNode(Node):
    """Publish measured 25 cm cells without inventing values between them."""

    def __init__(self) -> None:
        super().__init__("wifi_visualizer_node")
        self.declare_parameter("db_path", "~/.ros/titania_wifi.db")
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("map_id", 0)
        self.declare_parameter("metric", "signal")
        self.declare_parameter("cell_size", 0.25)
        self.declare_parameter("publish_frequency", 1.0)
        self.declare_parameter("highlight_unmeasured", False)
        self.declare_parameter("marker_alpha", 0.65)
        self.declare_parameter("metric_min", float("nan"))
        self.declare_parameter("metric_max", float("nan"))

        self.database = DatabaseManager(str(self.get_parameter("db_path").value))
        requested_map = int(self.get_parameter("map_id").value)
        map_path = str(self.get_parameter("map_yaml_path").value)
        if requested_map:
            map_data = self.database.get_map(requested_map)
        elif map_path:
            map_id = self.database.register_map(load_map(map_path))
            map_data = self.database.get_map(map_id)
        else:
            map_data = self.database.latest_map()
        if map_data is None:
            raise RuntimeError("The Wi-Fi database contains no captured map")
        self.map_data = map_data
        self.map_id = int(map_data["id"])
        self.cell_size = float(self.get_parameter("cell_size").value)
        metric = str(self.get_parameter("metric").value)
        if metric not in METRICS:
            raise ValueError(f"Unsupported Wi-Fi visualization metric: {metric}")
        self.metric = metric

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            MarkerArray, "/wifi_monitor/cells", qos
        )
        frequency = float(self.get_parameter("publish_frequency").value)
        if frequency <= 0:
            raise ValueError("publish_frequency must be positive")
        self.create_timer(1.0 / frequency, self.publish)
        self.publish()

    def destroy_node(self) -> None:
        self.database.close()
        super().destroy_node()

    def _color(self, value: Any, minimum: float, maximum: float) -> ColorRGBA:
        if self.metric == "access_point":
            digest = hashlib.sha256(str(value).encode()).digest()
            hue = int.from_bytes(digest[:2], "big") / 65535.0
            red, green, blue = colorsys.hsv_to_rgb(hue, 0.8, 1.0)
        else:
            normalized = (
                0.5
                if maximum == minimum
                else max(0.0, min(1.0, (float(value) - minimum) / (maximum - minimum)))
            )
            if METRICS[self.metric][4]:
                normalized = 1.0 - normalized
            hue = normalized / 3.0
            red, green, blue = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
        return ColorRGBA(
            r=float(red),
            g=float(green),
            b=float(blue),
            a=float(self.get_parameter("marker_alpha").value),
        )

    def _metric_rows(self) -> list[Tuple[Dict[str, Any], Any]]:
        metric = str(self.get_parameter("metric").value)
        if metric not in METRICS:
            self.get_logger().error(
                f"Ignoring unsupported visualization metric: {metric}"
            )
            return []
        self.metric = metric
        source, field, default_min, default_max, _ = METRICS[self.metric]
        records = (
            self.database.get_cells(self.map_id)
            if source == "cell"
            else self.database.get_surveys(self.map_id)
        )
        rows = []
        for record in records:
            value = record.get(field)
            if self.metric == "age" and value:
                observed = datetime.fromisoformat(str(value))
                value = (
                    datetime.now(timezone.utc) - observed
                ).total_seconds() / 86400.0
            if value is not None:
                rows.append((record, value))
        configured_min = float(self.get_parameter("metric_min").value)
        configured_max = float(self.get_parameter("metric_max").value)
        self.minimum = (
            configured_min if configured_min == configured_min else default_min
        )
        self.maximum = (
            configured_max if configured_max == configured_max else default_max
        )
        return rows

    def _cube_list(self, marker_id: int, namespace: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.cell_size
        marker.scale.y = self.cell_size
        marker.scale.z = 0.025
        return marker

    def _free_unmeasured(
        self, measured: set[Tuple[int, int]]
    ) -> Optional[Marker]:
        if not bool(self.get_parameter("highlight_unmeasured").value):
            return None
        image = decode_pgm(self.map_data["image_data"])
        resolution = float(self.map_data["resolution"])
        origin_x = float(self.map_data["origin_x"])
        origin_y = float(self.map_data["origin_y"])
        cells_wide = int(self.map_data["width"] * resolution / self.cell_size) + 1
        cells_high = int(self.map_data["height"] * resolution / self.cell_size) + 1
        marker = self._cube_list(1, "wifi_unmeasured")
        marker.scale.z = 0.01
        gray = ColorRGBA(r=0.35, g=0.35, b=0.35, a=0.22)
        for grid_y in range(cells_high):
            for grid_x in range(cells_wide):
                x = origin_x + (grid_x + 0.5) * self.cell_size
                y = origin_y + (grid_y + 0.5) * self.cell_size
                cell = (
                    int(x // self.cell_size),
                    int(y // self.cell_size),
                )
                if cell in measured:
                    continue
                pixel_x = int((x - origin_x) / resolution)
                pixel_y = int(self.map_data["height"]) - 1 - int(
                    (y - origin_y) / resolution
                )
                if not (
                    0 <= pixel_x < int(self.map_data["width"])
                    and 0 <= pixel_y < int(self.map_data["height"])
                ):
                    continue
                occupancy = (
                    image[pixel_y, pixel_x]
                    if int(self.map_data["negate"])
                    else 1.0 - image[pixel_y, pixel_x]
                )
                if occupancy < float(self.map_data["free_thresh"]):
                    marker.points.append(Point(x=x, y=y, z=0.01))
                    marker.colors.append(gray)
        return marker

    def publish(self) -> None:
        rows = self._metric_rows()
        measured_marker = self._cube_list(0, f"wifi_{self.metric}")
        measured = set()
        for record, value in rows:
            measured.add((int(record["cell_x"]), int(record["cell_y"])))
            x = (
                float(record["center_x"])
                if "center_x" in record
                else (int(record["cell_x"]) + 0.5) * self.cell_size
            )
            y = (
                float(record["center_y"])
                if "center_y" in record
                else (int(record["cell_y"]) + 0.5) * self.cell_size
            )
            measured_marker.points.append(Point(x=x, y=y, z=0.03))
            measured_marker.colors.append(
                self._color(value, self.minimum, self.maximum)
            )
        markers = [measured_marker]
        unmeasured = self._free_unmeasured(measured)
        if unmeasured is not None:
            markers.append(unmeasured)
        else:
            delete_marker = self._cube_list(1, "wifi_unmeasured")
            delete_marker.action = Marker.DELETE
            markers.append(delete_marker)
        self.publisher.publish(MarkerArray(markers=markers))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WifiVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
