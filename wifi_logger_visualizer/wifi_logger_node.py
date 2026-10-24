#!/usr/bin/env python3
import json
import subprocess
from typing import Optional, Tuple
import uuid

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from wifi_logger_visualizer.database_manager import DatabaseManager
from wifi_logger_visualizer.map_data import cell_center, cell_for_position, load_map
from wifi_logger_visualizer.wifi_data_fetcher import WiFiDataFetcher


class WifiDataCollector(Node):
    """Persist lightweight Wi-Fi observations at trustworthy map poses."""

    def __init__(self) -> None:
        super().__init__("wifi_logger_node")
        self.declare_parameter("db_path", "~/.ros/titania_wifi.db")
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("wifi_interface", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("cell_size", 0.25)
        self.declare_parameter("update_interval", 1.0)
        self.declare_parameter("pose_max_age", 1.0)
        self.declare_parameter("visit_gap", 10.0)

        map_path = str(self.get_parameter("map_yaml_path").value)
        if not map_path:
            raise ValueError("map_yaml_path is required")
        self.cell_size = float(self.get_parameter("cell_size").value)
        if self.cell_size <= 0:
            raise ValueError("cell_size must be positive")

        self.database = DatabaseManager(str(self.get_parameter("db_path").value))
        self.map_id = self.database.register_map(load_map(map_path))
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.pose_max_age = float(self.get_parameter("pose_max_age").value)
        self.visit_gap = float(self.get_parameter("visit_gap").value)
        self.fetcher = WiFiDataFetcher(
            str(self.get_parameter("wifi_interface").value)
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        recovery_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Bool,
            "/localization/recovery_active",
            self._recovery_callback,
            recovery_qos,
        )
        self.status_publisher = self.create_publisher(
            String, "/wifi_monitor/status", 10
        )
        self.recovery_active = False
        self.last_cell: Optional[Tuple[int, int]] = None
        self.last_sample_time: Optional[Time] = None
        self.visit_id = str(uuid.uuid4())
        self.create_timer(
            float(self.get_parameter("update_interval").value),
            self._sample,
        )
        self.get_logger().info(
            f"Passive Wi-Fi monitoring map_id={self.map_id}, "
            f"interface={self.fetcher.wifi_interface}, cell_size={self.cell_size} m"
        )

    def destroy_node(self) -> None:
        self.database.close()
        super().destroy_node()

    def _recovery_callback(self, message: Bool) -> None:
        self.recovery_active = message.data

    def _pose(self) -> Optional[Tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Skipping Wi-Fi sample without {self.map_frame} to "
                f"{self.robot_frame} transform: {error}",
                throttle_duration_sec=10.0,
            )
            return None
        stamp = Time.from_msg(transform.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age > self.pose_max_age:
            self.get_logger().warning(
                f"Skipping Wi-Fi sample because pose is {age:.1f}s old",
                throttle_duration_sec=10.0,
            )
            return None
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _sample(self) -> None:
        if self.recovery_active:
            return
        pose = self._pose()
        if pose is None:
            return
        now = self.get_clock().now()
        cell = cell_for_position(*pose, self.cell_size)
        gap = (
            (now - self.last_sample_time).nanoseconds / 1e9
            if self.last_sample_time is not None
            else 0.0
        )
        if cell != self.last_cell or gap > self.visit_gap:
            self.visit_id = str(uuid.uuid4())
        try:
            sample = self.fetcher.get_wifi_data()
        except (OSError, RuntimeError, subprocess.SubprocessError) as error:
            self.get_logger().error(f"Unable to read Wi-Fi state: {error}")
            return

        center = cell_center(*cell, self.cell_size)
        self.database.upsert_passive(
            self.map_id,
            *cell,
            *center,
            self.visit_id,
            sample,
        )
        self.last_cell = cell
        self.last_sample_time = now
        status = {
            "map_id": self.map_id,
            "cell": cell,
            "position": pose,
            **sample,
        }
        message = String()
        message.data = json.dumps(status, sort_keys=True)
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WifiDataCollector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
