#!/usr/bin/env python3
import json
import math
import subprocess
import sqlite3
from typing import Optional, Tuple
import uuid

from rclpy.duration import Duration
from rclpy.node import Node
from wifi_logger_visualizer.ros_runtime import run_node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import OccupancyGrid
from wifi_logger_visualizer.live_map import occupancy_snapshot
from wifi_logger_visualizer.orientation_data import live_orientation_document
from wifi_logger_visualizer.wifi_measurement import decode_measurement
from tf2_ros import Buffer, TransformException, TransformListener

from wifi_logger_visualizer.subprocess_trace import node_subprocess_trace
from wifi_logger_visualizer.database_manager import DatabaseManager
from wifi_logger_visualizer.map_data import cell_center, cell_for_position, load_map
from wifi_logger_visualizer.wifi_data_fetcher import WiFiDataFetcher
from wifi_logger_visualizer.gps_data import gps_metadata


class WifiDataCollector(Node):
    """Persist lightweight Wi-Fi observations at trustworthy map poses."""

    def __init__(self) -> None:
        super().__init__("wifi_logger_node")
        self.declare_parameter("db_path", "~/.ros/titania_wifi.db")
        self.declare_parameter("map_yaml_path", "")
        for name, value in dict(run_id=0, map_source='saved', map_topic='map',
                                map_update_interval=2., wifi_source='hardware',
                                wifi_topic='wifi/measurements', wifi_max_age=2.,
                                orientation_topic='localization/map_orientation', orientation_max_age=2.).items():
            self.declare_parameter(name, value)
        self.run_id = int(self.get_parameter('run_id').value)
        self.map_source = str(self.get_parameter('map_source').value)
        self.wifi_source = str(self.get_parameter('wifi_source').value)
        if self.map_source not in ('saved', 'live') or self.wifi_source not in ('hardware', 'topic'):
            raise ValueError('Invalid map_source or wifi_source')
        self.wifi_max_age = float(self.get_parameter('wifi_max_age').value)
        self.map_update_interval = float(self.get_parameter('map_update_interval').value)
        if any(not math.isfinite(v) or v <= 0 for v in (self.wifi_max_age, self.map_update_interval)):
            raise ValueError('Input age and map update interval must be positive and finite')
        self.latest_wifi = None
        self.last_wifi_stamp = None
        self.last_clock_ns = None
        self.pending_map = None
        self.latest_orientation = None
        self.orientation_max_age = float(self.get_parameter("orientation_max_age").value)
        if not math.isfinite(self.orientation_max_age) or self.orientation_max_age <= 0:
            raise ValueError("orientation_max_age must be finite and positive")
        self.declare_parameter("trace_subprocess_runs", True)
        self._trace_subprocess = node_subprocess_trace(self)
        self.declare_parameter("wifi_interface", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("cell_size", 0.25)
        self.declare_parameter("update_interval", 1.0)
        self.declare_parameter("pose_max_age", 1.0)
        self.declare_parameter("visit_gap", 10.0)
        self.declare_parameter("use_gps", False)
        self.declare_parameter("gps_topic", "/gps/filtered")
        self.declare_parameter("gps_max_age", 2.0)
        self.use_gps = bool(self.get_parameter("use_gps").value)
        self.gps_max_age = float(self.get_parameter("gps_max_age").value)
        if not math.isfinite(self.gps_max_age) or self.gps_max_age <= 0:
            raise ValueError("gps_max_age must be finite and positive")
        self.latest_gps = None
        self.gps_subscription = None
        if self.use_gps:
            self.gps_subscription = self.create_subscription(
                NavSatFix, str(self.get_parameter("gps_topic").value),
                self._gps_callback, qos_profile_sensor_data,
            )

        map_path = str(self.get_parameter("map_yaml_path").value)
        if self.map_source == "saved" and not map_path:
            raise ValueError("map_yaml_path is required")
        self.cell_size = float(self.get_parameter("cell_size").value)
        if self.cell_size <= 0:
            raise ValueError("cell_size must be positive")

        self.database = DatabaseManager(str(self.get_parameter("db_path").value))
        if not self.run_id and self.map_source == 'live':
            raise ValueError('Live collection must be started through the suite launch')
        self.map_id = (self.database.register_map(load_map(map_path), self.run_id)
                       if self.map_source == 'saved' else None)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.pose_max_age = float(self.get_parameter("pose_max_age").value)
        self.visit_gap = float(self.get_parameter("visit_gap").value)
        self.fetcher = None
        if self.wifi_source == 'hardware':
            self.fetcher = WiFiDataFetcher(str(self.get_parameter('wifi_interface').value), trace=self._trace_subprocess)
        else:
            self.create_subscription(String, str(self.get_parameter('wifi_topic').value), self._wifi_callback, 10)
        if self.map_source == 'live':
            self.create_subscription(String, str(self.get_parameter('orientation_topic').value),
                                     self._orientation_callback, 10)
            self.create_subscription(OccupancyGrid, str(self.get_parameter('map_topic').value),
                                     self._map_callback, qos_profile_sensor_data)
            self.create_timer(self.map_update_interval, self._store_live_map)

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
        self.create_timer(5.0, self._refresh_map_orientation)
        self.get_logger().info(
            f"Passive Wi-Fi monitoring map_id={self.map_id}, "
            f"interface={self.fetcher.wifi_interface if self.fetcher else 'topic'}, cell_size={self.cell_size} m"
        )

    def _wifi_callback(self, message):
        try:
            stamp, sample = decode_measurement(message.data)
        except (ValueError, TypeError) as error:
            self.get_logger().warning(f'Ignoring invalid Wi-Fi measurement: {error}', throttle_duration_sec=10.)
            return
        now = self.get_clock().now().nanoseconds
        if stamp <= now and (now-stamp)/1e9 <= self.wifi_max_age:
            if self.latest_wifi is None or stamp > self.latest_wifi[0]:
                self.latest_wifi = (stamp, sample)

    def _map_callback(self, message):
        self.pending_map = message

    def _orientation_callback(self, message):
        try:
            estimate = json.loads(message.data)
            if not isinstance(estimate, dict):
                raise ValueError('Expected an orientation object')
            self.latest_orientation = estimate
        except (ValueError, TypeError) as error:
            self.latest_orientation = None
            self.get_logger().warning(f'Ignoring malformed live orientation: {error}', throttle_duration_sec=10.)

    def _store_live_map(self):
        message, self.pending_map = self.pending_map, None
        try:
            snapshot = (occupancy_snapshot(message, self.map_frame) if message is not None
                        else self.database.get_map(self.map_id) if self.map_id is not None else None)
            if snapshot is None:
                return
            document = live_orientation_document(snapshot, self.latest_orientation,
                         self.get_clock().now().nanoseconds, self.orientation_max_age,
                         self.map_frame, self.robot_frame)
            self.map_id = self.database.update_live_map(self.run_id, snapshot, document)
        except (ValueError, sqlite3.Error) as error:
            self.get_logger().warning(f'Could not capture live map: {error}', throttle_duration_sec=10.)

    def _refresh_map_orientation(self) -> None:
        """Import metadata saved after collection began, even without pose data."""
        if self.map_source == 'live' or self.map_id is None:
            return
        try:
            self.database.refresh_map_orientation(self.map_id)
        except sqlite3.Error as error:
            self.get_logger().warning(f"Could not refresh saved map orientation: {error}")

    def destroy_node(self) -> None:
        self.database.close()
        super().destroy_node()

    def _recovery_callback(self, message: Bool) -> None:
        self.recovery_active = message.data

    def _gps_callback(self, message: NavSatFix) -> None:
        # Cache invalid fixes too, so loss of fix immediately invalidates GPS.
        self.latest_gps = message

    def _pose(self, stamp=None) -> Optional[Tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                stamp if stamp is not None else Time(),
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
        if age < 0 or age > self.pose_max_age:
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
        topic_stamp = None
        if getattr(self, 'wifi_source', 'hardware') == 'topic':
            now_ns = self.get_clock().now().nanoseconds
            if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
                self.latest_wifi = None
                self.last_wifi_stamp = None
                self.last_sample_time = None
                self.last_cell = None
            self.last_clock_ns = now_ns
            if self.map_id is None or self.latest_wifi is None:
                return
            topic_stamp, topic_sample = self.latest_wifi
            if topic_stamp == self.last_wifi_stamp or not 0 <= (now_ns-topic_stamp)/1e9 <= self.wifi_max_age:
                return
            pose = self._pose(Time(nanoseconds=topic_stamp, clock_type=self.get_clock().clock_type))
        else:
            if getattr(self, 'map_id', None) is None:
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
            sample = dict(topic_sample) if topic_stamp is not None else self.fetcher.get_wifi_data()
        except (OSError, RuntimeError, subprocess.SubprocessError) as error:
            self.get_logger().error(f"Unable to read Wi-Fi state: {error}")
            return

        if self.use_gps:
            sample.update(gps_metadata(
                self.latest_gps, self.get_clock().now().nanoseconds,
                self.gps_max_age,
            ))
        center = cell_center(*cell, self.cell_size)
        self.database.upsert_passive(
            self.map_id,
            *cell,
            *center,
            self.visit_id,
            sample,
        )
        self.last_wifi_stamp = topic_stamp
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
    run_node(WifiDataCollector, args=args)


if __name__ == "__main__":
    main()
