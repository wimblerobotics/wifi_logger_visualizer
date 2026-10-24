#!/usr/bin/env python3
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from datetime import datetime, timezone
import json
import math
import re
import socket
import subprocess
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from wifi_logger_visualizer.database_manager import DatabaseManager
from wifi_logger_visualizer.map_data import cell_for_position, load_map
from wifi_logger_visualizer.wifi_data_fetcher import WiFiDataFetcher


def parse_ping(output: str) -> Dict[str, Optional[float]]:
    loss = re.search(r"([\d.]+)% packet loss", output)
    timing = re.search(
        r"(?:rtt|round-trip) min/avg/max/(?:mdev|stddev) = "
        r"[\d.]+/([\d.]+)/[\d.]+/([\d.]+) ms",
        output,
    )
    return {
        "latency_ms": float(timing.group(1)) if timing else None,
        "jitter_ms": float(timing.group(2)) if timing else None,
        "packet_loss_percent": float(loss.group(1)) if loss else None,
    }


def iperf_mbps(output: str) -> float:
    result = json.loads(output)
    summary = result.get("end", {}).get("sum_received")
    if not summary:
        summary = result.get("end", {}).get("sum")
    if not summary or "bits_per_second" not in summary:
        raise ValueError("iperf3 JSON omitted received throughput")
    return float(summary["bits_per_second"]) / 1_000_000.0


def run_ping(host: str, count: int = 10, interval: float = 0.2) -> Dict[str, Any]:
    completed = subprocess.run(
        ["ping", "-n", "-c", str(count), "-i", str(interval), "-W", "1", host],
        check=False,
        capture_output=True,
        text=True,
        timeout=max(10.0, count * interval + 5.0),
    )
    parsed = parse_ping(completed.stdout + completed.stderr)
    if completed.returncode and parsed["packet_loss_percent"] is None:
        raise RuntimeError(completed.stderr.strip() or "ping failed")
    return parsed


def run_iperf(host: str, duration: int, reverse: bool = False) -> float:
    command = [
        "iperf3",
        "-c",
        host,
        "--json",
        "--time",
        str(duration),
        "--omit",
        "1",
    ]
    if reverse:
        command.append("--reverse")
    completed = subprocess.run(
        command,
        check=True,
        capture_output=True,
        text=True,
        timeout=duration + 15,
    )
    return iperf_mbps(completed.stdout)


def perform_survey(
    target_host: str,
    duration: int,
    bssid: Optional[str],
) -> Dict[str, Any]:
    target_address = socket.gethostbyname(target_host)
    result: Dict[str, Any] = {
        "measured_at": datetime.now(timezone.utc).isoformat(),
        "target_address": target_address,
        "bssid": bssid,
        "success": False,
    }
    try:
        idle = run_ping(target_address)
        result.update(
            idle_latency_ms=idle["latency_ms"],
            idle_jitter_ms=idle["jitter_ms"],
            idle_packet_loss_percent=idle["packet_loss_percent"],
        )
        ping_process = subprocess.Popen(
            [
                "ping",
                "-n",
                "-c",
                str(max(20, duration * 5)),
                "-i",
                "0.2",
                "-W",
                "1",
                target_address,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            result["upload_mbps"] = run_iperf(target_address, duration)
        finally:
            ping_stdout, ping_stderr = ping_process.communicate(
                timeout=max(15, duration + 10)
            )
        if ping_process.returncode and "packet loss" not in ping_stdout:
            raise RuntimeError(ping_stderr.strip() or "loaded ping failed")
        loaded = parse_ping(ping_stdout + ping_stderr)
        result.update(
            loaded_latency_ms=loaded["latency_ms"],
            loaded_jitter_ms=loaded["jitter_ms"],
            loaded_packet_loss_percent=loaded["packet_loss_percent"],
        )
        result["download_mbps"] = run_iperf(
            target_address, duration, reverse=True
        )
        result["success"] = True
        result["error_text"] = None
    except (
        OSError,
        RuntimeError,
        ValueError,
        subprocess.SubprocessError,
    ) as error:
        result["error_text"] = str(error)
    return result


class WifiSurveyNode(Node):
    """Run active performance tests only after Titania has remained still."""

    def __init__(self) -> None:
        super().__init__("wifi_survey_node")
        self.declare_parameter("db_path", "~/.ros/titania_wifi.db")
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("target_host", "amdc")
        self.declare_parameter("wifi_interface", "")
        self.declare_parameter("cell_size", 0.25)
        self.declare_parameter("stationary_seconds", 3.0)
        self.declare_parameter("stationary_tolerance", 0.04)
        self.declare_parameter("revisit_interval", 300.0)
        self.declare_parameter("iperf_duration", 3)
        map_path = str(self.get_parameter("map_yaml_path").value)
        if not map_path:
            raise ValueError("map_yaml_path is required")

        self.db_path = str(self.get_parameter("db_path").value)
        database = DatabaseManager(self.db_path)
        self.map_id = database.register_map(load_map(map_path))
        database.close()
        self.target_host = str(self.get_parameter("target_host").value)
        self.cell_size = float(self.get_parameter("cell_size").value)
        self.stationary_seconds = float(
            self.get_parameter("stationary_seconds").value
        )
        self.stationary_tolerance = float(
            self.get_parameter("stationary_tolerance").value
        )
        self.revisit_interval = float(
            self.get_parameter("revisit_interval").value
        )
        self.iperf_duration = int(self.get_parameter("iperf_duration").value)
        self.fetcher = WiFiDataFetcher(
            str(self.get_parameter("wifi_interface").value)
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Bool,
            "/localization/recovery_active",
            self._recovery_callback,
            qos,
        )
        self.status_publisher = self.create_publisher(
            String, "/wifi_survey/status", qos
        )
        self.recovery_active = False
        self.positions: deque[Tuple[float, float, float]] = deque()
        self.last_survey: Dict[Tuple[int, int], float] = {}
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.future: Optional[Future] = None
        self.pending_cell: Optional[Tuple[int, int]] = None
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f"Wi-Fi survey mode targets {self.target_host}; tests begin only "
            f"after {self.stationary_seconds:.1f}s stationary"
        )

    def destroy_node(self) -> None:
        self.executor.shutdown(wait=True, cancel_futures=False)
        super().destroy_node()

    def _recovery_callback(self, message: Bool) -> None:
        self.recovery_active = message.data
        if message.data:
            self.positions.clear()

    def _pose(self) -> Optional[Tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", Time(), timeout=Duration(seconds=0.1)
            )
        except TransformException:
            return None
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _stationary(self, now: float) -> bool:
        while self.positions and now - self.positions[0][0] > self.stationary_seconds:
            self.positions.popleft()
        if len(self.positions) < 2:
            return False
        duration = self.positions[-1][0] - self.positions[0][0]
        if duration < self.stationary_seconds * 0.9:
            return False
        first_x, first_y = self.positions[0][1:]
        return all(
            math.hypot(x - first_x, y - first_y) <= self.stationary_tolerance
            for _, x, y in self.positions
        )

    def _tick(self) -> None:
        if self.future is not None and self.future.done():
            self._finish_survey()
        if self.recovery_active or self.future is not None:
            return
        pose = self._pose()
        if pose is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        self.positions.append((now, *pose))
        if not self._stationary(now):
            return
        cell = cell_for_position(*pose, self.cell_size)
        if now - self.last_survey.get(cell, -float("inf")) < self.revisit_interval:
            return
        try:
            bssid = self.fetcher.get_wifi_data().get("bssid")
        except (OSError, RuntimeError, subprocess.SubprocessError):
            bssid = None
        self.pending_cell = cell
        self.future = self.executor.submit(
            perform_survey,
            self.target_host,
            self.iperf_duration,
            bssid,
        )
        self.get_logger().info(f"Surveying cell {cell} against {self.target_host}")

    def _finish_survey(self) -> None:
        assert self.future is not None
        assert self.pending_cell is not None
        result = self.future.result()
        cell = self.pending_cell
        database = DatabaseManager(self.db_path)
        try:
            database.upsert_survey(
                self.map_id, *cell, self.target_host, result
            )
        finally:
            database.close()
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_survey[cell] = now
        message = String()
        message.data = json.dumps({"cell": cell, **result}, sort_keys=True)
        self.status_publisher.publish(message)
        if result["success"]:
            self.get_logger().info(
                f"Survey cell {cell}: upload={result['upload_mbps']:.1f} "
                f"Mb/s download={result['download_mbps']:.1f} Mb/s"
            )
        else:
            self.get_logger().error(
                f"Survey failed in cell {cell}: {result['error_text']}"
            )
        self.future = None
        self.pending_cell = None
        self.positions.clear()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WifiSurveyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
