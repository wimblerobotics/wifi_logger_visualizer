"""Exercise real ROS clock, TF, grid and measurement transport end to end."""
import time
import json

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from wifi_logger_visualizer.database_manager import DatabaseManager
from wifi_logger_visualizer.survey_sim_node import SurveySimulator
from wifi_logger_visualizer.wifi_logger_node import WifiDataCollector


def test_simulator_collects_live_map_over_ros_topics(tmp_path):
    path = str(tmp_path/'wifi.db')
    db = DatabaseManager(path)
    run = db.start_run('live', 'simulated')
    db.close()
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', f'db_path:={path}',
                     '-p', f'run_id:={run}', '-p', 'map_source:=live', '-p', 'wifi_source:=topic',
                     '-p', 'map_update_interval:=0.2', '-p', 'update_interval:=0.1',
                     '-p', 'publish_frequency:=10.0'])
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        helper = Node('wifi_integration_driver')
        nodes.append(helper)
        sim = SurveySimulator()
        nodes.append(sim)
        logger = WifiDataCollector()
        nodes.append(logger)
        for node in nodes:
            executor.add_node(node)
        clock = helper.create_publisher(Clock, '/clock', 10)
        maps = helper.create_publisher(OccupancyGrid, 'map', 10)
        orientations = helper.create_publisher(String, 'localization/map_orientation', 10)
        broadcaster = TransformBroadcaster(helper)
        # Allow DDS discovery, then continuously advance simulation with fresh TF.
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        for tick in range(100):
            stamp = Clock()
            stamp.clock.sec = 10 + tick//10
            stamp.clock.nanosec = (tick%10)*100_000_000
            clock.publish(stamp)
            orientations.publish(String(data=json.dumps(dict(schema_version=1, valid=True, reason='',
                map_frame='map', robot_frame='base_link', published_at_ros_ns=stamp.clock.sec*1_000_000_000+stamp.clock.nanosec,
                stamp=stamp.clock.sec+stamp.clock.nanosec/1e9, reference='true_north', source='imu_and_tf',
                map_x_heading_from_east_deg=30., north_yaw_in_map_deg=60., uncertainty_deg=5.))))
            tf = TransformStamped()
            tf.header.stamp = stamp.clock
            tf.header.frame_id = 'map'
            tf.child_frame_id = 'base_link'
            tf.transform.rotation.w = 1.
            tf.transform.translation.x = tick*.05
            broadcaster.sendTransform(tf)
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.header.stamp = stamp.clock
            grid.info.width = 40 if tick < 50 else 48
            grid.info.height = 40
            grid.info.resolution = .25
            grid.info.origin.position.x = -2.
            grid.info.origin.position.y = -2.
            grid.info.origin.orientation.w = 1.
            grid.data = [0]*(grid.info.width*grid.info.height)
            if tick % 5 == 0:
                maps.publish(grid)
            until = time.monotonic()+.025
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.002)
        db = DatabaseManager(path)
        try:
            maps = db.list_maps()
            assert len(maps) == 1 and maps[0]['width'] == 48
            assert maps[0]['revision'] == 2
            row = db.get_map(maps[0]['id'])
            assert row['orientation_valid'] == 1
            assert row['map_x_heading_from_east_deg'] == 30.
            cells = db.get_cells(maps[0]['id'])
            assert len(cells) >= 5
            assert all(cell['interface'] == 'simulated' for cell in cells)
            assert len({round(cell['signal_dbm_last'], 1) for cell in cells}) > 1
        finally:
            db.close()
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.try_shutdown()
