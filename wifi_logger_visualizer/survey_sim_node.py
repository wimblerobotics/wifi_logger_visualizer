#!/usr/bin/env python3
"""Publish synthetic passive Wi-Fi measurements for an ordinary logger.

Launch through wifi_logger_visualizer.launch.py with wifi_source:=simulated,
use_sim_time:=true and either map_source:=live or a saved map. This node waits
for a fresh map-to-robot transform, anchors one access point at the first robot
position plus an offset, and generates signal/rates from subsequent distance.
It never publishes robot TF, GPS, or active iperf results and never writes SQLite.
A backwards clock jump resets the anchor, counters and random sequence.
"""
import math
import random

from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from wifi_logger_visualizer.ros_runtime import run_node
from wifi_logger_visualizer.wifi_measurement import encode_measurement


class SurveySimulator(Node):
    def __init__(self):
        super().__init__('survey_sim_node')
        defaults = dict(wifi_topic='wifi/measurements', map_frame='map', robot_frame='base_link',
                        publish_frequency=2., pose_max_age=1., seed=42, ap_offset_x=0., ap_offset_y=0.,
                        reference_signal_dbm=-35., path_loss_exponent=2.5, noise_stddev=0.,
                        disconnect_dbm=-90., max_rate_mbps=300., ssid='Simulation',
                        bssid='02:00:00:00:00:01', frequency_mhz=5200.)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = {name: self.get_parameter(name).value for name in defaults}
        for name in ('publish_frequency', 'pose_max_age', 'path_loss_exponent', 'max_rate_mbps', 'frequency_mhz'):
            if not math.isfinite(self.config[name]) or self.config[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('ap_offset_x', 'ap_offset_y', 'reference_signal_dbm', 'noise_stddev', 'disconnect_dbm'):
            if not math.isfinite(self.config[name]):
                raise ValueError(f'{name} must be finite')
        if self.config['noise_stddev'] < 0 or self.config['reference_signal_dbm'] <= self.config['disconnect_dbm']:
            raise ValueError('Invalid noise or signal limits')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(String, self.config['wifi_topic'], 10)
        self.anchor = None
        self.previous_time = None
        self.retries = self.failed = 0
        self.random = random.Random(self.config['seed'])
        self.create_timer(1. / self.config['publish_frequency'], self._tick)

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        if self.previous_time is not None and now < self.previous_time:
            self.anchor = None
            self.retries = self.failed = 0
            self.random.seed(self.config['seed'])
        self.previous_time = now
        if now <= 0:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.config['map_frame'], self.config['robot_frame'], Time())
        except TransformException:
            self.get_logger().info('Waiting for robot pose to initialize simulated Wi-Fi', throttle_duration_sec=10.)
            return
        age = (now-Time.from_msg(tf.header.stamp).nanoseconds)/1e9
        if age < 0 or age > self.config['pose_max_age']:
            return
        x, y = tf.transform.translation.x, tf.transform.translation.y
        if self.anchor is None:
            self.anchor = (x+self.config['ap_offset_x'], y+self.config['ap_offset_y'])
            self.get_logger().info(f'Simulated access point anchored at {self.anchor}')
        distance = max(1., math.hypot(x-self.anchor[0], y-self.anchor[1]))
        signal = max(-120., min(-20., self.config['reference_signal_dbm']
                     -10*self.config['path_loss_exponent']*math.log10(distance)
                     +self.random.gauss(0., self.config['noise_stddev'])))
        connected = signal >= self.config['disconnect_dbm']
        quality = max(0., min(1., (signal-self.config['disconnect_dbm']) /
                              (self.config['reference_signal_dbm']-self.config['disconnect_dbm'])))
        self.retries += round((1-quality)*4)
        self.failed += int(not connected)
        sample = dict(connected=connected, interface='simulated', ssid=self.config['ssid'] if connected else None,
                      bssid=self.config['bssid'] if connected else None, frequency_mhz=self.config['frequency_mhz'],
                      signal_dbm=signal if connected else None,
                      tx_bitrate_mbps=self.config['max_rate_mbps']*quality if connected else None,
                      rx_bitrate_mbps=self.config['max_rate_mbps']*quality if connected else None,
                      tx_retries=self.retries, tx_failed=self.failed)
        self.publisher.publish(String(data=encode_measurement(Time.from_msg(tf.header.stamp).nanoseconds, sample)))


def main(args=None):
    run_node(SurveySimulator, args=args)


if __name__ == '__main__':
    main()
