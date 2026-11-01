"""Topic measurements must carry valid, precise ROS stamps and radio fields."""
import json

import pytest
from wifi_logger_visualizer.wifi_measurement import encode_measurement, decode_measurement


def sample():
    return dict(connected=True, interface='simulated', ssid='Simulation', bssid='02:00:00:00:00:01',
                frequency_mhz=5200., signal_dbm=-40., tx_bitrate_mbps=200., rx_bitrate_mbps=200.,
                tx_retries=3, tx_failed=0)


def test_contract_preserves_stamp_and_fields():
    stamp = 1793488308889809071
    assert decode_measurement(encode_measurement(stamp, sample())) == (stamp, sample())


@pytest.mark.parametrize('key,value', [('signal_dbm', float('nan')), ('tx_retries', -1),
                                      ('connected', 1), ('bssid', 'not a mac'), ('ssid', 42)])
def test_invalid_radio_fields_are_rejected(key, value):
    fields = sample()
    fields[key] = value
    text = json.dumps(dict(schema_version=1, stamp_ns='123', sample=fields))
    with pytest.raises(ValueError):
        decode_measurement(text)


@pytest.mark.parametrize('stamp', ['0', '-1', 'nan', 123, None])
def test_invalid_stamps_are_rejected(stamp):
    with pytest.raises(ValueError):
        decode_measurement(json.dumps(dict(schema_version=1, stamp_ns=stamp, sample=sample())))


def test_topic_collection_uses_measurement_time_once_and_rejects_stale_data():
    from types import SimpleNamespace as NS
    from unittest.mock import Mock
    from rclpy.clock_type import ClockType
    from rclpy.time import Time
    from wifi_logger_visualizer.wifi_logger_node import WifiDataCollector
    now = Time(seconds=11, clock_type=ClockType.ROS_TIME)
    clock = NS(now=lambda: now, clock_type=ClockType.ROS_TIME)
    pose = Mock(return_value=(1., 2.))
    node = NS(recovery_active=False, _pose=pose, map_id=1, wifi_source='topic',
              latest_wifi=(10_500_000_000, sample()), last_wifi_stamp=None, last_clock_ns=None,
              wifi_max_age=2., get_clock=lambda: clock, cell_size=.25, last_sample_time=None,
              visit_gap=10., last_cell=None, visit_id='visit', database=Mock(),
              status_publisher=Mock(), use_gps=False)
    WifiDataCollector._sample(node)
    assert pose.call_args.args[0].nanoseconds == 10_500_000_000
    assert node.database.upsert_passive.call_count == 1
    WifiDataCollector._sample(node)
    assert node.database.upsert_passive.call_count == 1
    node.latest_wifi = (8_000_000_000, sample())
    WifiDataCollector._sample(node)
    assert node.database.upsert_passive.call_count == 1
    # Clock rewind clears old samples and visit state instead of logging future data.
    now = Time(seconds=5, clock_type=ClockType.ROS_TIME)
    WifiDataCollector._sample(node)
    assert node.latest_wifi is None and node.last_sample_time is None
    assert node.database.upsert_passive.call_count == 1
