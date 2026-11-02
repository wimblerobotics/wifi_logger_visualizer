import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from wifi_logger_visualizer.gps_data import gps_metadata


def fix(**changes):
    values = dict(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0),
                               frame_id='gps_antenna'),
        status=SimpleNamespace(status=0, service=1),
        latitude=33.95, longitude=-105.33, altitude=2300.1234,
        position_covariance=[1., 0., 0., 0., 1., 0., 0., 0., 4.],
        position_covariance_type=2,
    )
    values.update(changes)
    return SimpleNamespace(**values)


def test_ordinary_fix_and_altitude_precision():
    result = gps_metadata(fix(), 11_000_000_000, 2.)
    assert result['gps_status'] == 0
    assert result['altitude'] == 2300.1234
    assert result['gps_stamp_ns'] == 10_000_000_000
    assert result['gps_frame_id'] == 'gps_antenna'
    assert json.loads(result['gps_covariance'])[8] == 4.


@pytest.mark.parametrize('now', [9_000_000_000, 12_000_000_001])
def test_future_and_stale_fixes(now):
    assert gps_metadata(fix(), now, 2.) == {}


def test_max_age_boundary():
    assert gps_metadata(fix(), 12_000_000_000, 2.)


@pytest.mark.parametrize('message', [
    None, fix(status=SimpleNamespace(status=-1, service=1)),
    fix(latitude=float('nan')), fix(longitude=181.), fix(latitude=-91.),
    fix(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id='')),
])
def test_invalid_fixes(message):
    assert gps_metadata(message, 11_000_000_000, 2.) == {}


def test_unavailable_altitude_keeps_horizontal_fix():
    result = gps_metadata(fix(altitude=float('nan')), 11_000_000_000, 2.)
    assert result['latitude'] == 33.95
    assert result['altitude'] is None
    json.dumps(result, allow_nan=False)


def test_unknown_or_invalid_covariance_is_not_reported_as_precision():
    for message in [fix(position_covariance_type=0),
                    fix(position_covariance=[float('nan')] * 9)]:
        result = gps_metadata(message, 11_000_000_000, 2.)
        assert result['gps_covariance'] is None
        assert result['gps_covariance_type'] == 0


def test_collection_persists_fresh_fix_and_continues_without_it():
    from wifi_logger_visualizer.wifi_logger_node import WifiDataCollector
    from rclpy.time import Time
    now = Time(seconds=11)
    node = SimpleNamespace(
        recovery_active=False, _pose=lambda: (1., 2.), map_id=1,
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        cell_size=.25, last_sample_time=None, visit_gap=10., last_cell=None,
        visit_id='visit', fetcher=SimpleNamespace(get_wifi_data=lambda: {'connected': True}),
        database=Mock(), status_publisher=Mock(), use_gps=True,
        latest_gps=fix(), gps_max_age=2.,
    )
    WifiDataCollector._sample(node)
    assert node.database.upsert_passive.call_args.args[-1]['altitude'] == 2300.1234
    node.latest_gps = fix(status=SimpleNamespace(status=-1, service=1))
    WifiDataCollector._sample(node)
    assert 'latitude' not in node.database.upsert_passive.call_args.args[-1]
    node.use_gps = False
    node.latest_gps = fix()
    WifiDataCollector._sample(node)
    assert 'latitude' not in node.database.upsert_passive.call_args.args[-1]
    assert node.database.upsert_passive.call_count == 3
