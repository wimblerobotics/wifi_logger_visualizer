import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from std_msgs.msg import String
from rviz_2d_overlay_msgs.msg import OverlayText
from wifi_logger_visualizer.wifi_overlay_node import WifiOverlayNode, format_status


def bridge():
    params = dict(horizontal_alignment=2, vertical_alignment=3,
                  horizontal_distance=10, vertical_distance=10,
                  width=1000, height=180, font="DejaVu Sans Mono", text_size=12.0,
                  foreground_color=[0.1, 0.1, 0.1, 1.0],
                  background_color=[1.0, 1.0, 1.0, 0.5])
    return SimpleNamespace(
        get_parameter=lambda key: SimpleNamespace(value=params[key]),
        publisher=Mock(), get_logger=Mock(return_value=Mock()),
    )


def test_converts_status_to_overlay():
    node = bridge()
    WifiOverlayNode._status(node, String(data=json.dumps({
        "connected": True, "ssid": "Office <WiFi>", "signal_dbm": -53.2,
        "tx_bitrate_mbps": 300.0, "rx_bitrate_mbps": None, "cell": [-1, 2],
    })))
    overlay = node.publisher.publish.call_args.args[0]
    assert isinstance(overlay, OverlayText)
    assert "Office &lt;WiFi&gt;" in overlay.text
    assert "-53.2 dBm" in overlay.text
    assert "TX: 300.0 Mb/s  RX: N/A" in overlay.text
    assert "[-1, 2]" in overlay.text
    assert overlay.width == 1000
    assert overlay.bg_color.a == 0.5


@pytest.mark.parametrize("payload", ["broken", "[]", "null", '"text"'])
def test_invalid_payload_is_skipped(payload):
    node = bridge()
    WifiOverlayNode._status(node, String(data=payload))
    node.publisher.publish.assert_not_called()
    node.get_logger().warning.assert_called_once()


def test_missing_fields_and_disconnected():
    assert "Wi-Fi: disconnected" in format_status({"connected": False})
    assert "Signal: N/A" in format_status({})
