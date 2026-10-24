from wifi_logger_visualizer.wifi_data_fetcher import (
    parse_iw_link,
    parse_iw_station,
)


def test_parse_iw_data():
    link = parse_iw_link(
        "wlp8s0",
        """
Connected to b8:7b:d4:b9:81:8f (on wlp8s0)
        SSID: livingroom
        freq: 5745.0
        signal: -49 dBm
        rx bitrate: 1080.6 MBit/s
        tx bitrate: 960.7 MBit/s
""",
    )
    station = parse_iw_station("tx retries:\t15735\ntx failed:\t2\n")
    assert link["bssid"] == "b8:7b:d4:b9:81:8f"
    assert link["signal_dbm"] == -49.0
    assert link["tx_bitrate_mbps"] == 960.7
    assert station == {"tx_retries": 15735.0, "tx_failed": 2.0}


def test_parse_disconnected():
    assert not parse_iw_link("wlan0", "Not connected.")["connected"]
