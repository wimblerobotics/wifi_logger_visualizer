import json

from wifi_logger_visualizer.survey_node import iperf_mbps, parse_ping


def test_parse_ping_summary():
    result = parse_ping(
        "10 packets transmitted, 10 received, 0% packet loss\n"
        "rtt min/avg/max/mdev = 1.972/36.312/145.246/55.289 ms\n"
    )
    assert result == {
        "latency_ms": 36.312,
        "jitter_ms": 55.289,
        "packet_loss_percent": 0.0,
    }


def test_parse_iperf_received_rate():
    output = json.dumps(
        {"end": {"sum_received": {"bits_per_second": 123_400_000}}}
    )
    assert iperf_mbps(output) == 123.4
