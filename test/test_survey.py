import json
import socket
import subprocess

import pytest
from unittest.mock import patch

from wifi_logger_visualizer.survey_node import iperf_mbps, parse_ping, perform_survey, run_iperf


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


def test_dns_failure_returns_failed_survey():
    with patch(
        "wifi_logger_visualizer.survey_node.socket.gethostbyname",
        side_effect=socket.gaierror(-3, "Temporary failure in name resolution"),
    ), patch("wifi_logger_visualizer.survey_node.run_ping") as ping:
        result = perform_survey("amdc", 3, "aa:bb:cc:dd:ee:ff")
    assert result["success"] is False
    assert result["target_address"] is None
    assert result["bssid"] == "aa:bb:cc:dd:ee:ff"
    assert "Temporary failure in name resolution" in result["error_text"]
    assert result["measured_at"]
    ping.assert_not_called()


@pytest.mark.parametrize("output,stderr,code,expected", [
    ('{"error": "server is busy"}', "", 1, "server is busy"),
    ("", "connection refused", 1, "connection refused"),
    ("unexpected output", "", 1, "unexpected output"),
    ('{"error": "test failed"}', "", 0, "test failed"),
])
def test_iperf_reports_diagnostic(output, stderr, code, expected):
    completed = subprocess.CompletedProcess([], code, output, stderr)
    with patch("wifi_logger_visualizer.survey_node.subprocess.run", return_value=completed):
        with pytest.raises(RuntimeError, match=expected) as error:
            run_iperf("172.17.1.130", 3, reverse=True)
    assert "download to 172.17.1.130" in str(error.value)


def test_iperf_reverse_success():
    completed = subprocess.CompletedProcess(
        [], 0, '{"end":{"sum_received":{"bits_per_second":577438178.2762268}}}', ""
    )
    with patch("wifi_logger_visualizer.survey_node.subprocess.run", return_value=completed) as run:
        assert run_iperf("172.17.1.130", 3, reverse=True) == pytest.approx(577.438178)
    assert "--reverse" in run.call_args.args[0]
