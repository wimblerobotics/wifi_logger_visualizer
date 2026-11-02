from types import SimpleNamespace
from unittest.mock import Mock, patch
import subprocess

from wifi_logger_visualizer.subprocess_trace import node_subprocess_trace, trace_command
from wifi_logger_visualizer.survey_node import run_iperf, run_ping
from wifi_logger_visualizer.wifi_data_fetcher import WiFiDataFetcher


def test_trace_quotes_arguments_and_preserves_command():
    trace = Mock()
    command = ['ping', 'host with spaces']
    assert trace_command(command, trace) is command
    trace.assert_called_once_with("ping 'host with spaces'")


def test_parameter_can_disable_and_reenable_tracing():
    parameter = SimpleNamespace(value=True)
    logger = Mock()
    node = SimpleNamespace(get_parameter=lambda key: parameter, get_logger=lambda: logger)
    trace = node_subprocess_trace(node)
    trace('iw dev')
    parameter.value = False
    trace('ping host')
    parameter.value = True
    trace('iperf3 -c host')
    assert logger.info.call_count == 2


def test_iperf_and_ping_trace_commands():
    trace = Mock()
    with patch('subprocess.run', return_value=subprocess.CompletedProcess([], 0,
               '{"end":{"sum_received":{"bits_per_second":1000000}}}', '')) as run:
        run_iperf('host', 3, reverse=True, trace=trace)
        trace.assert_called_with('iperf3 -c host --json --time 3 --omit 1 --reverse')
        run_ping('host', trace=trace)
        trace.assert_called_with('ping -n -c 10 -i 0.2 -W 1 host')
        assert run.call_count == 2


def test_iw_detection_link_and_station_are_traced():
    trace = Mock()
    outputs = ['Interface wlan0', 'Connected to aa:bb:cc:dd:ee:ff', 'tx retries: 2']
    with patch('subprocess.run', side_effect=[subprocess.CompletedProcess([], 0, text, '') for text in outputs]):
        fetcher = WiFiDataFetcher(trace=trace)
        fetcher.get_wifi_data()
    assert [call.args[0] for call in trace.call_args_list] == [
        'iw dev', 'iw dev wlan0 link', 'iw dev wlan0 station dump',
    ]
