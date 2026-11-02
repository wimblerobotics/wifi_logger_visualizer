"""Expected ROS shutdown paths are quiet; other errors remain observable."""
import importlib
from unittest.mock import Mock, patch

import pytest
from rclpy.executors import ExternalShutdownException

from wifi_logger_visualizer import ros_runtime


@pytest.fixture(autouse=True)
def executor():
    with patch.object(ros_runtime, 'SingleThreadedExecutor') as factory:
        yield factory.return_value


@pytest.mark.parametrize('stop', [None, KeyboardInterrupt(), ExternalShutdownException()])
def test_normal_exit_and_expected_interrupts_release_node_and_context(stop, executor):
    node = Mock()
    factory = Mock(return_value=node)
    with patch.object(ros_runtime.rclpy, 'init') as initialize, \
         patch.object(ros_runtime.rclpy, 'spin', side_effect=stop), \
         patch.object(ros_runtime.rclpy, 'try_shutdown') as shutdown, \
         patch.object(ros_runtime.rclpy, 'shutdown') as unconditional_shutdown:
        ros_runtime.run_node(factory, args=['--ros-args'])
    initialize.assert_called_once_with(args=['--ros-args'])
    node.destroy_node.assert_called_once()
    executor.shutdown.assert_called_once()
    shutdown.assert_called_once()
    unconditional_shutdown.assert_not_called()


def test_executor_stops_before_node_and_context_are_released(executor):
    events = []
    node = Mock()
    executor.shutdown.side_effect = lambda: events.append('executor')
    node.destroy_node.side_effect = lambda: events.append('node')
    with patch.object(ros_runtime.rclpy, 'init'), \
         patch.object(ros_runtime.rclpy, 'spin'), \
         patch.object(ros_runtime.rclpy, 'try_shutdown', side_effect=lambda: events.append('context')):
        ros_runtime.run_node(lambda: node)
    assert events == ['executor', 'node', 'context']


def test_callback_failure_is_not_hidden():
    node = Mock()
    with patch.object(ros_runtime.rclpy, 'init'), \
         patch.object(ros_runtime.rclpy, 'spin', side_effect=RuntimeError('callback failure')), \
         patch.object(ros_runtime.rclpy, 'try_shutdown') as shutdown:
        with pytest.raises(RuntimeError, match='callback failure'):
            ros_runtime.run_node(lambda: node)
    node.destroy_node.assert_called_once()
    shutdown.assert_called_once()


def test_failed_construction_still_releases_context():
    with patch.object(ros_runtime.rclpy, 'init'), \
         patch.object(ros_runtime.rclpy, 'try_shutdown') as shutdown:
        with pytest.raises(ValueError, match='bad config'):
            ros_runtime.run_node(Mock(side_effect=ValueError('bad config')))
    shutdown.assert_called_once()


def test_failed_node_cleanup_still_releases_context():
    node = Mock()
    node.destroy_node.side_effect = RuntimeError('cleanup failure')
    with patch.object(ros_runtime.rclpy, 'init'), \
         patch.object(ros_runtime.rclpy, 'spin'), \
         patch.object(ros_runtime.rclpy, 'try_shutdown') as shutdown:
        with pytest.raises(RuntimeError, match='cleanup failure'):
            ros_runtime.run_node(lambda: node)
    shutdown.assert_called_once()


@pytest.mark.parametrize('error_type', [ros_runtime.rclpy_implementation.RCLError,
                                       ros_runtime.InvalidHandle])
@pytest.mark.parametrize('context_ok', [False, True])
def test_native_context_teardown_race_is_quiet_only_after_shutdown(error_type, context_ok):
    node = Mock()
    with patch.object(ros_runtime.rclpy, 'init'), \
         patch.object(ros_runtime.rclpy, 'spin', side_effect=error_type('context invalid')), \
         patch.object(ros_runtime.rclpy, 'ok', return_value=context_ok), \
         patch.object(ros_runtime.rclpy, 'try_shutdown') as shutdown:
        if context_ok:
            with pytest.raises(error_type):
                ros_runtime.run_node(lambda: node)
        else:
            ros_runtime.run_node(lambda: node)
    node.destroy_node.assert_called_once()
    shutdown.assert_called_once()


@pytest.mark.parametrize('module_name,factory_name', [
    ('wifi_logger_node', 'WifiDataCollector'),
    ('wifi_visualizer_node', 'WifiVisualizerNode'),
    ('wifi_overlay_node', 'WifiOverlayNode'),
    ('survey_node', 'WifiSurveyNode'),
])
def test_every_wifi_ros_entry_point_uses_the_shared_lifecycle(module_name, factory_name):
    module = importlib.import_module('wifi_logger_visualizer.' + module_name)
    with patch.object(module, 'run_node') as runner:
        module.main(args=['--ros-args'])
    runner.assert_called_once_with(getattr(module, factory_name), args=['--ros-args'])
