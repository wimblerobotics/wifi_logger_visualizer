"""Suite startup waits non-blockingly for a complete saved raster map."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
from launch import LaunchContext
from launch.actions import LogInfo, OpaqueFunction, TimerAction

from wifi_logger_visualizer.map_startup import wait_for_saved_map


def ready_files(tmp_path):
    path = tmp_path / 'map.yaml'
    path.write_text('image: map.pgm\nresolution: 0.25\norigin: [0, 0, 0]\n')
    (tmp_path / 'map.pgm').write_bytes(b'P5\n2 2\n255\n' + bytes([0, 255, 255, 0]))
    return path


def retry(actions, context):
    timers = [action for action in actions if isinstance(action, TimerAction)]
    assert len(timers) == 1
    return list(timers[0].actions)[0].execute(context)


def test_waits_for_yaml_image_and_complete_pixels_then_starts_once(tmp_path):
    context = LaunchContext()
    marker = LogInfo(msg='start nodes')
    gate = wait_for_saved_map(tmp_path / 'map.yaml', [marker], .1)
    waiting = gate.execute(context)
    assert marker not in waiting
    assert any(isinstance(action, LogInfo) for action in waiting)
    (tmp_path / 'map.yaml').write_text('image: map.pgm\nresolution: 0.25\n')
    waiting = retry(waiting, context)
    assert marker not in waiting
    (tmp_path / 'map.pgm').write_bytes(b'P5\n2 2\n255\n\xff')
    waiting = retry(waiting, context)
    assert marker not in waiting
    ready_files(tmp_path)
    started = retry(waiting, context)
    assert marker in started
    assert not any(isinstance(action, TimerAction) for action in started)
    assert gate.execute(context) == []


def test_existing_map_starts_without_waiting_for_sidecar(tmp_path):
    context = LaunchContext()
    marker = LogInfo(msg='start nodes')
    actions = wait_for_saved_map(ready_files(tmp_path), [marker]).execute(context)
    assert marker in actions
    assert not any(isinstance(action, TimerAction) for action in actions)


@pytest.mark.parametrize('contents', ['image: [', 'resolution: 0.25\n', '[]\n'])
def test_partial_yaml_is_a_waiting_state(tmp_path, contents):
    path = tmp_path / 'map.yaml'
    path.write_text(contents)
    assert any(isinstance(action, TimerAction) for action in
               wait_for_saved_map(path, []).execute(LaunchContext()))


@pytest.mark.parametrize('interval', [0, -1, float('nan'), float('inf'), True])
def test_invalid_polling_interval_is_a_configuration_error(tmp_path, interval):
    with pytest.raises(ValueError, match='map_check_interval'):
        wait_for_saved_map(tmp_path / 'map.yaml', [], interval)


def test_missing_configured_filename_is_a_configuration_error():
    with pytest.raises(ValueError, match='map_yaml_path'):
        wait_for_saved_map('', [])


def test_package_launch_uses_the_wait_gate(tmp_path):
    path = Path(__file__).parents[1] / 'launch/wifi_logger_visualizer.launch.py'
    spec = importlib.util.spec_from_file_location('wifi_launch_wait_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with patch.object(module, 'get_package_share_directory', return_value=str(tmp_path)):
        description = module.generate_launch_description()
    callback = next(action for action in description.entities if isinstance(action, OpaqueFunction))
    context = LaunchContext()
    context.launch_configurations.update(monitoring='true', survey='false', visualization='true',
                                         map_yaml_path=str(tmp_path / 'later.yaml'), map_check_interval='5',
                                         config_filepath=str(Path(__file__).parents[1] / 'config/wifi_logger_config.yaml'),
                                         db_path=str(tmp_path / 'wifi.db'))
    actions = callback.execute(context)
    gate = next(action for action in actions if isinstance(action, OpaqueFunction))
    assert any(isinstance(action, TimerAction) for action in gate.execute(context))


def test_launch_keeps_running_and_preserves_namespace_while_map_is_saved(tmp_path):
    from launch import LaunchDescription, LaunchService
    from launch.actions import EmitEvent, GroupAction
    from launch.events import Shutdown
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import PushRosNamespace
    state = dict(other_started=False, starts=[])
    def start_other(context):
        state['other_started'] = True
    def start_wifi(context):
        assert state['other_started']
        state['starts'].append(LaunchConfiguration('ros_namespace').perform(context))
        return [EmitEvent(event=Shutdown(reason='test complete'))]
    def save_map(context):
        ready_files(tmp_path)
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        OpaqueFunction(function=start_other),
        GroupAction([
            PushRosNamespace('robot1'),
            wait_for_saved_map(tmp_path / 'map.yaml', [OpaqueFunction(function=start_wifi)], .03),
        ]),
        TimerAction(period=.12, actions=[OpaqueFunction(function=save_map)]),
        TimerAction(period=2., actions=[EmitEvent(event=Shutdown(reason='test timeout'))]),
    ]))
    assert service.run() == 0
    assert state['starts'] == ['/robot1']
