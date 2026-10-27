import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
from launch import LaunchContext
from launch.actions import ExecuteProcess
import yaml

spec = importlib.util.spec_from_file_location(
    'web_viewer_launch', Path(__file__).parents[1] / 'launch/web_viewer.launch.py'
)
viewer_launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(viewer_launch)


@pytest.fixture
def context(tmp_path):
    config = tmp_path / 'config.yaml'
    config.write_text(yaml.safe_dump({
        '/**': {'ros__parameters': {'db_path': '/tmp/config.db', 'cell_size': 0.5}},
        '/**/wifi_web_viewer': {'ros__parameters': {
            'host': '127.0.0.1', 'port': 9000, 'node_names': '/tmp/names.yaml',
        }},
    }))
    ctx = LaunchContext()
    ctx.launch_configurations.update(config_filepath=str(config), db='', host='',
                                     port='', cell_size='', node_names='')
    return ctx


def command(context):
    with patch.object(viewer_launch, 'ExecuteProcess', wraps=ExecuteProcess) as process:
        viewer_launch._launch_viewer(context)
    return process.call_args.kwargs['cmd'][1:]


def test_uses_yaml_settings(context):
    assert command(context) == [
        '--db', '/tmp/config.db', '--host', '127.0.0.1', '--port', '9000',
        '--cell-size', '0.5', '--node-names', '/tmp/names.yaml',
    ]


def test_arguments_override_yaml(context):
    context.launch_configurations.update(db='/tmp/override.db', host='0.0.0.0',
                                         port='8088', cell_size='0.25', node_names='/tmp/other.yaml')
    assert command(context) == [
        '--db', '/tmp/override.db', '--host', '0.0.0.0', '--port', '8088',
        '--cell-size', '0.25', '--node-names', '/tmp/other.yaml',
    ]


@pytest.mark.parametrize('key,value', [('port', '80888'), ('port', 'no'),
                                      ('cell_size', '0'), ('cell_size', 'nan')])
def test_rejects_invalid_settings(context, key, value):
    context.launch_configurations[key] = value
    with pytest.raises(ValueError):
        command(context)
