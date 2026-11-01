"""The coordinator selects inputs and performs startup cleanup exactly once."""
from pathlib import Path
from unittest.mock import patch

import pytest
from launch import LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from wifi_logger_visualizer.database_manager import DatabaseManager
from wifi_logger_visualizer.suite_launch import ARGUMENTS, launch_suite


def context(tmp_path, **overrides):
    ctx = LaunchContext()
    ctx.launch_configurations.update(ARGUMENTS, config_filepath=str(Path(__file__).parents[1]/'config/wifi_logger_config.yaml'),
                                     db_path=str(tmp_path/'wifi.db'), **overrides)
    return ctx


def test_sim_live_starts_without_saved_map_and_shares_one_run(tmp_path):
    ctx = context(tmp_path, map_source='live', wifi_source='simulated', use_sim_time='true')
    actions = launch_suite(ctx)
    assert not any(isinstance(a, OpaqueFunction) for a in actions)
    nodes = [a for a in actions if isinstance(a, Node)]
    assert len(nodes) == 4  # simulator, collector, visualizer, overlay
    db = DatabaseManager(str(tmp_path/'wifi.db'))
    assert db.conn.execute('SELECT COUNT(*) FROM collection_runs').fetchone()[0] == 1
    run = db.get_run(1)
    assert run['temporary'] and run['wifi_source'] == 'simulated'
    assert db.list_maps() == []
    db.close()


def test_saved_gate_has_simulator_outside_it(tmp_path):
    actions = launch_suite(context(tmp_path, wifi_source='simulated', map_yaml_path=str(tmp_path/'not-yet.yaml')))
    assert any(isinstance(a, Node) for a in actions)
    assert any(isinstance(a, OpaqueFunction) for a in actions)


def test_invalid_launch_does_not_delete_existing_run(tmp_path):
    db = DatabaseManager(str(tmp_path/'wifi.db'))
    run = db.start_run('live', 'simulated')
    db.close()
    with pytest.raises(ValueError, match='Active surveys'):
        launch_suite(context(tmp_path, map_source='live', wifi_source='simulated', survey='true'))
    db = DatabaseManager(str(tmp_path/'wifi.db'))
    assert db.get_run(run)
    db.close()
