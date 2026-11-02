"""Coordinate one collection run and launch hardware or topic-based acquisition.

Called by the package launch file and robot launch adapters. Source choices are
launch arguments, not YAML settings. Cleanup occurs once here, before any node
is started; nodes receive the resulting run ID. Saved maps keep the nonblocking
readiness gate, while live runs wait for occupancy grids inside the logger.
"""
import math
import os

import yaml
from launch.actions import ExecuteProcess, LogInfo
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage

from wifi_logger_visualizer.database_manager import DatabaseManager
from wifi_logger_visualizer.map_startup import wait_for_saved_map


ARGUMENTS = dict(config_filepath='', monitoring='true', survey='false', visualization='true',
                 overlay='true', web_viewer='true', db_path='', map_yaml_path='', cell_size='',
                 target_host='', map_check_interval='', map_source='saved', wifi_source='hardware',
                 map_topic='map', wifi_topic='wifi/measurements', use_sim_time='false',
                 temporary_run='auto', wifi_sim_seed='42')


def launch_suite(context):
    args = {key: LaunchConfiguration(key, default=value).perform(context) for key, value in ARGUMENTS.items()}
    logger = get_logger('wifi_survey_v2.launch')
    for name, value in args.items():
        logger.info(f'{name}={value!r}')
    def flag(name):
        value = args[name].lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return value == 'true'
    enabled = {key: flag(key) for key in ('monitoring', 'survey', 'visualization', 'overlay', 'web_viewer', 'use_sim_time')}
    if not any(enabled[key] for key in ('monitoring', 'survey', 'visualization')):
        return []
    config = os.path.abspath(os.path.expanduser(args['config_filepath']))
    with open(config) as stream:
        settings = yaml.safe_load(stream) or {}
    logger.info('Loaded parameter settings:\n' + yaml.safe_dump(settings, sort_keys=False).rstrip())
    defaults = settings.get('/**', {}).get('ros__parameters', {})
    def value(key, fallback):
        return args[key] or defaults.get(key, fallback)
    map_source, wifi_source = args['map_source'], args['wifi_source']
    if map_source not in ('saved', 'live') or wifi_source not in ('hardware', 'topic', 'simulated'):
        raise ValueError('map_source must be saved/live; wifi_source must be hardware/topic/simulated')
    if args['temporary_run'] not in ('auto', 'true', 'false'):
        raise ValueError('temporary_run must be auto, true or false')
    if enabled['survey'] and wifi_source != 'hardware':
        raise ValueError('Active surveys run real ping/iperf3; disable survey for simulated/topic input')
    path = str(value('map_yaml_path', ''))
    if map_source == 'saved' and not path:
        raise ValueError('map_yaml_path is required with map_source:=saved')
    db_path = os.path.abspath(os.path.expanduser(str(value('db_path', '~/.ros/titania_wifi.db'))))
    cell_size = float(value('cell_size', .25))
    interval = float(value('map_check_interval', 5.))
    if any(not math.isfinite(v) or v <= 0 for v in (cell_size, interval)):
        raise ValueError('cell_size and map_check_interval must be positive and finite')
    seed = int(args['wifi_sim_seed'])
    temporary = None if args['temporary_run'] == 'auto' else args['temporary_run'] == 'true'
    database = DatabaseManager(db_path)
    try:
        run_id = database.start_run(map_source, wifi_source, temporary, cell_size=cell_size)
    finally:
        database.close()
    common = dict(db_path=db_path, run_id=run_id, map_source=map_source,
                  map_yaml_path=os.path.abspath(os.path.expanduser(path)) if path else '',
                  cell_size=cell_size, use_sim_time=enabled['use_sim_time'],
                  wifi_source='hardware' if wifi_source == 'hardware' else 'topic',
                  wifi_topic=args['wifi_topic'], map_topic=args['map_topic'])
    logger.info('Effective node overrides:\n' + yaml.safe_dump(common, sort_keys=False).rstrip())
    remappings = [(name, name.lstrip('/')) for name in (
        '/tf', '/tf_static', '/gps/filtered', '/localization/recovery_active',
        '/wifi_monitor/status', '/wifi_monitor/cells', '/wifi_survey/status')]
    nodes = []
    for active, executable, name in (
        (enabled['monitoring'] or enabled['survey'] or map_source == 'live', 'wifi_logger_node.py', 'wifi_logger_node'),
        (enabled['visualization'], 'wifi_visualizer_node.py', 'wifi_visualizer_node'),
        (enabled['survey'], 'survey_node.py', 'wifi_survey_node'),
        (enabled['overlay'], 'wifi_overlay_node.py', 'wifi_overlay_node'),
    ):
        if active:
            params = dict(common)
            if executable == 'survey_node.py':
                params['target_host'] = str(value('target_host', 'amdc'))
            nodes.append(Node(package='wifi_logger_visualizer', executable=executable, name=name,
                              parameters=[config, params], remappings=remappings, output='screen'))
    if enabled['web_viewer']:
        viewer = settings.get('/**/wifi_web_viewer', {}).get('ros__parameters', {})
        command = [ExecutableInPackage(package='wifi_logger_visualizer', executable='web_viewer.py'),
                   '--read-only', '--db', db_path, '--cell-size', str(cell_size), '--host', str(viewer.get('host', '0.0.0.0')),
                   '--port', str(viewer.get('port', 8088))]
        if defaults.get('use_gps', False):
            command.append('--use-gps')
        if viewer.get('node_names'):
            command.extend(['--node-names', os.path.expanduser(str(viewer['node_names']))])
        nodes.append(ExecuteProcess(cmd=command, output='screen', additional_env={'PYTHONUNBUFFERED': '1'}))
    actions = [LogInfo(msg=f'Wi-Fi collection run {run_id}: {map_source}/{wifi_source}')]
    if wifi_source == 'simulated':
        pose_settings = settings.get('/wifi_logger_node', {}).get('ros__parameters', {})
        actions.append(Node(package='wifi_logger_visualizer', executable='survey_sim_node.py',
                            name='survey_sim_node', parameters=[config, dict(
                                use_sim_time=enabled['use_sim_time'], wifi_topic=args['wifi_topic'],
                                seed=seed, map_frame=pose_settings.get('map_frame', 'map'),
                                robot_frame=pose_settings.get('robot_frame', 'base_link'))], remappings=remappings, output='screen'))
    if map_source == 'saved':
        actions.append(wait_for_saved_map(common['map_yaml_path'], nodes, interval))
    else:
        actions.extend(nodes)
    return actions
