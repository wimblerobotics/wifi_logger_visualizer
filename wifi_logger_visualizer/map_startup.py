"""Non-blocking launch gate for a saved map that may not exist at bringup.

Localization/mapping can continue while Wi-Fi waits for its configured raster
YAML and complete PGM image. Wall-clock TimerActions retry without blocking the
launch executor or requiring simulation time to advance. Once ready, the gate
releases its actions exactly once; it does not reload maps during collection.
"""
import math
import os

from launch.actions import LogInfo, OpaqueFunction, TimerAction
import yaml

from wifi_logger_visualizer.map_data import decode_pgm, load_map


def wait_for_saved_map(map_yaml_path, actions, retry_interval=5.0):
    """Return a launch action that waits until the saved raster can be loaded.

    A path must be configured even when the file is not yet saved. Both the
    YAML and complete image are validated so a partially written save does not
    trigger the nodes' constructors. Missing/unreadable/partial files are normal
    waiting states, reported at INFO level. Timers cancel on launch shutdown.
    """
    if not map_yaml_path:
        raise ValueError('map_yaml_path must name the map file to wait for')
    if isinstance(retry_interval, bool):
        raise ValueError('map_check_interval must be finite and positive')
    retry_interval = float(retry_interval)
    if not math.isfinite(retry_interval) or retry_interval <= 0:
        raise ValueError('map_check_interval must be finite and positive')
    path = os.path.abspath(os.path.expanduser(str(map_yaml_path)))
    released = False

    def check(context):
        nonlocal released
        if released:
            return []
        check_log = LogInfo(msg=f'Wi-Fi V2 checking saved map: {path}')
        try:
            captured = load_map(path)
            decode_pgm(captured['image_data'])
            if (captured['width'] <= 0 or captured['height'] <= 0
                    or not math.isfinite(captured['resolution']) or captured['resolution'] <= 0):
                raise ValueError('Map dimensions and resolution must be positive and finite')
        except (OSError, UnicodeError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
            return [
                check_log,
                LogInfo(msg=f'Wi-Fi V2 waiting for saved map {path}: {error}. '
                            f'Checking again in {retry_interval:g} seconds.'),
                TimerAction(period=retry_interval, actions=[OpaqueFunction(function=check)]),
            ]
        released = True
        return [check_log, LogInfo(msg=f'Wi-Fi V2 saved map is ready: {path}; starting enabled services.'), *actions]

    return OpaqueFunction(function=check)
