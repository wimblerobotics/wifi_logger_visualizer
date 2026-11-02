"""Saved map orientation must match the database snapshot before display."""
import math
from pathlib import Path

import pytest
import yaml

from wifi_logger_visualizer.map_data import load_map
from wifi_logger_visualizer.orientation_data import load_saved_orientation


def map_and_sidecar(tmp_path, heading=0., yaw=0.):
    path = tmp_path / 'garden.yaml'
    (tmp_path / 'garden.pgm').write_bytes(b'P5\n2 2\n255\n' + bytes([255, 255, 255, 0]))
    path.write_text(f'image: garden.pgm\nresolution: 0.25\norigin: [0, 0, {yaw}]\n')
    captured = load_map(str(path))
    sidecar = path.with_suffix('.orientation.yaml')
    data = dict(schema_version=1, map_file=path.name, map_content_hash=captured['content_hash'],
                saved_at='2026-10-28T04:10:24+00:00', orientation=dict(
                    valid=True, reference='true_north', source='imu_and_tf',
                    map_x_heading_from_east_deg=heading,
                    north_yaw_in_map_deg=(90.-heading) % 360., uncertainty_deg=15.))
    sidecar.write_text(yaml.safe_dump(data))
    return captured, sidecar, data


@pytest.mark.parametrize('heading,yaw,rotation', [
    (0., 0., 0.), (90., 0., 90.), (-90., 0., 270.),
    (30., math.pi/2, 120.), (180., math.pi, 0.),
])
def test_compass_accounts_for_map_heading_and_raster_yaw(tmp_path, heading, yaw, rotation):
    captured, _, _ = map_and_sidecar(tmp_path, heading, yaw)
    result = load_saved_orientation(captured)
    assert result['valid']
    assert result['north_rotation_in_image_deg'] == pytest.approx(rotation)
    assert result['uncertainty_deg'] == 15.


def test_wrong_hash_cannot_supply_orientation_for_snapshot(tmp_path):
    captured, sidecar, data = map_and_sidecar(tmp_path)
    data['map_content_hash'] = 'different-map'
    sidecar.write_text(yaml.safe_dump(data))
    result = load_saved_orientation(captured)
    assert not result['valid']
    assert 'does not match' in result['reason']


def test_uses_captured_hash_not_a_later_overwritten_image(tmp_path):
    captured, _, _ = map_and_sidecar(tmp_path)
    (tmp_path / 'garden.pgm').write_bytes(b'new image bytes')
    assert load_saved_orientation(captured)['valid']


@pytest.mark.parametrize('updates', [
    {'valid': False, 'reason': 'Heading unavailable'},
    {'reference': 'magnetic'}, {'map_x_heading_from_east_deg': float('nan')},
    {'map_x_heading_from_east_deg': True}, {'uncertainty_deg': -1.},
    {'uncertainty_deg': float('inf')}, {'north_yaw_in_map_deg': 180.},
])
def test_invalid_saved_estimates_are_not_drawn(tmp_path, updates):
    captured, sidecar, data = map_and_sidecar(tmp_path)
    data['orientation'].update(updates)
    sidecar.write_text(yaml.safe_dump(data))
    assert not load_saved_orientation(captured)['valid']


def test_missing_invalid_and_unsupported_sidecar_are_nonfatal(tmp_path):
    captured, sidecar, data = map_and_sidecar(tmp_path)
    sidecar.unlink()
    assert not load_saved_orientation(captured)['valid']
    sidecar.write_text('orientation: [')
    assert not load_saved_orientation(captured)['valid']
    for value in ([], None, {'schema_version': 2}, {'schema_version': True}):
        sidecar.write_text(yaml.safe_dump(value))
        assert not load_saved_orientation(captured)['valid']
    assert not load_saved_orientation({})['valid']


def test_offline_sidecar_beside_database_can_replace_missing_original(tmp_path):
    original = tmp_path / 'robot'
    original.mkdir()
    captured, sidecar, _ = map_and_sidecar(original)
    offline = tmp_path / 'offline'
    offline.mkdir()
    (offline / sidecar.name).write_bytes(sidecar.read_bytes())
    sidecar.unlink()
    assert load_saved_orientation(captured, offline / 'snapshot.db')['valid']


def test_matching_offline_copy_is_used_when_original_sidecar_is_for_new_map(tmp_path):
    original = tmp_path / 'robot'
    original.mkdir()
    captured, sidecar, data = map_and_sidecar(original)
    offline = tmp_path / 'offline'
    offline.mkdir()
    (offline / sidecar.name).write_bytes(sidecar.read_bytes())
    data['map_content_hash'] = 'overwritten-map'
    sidecar.write_text(yaml.safe_dump(data))
    assert load_saved_orientation(captured, offline / 'snapshot.db')['valid']


def test_unknown_uncertainty_is_explicitly_nullable(tmp_path):
    captured, sidecar, data = map_and_sidecar(tmp_path)
    data['orientation'].pop('uncertainty_deg')
    sidecar.write_text(yaml.safe_dump(data))
    result = load_saved_orientation(captured)
    assert result['valid']
    assert result['uncertainty_deg'] is None
