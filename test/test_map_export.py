"""Exported standard map files reload with hash-bound geographic orientation."""
import io
import zipfile

import pytest
import yaml

from wifi_logger_visualizer.database_manager import DatabaseManager
from wifi_logger_visualizer.live_map import occupancy_snapshot
from wifi_logger_visualizer.map_data import load_map, decode_pgm
from wifi_logger_visualizer.map_export import export_map_bundle
from wifi_logger_visualizer.orientation_data import live_orientation_document, load_saved_orientation
from test_collection_runs import grid
from test_database import write_map


def estimate(**changes):
    value = dict(schema_version=1, valid=True, reason='', map_frame='map', robot_frame='base_link',
                 published_at_ros_ns=11_000_000_000, stamp=10.5, reference='true_north',
                 source='imu_and_tf', session_id='test', map_x_heading_from_east_deg=30.,
                 north_yaw_in_map_deg=60., uncertainty_deg=5.)
    value.update(changes)
    return value


def test_live_export_round_trip_and_reimport_from_read_only_database(tmp_path):
    path = str(tmp_path/'wifi.db')
    db = DatabaseManager(path)
    run = db.start_run('live', 'simulated')
    snapshot = occupancy_snapshot(grid())
    document = live_orientation_document(snapshot, estimate(), 11_000_000_000, 2., 'map', 'base_link')
    map_id = db.update_live_map(run, snapshot, document)
    assert db.get_map(map_id)['orientation_valid'] == 1
    before = db.get_map(map_id)
    db.close()
    reader = DatabaseManager(path, read_only=True)
    bundle = export_map_bundle(reader, map_id, 'office')
    assert reader.get_map(map_id) == before
    assert reader.get_run(run)['temporary'] == 1
    reader.close()
    output = tmp_path/'export'
    output.mkdir()
    with zipfile.ZipFile(io.BytesIO(bundle)) as archive:
        assert set(archive.namelist()) == {'office.pgm', 'office.yaml', 'office.orientation.yaml'}
        assert all((entry.external_attr >> 16) & 0o777 == 0o644 for entry in archive.infolist())
        archive.extractall(output)
    loaded = load_map(str(output/'office.yaml'))
    assert loaded['image_data'] == snapshot['image_data']
    assert loaded['content_hash'] != snapshot['content_hash']  # renamed image in YAML
    exported = yaml.safe_load((output/'office.orientation.yaml').read_text())
    assert exported['map_content_hash'] == loaded['content_hash']
    assert exported['orientation'] == document['orientation']
    assert exported['provenance']['source_revision'] == 1
    assert load_saved_orientation(loaded)['north_rotation_in_image_deg'] == 30.
    # Standard trinary thresholds keep gray unknown rather than loading it as free.
    settings = yaml.safe_load((output/'office.yaml').read_text())
    assert settings['mode'] == 'trinary'
    unknown_occupancy = 1-decode_pgm(loaded['image_data'])[0, 0]
    assert settings['free_thresh'] < unknown_occupancy < settings['occupied_thresh']
    fresh = DatabaseManager(str(tmp_path/'reimport.db'))
    row = fresh.get_map(fresh.register_map(loaded))
    assert row['orientation_valid'] == 1
    assert row['map_x_heading_from_east_deg'] == 30.
    fresh.close()


@pytest.mark.parametrize('changes', [dict(map_frame='odom'), dict(robot_frame='wrong'),
    dict(published_at_ros_ns=1), dict(stamp=1.), dict(stamp=12.), dict(schema_version=2),
    dict(map_x_heading_from_east_deg=float('nan')), dict(uncertainty_deg=-1.),
    dict(valid=False, reason='Waiting for heading')])
def test_bad_live_orientation_cannot_be_exported_as_valid(tmp_path, changes):
    db = DatabaseManager(str(tmp_path/'wifi.db'))
    run = db.start_run('live', 'hardware')
    snapshot = occupancy_snapshot(grid())
    doc = live_orientation_document(snapshot, estimate(**changes), 11_000_000_000, 2., 'map', 'base_link')
    map_id = db.update_live_map(run, snapshot, doc)
    assert db.get_map(map_id)['orientation_valid'] == 0
    with zipfile.ZipFile(io.BytesIO(export_map_bundle(db, map_id, 'map'))) as archive:
        sidecar = yaml.safe_load(archive.read('map.orientation.yaml'))
    assert sidecar['orientation']['valid'] is False
    assert sidecar['orientation']['reason']
    db.close()


def test_orientation_can_arrive_after_last_grid_then_invalidates_on_new_grid(tmp_path):
    db = DatabaseManager(str(tmp_path/'wifi.db'))
    run = db.start_run('live', 'hardware')
    snapshot = occupancy_snapshot(grid())
    map_id = db.update_live_map(run, snapshot)
    doc = live_orientation_document(snapshot, estimate(), 11_000_000_000, 2., 'map', 'base_link')
    db.update_live_map(run, snapshot, doc)
    assert db.get_map(map_id)['orientation_valid'] == 1
    assert db.get_map(map_id)['revision'] == 1
    snapshot = occupancy_snapshot(grid(x=-1.))
    db.update_live_map(run, snapshot)
    assert db.get_map(map_id)['orientation_valid'] is None
    db.close()


def test_saved_map_exports_original_image_with_explicit_missing_orientation(tmp_path):
    db = DatabaseManager(str(tmp_path/'wifi.db'))
    snapshot = load_map(str(write_map(tmp_path)))
    map_id = db.register_map(snapshot)
    with zipfile.ZipFile(io.BytesIO(export_map_bundle(db, map_id, 'saved'))) as archive:
        assert archive.read('saved.pgm') == snapshot['image_data']
        assert not yaml.safe_load(archive.read('saved.orientation.yaml'))['orientation']['valid']
    db.close()


@pytest.mark.parametrize('name', ['', '../map', '/tmp/map', 'map.yaml', 'x\nInjected', None, 'a'*101])
def test_export_rejects_invalid_names_before_reading_database(name):
    with pytest.raises(ValueError, match='Map name'):
        export_map_bundle(None, 1, name)
