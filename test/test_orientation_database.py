"""Orientation snapshots remain portable and bound to captured map geometry."""
import pytest
import yaml

from wifi_logger_visualizer.database_manager import DatabaseManager
from wifi_logger_visualizer.orientation_data import load_saved_orientation
from test_orientation_data import map_and_sidecar


def fixture(tmp_path):
    captured, sidecar, document = map_and_sidecar(tmp_path, heading=30.)
    document['geometry'] = dict(resolution=.25, origin=[0, 0, 0])
    raw = '# Original diagnostics retained\n' + yaml.safe_dump(document)
    sidecar.write_text(raw)
    return captured, sidecar, document, raw


def test_snapshot_is_complete_and_portable(tmp_path):
    captured, sidecar, _, raw = fixture(tmp_path)
    db = DatabaseManager(str(tmp_path / 'wifi.db'))
    map_id = db.register_map(captured)
    row = db.get_map(map_id)
    assert row['orientation_yaml'] == raw
    assert row['orientation_valid'] == 1
    assert row['orientation_captured_at']
    assert row['orientation_saved_at'] == '2026-10-28T04:10:24+00:00'
    sidecar.unlink()
    assert load_saved_orientation(row)['north_rotation_in_image_deg'] == pytest.approx(30.)
    db.refresh_map_orientation(map_id)
    assert db.get_map(map_id) == row
    db.close()
    reader = DatabaseManager(str(tmp_path / 'wifi.db'), read_only=True)
    assert load_saved_orientation(reader.get_map(map_id))['valid']
    reader.close()


def test_late_import_and_reregistration(tmp_path):
    captured, sidecar, _, raw = fixture(tmp_path)
    sidecar.unlink()
    db = DatabaseManager(str(tmp_path / 'wifi.db'))
    map_id = db.register_map(captured)
    assert db.get_map(map_id)['orientation_valid'] is None
    sidecar.write_text(raw)
    db.refresh_map_orientation(map_id)
    assert db.get_map(map_id)['orientation_valid'] == 1
    assert db.register_map(captured) == map_id
    row = db.get_map(map_id)
    db.refresh_map_orientation(map_id)
    assert db.get_map(map_id) == row
    db.close()


@pytest.mark.parametrize('kind', ['hash', 'geometry', 'schema', 'estimate', 'malformed'])
def test_bad_replacements_preserve_valid_snapshot(tmp_path, kind):
    captured, sidecar, doc, _ = fixture(tmp_path)
    db = DatabaseManager(str(tmp_path / 'wifi.db'))
    map_id = db.register_map(captured)
    before = db.get_map(map_id)
    if kind == 'hash':
        doc['map_content_hash'] = 'another map'
    elif kind == 'geometry':
        doc['geometry']['origin'][0] = 10
    elif kind == 'schema':
        doc['schema_version'] = 2
    elif kind == 'estimate':
        doc['orientation']['valid'] = False
    sidecar.write_text('orientation: [' if kind == 'malformed' else yaml.safe_dump(doc))
    db.refresh_map_orientation(map_id)
    assert db.get_map(map_id) == before
    db.close()


def test_invalid_geometry_is_recorded_but_not_displayed(tmp_path):
    captured, sidecar, doc, _ = fixture(tmp_path)
    doc['geometry']['resolution'] = 1.
    sidecar.write_text(yaml.safe_dump(doc))
    db = DatabaseManager(str(tmp_path / 'wifi.db'))
    row = db.get_map(db.register_map(captured))
    assert row['orientation_valid'] == 0
    assert row['map_x_heading_from_east_deg'] is None
    assert 'geometry' in load_saved_orientation(row)['reason']
    assert row['orientation_yaml'] == sidecar.read_text()
    db.close()
