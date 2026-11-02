from pathlib import Path

from wifi_logger_visualizer.database_manager import DatabaseManager
from wifi_logger_visualizer.map_data import (
    cell_center,
    cell_for_position,
    free_cell_centers,
    load_map,
)


def write_map(directory: Path) -> Path:
    image = directory / "house.pgm"
    image.write_bytes(b"P5\n2 2\n255\n" + bytes([0, 255, 255, 0]))
    yaml = directory / "house.yaml"
    yaml.write_text(
        "image: house.pgm\nresolution: 0.1\norigin: [1, 2, 0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )
    return yaml


def test_map_content_identity_changes_with_image(tmp_path):
    yaml = write_map(tmp_path)
    first = load_map(str(yaml))
    (tmp_path / "house.pgm").write_bytes(
        b"P5\n2 2\n255\n" + bytes([255, 255, 255, 0])
    )
    second = load_map(str(yaml))
    assert first["content_hash"] != second["content_hash"]


def test_latest_visit_replaces_previous_window(tmp_path):
    database = DatabaseManager(str(tmp_path / "wifi.db"))
    map_id = database.register_map(load_map(str(write_map(tmp_path))))
    cell = cell_for_position(-0.01, 0.26, 0.25)
    assert cell == (-1, 1)
    center = cell_center(*cell, 0.25)
    database.upsert_passive(
        map_id,
        *cell,
        *center,
        "visit-a",
        {"connected": True, "signal_dbm": -70.0},
    )
    database.upsert_passive(
        map_id,
        *cell,
        *center,
        "visit-a",
        {"connected": True, "signal_dbm": -60.0},
    )
    first = database.get_cells(map_id)[0]
    assert first["signal_dbm_min"] == -70.0
    assert first["signal_dbm_max"] == -60.0
    assert first["sample_count"] == 2

    database.upsert_passive(
        map_id,
        *cell,
        *center,
        "visit-b",
        {"connected": True, "signal_dbm": -80.0},
    )
    replaced = database.get_cells(map_id)[0]
    assert replaced["signal_dbm_min"] == -80.0
    assert replaced["signal_dbm_max"] == -80.0
    assert replaced["signal_dbm_last"] == -80.0
    assert replaced["sample_count"] == 1


def test_free_cell_centers_use_database_cell_coordinates(tmp_path):
    map_data = load_map(str(write_map(tmp_path)))
    centers = free_cell_centers(map_data, 0.25)

    assert centers
    for x, y in centers:
        assert (x / 0.25 - 0.5).is_integer()
        assert (y / 0.25 - 0.5).is_integer()


def test_read_only_viewer_reads_while_writer_transaction_is_open(tmp_path):
    import sqlite3
    import pytest

    path = str(tmp_path / 'wifi.db')
    writer = DatabaseManager(path)
    writer.register_map(load_map(str(write_map(tmp_path))))
    writer.conn.execute('BEGIN IMMEDIATE')
    reader = DatabaseManager(path, read_only=True)
    try:
        assert len(reader.list_maps()) == 1
        with pytest.raises(sqlite3.OperationalError, match='readonly'):
            reader.conn.execute('DELETE FROM maps')
    finally:
        reader.close()
        writer.conn.rollback()
        writer.close()


def test_read_only_does_not_create_missing_database(tmp_path):
    import sqlite3
    import pytest

    path = tmp_path / 'missing.db'
    with pytest.raises(sqlite3.OperationalError):
        DatabaseManager(str(path), read_only=True)
    assert not path.exists()


def test_gps_tracks_latest_sample_and_never_survives_missing_fix(tmp_path):
    database = DatabaseManager(str(tmp_path / 'wifi.db'))
    map_id = database.register_map(load_map(str(write_map(tmp_path))))
    def store(visit, **gps):
        database.upsert_passive(map_id, 0, 0, .125, .125, visit,
                                {'signal_dbm': -60., **gps})
        return database.get_cells(map_id)[0]
    assert store('a', latitude=1., longitude=2., altitude=3.)['altitude'] == 3.
    assert store('a', latitude=4., longitude=5., altitude=6.)['latitude'] == 4.
    assert store('a')['latitude'] is None
    store('a', latitude=7.)
    assert store('b')['latitude'] is None
    database.close()


def test_new_database_contains_gps_columns_and_reopens_without_data_loss(tmp_path):
    from wifi_logger_visualizer.gps_data import GPS_COLUMNS
    path = str(tmp_path / 'wifi.db')
    database = DatabaseManager(path)
    map_id = database.register_map(load_map(str(write_map(tmp_path))))
    database.upsert_passive(map_id, 0, 0, .125, .125, 'a', {'signal_dbm': -55.})
    columns = {row[1]: row[2] for row in database.conn.execute('PRAGMA table_info(wifi_cells)')}
    assert all(columns[name] == sql_type for name, sql_type in GPS_COLUMNS.items())
    database.close()
    # The viewer can read a newly created database without writing to it.
    reader = DatabaseManager(path, read_only=True)
    assert reader.get_cells(map_id)[0]['signal_dbm_last'] == -55.
    reader.close()
    for _ in range(2):
        database = DatabaseManager(path)
        row = database.get_cells(map_id)[0]
        assert row['signal_dbm_last'] == -55.
        assert all(row[column] is None for column in GPS_COLUMNS)
        database.close()
