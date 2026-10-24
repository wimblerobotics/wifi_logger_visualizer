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
