"""Run-scoped ownership protects keepers and deletes temporary data completely."""
from types import SimpleNamespace as NS

import pytest

from wifi_logger_visualizer.database_manager import DatabaseManager
from wifi_logger_visualizer.live_map import occupancy_snapshot
from wifi_logger_visualizer.map_data import decode_pgm, load_map
from test_database import write_map


def grid(width=2, height=2, x=0., data=None):
    return NS(header=NS(frame_id='map'), info=NS(width=width, height=height, resolution=.25,
              origin=NS(position=NS(x=x, y=0.), orientation=NS(x=0., y=0., z=0., w=1.))),
              data=data if data is not None else [0, 100, -1, 50])


def add_cell(db, map_id):
    db.upsert_passive(map_id, 0, 0, .125, .125, 'visit', dict(connected=True, signal_dbm=-40.))


def test_cleanup_cascades_but_promotion_preserves_all_data(tmp_path):
    db = DatabaseManager(str(tmp_path/'wifi.db'))
    snapshot = load_map(str(write_map(tmp_path)))
    first = db.start_run('saved', 'simulated')
    first_map = db.register_map(snapshot, first)
    add_cell(db, first_map)
    # Include survey data in the ownership/cascade test, without running iperf.
    db.conn.execute('INSERT INTO wifi_surveys(map_id,cell_x,cell_y,measured_at,target_host,success) VALUES(?,0,0,?,?,1)',
                    (first_map, 'now', 'test'))
    db.conn.commit()
    assert db.keep_run(first)
    disposable = db.start_run('saved', 'simulated')
    disposable_map = db.register_map(snapshot, disposable)
    add_cell(db, disposable_map)
    assert disposable_map != first_map
    third = db.start_run('live', 'hardware')
    assert db.get_run(disposable) is None
    assert db.get_map(disposable_map) is None
    assert not db.get_cells(disposable_map)
    assert db.get_map(first_map)
    assert len(db.get_cells(first_map)) == len(db.get_surveys(first_map)) == 1
    assert db.get_run(third)['temporary'] == 1
    assert not db.keep_run(disposable)
    db.close()


def test_ending_and_viewing_do_not_cleanup(tmp_path):
    path = str(tmp_path/'wifi.db')
    db = DatabaseManager(path)
    run = db.start_run('live', 'simulated')
    map_id = db.update_live_map(run, occupancy_snapshot(grid()))
    db.close()
    db = DatabaseManager(path)
    assert db.get_run(run)['temporary']
    assert db.get_map(map_id)
    assert db.keep_run(run)
    db.start_run()
    assert db.get_map(map_id)
    db.close()


def test_live_map_replaces_single_row_and_preserves_cells(tmp_path):
    db = DatabaseManager(str(tmp_path/'wifi.db'))
    run = db.start_run('live', 'hardware')
    first = occupancy_snapshot(grid())
    map_id = db.update_live_map(run, first)
    add_cell(db, map_id)
    assert db.update_live_map(run, first) == map_id
    assert db.get_map(map_id)['revision'] == 1
    updated = occupancy_snapshot(grid(3, 2, -1., [0]*6))
    assert db.update_live_map(run, updated) == map_id
    row = db.get_map(map_id)
    assert row['revision'] == 2 and row['width'] == 3 and row['origin_x'] == -1.
    assert len(db.list_maps()) == 1 and len(db.get_cells(map_id)) == 1
    db.close()


def test_grid_conversion_preserves_unknown_and_flips_y():
    snapshot = occupancy_snapshot(grid())
    pixels = decode_pgm(snapshot['image_data'])
    assert pixels[0, 0] == pytest.approx(205/255)
    assert pixels[1, 0] == pytest.approx(254/255)
    assert pixels[1, 1] == 0.


@pytest.mark.parametrize('kind', ['pixels', 'frame', 'resolution', 'rotation'])
def test_invalid_grid_cannot_replace_snapshot(kind):
    message = grid()
    if kind == 'pixels':
        message.data = [101]*4
    elif kind == 'frame':
        message.header.frame_id = 'odom'
    elif kind == 'resolution':
        message.info.resolution = 0.
    else:
        message.info.origin.orientation.z = .5
    with pytest.raises(ValueError):
        occupancy_snapshot(message)


def test_concurrent_nodes_share_one_saved_map_within_run(tmp_path):
    from concurrent.futures import ThreadPoolExecutor
    path = str(tmp_path/'wifi.db')
    db = DatabaseManager(path)
    run = db.start_run()
    db.close()
    snapshot = load_map(str(write_map(tmp_path)))
    def register(_):
        writer = DatabaseManager(path)
        try:
            return writer.register_map(snapshot, run)
        finally:
            writer.close()
    with ThreadPoolExecutor(max_workers=4) as pool:
        ids = list(pool.map(register, range(8)))
    assert len(set(ids)) == 1
    db = DatabaseManager(path)
    assert len(db.list_maps()) == 1
    db.close()
