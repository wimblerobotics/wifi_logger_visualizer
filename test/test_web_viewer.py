from unittest.mock import Mock, patch

import pytest

from wifi_logger_visualizer.web_viewer import ViewerHandler


@pytest.mark.parametrize('error', [ConnectionResetError, BrokenPipeError])
def test_client_disconnect_during_response_is_ignored(error):
    handler = object.__new__(ViewerHandler)
    handler.path = '/'
    handler.log_message = Mock()
    handler.send_response = Mock()
    handler.send_header = Mock()
    handler.end_headers = Mock()
    handler.wfile = Mock()
    handler.wfile.write.side_effect = error()
    handler.do_GET()
    handler.wfile.write.assert_called_once()


def test_unexpected_request_errors_are_not_hidden():
    handler = object.__new__(ViewerHandler)
    handler.path = '/'
    handler.log_message = Mock()
    handler._handle_get = Mock(side_effect=RuntimeError('unexpected'))
    with pytest.raises(RuntimeError, match='unexpected'):
        handler.do_GET()


def test_favicon_does_not_open_database():
    handler = object.__new__(ViewerHandler)
    handler.path = '/favicon.ico'
    handler.log_message = Mock()
    handler.send_response = Mock()
    handler.end_headers = Mock()
    with patch('wifi_logger_visualizer.web_viewer.DatabaseManager') as database:
        handler.do_GET()
    database.assert_not_called()
    handler.send_response.assert_called_once_with(204)
    handler.end_headers.assert_called_once()


def test_startup_prints_effective_parameters(capsys, tmp_path):
    import json
    from wifi_logger_visualizer.web_viewer import main

    database = tmp_path / 'wifi.db'
    names = tmp_path / 'names.yaml'
    server = Mock()
    server.serve_forever.side_effect = KeyboardInterrupt
    with patch('sys.argv', ['web_viewer.py', '--db', str(database),
                           '--host', '0.0.0.0', '--port', '8089', '--cell-size', '0.5']), \
            patch('wifi_logger_visualizer.web_viewer.default_node_names_path', return_value=names), \
            patch('wifi_logger_visualizer.web_viewer.load_node_names', return_value={}), \
            patch('wifi_logger_visualizer.web_viewer.ViewerHTTPServer', return_value=server):
        main()
    output = capsys.readouterr().out
    data, _ = json.JSONDecoder().raw_decode(output.split('Wi-Fi viewer parameters:\n')[1])
    assert data == dict(db=str(database), host='0.0.0.0', port=8089,
                        cell_size=0.5, node_names=str(names), snapshot=None, use_gps=False, analysis=False)
    server.server_close.assert_called_once()


def test_idle_browser_connection_does_not_block_other_requests(tmp_path):
    import http.client
    import socket
    import threading
    from wifi_logger_visualizer.web_viewer import ViewerHTTPServer

    from concurrent.futures import ThreadPoolExecutor
    import json
    from wifi_logger_visualizer.database_manager import DatabaseManager
    from wifi_logger_visualizer.map_data import load_map

    (tmp_path / 'map.pgm').write_bytes(b'P5\n2 2\n255\n' + bytes([0, 255, 255, 0]))
    map_path = tmp_path / 'map.yaml'
    map_path.write_text('image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n'
                        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    database = DatabaseManager(str(tmp_path / 'wifi.db'))
    database.register_map(load_map(str(map_path)))
    database.close()
    idle_accepted = threading.Event()

    class Handler(ViewerHandler):
        def setup(self):
            super().setup()
            idle_accepted.set()

        def log_message(self, *args):
            pass

    Handler.db_path = str(tmp_path / 'wifi.db')
    server = ViewerHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    idle = None
    client = None
    try:
        idle = socket.create_connection(server.server_address, timeout=2)
        assert idle_accepted.wait(2)
        client = http.client.HTTPConnection(*server.server_address, timeout=2)
        client.request('GET', '/')
        response = client.getresponse()
        assert response.status == 200
        assert b'Wi-Fi coverage' in response.read()
        assert idle.fileno() != -1

        def request(path):
            connection = http.client.HTTPConnection(*server.server_address, timeout=3)
            try:
                connection.request('GET', path)
                response = connection.getresponse()
                assert response.status == 200
                return response.read()
            finally:
                connection.close()

        with ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(request, ['/', '/api/data', '/map.png?map_id=1'] * 10))
        assert len(json.loads(results[1])['maps']) == 1
        assert results[2].startswith(b'\x89PNG')
        client.close()
        client = http.client.HTTPConnection(*server.server_address, timeout=2)
        client.request('GET', '/favicon.ico')
        assert client.getresponse().status == 204
    finally:
        if client:
            client.close()
        if idle:
            idle.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


@pytest.mark.parametrize('hostname,expected', [('seggy.local', 'Seggy'),
                                              ('seggy', 'Seggy'),
                                              ('titania.example.org', 'Titania')])
def test_robot_name_from_network_hostname(hostname, expected):
    from wifi_logger_visualizer.web_viewer import robot_name, rendered_html

    with patch('wifi_logger_visualizer.web_viewer.socket.gethostname', return_value=hostname):
        assert robot_name() == expected
        page = rendered_html({})
    assert f'<title>{expected} Wi-Fi Map</title>' in page
    assert f'<h2>{expected} Wi-Fi coverage</h2>' in page
    assert f'link.download="{expected.lower()}-wifi-map.png"' in page


def test_api_preserves_nanosecond_gps_timestamp():
    import json
    handler = object.__new__(ViewerHandler)
    handler.path = '/api/data'
    handler._map_id = Mock(return_value=1)
    handler._send = Mock()
    database = Mock()
    database.get_map.return_value = {'id': 1}
    database.list_maps.return_value = []
    database.get_surveys.return_value = []
    database.get_cells.return_value = [
        {'gps_stamp_ns': 1792969795123456789, 'latitude': 33.95},
        {'gps_stamp_ns': None}, {},
    ]
    with patch('wifi_logger_visualizer.web_viewer.DatabaseManager', return_value=database), \
            patch('wifi_logger_visualizer.web_viewer.free_cell_centers', return_value=[]):
        handler._handle_get()
    cells = json.loads(handler._send.call_args.args[0])['cells']
    assert cells[0]['gps_stamp_ns'] == '1792969795123456789'
    assert cells[0]['latitude'] == 33.95
    assert cells[1]['gps_stamp_ns'] is None
    assert cells[2] == {}


def test_gps_display_is_opt_in():
    from wifi_logger_visualizer.web_viewer import rendered_html
    assert 'const useGps=false;' in rendered_html({})
    assert 'const useGps=true;' in rendered_html({}, use_gps=True)


def test_api_includes_saved_orientation(tmp_path):
    import json
    from wifi_logger_visualizer.map_data import load_map
    import yaml
    path = tmp_path / 'map.yaml'
    (tmp_path / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    path.write_text('image: map.pgm\nresolution: 0.25\norigin: [0, 0, 0]\n')
    captured = load_map(str(path))
    path.with_suffix('.orientation.yaml').write_text(yaml.safe_dump(dict(
        schema_version=1, map_content_hash=captured['content_hash'],
        orientation=dict(valid=True, reference='true_north',
                         map_x_heading_from_east_deg=30., uncertainty_deg=2.))))
    handler = object.__new__(ViewerHandler)
    handler.path = '/api/data'
    handler.db_path = str(tmp_path / 'wifi.db')
    handler._map_id = Mock(return_value=1)
    handler._send = Mock()
    database = Mock()
    database.get_map.return_value = captured
    database.list_maps.return_value = []
    database.get_surveys.return_value = []
    database.get_cells.return_value = []
    with patch('wifi_logger_visualizer.web_viewer.DatabaseManager', return_value=database):
        handler._handle_get()
    response = json.loads(handler._send.call_args.args[0])
    assert response['orientation']['valid']
    assert response['orientation']['north_rotation_in_image_deg'] == 30.


def test_keep_run_http_promotes_and_survives_next_start(tmp_path):
    import http.client
    import json
    import threading
    from wifi_logger_visualizer.web_viewer import ViewerHTTPServer
    from wifi_logger_visualizer.database_manager import DatabaseManager
    from wifi_logger_visualizer.map_data import load_map
    from test_database import write_map

    path = str(tmp_path/'wifi.db')
    db = DatabaseManager(path)
    run = db.start_run('saved', 'simulated')
    map_id = db.register_map(load_map(str(write_map(tmp_path))), run)
    db.close()
    class Handler(ViewerHandler):
        db_path = path
        read_only = False
        def log_message(self, *args):
            pass
    server = ViewerHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        conn = http.client.HTTPConnection(*server.server_address, timeout=5)
        conn.request('GET', '/api/data')
        response = conn.getresponse()
        assert response.status == 200
        assert json.loads(response.read())['run']['temporary'] == 1
        conn.request('POST', '/api/keep-run', json.dumps({'run_id': run}), {'Content-Type': 'application/json'})
        response = conn.getresponse()
        assert response.status == 200
        response.read()
        conn.close()
        db = DatabaseManager(path)
        db.start_run('live', 'simulated')
        assert db.get_map(map_id)
        assert db.get_run(run)['temporary'] == 0
        db.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


def test_read_only_viewer_rejects_promotion_before_opening_database():
    handler = object.__new__(ViewerHandler)
    handler.read_only = True
    handler.path = '/api/keep-run'
    handler.send_error = Mock()
    with patch('wifi_logger_visualizer.web_viewer.DatabaseManager') as database:
        handler.do_POST()
    database.assert_not_called()
    assert handler.send_error.call_args.args[0] == 403


def test_viewer_html_exposes_mode_to_promotion_controls():
    from wifi_logger_visualizer.web_viewer import rendered_html
    assert 'const readOnly=true;' in rendered_html({})
    assert 'const readOnly=false;' in rendered_html({}, read_only=False)


def test_get_uses_database_read_only_mode():
    handler = object.__new__(ViewerHandler)
    handler.path = '/api/data'
    handler.read_only = True
    handler.db_path = '/tmp/read-only-example.db'
    handler._map_id = Mock(return_value=1)
    handler.send_error = Mock()
    database = Mock()
    database.get_map.return_value = None
    with patch('wifi_logger_visualizer.web_viewer.DatabaseManager', return_value=database) as factory:
        handler._handle_get()
    factory.assert_called_once_with(handler.db_path, read_only=True)


def test_port_conflict_reports_actionable_message_without_traceback(tmp_path, capsys):
    import errno
    from wifi_logger_visualizer.web_viewer import main
    with patch('sys.argv', ['web_viewer.py', '--node-names', str(tmp_path/'names.yaml'), '--port', '8088']), \
         patch('wifi_logger_visualizer.web_viewer.load_node_names', return_value={}), \
         patch('wifi_logger_visualizer.web_viewer.ViewerHTTPServer',
               side_effect=OSError(errno.EADDRINUSE, 'Address already in use')):
        with pytest.raises(SystemExit) as stopped:
            main()
    message = str(stopped.value)
    assert '8088 is already in use' in message
    assert 'robot launch' in message
    assert 'port:=8089' in message and '--port 8089' in message
    assert 'Traceback' not in message + capsys.readouterr().err


def test_unrelated_server_startup_error_is_not_hidden(tmp_path):
    import errno
    from wifi_logger_visualizer.web_viewer import main
    error = OSError(errno.EACCES, 'Permission denied')
    with patch('sys.argv', ['web_viewer.py', '--node-names', str(tmp_path/'names.yaml')]), \
         patch('wifi_logger_visualizer.web_viewer.load_node_names', return_value={}), \
         patch('wifi_logger_visualizer.web_viewer.ViewerHTTPServer', side_effect=error):
        with pytest.raises(OSError) as failure:
            main()
    assert failure.value is error


def test_read_only_viewer_downloads_standard_map_bundle(tmp_path):
    import io
    import json
    import zipfile
    from wifi_logger_visualizer.database_manager import DatabaseManager
    from wifi_logger_visualizer.map_data import load_map
    from test_database import write_map
    path = str(tmp_path/'wifi.db')
    db = DatabaseManager(path)
    map_id = db.register_map(load_map(str(write_map(tmp_path))))
    db.close()
    handler = object.__new__(ViewerHandler)
    handler.path = '/api/export-map'
    handler.read_only = True
    handler.db_path = path
    data = json.dumps(dict(map_id=map_id, name='office')).encode()
    handler.headers = {'Content-Length': str(len(data)), 'Content-Type': 'application/json'}
    handler.rfile = io.BytesIO(data)
    handler._send = Mock()
    handler.send_error = Mock()
    handler.do_POST()
    handler.send_error.assert_not_called()
    body, content_type = handler._send.call_args.args
    assert content_type == 'application/zip'
    assert handler._send.call_args.kwargs['filename'] == 'office.zip'
    with zipfile.ZipFile(io.BytesIO(body)) as archive:
        assert 'office.yaml' in archive.namelist()
