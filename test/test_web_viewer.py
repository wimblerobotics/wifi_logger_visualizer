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
                        cell_size=0.5, node_names=str(names), snapshot=None)
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
