"""Contract, validation and real loopback socket tests; no ROS/hardware required."""
import http.client
import json
import math
import os
import socket
import threading
import time
import unittest
from unittest.mock import patch

from mobile_gateway.server import TelemetryServer, token_from_environment
from mobile_gateway.state import TelemetryState

TOKEN = 'test-only-token-' + 'a' * 32


class StateTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.state = TelemetryState(clock=lambda: self.now)

    def odometry(self, **changes):
        args = dict(frame_id='odom', child_frame_id='base_link', x=1., y=2.,
                    qx=0., qy=0., qz=math.sin(.3/2), qw=math.cos(.3/2),
                    linear_x=.2, linear_y=0., angular_z=.1)
        args.update(changes)
        self.state.odometry(**args)

    def test_missing_and_monotonic_stale_boundary(self):
        self.assertEqual(self.state.snapshot(), {'api_version': 1, 'source': 'ros2',
            'odometry': {'available': False}, 'scan': {'available': False}})
        self.odometry()
        self.now += 2
        self.assertTrue(self.state.snapshot()['odometry']['available'])
        self.now += .001
        stale = self.state.snapshot()['odometry']
        self.assertFalse(stale['available'])
        self.assertNotIn('x', stale)
        self.assertAlmostEqual(stale['age_seconds'], 2.001)

    def test_fixture_source_is_explicit(self):
        self.assertEqual(TelemetryState(source="fixture").snapshot()["source"], "fixture")

    def test_quaternion_normalization_and_heading(self):
        self.odometry(qz=1e300, qw=1e300)
        self.assertAlmostEqual(self.state.snapshot()['odometry']['heading'], math.pi / 2)

    def test_invalid_odometry_invalidates_previous_sample(self):
        for changes in ({'x': math.nan}, {'angular_z': math.inf},
                        {'qx': 0., 'qy': 0., 'qz': 0., 'qw': 0.}):
            with self.subTest(changes=changes):
                self.odometry()
                self.odometry(**changes)
                self.assertFalse(self.state.snapshot()['odometry']['available'])
                json.dumps(self.state.snapshot(), allow_nan=False)

    def test_scan_filters_nonfinite_and_out_of_range(self):
        self.state.scan(frame_id='laser', ranges=[math.nan, math.inf, -.1, .1, 2., 10.1],
                        range_min=.1, range_max=10.)
        scan = self.state.snapshot()['scan']
        self.assertTrue(scan['available'])
        self.assertEqual(scan['nearest_m'], .1)

    def test_fresh_scan_without_returns_is_available_and_null(self):
        for ranges in ([], [math.inf, math.nan, -1., 11.]):
            self.state.scan(frame_id='laser', ranges=ranges, range_min=.1, range_max=10.)
            scan = self.state.snapshot()['scan']
            self.assertTrue(scan['available'])
            self.assertIsNone(scan['nearest_m'])
        self.now += 2.01
        self.assertFalse(self.state.snapshot()['scan']['available'])

    def test_invalid_scan_bounds(self):
        for bounds in ((2., 1.), (math.nan, 10.), (.1, math.inf)):
            self.state.scan(frame_id='laser', ranges=[1.], range_min=bounds[0], range_max=bounds[1])
            self.assertFalse(self.state.snapshot()['scan']['available'])


class SocketTests(unittest.TestCase):
    def setUp(self):
        self.server = TelemetryServer(('127.0.0.1', 0), TelemetryState(), TOKEN,
                                      socket_timeout=.2)
        self.worker = threading.Thread(target=self.server.serve_forever,
                                       kwargs={'poll_interval': .01}, daemon=True)
        self.worker.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.worker.join(2)

    def request(self, method='GET', path='/v1/telemetry', auth='Bearer ' + TOKEN):
        conn = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=2)
        try:
            conn.request(method, path, headers={} if auth is None else {'Authorization': auth})
            response = conn.getresponse()
            return response.status, dict(response.getheaders()), response.read()
        finally:
            conn.close()

    def test_authenticated_response(self):
        status, headers, body = self.request()
        self.assertEqual(status, 200)
        self.assertEqual(headers['Cache-Control'], 'no-store')
        self.assertEqual(headers['Connection'], 'close')
        self.assertEqual(json.loads(body)['api_version'], 1)

    def test_authentication_required_and_exact(self):
        for auth in (None, '', TOKEN, 'Bearer wrong', 'bearer ' + TOKEN):
            with self.subTest(auth=auth):
                status, headers, body = self.request(auth=auth)
                self.assertEqual(status, 401)
                self.assertIn('WWW-Authenticate', headers)
                self.assertNotIn(TOKEN.encode(), body)

    def test_duplicate_authorization_rejected(self):
        conn = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=2)
        try:
            conn.putrequest('GET', '/v1/telemetry')
            conn.putheader('Authorization', 'Bearer ' + TOKEN)
            conn.putheader('Authorization', 'Bearer ' + TOKEN)
            conn.endheaders()
            self.assertEqual(conn.getresponse().status, 401)
        finally:
            conn.close()

    def test_methods_and_paths(self):
        for method in ('POST', 'PUT', 'DELETE', 'OPTIONS', 'HEAD', 'PATCH', 'CUSTOM'):
            status, headers, _ = self.request(method)
            self.assertEqual(status, 405, method)
            self.assertEqual(headers['Allow'], 'GET')
        for path in ('/cmd_vel', '/v1/telemetry?token=bad', '/v1/telemetry/', '/v1/map'):
            self.assertEqual(self.request(path=path)[0], 404)

    def test_disconnect_and_partial_request_do_not_stop_server(self):
        with socket.create_connection(self.server.server_address, timeout=2) as client:
            client.sendall(('GET /v1/telemetry HTTP/1.1\r\nAuthorization: Bearer ' + TOKEN + '\r\n\r\n').encode())
        with socket.create_connection(self.server.server_address, timeout=2) as client:
            client.sendall(b'GET /v1/telemetry HTTP/1.1\r\n')
            self.assertEqual(client.recv(1), b'')  # bounded idle socket timeout
        self.assertEqual(self.request()[0], 200)

    def test_slow_drip_deadline_releases_client_slot(self):
        server = TelemetryServer(('127.0.0.1', 0), TelemetryState(), TOKEN,
                                 max_clients=1, socket_timeout=1., request_timeout=.15)
        worker = threading.Thread(target=server.serve_forever,
                                  kwargs={'poll_interval': .01}, daemon=True)
        worker.start()
        try:
            with socket.create_connection(server.server_address, timeout=1) as client:
                started = time.monotonic()
                # Every byte arrives before the inactivity timeout, but the
                # total deadline must still terminate this unfinished request.
                for byte in b'GET /v1/telemetry HTTP/1.1\r\n':
                    try:
                        client.sendall(bytes([byte]))
                    except OSError:
                        break
                    time.sleep(.03)
                try:
                    self.assertEqual(client.recv(1), b'')
                except ConnectionResetError:
                    pass  # TCP may reset when closing with unread request bytes.
                self.assertLess(time.monotonic() - started, .8)
            # Deadline cleanup must release the only worker slot.
            deadline = time.monotonic() + 1
            while True:
                conn = http.client.HTTPConnection(*server.server_address, timeout=1)
                try:
                    conn.request('GET', '/v1/telemetry',
                                 headers={'Authorization': 'Bearer ' + TOKEN})
                    self.assertEqual(conn.getresponse().status, 200)
                    break
                except (OSError, http.client.HTTPException):
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(.01)
                finally:
                    conn.close()
        finally:
            server.shutdown()
            server.server_close()
            worker.join(2)

    def test_invalid_tokens_fail_startup(self):
        for token in ('', 'short', 'x' * 31, 'x' * 32 + '\n', 'é' * 32):
            with patch.dict(os.environ, {'HELIOS_GATEWAY_TOKEN': token}):
                with self.assertRaises(ValueError):
                    token_from_environment()
        with patch.dict(os.environ, {'HELIOS_GATEWAY_TOKEN': TOKEN}):
            self.assertEqual(token_from_environment(), TOKEN)


if __name__ == '__main__':
    unittest.main()
