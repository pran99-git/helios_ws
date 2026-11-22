"""Bounded read-only HTTP server; put a TLS reverse proxy in front remotely."""
import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import os
import socket
import threading


def token_from_environment():
    token = os.environ.get("HELIOS_GATEWAY_TOKEN", "")
    if len(token) < 32 or not token.isascii() or any(c.isspace() for c in token):
        raise ValueError("HELIOS_GATEWAY_TOKEN must contain at least 32 ASCII non-whitespace characters")
    return token


class TelemetryServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    request_queue_size = 16

    def __init__(self, address, state, token, max_clients=8, socket_timeout=3.0, request_timeout=5.0, operations=None):
        if len(token) < 32 or not token.isascii() or any(c.isspace() for c in token):
            raise ValueError("A token of at least 32 ASCII non-whitespace characters is required")
        if max_clients < 1 or socket_timeout <= 0 or request_timeout <= 0:
            raise ValueError("Client limit and socket timeout must be positive")
        self.operations = operations
        self.state = state
        self._token = token.encode("ascii")
        self._slots = threading.BoundedSemaphore(max_clients)
        self.socket_timeout = socket_timeout
        self.request_timeout = request_timeout
        super().__init__(address, TelemetryHandler)

    def process_request(self, request, client_address):
        request.settimeout(self.socket_timeout)
        if not self._slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self._slots.release()
            raise

    def process_request_thread(self, request, client_address):
        # An inactivity timeout alone does not bound clients that drip headers.
        def expire():
            try:
                request.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

        deadline = threading.Timer(self.request_timeout, expire)
        deadline.daemon = True
        deadline.start()
        try:
            super().process_request_thread(request, client_address)
        finally:
            deadline.cancel()
            self._slots.release()

    def handle_error(self, request, client_address):
        # Never print raw requests/headers or token-bearing exceptions.
        pass


class TelemetryHandler(BaseHTTPRequestHandler):
    server_version = "HeliosTelemetry/1"
    sys_version = ""

    def log_message(self, format, *args):
        # No request logging: query strings or malformed headers may contain secrets.
        pass

    def _reply(self, status, payload):
        body = json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        if status == 401:
            self.send_header("WWW-Authenticate", 'Bearer realm="helios"')
        if status == 405:
            self.send_header("Allow", getattr(self, "allowed_method", "GET"))
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)
        self.close_connection = True

    def _body(self):
        if self.headers.get_all("Transfer-Encoding"):
            raise ValueError("Chunked requests are unsupported")
        lengths = self.headers.get_all("Content-Length", [])
        if len(lengths) != 1 or not lengths[0].isdigit():
            raise ValueError("Exactly one Content-Length is required")
        length = int(lengths[0])
        if not 0 < length <= 16384:
            raise ValueError("JSON request must be 1-16384 bytes")
        if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
            raise ValueError("Content-Type must be application/json")
        raw = self.rfile.read(length)
        if len(raw) != length:
            raise ValueError("Incomplete body")
        def pairs(items):
            result = {}
            for key, value in items:
                if key in result:
                    raise ValueError("Duplicate JSON fields")
                result[key] = value
            return result
        def invalid_number(value):
            raise ValueError("Non-finite JSON number")
        body = json.loads(raw, object_pairs_hook=pairs, parse_constant=invalid_number)
        if not isinstance(body, dict):
            raise ValueError("JSON body must be an object")
        return body

    def _dispatch(self):
        paths = self.path.split("/")
        exact = self.path in ("/v1/telemetry", "/v1/operations", "/v1/control/heartbeat", "/v1/operations/stop-all")
        dynamic = (len(paths) == 5 and paths[1:3] == ["v1", "operations"] and paths[4] == "start") or (len(paths) in (4, 5) and paths[1:3] == ["v1", "jobs"] and (len(paths) == 4 or paths[4] == "stop"))
        if not (exact or dynamic) or "?" in self.path:
            self._reply(404, {"error": "not_found"})
            return
        headers = self.headers.get_all("Authorization", [])
        provided = headers[0].encode("utf-8") if len(headers) == 1 else b""
        if not hmac.compare_digest(provided, b"Bearer " + self.server._token):
            self._reply(401, {"error": "unauthorized"})
            return
        is_read = self.path in ("/v1/telemetry", "/v1/operations") or (paths[1:3] == ["v1", "jobs"] and len(paths) == 4)
        self.allowed_method = "GET" if is_read else "POST"
        if self.command != self.allowed_method:
            self._reply(405, {"error": "method_not_allowed"})
            return
        if self.path == "/v1/telemetry":
            self._reply(200, self.server.state.snapshot())
            return
        manager = self.server.operations
        if manager is None:
            self._reply(403, {"error": "commands_disabled", "message": "Operation manager is disabled."})
            return
        from .operations import OperationError
        try:
            if self.path == "/v1/operations":
                self._reply(200, manager.catalog())
                return
            if is_read:
                self._reply(200, manager.job(paths[3]))
                return
            body = self._body()
            if self.path == "/v1/control/heartbeat":
                if set(body) != {"client_id"}:
                    raise ValueError("Expected only client_id")
                result = manager.heartbeat(body.get("client_id"))
            elif self.path == "/v1/operations/stop-all":
                if set(body) != {"client_id"}:
                    raise ValueError("Expected only client_id")
                result = manager.stop_all(body.get("client_id"))
            elif paths[2] == "jobs":
                if set(body) != {"client_id"}:
                    raise ValueError("Expected only client_id")
                result = manager.stop(paths[3], body.get("client_id"))
            else:
                result = manager.start(paths[3], body)
            self._reply(200 if self.path == "/v1/control/heartbeat" else 202, result)
        except OperationError as error:
            self._reply(error.status, {"error": error.code, "message": str(error)})
        except (ValueError, UnicodeError, RecursionError) as error:
            self._reply(400, {"error": "invalid_request", "message": str(error)[:200]})

    # BaseHTTPRequestHandler normally emits 501 for unknown methods. All parsed
    # HTTP methods use the same explicit read-only dispatch instead.
    def __getattr__(self, name):
        if name.startswith("do_"):
            return self._dispatch
        raise AttributeError(name)


def arguments(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--enable-commands", action="store_true")
    parser.add_argument("--exclusive-stack-control", action="store_true", help="Attest no laptop or other process controls the ROS stack")
    parser.add_argument("--workspace", default=".", help="Built trusted Helios workspace root")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--behind-tls-proxy", action="store_true",
                        help="Acknowledge a secured TLS proxy and firewall protect a non-loopback listener")
    args, ros_args = parser.parse_known_args()
    try:
        loopback = ipaddress.ip_address(args.host).is_loopback
    except ValueError:
        parser.error("--host must be a numeric IPv4 bind address")
    if ":" in args.host:
        parser.error("Only IPv4 listeners are supported")
    if not loopback and not args.behind_tls_proxy:
        parser.error("Non-loopback binding requires --behind-tls-proxy and a configured TLS proxy/firewall")
    if not 0 <= args.port <= 65535:
        parser.error("--port must be between 0 and 65535")
    return args, ros_args
