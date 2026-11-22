# Helios mobile telemetry gateway

**Start here:** [Pranav’s complete Jetson-to-iPhone setup and handoff](PRANAV_SETUP.md).

A small ROS 2 Python package providing authenticated HTTP telemetry and opt-in managed runbook operations for a mobile client. By default only telemetry is enabled. An explicitly enabled operation manager can start/stop allowlisted ROS launch processes and run fixed save/recovery/diagnostic commands. It never accepts arbitrary shell input or submits navigation goals. No physical hardware was actuated during development. See [the operation catalog and API](GATEWAY_OPERATIONS.md).

## Repository integration

The source audit identifies these interfaces:

| Input / existing capability | Repository evidence | Gateway use |
| --- | --- | --- |
| `/odometry/filtered`, `nav_msgs/msg/Odometry` | [EKF launch](https://github.com/tanmayg1502/helios_ws/blob/codex/mobile-telemetry-gateway/src/perception_pkg/sensor_fusion/launch/ekf.launch.py), [EKF config](https://github.com/tanmayg1502/helios_ws/blob/codex/mobile-telemetry-gateway/src/perception_pkg/sensor_fusion/config/ekf.yaml) | Pose and planar velocity |
| `/scan`, `sensor_msgs/msg/LaserScan`, frame `laser` | [LiDAR launch](https://github.com/tanmayg1502/helios_ws/blob/codex/mobile-telemetry-gateway/src/perception_pkg/lidar/custom_config/launch/lidar.launch.py), [LiDAR config](https://github.com/tanmayg1502/helios_ws/blob/codex/mobile-telemetry-gateway/src/perception_pkg/lidar/custom_config/config/urg_node2.yaml) | Nearest valid return |
| `/map` and localization | [Mapping package](https://github.com/tanmayg1502/helios_ws/blob/codex/mobile-telemetry-gateway/src/mapping_localization_pkg/README.md) | Not exposed |
| Nav2 goals and `/cmd_vel` | [Navigation launch](https://github.com/tanmayg1502/helios_ws/blob/codex/mobile-telemetry-gateway/src/navigation_pkg/launch/navigation.launch.py), [navigation docs](https://github.com/tanmayg1502/helios_ws/blob/codex/mobile-telemetry-gateway/src/navigation_pkg/README.md) | Not exposed |

Subscriptions use ROS sensor-data QoS (best effort, volatile). Topic names are configurable via ROS parameters. Gateway freshness measures monotonic time since callback receipt, **not** the ROS message timestamp or a guarantee that the sensor measurement itself is current. Delayed or replayed ROS data can therefore appear fresh. Odometry is in the supplied odometry frame; it is not a map-relative location. The nearest return is a scan summary, not a collision or clearance guarantee.

## API version 1

`GET /v1/telemetry` with exactly one `Authorization: Bearer <token>` header. Paths are exact; no query parameters. Responses have `Content-Type: application/json`, `Cache-Control: no-store`, and close the connection. Suggested client polling: 2 Hz, one outstanding request, with a finite timeout and reconnect backoff. A telemetry-only disconnect has no robot-side action; an operation control lease expires without heartbeats and requests owned-process shutdown; clients must immediately mark their display disconnected and must not treat cached telemetry as live.

```json
{"api_version":1,"source":"ros2","odometry":{"available":true,"age_seconds":0.1,"frame_id":"odom","child_frame_id":"base_link","x":1.0,"y":2.0,"heading":0.3,"linear_x":0.2,"linear_y":0.0,"angular_z":0.1},"scan":{"available":true,"age_seconds":0.1,"frame_id":"laser","nearest_m":1.2}}
```

`source` is `ros2` for the ROS adapter and `fixture` for synthetic data; clients must label fixture data as simulated. This describes the adapter, not proof of physical hardware (ROS can replay bags).

Distances are meters, heading radians, linear velocity meters/second, angular velocity radians/second. Heading is yaw from the normalized quaternion. Numeric JSON values are finite. Missing data is `{"available":false}`; invalid or older-than-2-second data is `{"available":false,"age_seconds":...}` with no measurement fields. At exactly 2 seconds a valid sample remains available. A fresh scan with no valid finite in-range returns is available with `"nearest_m":null`; this does **not** mean the space is clear. Consumers must accept missing optional fields when unavailable and ignore unknown fields for future additive changes.

Invalid credentials return 401 (`unauthorized`); authenticated non-GET methods return 405 (`method_not_allowed`, `Allow: GET`); unknown paths return 404 (`not_found`). These errors contain no credentials. A HEAD response has no body. This telemetry endpoint does not command motion. The separately enabled [operation API](GATEWAY_OPERATIONS.md) can launch motion-capable stacks and recovery services; it has an expiring control lease and best-effort graceful process shutdown. There is no emergency-stop, velocity, map-rendering, mission or navigation-goal API.

## ROS 2 deployment

On the robot's Linux host with its existing ROS 2 environment (the repository uses Jazzy), from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/mobile_gateway --ignore-src -r -y
colcon build --packages-select mobile_gateway
source install/setup.bash
export HELIOS_GATEWAY_TOKEN="$(python3 -c 'import secrets; print(secrets.token_urlsafe(32))')"
ros2 run mobile_gateway mobile_gateway --host 127.0.0.1 --port 8080
```

Generate and securely retain one token for the gateway and mobile client; regenerating it invalidates the previous one when the gateway restarts. Startup rejects tokens shorter than 32 ASCII non-whitespace characters. Do not commit it, put it in a URL, or paste it into logs. A service manager may load it from a permissions-restricted environment file; avoid command-line token arguments. Configure the same ROS domain and middleware environment as the publishers. Launch existing sensors separately according to their own operator procedures; starting this gateway alone yields unavailable telemetry.

Topic overrides:

```bash
ros2 run mobile_gateway mobile_gateway --ros-args \
  -p odometry_topic:=/odometry/filtered -p scan_topic:=/scan
```

Local verification from a shell that has the token:

```bash
curl --fail --silent --show-error \
  -H "Authorization: Bearer ${HELIOS_GATEWAY_TOKEN}" \
  http://127.0.0.1:8080/v1/telemetry
```

For off-host use, terminate trusted TLS at a reverse proxy on the robot. For example, an nginx `http` configuration fragment, with a certificate trusted by the phone and matching `robot.example.net`:

```nginx
server {
    listen 443 ssl;
    server_name robot.example.net;
    ssl_certificate /etc/helios/tls/fullchain.pem;
    ssl_certificate_key /etc/helios/tls/privkey.pem;
    ssl_protocols TLSv1.2 TLSv1.3;
    access_log off;
    client_header_timeout 5s;
    client_body_timeout 5s;
    client_max_body_size 1k;
    location = /v1/telemetry {
        proxy_pass http://127.0.0.1:8080;
        proxy_set_header Authorization $http_authorization;
        proxy_connect_timeout 2s;
        proxy_read_timeout 5s;
    }
    location / { return 404; }
}
```

Configure DNS/routing, install the certificate and proxy separately, and restrict inbound 443 to the intended trusted LAN/VPN. Keep port 8080 loopback-only. The mobile base URL is `https://robot.example.net`; do not disable certificate validation. There is no plaintext off-host recommendation. Non-loopback binding requires explicit `--behind-tls-proxy`; this flag acknowledges deployment responsibilities and does not implement TLS or a firewall. The Python server is intentionally small (8 concurrent clients, 3-second socket inactivity timeout, 5-second total request deadline), not a public Internet service. Reverse-proxy hardening and network policy remain deployment responsibilities.

## Local synthetic fixture and tests

These commands require Python 3 and no ROS installation. From the workspace root:

```bash
PYTHONPATH=src/mobile_gateway python3 -m unittest discover -s src/mobile_gateway/test -v
export HELIOS_GATEWAY_TOKEN="$(python3 -c 'import secrets; print(secrets.token_urlsafe(32))')"
PYTHONPATH=src/mobile_gateway python3 -m mobile_gateway.fixture --port 8080
```

The fixture continuously generates synthetic odometry and scan data using the same HTTP server and state validation as the ROS adapter. Its operation API is enabled with a simulated backend: services remain running and actions complete synthetically, with no subprocesses or files created. It prints a synthetic-only banner and never imports ROS or contacts hardware. Use the loopback URL for a same-host development client; a phone requires the TLS deployment above. Stop with Ctrl-C. Never present fixture values as live robot measurements.

Tests exercise actual loopback sockets, authentication and duplicate headers, method/path rejection, disconnect recovery, socket timeout and slow-drip deadline/slot recovery, monotonic stale boundaries, quaternion normalization, finite-value rejection, and scan filtering/null semantics. Local tests do not prove ROS discovery/QoS, colcon installation on the robot, trusted TLS on a phone, real sensor freshness, or robot connectivity. Those require deployment validation by an operator. No hardware, map rendering, or navigation validation is claimed.

---
This file is part of the mirrored [Helios setup bundle](README.md). Shell commands and source paths refer to the original app or robot repository root, as specified in the guide.
