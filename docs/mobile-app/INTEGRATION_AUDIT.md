# Helios robot integration evidence

Audited 2026-09-28 against [pran99-git/helios_ws](https://github.com/pran99-git/helios_ws), commit [`6f06f89dfabf501fcf4ce21b35c9f652af633acd`](https://github.com/pran99-git/helios_ws/tree/6f06f89dfabf501fcf4ce21b35c9f652af633acd). Paths below are relative to that pinned repository. This is a source audit, not a hardware verification.

## Implemented companion integration

After this original-source audit, a separate `mobile_gateway` package was implemented for authenticated read-only `GET /v1/telemetry` (odometry and lidar summary). The iOS client now implements that agreed API, with a local synthetic fixture used for protocol validation. See `README.md` and `VALIDATION.md`. References below to a missing gateway describe the pinned original commit, not the new contribution. Robot deployment, trusted HTTPS endpoint and hardware verification remain outstanding. The companion gateway has since added 20 explicitly enabled managed operational commands; see `OPERATIONS.md`. Map streaming, video, direct velocity and navigation-goal submission remain unimplemented.

## Robot and current capability

The root `README.md` describes a Lynxmotion A4WD3 four-wheel mecanum rover, NVIDIA Jetson AGX Orin, Ubuntu 24.04 / ROS 2 Jazzy, RoboClaw controllers, Hokuyo UST-10LX lidar, and ZED 2i stereo camera/IMU. It supports manual Bluetooth gamepad driving, odometry, 2D/3D mapping, and localization. Nav2 is configured and launches, but the repository explicitly says full autonomous goal-to-goal driving has not been validated. Do not present autonomous navigation as proven functionality.

At the time of this original audit, the phone app started in a clearly labeled local demonstration. Current builds start in real robot mode; samples require eligible developer opt-in (see DEVELOPER_MODE.md). Simulated pose, illustrative map, and motion are not readings or commands exchanged with this robot. No physical hardware connection has been established or verified; the client has been exercised against a synthetic gateway.

## Actual ROS contracts

| Interface | Type / meaning | Evidence |
| --- | --- | --- |
| `/odometry/filtered` | `nav_msgs/msg/Odometry`, fused planar pose and body velocity; nominal 30 Hz; pose is in `odom`, not globally corrected `map` coordinates | `src/perception_pkg/sensor_fusion/config/ekf.yaml`, `src/perception_pkg/sensor_fusion/README.md` |
| `/wheel/odometry` | `nav_msgs/msg/Odometry`, encoder-derived odometry; roughly encoder publication rate | `src/perception_pkg/wheel_odometry/wheel_odometry/wheel_odometry_node.py`, `config/wheel_odometry.yaml` in that package |
| `/zed/odom_with_cov` | `nav_msgs/msg/Odometry`, camera odometry with repaired twist covariance for EKF input | `src/perception_pkg/camera/custom_covariance/custom_covariance/zed_odom_covariance_node.py` |
| `/scan` | `sensor_msgs/msg/LaserScan`, planar lidar in `laser` frame; nominal 40 Hz | `src/perception_pkg/lidar/custom_config/launch/lidar.launch.py`, `src/perception_pkg/README.md` |
| `/map` | `nav_msgs/msg/OccupancyGrid`, 2D SLAM or saved-map output; configured SLAM resolution 0.05 m/cell | `src/mapping_localization_pkg/README.md`, `slam_toolbox/config/slam_toolbox.yaml` in that package |
| `/rtabmap/map` | Default RTAB-Map occupancy grid; launch argument `map_topic:=/map` is required to serve Nav2's expected topic | `src/mapping_localization_pkg/rtabmap/launch/rtabmap.launch.py` |
| `/roboclaw/wheel_encoders` | `sensor_msgs/msg/JointState`; `position` contains raw quadrature counts, **not radians**; four corner names; nominal 30 Hz | `src/low_level_control_pkg/roboclaw/roboclaw_driver_node.py` |
| `/zed/zed_node/rgb/color/rect/image` | Rectified camera image ROS topic, nominal 30 Hz; no phone-ready video URL | `src/perception_pkg/README.md`, `src/mapping_localization_pkg/rtabmap/launch/rtabmap.launch.py` |
| `/cmd_vel` | `geometry_msgs/msg/Twist`; `linear.x` forward m/s, `linear.y` lateral m/s, `angular.z` yaw rad/s in body frame | `src/low_level_control_pkg/roboclaw/roboclaw_driver_node.py` |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose`, goal pose in `map` frame | `src/navigation_pkg/README.md` |
| `/amcl_recovery/relocalize`, `/amcl_recovery/abort` | `std_srvs/srv/Trigger`; start localization recovery or cancel its active behavior; recovery can physically spin/drive | `src/mapping_localization_pkg/localization/amcl_recovery_node.py` |

The TF chain is `map → odom → base_link → sensors`. A display overlaying the fused pose or a scan on a map must apply the appropriate timestamped TF transforms. Treating odometry coordinates as map coordinates would be incorrect. Exactly one active mapping/localization source should own `map → odom`; the EKF alone owns `odom → base_link`.

Configured rates are expectations, not observed mobile connection rates. A gateway must preserve timestamps and frame IDs and choose compatible ROS QoS. A missing topic must appear as unavailable, not as a zero or a healthy sensor.

## Drive behavior and limits

`src/low_level_control_pkg/config/roboclaw.yaml` sets ±0.40 m/s forward/lateral, ±0.8 rad/s yaw, control rate 20 Hz, and a 0.5-second stale-command watchdog. The Python driver's fallback defaults are lower, so source-level defaults must not be mistaken for the deployed launch configuration. Actual deployed parameters still require confirmation.

The driver clamps finite body commands and scales wheel speeds to controller limits. The watchdog stops through the configured acceleration ramp (`drive_accel: 5000` counts/s²): it is **not an instantaneous physical emergency stop**. Its YAML estimates approximately 0.28 m total travel after loss of the command stream at 0.40 m/s, under its stated assumptions. A phone stop button must not be represented as a certified emergency stop.

`src/navigation_pkg/README.md` warns that joystick teleop and Nav2 both publish `/cmd_vel` without arbitration: whichever message arrives last wins. There is no independent `collision_monitor`; current costmaps use lidar only and can miss obstacles below the 17.6 cm laser plane. Mobile driving requires server-side control ownership and coordination with both navigation and automatic AMCL recovery, not simply another `/cmd_vel` publisher.

## Networking: what exists and what does not

The documented `192.168.0.10:10940` address is the **Hokuyo scanner's SCIP/TCP endpoint**, on the Jetson's wired sensor subnet. It is not a Jetson/mobile application endpoint. Evidence: `src/perception_pkg/lidar/custom_config/config/urg_node2.yaml` and `src/perception_pkg/README.md`.

The repository describes SSH/onboard operation. Inspection found no rosbridge deployment, HTTP API, WebSocket gateway, MQTT contract, phone discovery mechanism, authentication service, or mobile pairing implementation in the checked-in application source. An iOS `URLSession` client cannot directly consume these ROS 2 DDS topics. No connection address, port, or JSON envelope should be invented and presented as an existing robot contract.

No published battery-state/charge-percentage interface was found. The 3S LiPo hardware description does not provide live voltage or state of charge. Wi-Fi signal, Jetson thermals, storage, mission queue, docking, and emergency-stop state likewise need explicit telemetry contracts before being shown as real readings. Camera ROS topics do not establish an RTSP, HLS, or WebRTC endpoint.

## Work required for real integration

1. Confirm Pranav's deployed commit, actual launch mode, ROS domain/RMW/QoS settings, topic names/types, frame tree, and runtime parameter values. Obtain read-only sample messages or a ROS bag for decoder and coordinate-transform fixtures.
2. Select and deploy a Jetson-side mobile gateway. A restricted rosbridge-based adapter or purpose-built ROS 2 gateway are potential designs, **not existing functionality**. Agree on versioned messages, topic allowlists, map encoding, scan decimation, timestamps, heartbeat, and error/capability reporting before implementing the iOS transport.
3. Supply the actual Jetson LAN host/address and reachable port, network topology, TLS/pairing/authentication design, and reconnect behavior. For iOS, add a truthful Local Network usage description when networking is implemented. Bonjour declarations are needed only if an agreed discovery service uses them. Store future credentials in Keychain.
4. Begin with read-only odometry, scan, and map subscriptions. Preserve unknown occupancy cells and invalid/infinite scan readings; handle map origin orientation/resolution and timestamped transforms; throttle large streams for phone bandwidth. Verify stale/disconnected states without hardware motion.
5. Before enabling actuation, implement a gateway-side exclusive control lease and timeout; arbitrate against joystick, Nav2 and AMCL recovery; bound commands to current configured limits; require an explicit held drive control; revoke ownership on disconnect/background/interruption; test cancellation and loss of connectivity with the robot owner. A client-side timer alone is insufficient.
6. Agree on a hardware stop procedure with the owner, then perform supervised commissioning. The repository's current autonomy limitations still apply. Goal sending/cancellation, relocalization, mapping mode changes, and map saving each need a separately specified, authorized gateway operation.

Remaining deployment decisions: Wi-Fi/LAN host and TLS setup, actual token and ROS domain, available map/ROS-bag samples, eventual driving/video scope, and hardware commissioning procedure. The first integration is read-only API v1. These do not block building and running the app's local demo; they do block a truthful live-robot connection.

---
This file is part of the mirrored [Helios setup bundle](README.md). Shell commands and source paths refer to the original app or robot repository root, as specified in the guide.
