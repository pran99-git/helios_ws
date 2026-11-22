# Helios for iOS

Native SwiftUI companion for Pranav's Helios rover. The app connects to the companion **mobile_gateway API v1** for fused odometry, a lidar summary, and explicitly enabled managed robot operations. It starts in real robot mode. Local samples and network fixtures require explicit developer-mode opt-in in eligible Debug or internal TestFlight builds; ordinary Release has no developer tools. See [developer-mode distribution and lifecycle](DEVELOPER_MODE.md). Requires Xcode 26.4, Swift 6.2+ toolchain (Swift 6 language mode), and iOS 26+. No third-party app dependencies.

## Open and run

Open `Helios.xcodeproj`, select **Helios**, choose an iPhone/iPad simulator, and Run. For a physical phone, set your development team and a unique app bundle identifier in Signing & Capabilities. XcodeGen is only needed after changing `project.yml`: run `xcodegen generate`.

## Connect to the robot

1. Deploy `src/mobile_gateway` from the companion [upstream PR #1](https://github.com/pran99-git/helios_ws/pull/1). Its [gateway README](GATEWAY_SETUP.md) contains the full ROS setup, topic overrides and TLS reverse-proxy example. This package must exist on the Jetson; the app cannot connect directly to ROS DDS.
2. In the Jetson workspace, build and launch the read-only gateway:

   ```sh
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src/mobile_gateway --ignore-src -r -y
   colcon build --packages-select mobile_gateway
   source install/setup.bash
   export HELIOS_GATEWAY_TOKEN="$(python3 -c 'import secrets; print(secrets.token_urlsafe(32))')"
   ros2 run mobile_gateway mobile_gateway --host 127.0.0.1 --port 8080
   ```

   Retain the generated token securely and enter that same value in the app. The gateway uses the same ROS domain/middleware environment as the robot's existing sensor publishers. Starting it does not start sensors or move the robot.
3. Configure a trusted HTTPS reverse proxy to that loopback listener, DNS/routing, and LAN/VPN access. The certificate must match the host and be trusted by the iPhone. Keep port 8080 private. The gateway README includes nginx configuration; the project does not provision certificates or change the robot network.
4. In the app's **Connect** tab, enter the base URL, such as `https://<configured-robot-host>` (no `/v1/telemetry` suffix), and the bearer token. Tap **Connect**, then **Overview**. Allow Local Network access if iOS asks.

The real hostname, certificate/trust setup, gateway installation, token and running ROS sensor topics must come from Pranav's deployment. None are inferred from the lidar's IP address. Robot deployment and physical connectivity have not been verified here.

## What the client does

- Authenticated `GET /v1/telemetry`, one outstanding request, approximately 1 Hz on success and 2-second retry interval after failures.
- Displays frame IDs, X/Y/yaw, forward/lateral/yaw velocity, and the nearest finite lidar return in SI units.
- Rejects unsupported versions and incomplete available data. Missing or stale sensors are shown as unavailable; no valid lidar return does not imply clear space.
- Includes network transit conservatively in a **monotonic** two-second freshness window. This is freshness since gateway callback receipt, not proof of the sensor's acquisition time; ROS bags/delayed publishers can appear fresh.
- Clears readings on request failures, manual disconnect, and app backgrounding. Automatically resumes/retries while enabled in real mode; switching modes disconnects and drops local control. Backgrounding exits developer mode.
- Real mode accepts only `source: ros2` and non-simulated jobs; missing/unknown/fixture sources are rejected. Developer mode accepts only fixtures and labels them simulated. ROS source identifies the adapter, not proof of hardware. The map tab is unavailable in real mode; synthetic geometry exists only after developer opt-in.
- HTTPS only off-device. Plain HTTP is accepted only for loopback in explicit fixture mode. Redirects are refused, credentials stay in memory, cookies/cache are disabled, telemetry responses are capped at 16 KiB and operation responses at 256 KiB, and requests have finite timeouts. The app never disables certificate validation.

The **Operations** tab now supports 20 repo-backed startup, shutdown, mapping, localization, recovery and diagnostic operations. It uses an explicit expiring control lease, typed parameter forms, confirmations and job/output views. See [OPERATIONS.md](IOS_APP_OPERATIONS.md) for the catalog and robot deployment flags. No direct velocity, emergency-stop, navigation-goal, camera, map-stream or battery API is present. This does not claim the rover's autonomy is hardware validated.

The interface uses an original WHOOP/Tesla-inspired dark instrument-panel design: charcoal surfaces, lime/cyan metrics, a decorative rover schematic, readable status and grouped controls. Demo and fixture data remain explicitly labeled.

## Test against the actual gateway fixture

The fixture uses the same HTTP server and telemetry state code as the ROS node but never loads ROS or contacts hardware. From the gateway checkout:

```sh
export HELIOS_GATEWAY_TOKEN=helios-local-fixture-token-32-chars-only
PYTHONPATH=src/mobile_gateway python3 -m mobile_gateway.fixture --port 18080
```

This is a disposable public test token, never a robot credential. In a Debug simulator build, first choose **Connect → Enable developer mode → Enable simulation**, then use `http://localhost:18080` and that test token. A real phone's `localhost` means the phone itself; use the HTTPS setup above for an off-device gateway.

From this iOS project:

```sh
HELIOS_TEST_ENDPOINT=http://localhost:18080 \
HELIOS_GATEWAY_TOKEN=helios-local-fixture-token-32-chars-only \
./Scripts/check-gateway.sh

xcodebuild -project Helios.xcodeproj -scheme Helios \
  -destination 'platform=iOS Simulator,name=iPhone 17e' \
  -derivedDataPath /tmp/helios-ios-verified test CODE_SIGNING_ALLOWED=NO

# Opt-in UI integration test; requires the fixture and test token above.
xcodebuild -project Helios.xcodeproj -scheme HeliosIntegration \
  -destination 'platform=iOS Simulator,name=iPhone 17e' \
  -derivedDataPath /tmp/helios-ios-verified test CODE_SIGNING_ALLOWED=NO
```

`check-gateway.sh` compiles the shared URLSession client and observable connection controller with `-D DEBUG`, then explicitly opts into fixture mode on macOS against the actual gateway fixture, covering auth, decoding, pause/resume, disconnect cancellation and recovery after corrected credentials. The iOS unit suite separately checks configuration, malformed/versioned data, freshness and demo playback. For delayed-response and automatic recovery tests against the actual gateway server:

```sh
PYTHONPATH=../helios-mobile-gateway/src/mobile_gateway python3 Tests/run_gateway_faults.py
```

The Debug UI suite verifies real-mode defaults, explicit opt-in, fixture telemetry and commands, local preview, and background reset. Release/TestFlight simulator UI checks verify tools stay closed without verified eligibility. The harness compile flag is test-only; it does not change the app Release configuration.

## Architecture

`App/RootView` owns `DemoSession`, `LiveConnection`, and the fail-closed `DeveloperAccess`. The demo timer runs only during opt-in. Feature views share those observable main-actor models. `GatewayClient` validates configuration, performs bounded authenticated requests and decodes typed API values. `LiveConnection` owns the cancellable polling task; generation checks prevent old responses from repopulating a disconnected/reconfigured screen. The separate `OperationsSession` polls the operation catalog, owns the explicit control lease, scopes cancellable HTTP requests, and retains command outcomes separately from connection status. No hardware networking code is coupled to the demo model. `GatewayOdometry` and `GatewayScan` enforce freshness for presentation.

[original integration audit](INTEGRATION_AUDIT.md) records the original upstream source audit, ROS contracts and future requirements for map transforms or control. `VALIDATION.md` records checks performed and their limits.

---
This file is part of the mirrored [Helios setup bundle](README.md). Shell commands and source paths refer to the original app or robot repository root, as specified in the guide.
