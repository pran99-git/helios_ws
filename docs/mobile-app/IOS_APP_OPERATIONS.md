# Phone operations

The Operations tab loads the authenticated gateway's supported command catalog. Current catalog: **20 operations** based on the upstream launch runbook and mapping/navigation documentation. This is managed robot-software control, not a general remote terminal.

## Available operations

| Group | Commands |
| --- | --- |
| Hardware/perception | Start/stop motor driver; sensors and fused odometry with camera/lidar options; physical joystick teleop |
| Mapping/localization | 2D SLAM mapping or saved-graph localization; 3D RTAB mapping or database localization; AMCL with optional recovery |
| Navigation | Start/stop the Nav2 stack |
| Map files | Save SLAM maps/pose graph; capture RTAB occupancy map; export a stopped RTAB database to point cloud |
| Recovery | Abort recovery; request relocalization; global localization |
| Diagnostics | Node list, topic list/types, and controller/planner/navigator lifecycle queries |

The gateway enforces dependencies, conflicting modes and constrained parameters. Motion-capable entries require a confirmation dialog. A running process is not proof that ROS nodes are ready: inspect telemetry and lifecycle diagnostics. Map inputs are validated basenames in the configured workspace, not arbitrary paths; existing outputs cannot be overwritten. The gateway documentation explains each parameter and file location.

Not exposed: direct phone joystick/velocity commands, navigation goal submission/cancellation, GUI tools, package installation/builds, device/network configuration, motor calibration or EEPROM changes. The robot's navigation still has its documented hardware-validation limitations. Camera and live map streaming are not implemented.

## Control session and lifecycle

1. Connect with the gateway address and token.
2. Open Operations and **Acquire control session**. Viewing telemetry/catalog alone never claims control.
3. Start prerequisites in order: motors → sensors → one mapper. Choose physical joystick or navigation; they cannot run together.
4. Open an operation, fill its typed inputs, and run it. Check the resulting job state and output; acceptance does not mean success.
5. Stop dependents before their providers, or use **Stop managed processes** for orderly shutdown. Save map outputs before shutdown.

The foreground app renews an exclusive lease every three seconds. After ten seconds without a heartbeat, the gateway requests shutdown of its owned work. Backgrounding, disconnecting or switching robot/developer modes stops renewal. Developer mode accepts only simulated fixture commands and exits on backgrounding; ordinary Release cannot enable it. Returning to the foreground does **not** silently reacquire control. An OS/gateway failure can prevent cleanup; this is not a physical deadman or emergency stop, and it cannot stop software launched from other terminals.

Before each mutation the client validates gateway identity through the catalog, using the same cancellation scope. Mutation requests are sent once. If the response is lost, the app reports that the outcome may be unknown and preserves that message across heartbeat renewals. Check jobs before repeating the operation. Request identifiers allow bounded server-side duplicate detection, but the app never automatically retries command mutations. Disconnect cancels outstanding HTTP sessions; this cannot undo commands already accepted by the server.

Active jobs and stop controls are above the catalog. Completed jobs are collapsed below it. Process output shows a short catalog preview; **Refresh output** fetches the latest bounded 1,024-byte log tail. Failed stops and server rejection messages remain visible.

## Enable on the Jetson

Install/build the gateway changes and source the complete trusted robot workspace. Stop laptop-managed robot stacks first; the gateway must be the sole manager. Then, with the token already loaded securely:

```sh
cd ~/helios_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run mobile_gateway mobile_gateway --workspace "$PWD" \
  --enable-commands --exclusive-stack-control --host 127.0.0.1 --port 8080
```

Keep the gateway unprivileged with the robot's existing device permissions. Configure trusted HTTPS and permit the documented operation POST routes in the reverse proxy; the old telemetry-only proxy route is insufficient. The gateway defaults to telemetry-only without those explicit flags.

[Authoritative gateway catalog, constraints, and proxy configuration](GATEWAY_OPERATIONS.md)

## Fixture validation

Start the companion fixture as described in README. The Debug app requires **Connect → Enable developer mode → Enable simulation**; ordinary Release rejects fixtures. The script explicitly compiles Debug tooling and selects fixture mode. It uses a simulated process backend, labels all responses/jobs as fixture data, and never executes ROS or hardware commands. Then run:

```sh
HELIOS_TEST_ENDPOINT=http://localhost:18080 \
HELIOS_GATEWAY_TOKEN=helios-local-fixture-token-32-chars-only \
./Scripts/check-operations.sh
```

The harness refuses to acquire control unless the endpoint identifies itself as a fixture. It checks lease acquisition, required confirmation, service starts, mode conflicts, dependency rejection, persistent command outcomes, shutdown and lease-expiry behavior. The opt-in HeliosIntegration simulator scheme also exercises on-screen confirmation/start/stop with the fixture.

No physical robot operation or stopping behavior has been validated here. Deployment requires supervised robot-side acceptance testing.

---
This file is part of the mirrored [Helios setup bundle](README.md). Shell commands and source paths refer to the original app or robot repository root, as specified in the guide.
