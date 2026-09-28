# Phone operation catalog

This catalog translates the commands in [`launch_helios.md`](../../launch_helios.md), the [mapping README](../mapping_localization_pkg/README.md), and the [navigation README](../navigation_pkg/README.md). Commands execute only when robot operations have been explicitly enabled in deployment. The synthetic fixture does not execute ROS or shell commands.

A started process is **not proof of ROS readiness**. Inspect its bounded logs, the lifecycle diagnostics and telemetry before proceeding. The gateway manages only processes it starts: never launch a second copy alongside laptop terminals. In particular, two RoboClaw processes can corrupt serial packets, and independent `/cmd_vel` publishers can compete. No gateway can stop externally launched publishers using its process registry.

| ID | Purpose and parameters | Dependencies / conflicts |
|---|---|---|
| `motors` | RoboClaw driver | Motion confirmation |
| `sensors` | Sensors + EKF; `camera`, `lidar` booleans default true | Motors; hold still for initial five seconds |
| `joystick` | Physical joystick; fixed default device | Motors; conflicts with navigation; motion confirmation |
| `slam_mapping` | 2D SLAM | Sensors; exclusive mapper |
| `slam_localization` | Saved graph; `map` basename or prefix, optional `x`, `y`, `heading` | Sensors; matching `.posegraph` and `.data`; exclusive mapper |
| `rtab_mapping` | 3D mapping; required new `name` | Sensors; exclusive mapper; creates `rtabmap_NAME.db` |
| `rtab_localization` | Saved `database` basename ending `.db` | Sensors; exclusive mapper |
| `amcl` | `map` basename ending `.yaml`; `family` is `slam_toolbox` (default) or `rtabmap`; `recovery` defaults false | Sensors; exclusive mapper; motion confirmation |
| `navigation` | Nav2 servers | Sensors + one mapper; conflicts with joystick; motion confirmation |
| `save_slam` | Save all four SLAM formats with required `name` | Active SLAM mapping |
| `save_rtab_map` | Save occupancy grid with required `name` | Active RTAB mapping/localization |
| `export_rtab_cloud` | Export `database` to required `name` | Both RTAB modes stopped |
| `recovery_abort` | Call AMCL recovery abort | AMCL |
| `recovery_relocalize` | Force autonomous recovery | AMCL + navigation; no joystick; motion confirmation |
| `global_localization` | Scatter AMCL particles | AMCL; motion confirmation because enabled recovery may react |
| `diagnostics_nodes` | ROS node list | None |
| `diagnostics_topics` | ROS topic list with types | None |
| `diagnostics_controller` | Controller lifecycle state | None |
| `diagnostics_planner` | Planner lifecycle state | None |
| `diagnostics_navigator` | Navigator lifecycle state | None |

The gateway deliberately uses a single mapper authority, including RTAB. RTAB gets `publish_tf_map:=true` and `map_topic:=/map` so navigation can consume the map. This excludes the runbook's optional parallel RTAB experiment. Because the upstream RTAB save script captures `/rtabmap/map`, the gateway's occupancy capture instead uses its same fixed `map_saver_cli` arguments with `/map`. Cloud export retains the upstream script and its filtering settings.

Map names contain only ASCII letters, digits, underscores, dots and hyphens, start with an alphanumeric character, contain no `..`, and have at most 80 characters. Paths never come from arbitrary client text. SLAM files live in `WORKSPACE/src/mapping_localization_pkg/slam_toolbox/maps`; RTAB files in the corresponding `rtabmap/maps`. Existing outputs are refused; symlinked map directories/files are refused. Ensure both directories exist and are writable by the gateway account. SLAM localization accepts the saved prefix (for example `slam_toolbox_lab`) or `.posegraph` filename; AMCL uses the full `.yaml` filename. No file upload/delete/rename API is provided. Trusted operator-managed map metadata and its referenced images remain a deployment responsibility.

Coordinates are finite metres bounded to ±10,000 and heading is radians within ±π. Unknown parameters and wrong JSON types are rejected. Process arguments are a fixed allowlist with separately passed validated arguments; client-provided shell code, environment overrides, launch filenames and arbitrary parameter files are unavailable. Operations inherit the administrator's pre-sourced ROS environment. Save commands may leave partial files after interruption or ROS failure: inspect them locally and choose a new output name for retry.

Launch order is motors → sensors → one mapper → navigation, or motors → sensors → joystick + mapper for manual mapping. Stop dependents before their providers. Stop RTAB gracefully before cloud export; its database flush can take time. AMCL recovery is disabled by default because upstream recovery can drive immediately after navigation becomes available. Enabling recovery requires a clear area and a local operator able to stop the robot. Nav2 has no physical deadman, lacks an independent collision monitor, and its laser cannot see obstacles lower than approximately 17.6 cm; see upstream navigation limitations.

Not exposed: package installation, builds, network/device setup, calibration, EEPROM changes, arbitrary shell commands, GUI tools, direct velocity control, and navigation goal submission. Although the README demonstrates the NavigateToPose action, safely cancelling an action requires more than terminating its CLI client; the phone does not expose it in this release. Recovery abort targets the recovery node only and is not an emergency stop for external navigation goals or external processes. Retain a physical emergency stop/local operator; deployment must be validated on the robot before use.

Validation in this change uses synthetic process runners and the fixture. No physical robot was actuated; ROS launch success, map saving, serial devices, lifecycle transitions and actual stopping distances require robot-side acceptance testing.

## Enable on the robot

First stop all laptop-launched robot stacks and command publishers. The gateway cannot take ownership of them. Build/source the complete trusted workspace according to the runbook; the gateway inherits this environment and never invokes a client-supplied shell setup command. Keep credentials, source scripts and map files controlled by the operator account. Run the gateway as an unprivileged account with the existing device permissions, not root.

```bash
cd ~/helios_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# HELIOS_GATEWAY_TOKEN is already loaded from your protected environment.
ros2 run mobile_gateway mobile_gateway --workspace "$PWD" \
  --enable-commands --exclusive-stack-control --host 127.0.0.1 --port 8080
```

`--exclusive-stack-control` is the operator's attestation that this is the only manager of the stack. Additional ROS graph checks reject recognized externally managed/duplicate nodes and unrecognized `/cmd_vel` publishers before a start. Discovery is asynchronous, so this is **not** a cross-process lock or proof that external nodes do not exist. Never start commands from a laptop or another gateway while this manager runs. Read-only mode can coexist with laptop-managed nodes. The same trusted TLS reverse proxy from the README is required for off-host clients; allow the listed POST paths as well as telemetry (example below).

A `running` job means only that the launch leader has not exited; it does not prove active lifecycle nodes, healthy sensors, finished camera calibration or a localized robot. Use telemetry and diagnostics, keep the rover still during sensor startup, and commission on hardware under local supervision.

## API and control ownership

Every route requires the same bearer authentication as telemetry. Request bodies are JSON objects, maximum 16 KiB, with `Content-Type: application/json` and one `Content-Length`. Chunked framing, duplicate fields, unknown operation parameters and nonfinite numbers are rejected. GET requests are observational and never acquire or renew control.

- `GET /v1/operations`: `{api_version,source,commands_enabled,lease,operations,jobs}`. `source` is `ros2` or `fixture`. Catalog entries contain `id,title,description,kind` (`service`/`action`), `requires_confirmation`, `requires`, `requires_any`, `conflicts`, and `parameters`. Parameter entries contain `name,type,required` and optional `default,options,min,max`.
- `POST /v1/control/heartbeat` with `{"client_id":"<client UUID>"}` acquires/renews the exclusive **10-second** lease. Returns `{api_version:1,lease:{client_id,remaining_seconds}}`. Renew every three seconds while the user actively controls the foreground app. Another owner receives 409. Viewing telemetry must not claim control implicitly.
- `POST /v1/operations/<id>/start` with `{"client_id":"...","request_id":"<unique UUID>","parameters":{},"confirm":true}` starts a job and returns HTTP 202 `{api_version:1,job:{...}}`. Confirmation is mandatory for catalog-marked operations; explain their motion effects before sending it. Dependencies must already be running; starts never implicitly launch prerequisites. ROS readiness is still an operator check.
- `GET /v1/jobs/<id>` returns `{api_version:1,job:{...}}`.
- `POST /v1/jobs/<id>/stop` with `{"client_id":"..."}` requests a graceful stop and returns HTTP 202 with the job. Active dependents cause 409; stop them first or use stop-all. Recovery/reinitialization service effects cannot be undone by killing their CLI; stopping those individual jobs returns 409 and instructs the client to use `recovery_abort` or stop-all.
- `POST /v1/operations/stop-all` with `{"client_id":"..."}` requests all owned jobs stop and returns HTTP 202 `{api_version:1,jobs:[...]}`. Poll status for completion.

Jobs have `id,operation_id,state,pid,exit_code,output,error,started_at,simulated`. States are `running,stopping,succeeded,failed,stopped,stop_failed`. Per-job detail log tails are capped at 1024 bytes; catalog/stop-all summaries return the final 128 characters, and errors the first 128 characters; history is capped at 128 jobs (restart after stopping all jobs when full). A `succeeded` action means the CLI exited successfully; ROS Trigger `success=false` is reported as failed. Save/export actions also require all expected output files to exist and be nonempty; this checks creation, not map quality. Logs and telemetry still need interpretation; a successful lifecycle query may report an inactive node. `started_at` is wall-clock Unix seconds; lease durations use monotonic time.

A unique `request_id` identifies one user intent. Repeating it with the same owner, operation, parameters and confirmation returns its existing job. Changing the intent returns 409. This protection exists only in current bounded in-memory history, not across gateway restart. Do not automatically retry mutations after network timeouts: refresh jobs to resolve the outcome. Return values never imply an operation completed merely because the HTTP request was accepted. Errors include `{error,message}`, using 400 for validation, 403 for disabled commands/missing confirmation, 404 for unknown jobs/operations, and 409 for ownership/dependency/conflict/state errors.

Lease expiration, app background/disconnect, prerequisite failure and gateway shutdown request graceful SIGINT of owned jobs. Motion producers are interrupted first, then remaining jobs in reverse launch/dependency order. All stop requests are sent before waiting, so a hung diagnostic cannot prevent interrupting motors. This is a **best-effort process shutdown, not a physical deadman or emergency stop**. It cannot prevent motion during the lease interval, guarantee OS scheduling/ROS delivery, or stop an unresponsive/external process. Stacks are deliberately not left running unattended after a mobile control lease disappears; save maps before leaving the app. SIGINT allows RTAB database flushing, but do not assume data was saved until process completion is observed.

Each launched command uses a new process session; only gateway-owned process groups are interrupted. There is no automatic SIGKILL escalation. A stop timeout or surviving group after the leader exits becomes `stop_failed`, blocks new starts, and requires local operator intervention; the gateway never signals a potentially reused PID after reaping the leader. No process ownership is reconstructed after a crash/restart. Check and clean up orphaned stacks locally before re-enabling. Process sessions that deliberately escape their group are outside this manager's control. Keep a hardware stop procedure available. A gateway kill/power loss bypasses lease cleanup; the robot driver's own watchdog and physical stop remain essential.

### Reverse proxy routes

Replace the README's exact telemetry location with this bounded location when enabling operations; retain TLS, authentication forwarding and network restrictions. Do not expose unrelated ROS services or a general proxy.

```nginx
location ~ ^/v1/(telemetry|operations|control/heartbeat|operations/stop-all|operations/[a-z_]+/start|jobs/[a-zA-Z0-9-]+(/stop)?)$ {
    proxy_pass http://127.0.0.1:8080;
    proxy_set_header Authorization $http_authorization;
    client_max_body_size 16k;
    proxy_connect_timeout 2s;
    proxy_read_timeout 5s;
}
```

The gateway defaults to loopback and must not be exposed directly to an untrusted network. Output logs may contain robot paths or ROS diagnostic details; only authorized operators should possess the bearer token.
