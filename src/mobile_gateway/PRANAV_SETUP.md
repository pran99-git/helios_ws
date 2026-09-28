# Pranav's Helios phone setup and handoff

Follow this once, in order: **merge and update the Jetson → prove local telemetry → add trusted HTTPS → install the iPhone app → verify telemetry → optionally enable supervised commands**. The gateway is code to deploy, not a service already running on your robot. No robot address, certificate, ROS domain, Apple team or production token has been provisioned for you.

The gateway contribution is [helios_ws PR #1](https://github.com/pran99-git/helios_ws/pull/1). The companion app is in the **private** [tanmayg1502/helios-ios repository](https://github.com/tanmayg1502/helios-ios). The two repositories are built/deployed separately. This guide is the sequential handoff; [gateway API details](README.md), [operation details](OPERATIONS.md), and the existing [robot runbook](../../launch_helios.md) provide deeper references.

## 1. Fill in these deployment values — required

Keep this table in your own deployment notes. Do not put tokens/private keys into GitHub, screenshots, issue comments or shared logs.

| Value | What you must supply |
|---|---|
| Jetson account and workspace | Existing non-root robot operator account; absolute workspace path, e.g. `/home/YOUR_USER/helios_ws` |
| Current robot software | Working Ubuntu/ROS installation, JetPack/CUDA and matching ZED SDK; the repository targets ROS 2 Jazzy |
| ROS discovery environment | Actual `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, discovery restrictions/configuration used by your publishers; leave an unset value unset rather than guessing |
| Robot network | Jetson LAN/VPN address, phone-reachable route, and DNS hostname you control |
| HTTPS identity | Hostname matching the certificate, full certificate chain, private-key paths, renewal plan, and trust on the iPhone |
| Phone access | Trusted LAN/VPN only; firewall approval for HTTPS, with SSH access preserved |
| App repository access | Pranav's GitHub account invited to the private iOS repository; accept the invitation before cloning |
| Apple development | Mac with Xcode 26.4; iPhone on iOS 26+; Apple development team and a unique bundle identifier you can sign |
| Robot supervision | Local operator, clear test area, known hardware stop procedure, power/device readiness |

**Do not use `192.168.0.10` as the app address.** It is the documented Hokuyo scanner address, not a Jetson HTTP gateway. Likewise, `localhost` on a physical phone means the phone. Examples below use `robot.example.net` as a placeholder; replace it everywhere with your actual host.

## 2. Review/merge and update the Jetson without losing work — required

Review PR #1 and merge it into **`pran99-git/helios_ws` → `main`** through the normal GitHub workflow. If it is already merged, skip the merge. These instructions assume the PR is merged; do not deploy an unreviewed branch by accident.

On the Jetson, first stop running robot processes through their existing operator procedure before changing software. Inspect the working copy, including submodules:

```bash
export HELIOS_WS="$HOME/helios_ws"   # Replace if your workspace is elsewhere.
cd "$HELIOS_WS"
git remote -v
git status --short
git branch --show-current
git submodule status --recursive
git submodule foreach --recursive 'git status --short'
```

**If there are local modifications, untracked work, local commits, or dirty submodules, preserve/reconcile them before switching or updating.** A parent-repository stash does not preserve all submodule work or ignored maps. Do not run `reset --hard`, `clean`, or forced submodule updates. Back up saved maps/databases as well as code; the map directories are intentionally Git-ignored. One simple full backup, if you have enough disk space, is:

```bash
# Copies the whole stopped workspace, including .git, submodules and ignored maps.
# Destination is a new sibling directory; this may be large.
cp -a "$HELIOS_WS" "${HELIOS_WS}.backup-$(date +%Y%m%d-%H%M%S)"
```

Once local work is safely accounted for and the checkout can be updated, verify which remote points to the original repository. The commands below assume **`origin` is exactly `https://github.com/pran99-git/helios_ws.git` (or its SSH equivalent)**. If not, substitute the verified upstream remote name; do not silently replace your remotes.

```bash
git fetch origin
git switch main
git merge --ff-only origin/main
git submodule sync --recursive
git submodule update --init --recursive
test -f src/mobile_gateway/package.xml
git log -1 --oneline
```

If fast-forward fails, stop and reconcile the branch; the guide does not discard divergent work. Do not use `git submodule update --remote`: use the revisions pinned by this workspace. Record the deployed commit in your notes. Both the ZED wrapper and Hokuyo driver are submodules.

## 3. Confirm existing robot prerequisites — required

This feature does not install/reconfigure the robot's operating system, SDK or devices. Confirm the established stack works according to the [perception instructions](../perception_pkg/README.md) and [motor/gamepad instructions](../low_level_control_pkg/README.md):

- ROS 2 Jazzy and the existing build tooling are available. The ZED SDK matches the installed JetPack/CUDA version; do not upgrade these blindly to install the app gateway.
- The installed udev configuration identifies the correct left/right RoboClaw ports; the gateway account has their existing read/write permissions. Do not substitute world-writable devices or run the gateway as root.
- Hokuyo Ethernet and ZED connectivity work. The first ZED start may spend minutes optimizing models.
- For joystick use, the paired gamepad is available as the documented `/dev/input/js0`, with the established axis mapping.
- Saved map directories are real directories (not symlinks), writable by this account, and have space for new maps/databases.

Read-only preflight examples, using the scanner address documented in this repository:

```bash
ls -l /dev/roboclaw_left /dev/roboclaw_right
ls -l /dev/input/js0     # Only required for gamepad operation.
ping -c 2 192.168.0.10
ip -brief address
```

A failed check is a device/network issue to fix with the existing runbook; it is not an app authentication issue. Do not open RoboClaw serial ports with a second process while a driver is running.

## 4. Build and source the workspace — required

Use a fresh Bash terminal. Preserve your known ROS domain/RMW/discovery settings; use the same settings for both the gateway and robot publishers in every terminal.

```bash
export HELIOS_WS="$HOME/helios_ws"   # Set your actual path in each new Jetson terminal.
cd "$HELIOS_WS"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/mobile_gateway --ignore-src -r -y
colcon build --packages-select mobile_gateway --symlink-install
source install/setup.bash
ros2 pkg prefix mobile_gateway
PYTHONPATH=src/mobile_gateway python3 -m unittest discover -s src/mobile_gateway/test -v
```

The targeted build assumes the existing robot workspace has already been built successfully. **For a new/unbuilt workspace, or if other robot packages changed**, resolve its dependencies and build the full stack using the repository's existing SDK/setup prerequisites:

```bash
export HELIOS_WS="$HOME/helios_ws"   # Set your actual path in each new Jetson terminal.
cd "$HELIOS_WS"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Do not ignore unresolved rosdep/build errors or delete the working environment to hide them. `rosdep` does not replace the vendor ZED SDK installation. Confirm the full command stack's packages are installed before enabling operations:

```bash
for pkg in mobile_gateway low_level_control_pkg sensor_fusion mapping_localization_pkg navigation_pkg; do
    ros2 pkg prefix "$pkg"
done
```

## 5. Create one durable production token — required

Generate it **once**, retain it securely, and reuse it across gateway restarts. This creates a permissions-restricted file and refuses to overwrite an existing token:

```bash
python3 - <<'PY'
import os
from pathlib import Path
import secrets
folder = Path.home() / '.config' / 'helios'
folder.mkdir(parents=True, exist_ok=True)
folder.chmod(0o700)
path = folder / 'gateway.env'
fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
with os.fdopen(fd, 'w') as stream:
    stream.write('export HELIOS_GATEWAY_TOKEN=' + secrets.token_urlsafe(32) + '\n')
print('Created protected gateway.env; token was not printed.')
PY
source "$HOME/.config/helios/gateway.env"
```

If the file already exists, reuse it with `source` rather than regenerating it. Retrieve only the token value in a trusted local editor/secure transfer when entering it into the phone. Never use the fixture's public test token for the robot. Avoid `set -x`, printing the environment, putting a token in a URL, or recording it in screenshots. The app currently keeps the URL/token **in memory**, so expect to re-enter them after relaunch; it does not provide persistent credential storage.

For rotation, stop managed work and the gateway, deliberately replace the protected file with a new generated token, restart, and update authorized phones. All holders of this shared token have operator authority when commands are enabled.

## 6. Prove loopback telemetry first — required

Start **telemetry-only**, without operation-enable flags, in a foreground Jetson terminal:

```bash
export HELIOS_WS="$HOME/helios_ws"   # Set your actual path in each new Jetson terminal.
cd "$HELIOS_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source "$HOME/.config/helios/gateway.env"
ros2 run mobile_gateway mobile_gateway --host 127.0.0.1 --port 8080
```

In a second Jetson terminal, load the same token and read both endpoints without putting the token in a command-line argument:

```bash
source "$HOME/.config/helios/gateway.env"
python3 - <<'PY'
import json, os, urllib.request
for route in ('telemetry', 'operations'):
    req = urllib.request.Request('http://127.0.0.1:8080/v1/' + route,
        headers={'Authorization': 'Bearer ' + os.environ['HELIOS_GATEWAY_TOKEN']})
    with urllib.request.urlopen(req, timeout=5) as response:
        data = json.load(response)
    if route == 'telemetry':
        print(json.dumps(data, indent=2))
    else:
        print('commands_enabled:', data['commands_enabled'])
PY
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8080/v1/telemetry
```

Expect `api_version: 1`, `source: "ros2"`, `commands_enabled: false`, and **401** for the final unauthenticated curl request. If no sensors are running, `available: false` is correct. `source: ros2` identifies the adapter, not proof that physical sensors are producing data.

With the gateway still telemetry-only, use the robot's existing supervised runbook to start/observe the known-working sensor stack if it is not already running. Starting that stack may include the motor driver; keep its normal hardware precautions. In a correctly sourced diagnostic terminal:

```bash
ros2 topic type /odometry/filtered   # nav_msgs/msg/Odometry
ros2 topic type /scan                # sensor_msgs/msg/LaserScan
ros2 topic hz /odometry/filtered     # Observe, then Ctrl-C.
ros2 topic hz /scan                  # Observe, then Ctrl-C.
```

The JSON should become available with the actual frame IDs and plausible SI values. Values older than two seconds since gateway receipt become unavailable. Receipt freshness does not validate the sensor timestamp; no finite laser return is `nearest_m: null`, not guaranteed clear space. Keep odometry separate from map coordinates. For actual alternate topic names, restart with `--ros-args -p odometry_topic:=/YOUR_ODOM_TOPIC -p scan_topic:=/YOUR_SCAN_TOPIC`.

## 7. Add trusted HTTPS and network access — required for a phone

The phone needs a route to the **Jetson**, not to the lidar subnet. Choose your existing trusted LAN/VPN; ensure the phone can resolve the chosen hostname to the reachable Jetson/proxy address. Reserve that address or manage DNS so it remains stable. Keep TCP 8080 bound to loopback. Permit TCP 443 only from the intended LAN/VPN and preserve SSH access; do not blindly enable/reset a firewall over SSH or port-forward this service to the public Internet.

Provision a valid server certificate for your real hostname, its full chain and private key through your network administrator or established certificate tooling. Plan renewals. A public CA certificate for a domain you control or a managed private CA can work; the certificate must match the URL and be trusted by the **iPhone**, not merely by the Mac. A manually installed private CA profile may need explicit SSL trust; follow [Apple's certificate-trust instructions](https://support.apple.com/en-ie/102390). Do not use `curl -k`, ATS exceptions or disabled certificate validation as the solution.

If using nginx on the Jetson, install it through your approved OS package process (`sudo apt install nginx` on a suitable Ubuntu host if it is not already installed). The following is a complete **server block inside nginx's `http` context**, for example a dedicated site file. Replace `robot.example.net` and both certificate paths before enabling it. Preserve existing nginx sites and resolve any listener conflicts.

```nginx
server {
    listen 443 ssl;
    server_name robot.example.net;
    ssl_certificate /ABSOLUTE/PATH/TO/fullchain.pem;
    ssl_certificate_key /ABSOLUTE/PATH/TO/privkey.pem;
    ssl_protocols TLSv1.2 TLSv1.3;
    access_log off;
    client_header_timeout 5s;
    client_body_timeout 5s;
    client_max_body_size 16k;

    # Both telemetry/catalog GETs and operation POSTs reach the gateway.
    # Method, token, lease and body validation still happen in the gateway.
    location ~ ^/v1/(telemetry|operations|control/heartbeat|operations/stop-all|operations/[a-z_]+/start|jobs/[a-zA-Z0-9-]+(/stop)?)$ {
        proxy_pass http://127.0.0.1:8080;
        proxy_set_header Authorization $http_authorization;
        proxy_connect_timeout 2s;
        proxy_read_timeout 5s;
    }
    location / { return 404; }
}
```

After placing/enabling the site according to the host's nginx layout:

```bash
sudo nginx -t
# Reload only after the test succeeds.
sudo systemctl reload nginx
```

From the Mac on the same intended LAN/VPN, verify DNS/routing/TLS **without sending a token**:

```bash
curl -sS -o /dev/null -w '%{http_code}\n' https://robot.example.net/v1/telemetry
curl -sS -o /dev/null -w '%{http_code}\n' https://robot.example.net/v1/operations
```

Both should return **401**, not a certificate error, redirect, 404 or 502. A private CA may require trust setup on the Mac too; Mac success does not prove iPhone trust. All API routes are served at this same origin; the app refuses redirects. A telemetry-only proxy location is insufficient for the Operations tab.

## 8. Get and install the iOS app — required

On the Mac, sign into GitHub with the invited account and accept the private repository invitation. A 404 for the app repository usually means the invitation/account access is missing, not that the robot gateway is down. Clone into a new directory:

```bash
git clone https://github.com/tanmayg1502/helios-ios.git
cd helios-ios
open Helios.xcodeproj
```

In **Xcode 26.4**:

1. Add your Apple account/team in Xcode settings. Select the **Helios** app target → **Signing & Capabilities**; choose your team, enable automatic signing if appropriate, and set a unique app bundle identifier. Keep the test targets' identifiers distinct if running tests. No signing identity/team was provisioned in the repository.
2. Pair/connect the iPhone, unlock it, and follow its trust prompts. Enable Developer Mode if required; see [Apple's Developer Mode instructions](https://developer.apple.com/documentation/xcode/enabling-developer-mode-on-a-device).
3. Select the **Helios** scheme and the physical iPhone as run destination, then Run. The target requires **iOS 26+**. Resolve signing/provisioning errors with your team's entitlements/device access; a private GitHub invitation does not grant Apple signing access.

Every build starts in real robot mode. Developer tools are a separate app setting, distinct from iOS device Developer Mode: **Connect → Enable developer mode → Enable simulation**, visible only in Debug or eligible dedicated TestFlight builds. Backgrounding exits this opt-in. See [distribution policy](../../docs/mobile-app/DEVELOPER_MODE.md).

The checked-in Xcode project is ready to open. XcodeGen is optional and needed only if you change `project.yml`; preserve signing changes when regenerating. This is a developer installation, not an App Store/TestFlight release; signing/provisioning lifetime depends on your team. For Xcode's device workflow, see [Apple's run-on-device guide](https://help.apple.com/xcode/mac/current/en.lproj/dev5a825a1ca.html).

## 9. Connect the phone in telemetry-only mode — required

Keep the gateway and proxy running. Put the phone on the intended LAN/VPN, then:

1. Open **Connect**.
2. Enter the base URL, e.g. `https://robot.example.net`. **Do not append `/v1/telemetry`**, a query, or credentials.
3. Enter the exact token **value** from the protected file, without `Bearer ` or `export HELIOS_GATEWAY_TOKEN=`.
4. Tap **Connect** and allow Local Network access if iOS asks.
5. Open **Overview** and verify frames, SI values and freshness against the Jetson's readings. Missing sensors must look unavailable. The **Map** tab says “Live map unavailable”; the gateway does not provide captured maps.
6. Open **Operations** to verify the catalog loads but commands are disabled. Do not acquire control at this stage.

Disconnect once and check readings clear, then reconnect. Closing/backgrounding the app clears live readings; restarting it loses the in-memory credentials. Real mode rejects `fixture` and unknown sources. Debug or eligible internal TestFlight builds can explicitly opt into fixture-only developer mode; see [developer-mode setup](../../docs/mobile-app/DEVELOPER_MODE.md). Production Release has no developer tools. Never use off-device plain HTTP or the public fixture token for the real robot.

## 10. Transfer to managed operations — optional, only after steps 1–9 pass

This is a deliberate change from observing existing laptop-run processes to letting the gateway own them. Keep a local operator and hardware stop procedure available. Starting motors enables reception of `/cmd_vel`; enabling navigation/AMCL recovery can cause autonomous motion. Phone controls are not an emergency stop.

1. Stop all laptop-launched robot stacks in the runbook's reverse order: navigation, mapping/localization, joystick, sensors, motors. Let RTAB flush; inspect for leftovers. Never start a duplicate motor driver.
2. Ctrl-C the **telemetry-only** gateway. Do not leave a second gateway on another port.
3. Verify the full workspace/device permissions and map directories from steps 3–4. Resolve orphaned/externally launched processes locally; the gateway cannot adopt or stop them.
4. Start the managed gateway in the same foreground terminal and trusted ROS environment:

```bash
export HELIOS_WS="$HOME/helios_ws"   # Set your actual path in each new Jetson terminal.
cd "$HELIOS_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source "$HOME/.config/helios/gateway.env"
ros2 run mobile_gateway mobile_gateway --workspace "$HELIOS_WS" \
  --enable-commands --exclusive-stack-control --host 127.0.0.1 --port 8080
```

`--exclusive-stack-control` is your attestation that no other terminal/gateway controls this stack. ROS graph checks help reject duplicates but cannot establish exclusivity across races/hidden nodes. Do not launch additional robot commands from a laptop while the gateway owns the stack. Read-only diagnostics remain useful.

## 11. First supervised startup and shutdown — optional managed-mode acceptance

Keep the phone awake and the app **foreground**. In **Operations**, explicitly tap **Acquire control session**; simply viewing telemetry/catalog does not claim the lease. Check **Control session active**. Another controlling client must finish/release by lease expiry before you can acquire it.

Start one stage at a time, read confirmation dialogs, and inspect job output/telemetry before the next:

1. **Motor driver**: confirm the movement-capable operation. Inspect startup/QPPS and driver output using the established runbook; `running` means only the launch leader is alive.
2. **Sensors and odometry**: normally keep camera/lidar enabled; hold the rover still for the initial five seconds. Wait for real odometry/scan. Initial ZED model optimization can take longer.
3. **One mapper/localizer** from the table below. The gateway deliberately permits only one map-transform authority, including RTAB.
4. For manual operation, **Joystick teleoperation** uses the physical controller and its shoulder-button deadman. For autonomous-stack commissioning, stop joystick first, then start **Navigation servers** with explicit confirmation. The two modes conflict. Starting Nav2 is not sending a goal; phone navigation-goal submission is unsupported.
5. Use node/topic lists and controller/planner/navigator lifecycle diagnostics. A successful query can report an inactive node; the CLI's `succeeded` does not mean navigation is ready.

To shut down, save desired map outputs first, then use **Stop managed processes** (stop-all). Observe jobs reaching stopped/terminal states; do not equate HTTP acceptance with completion. Individual stops reject active dependents. Failed stops require local intervention, not repeated blind starts. RTAB needs graceful shutdown for its database; no automatic SIGKILL escalation is used.

**Foreground/lease behavior:** the app renews every three seconds. Ten seconds without renewal requests gateway-owned shutdown. Locking the phone, changing apps, backgrounding, disconnecting, or switching robot/developer modes stops renewal; foregrounding does not silently reacquire. Save before leaving. Stop-all is the intentional shutdown path; lease expiry is a fallback, not an instantaneous physical stop. Network/OS/gateway failure can prevent cleanup and external processes are outside this control.

## 12. Mapping, localization and recovery recipes — optional

All names are simple basenames, maximum 80 ASCII letters/digits/underscores/dots/hyphens, beginning alphanumeric, with no `..` or path separators. Use a new name per save; overwrite is refused. Files live under the configured workspace:

| Phone operation | Inputs and expected files/order |
|---|---|
| SLAM mapping → Save SLAM map and graph | Choose save name `lab_run`; creates `slam_toolbox/maps/slam_toolbox_lab_run.{pgm,yaml,posegraph,data}` under `src/mapping_localization_pkg/` |
| SLAM localization | Stop the previous mapper; use `map=slam_toolbox_lab_run` (or `.posegraph` filename); matching `.data` required. Supply initial `x,y,heading` in metres/radians appropriate to the saved map |
| AMCL localization | Stop previous mapper; `family=slam_toolbox`, `map=slam_toolbox_lab_run.yaml`. Keep `recovery=false` for initial commissioning. Manual localization still requires the existing robot tools/procedure |
| RTAB mapping | Choose new `name=lab_run`; writes `rtabmap/maps/rtabmap_lab_run.db`; gateway owns map TF and publishes occupancy on `/map` |
| Save RTAB occupancy map | **While RTAB is running**, choose save name `lab_run`; creates `.pgm` + `.yaml` with `rtabmap_lab_run` prefix |
| Export stopped RTAB cloud | Gracefully stop RTAB and wait for completion first. Set `database=rtabmap_lab_run.db`, new output `name=lab_cloud`; creates `rtabmap_lab_cloud_cloud.ply` |
| RTAB localization | Stop other mapper; `database=rtabmap_lab_run.db` |
| AMCL against RTAB grid | `family=rtabmap`, `map=rtabmap_lab_run.yaml` |

Wait for successful save/export results and confirm the expected nonempty files locally before relying on them. Files being nonempty does not establish map quality. Back up maps separately; they are ignored by Git. Partial failed saves require local inspection and a new name; there is no phone upload/delete/rename browser.

AMCL `recovery=true` is a separate motion-capable choice. With Nav2 present it can spin/drive without a physical deadman, including on startup/loss of localization. **Force AMCL recovery** requires AMCL + navigation; **Abort AMCL recovery** targets only that recovery behavior. **Scatter AMCL particles** may trigger enabled recovery. Killing a service CLI cannot undo its effect, so individual stop for recovery/reinitialization requests is rejected: use the recovery-abort operation or stop-all as appropriate. Neither replaces the hardware stop. Upstream full autonomous driving remains unvalidated and the laser plane misses sufficiently low obstacles.

## 13. Persistence, reboot and normal reuse

This handoff uses a **foreground gateway**, not an installed gateway system service. Keep that terminal/session alive. Ending it unexpectedly or rebooting stops availability; a crash may leave child processes, and ownership is not reconstructed after restart. Do not assume SSH loss cleanly stops all jobs. Prefer the local console during commissioning; if using a persistent terminal tool, keep responsibility for the process explicit.

After each reboot/session, recheck device/network state, source ROS and `install/setup.bash`, load the same protected token, and start the gateway in the intended mode. Reconnect the phone and explicitly reacquire control when wanted. Job history/leases are in memory and reset on restart; map files persist. Nginx may be a system service, but that does not start the gateway or robot stack.

An automatic gateway service is **not configured by this change**. Design one only after hardware acceptance, with the correct unprivileged account, pre-sourced ROS environment, protected token, adequate graceful-stop window, and restart/orphan policy. Do not add an unconditional auto-restart service for motion-capable stacks as a shortcut. Software updates/token rotation require planned managed shutdown first.

## 14. Troubleshooting

| Symptom | Check / action |
|---|---|
| `mobile_gateway` package missing | PR merged and pulled into the actual workspace? Build passed? Same terminal sourced `install/setup.bash`? |
| Build/SDK error | Resolve existing Jazzy/vendor dependencies; do not treat ZED SDK as a pip dependency or suppress build failures |
| Local connection refused | Gateway running? Port conflict? `ss -ltnp` on Jetson should show loopback `127.0.0.1:8080` |
| 401 | Same durable token in process and app? Enter value only; restart gateway after rotation. Do not paste secrets into bug reports |
| TLS/DNS/connect error | Phone LAN/VPN route, hostname resolution, certificate name/full chain/expiry and phone trust; correct device clock. No insecure bypass |
| 502 through nginx | Proxy reaches wrong address/port or foreground gateway exited |
| Telemetry works, Operations 404/405 | Proxy must include operation/job/heartbeat routes and POST forwarding; use the complete block in step 7 |
| Commands disabled (403) | Expected in initial mode; follow explicit takeover in step 10 before enabling |
| Missing/stale sensors | Publishers active, matching ROS domain/RMW/discovery/QoS and topics? Inspect actual ROS data; don't invent zero readings |
| Lease required/owned | Explicitly Acquire in foreground; wait for other owner's expiry. Backgrounding stops renewal; don't repeatedly retry commands |
| Dependencies/conflict/external-stack error | Read job/catalog prerequisites; stop joystick before Nav2 and previous mapper before another. Resolve outside processes locally |
| `running` but no robot behavior | Launch-leader state is not ROS readiness; refresh bounded output, lifecycle diagnostics and local logs |
| Command timeout/unknown result | Refresh jobs first. Client does not retry mutations automatically; accepted commands can outlive canceled HTTP requests |
| `stop_failed` / orphan / history full | Use local operator procedure, inspect/stop leftovers without blind force-kills, then restart gateway only after all work is accounted for. History limit is 128 jobs |
| Map path/overwrite error | Supply basename in correct family, not an absolute path; ensure matching files, no symlinks, writable directory and a new output name |
| Private iOS repo 404 | Repository invitation/account access; ask its owner for access |
| Xcode signing/install failure | Correct team, bundle identifier, device provisioning, trust/Developer Mode, supported OS; not a ROS issue |
| iOS Local Network denied | Enable Helios Local Network permission in iOS settings; verify VPN policy allows the robot route |
| Gateway mode mismatch | Real mode requires the actual ROS gateway. Fixture testing requires eligible developer mode; return to real robot mode before hardware acceptance |

## 15. Acceptance and what remains unverified

Record the gateway and app commits, robot environment/domain, hostname/certificate expiry, operator and test date. Do not record the token.

- [ ] Updated workspace/submodules and preserved local code/maps; build and local tests pass.
- [ ] Loopback authenticated telemetry works; unauthenticated requests return 401; commands initially disabled.
- [ ] Trusted HTTPS works from the intended phone network; 8080 is private; complete routes are configured.
- [ ] Signed app runs on physical iPhone; correct URL/token, local-network permission, real frames/telemetry and unavailable states verified.
- [ ] For managed mode: external stacks stopped, explicit exclusive takeover, lease/confirmations/dependencies inspected, each startup stage supervised.
- [ ] Map outputs checked if used; stop-all, failure handling and lease loss tested in a controlled robot acceptance procedure with a local hardware stop available.
- [ ] Operator understands reboot/foreground limits and has a token/certificate renewal and map-backup plan.

Initial integration validation covered **54 gateway tests, a package wheel build, Python 3.12 CI, 10 app unit tests, two iOS simulator UI tests, and production-client fixture/fault harnesses**. Operations tests used simulated/mocked processes; no physical robot was actuated. See [the latest app validation record](../../docs/mobile-app/VALIDATION.md) for subsequent developer-mode checks. These do not prove Jetson ROS discovery, real map saving, serial access, physical stopping, phone TLS/signing or autonomous navigation.

Unsupported in this release: arbitrary terminal commands, package/OS administration, calibration/EEPROM writes, direct phone joystick/velocity control, navigation goal submission/cancellation, camera video, live map streaming, battery telemetry and physical emergency-stop certification. The phone can manage the documented software catalog; it is not a replacement for the robot's hardware stop and supervised commissioning.
