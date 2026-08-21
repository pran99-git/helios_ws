# custom_config

Workspace-owned configuration and launch for the Hokuyo UST-10LX.

It sits next to `urg_node2` so that everything laser-related lives in `LiDAR/`,
mirroring `Camera/custom_covariance`. But **it is our code, not the vendor's**:
`urg_node2` is a git submodule pinned to an upstream commit, and anything
written inside it is reverted by `git submodule update`. This package is outside
that boundary, which is the entire reason it exists.

```
config/urg_node2.yaml    driver parameters — IP, angle limits, frame_id
launch/lidar.launch.py   replaces the submodule's launch file
rviz/lidar_scan.rviz     layout for checking the laser on its own
```

---

## Why we own the launch file

Upstream's `urg_node2.launch.py` hardcodes reading `config/params_ether.yaml`
out of its own `share/` directory and exposes **no argument to override it**. So
the only way to change a LiDAR parameter was to edit tracked content inside the
pinned submodule — where the next `git submodule update` silently throws it
away.

`lidar.launch.py` declares the same `LifecycleNode` against
`config/urg_node2.yaml` here instead. The submodule stays pristine and is used
purely as a driver binary. Behaviour is otherwise identical to upstream's: the
node does not auto-activate, so it is driven `configure -> activate` through
event handlers.

## Why the params file is keyed `/**:`

Not `urg_node2:`, and that is deliberate.

Launch configurations leak between sibling includes in the same
`LaunchDescription`, and this driver has already come up once under the **wrong
node name** — as `/zed_node`, because the ZED include's `node_name` default was
already set in the shared context by the time the LiDAR include ran, and
`DeclareLaunchArgument` only fills a value that is not already present.

A node-name-keyed params file would have silently applied **nothing** in that
state, leaving the driver on its built-in defaults: wrong IP, wrong `frame_id`,
no error message. `/**` applies regardless of what the node ends up being called.

`bringup.launch.py` also passes `node_name:='urg_node2'` explicitly for the same
reason — belt and braces on a failure that is invisible when it happens.

---

## Running it

`sensor_fusion/launch/bringup.launch.py` includes this automatically, gated on
its `lidar:=` argument. Nothing extra to launch.

The laser on its own:

```bash
ros2 launch custom_config lidar.launch.py
ros2 launch custom_config lidar.launch.py scan_topic:=/scan_raw
```

Checking it:

```bash
ros2 topic hz /scan                              # ~40 Hz
ros2 node list                                   # must show /urg_node2
ros2 lifecycle get /urg_node2                    # active [3]
rviz2 -d $(ros2 pkg prefix --share custom_config)/rviz/lidar_scan.rviz   # Fixed Frame: laser
```

---

## Network

The scanner talks over **Ethernet**, not USB. It ships at `192.168.0.10`, so the
Jetson's wired interface must be on that subnet:

```bash
nmcli con up lidar-ethernet     # the profile already on this machine
ping -c 3 192.168.0.10          # must reply before you launch anything
```

The wired interface on this Jetson is **`end0`** — check for `NO-CARRIER` with
`ip -brief link show end0` if the ping fails. A `could not open ethernet port`
error at launch usually just means the scanner's TCP port was not ready yet;
wait a few seconds and relaunch.

---

## Related

- `sensor_fusion/launch/bringup.launch.py` — includes this launch file
- `helios_description/urdf/sensors.xacro` — `laser_mount_joint`, slam_toolbox's
  scan-matching lever arm. Hand-measured; re-check it after any remount, and
  note its `rpy="0 0 0"` assumes the cable and dead zone face aft
- `Camera/custom_covariance/` — the same pattern on the camera side
