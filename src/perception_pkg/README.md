# perception_pkg

Everything sensing-related: read every sensor, convert the raw readings into
something meaningful, and fuse them into a single estimate of where the robot
is. The layer stops there: it produces `/odometry/filtered` and hands off to
`mapping_localization_pkg`.

> **`perception_pkg` is a directory, not a ROS package.** There is no
> `package.xml` here. It groups seven things: five ROS packages you build and
> launch by name (`sensor_fusion`, `wheel_odometry`, `custom_covariance`,
> `zed_custom_tuning`, `custom_config`) and two third-party drivers vendored as
> git submodules. `colcon` finds the packages nested inside automatically,
> however deep.

**Read [the root README](../../README.md) first** for the system overview.

---

## What's in here

```
perception_pkg/
├── sensor_fusion/           ROS package: the EKF and the whole-stack bring-up
│   └── rviz/                sensor-check layouts (see below)
├── wheel_odometry/          ROS package: encoder counts to position estimate
├── camera/
│   ├── zed-ros2-wrapper/    git submodule: Stereolabs ZED 2i driver
│   ├── custom_covariance/   ROS package, OURS. Fixes the ZED's missing
│   │                        twist covariance before the EKF sees it.
│   └── zed_custom_tuning/   ROS package, OURS. Our departures from
│                            Stereolabs' shipped parameter defaults.
└── lidar/
    ├── urg_node2/           git submodule: Hokuyo driver
    └── custom_config/       ROS package, OURS. The Hokuyo's parameters,
                             launch file and laser-only RViz layout.
```

**Everything for one sensor lives in that sensor's folder**, and each folder has
the same internal boundary: the submodule is vendor content that
`git submodule update` will revert, and the package(s) beside it are ours and
safe to edit. Anything we write about a sensor belongs on our side of that line.
That is the whole reason those packages exist.

`camera/` has two of ours rather than one because the camera needs two
different kinds of thing: a *running node* (`custom_covariance`) and a *config
layer* (`zed_custom_tuning`). Folding them together would produce a package
whose name could not honestly describe its contents.

What stays in `sensor_fusion/` is the fusion layer itself: the EKF, its config,
the whole-stack bring-up, and the one RViz layout that spans both sensors.

`perception_pkg/` itself is a plain container directory, **not** a ROS package:
it has no `package.xml`, so nothing placed directly at this level is installable
or reachable through `ros2 launch`. Everything must live inside one of the
packages below it.

| Component | Read this |
|---|---|
| `sensor_fusion` | [README](sensor_fusion/README.md): the EKF, transform ownership, bring-up |
| `wheel_odometry` | [README](wheel_odometry/README.md): mecanum kinematics, calibration |
| `custom_covariance` | [README](camera/custom_covariance/README.md): why the ZED's odometry must be republished before the EKF can fuse it |
| `zed_custom_tuning` | [README](camera/zed_custom_tuning/README.md): our ZED parameter overrides, and which ones the wrapper silently ignores |
| `custom_config` | [README](lidar/custom_config/README.md): why the Hokuyo's parameters and launch live outside the submodule |
| ZED wrapper | Upstream [README](camera/zed-ros2-wrapper/README.md) |
| Hokuyo driver | Upstream [README](lidar/urg_node2/README.md) |

There are two RViz layouts for checking things before involving mapping:
`lidar_scan.rviz` (laser alone) installs with `custom_config`, and
`visual_odometry_with_lidar.rviz` (camera and laser together) installs with
`sensor_fusion`. Both load by package path rather than source-tree path. See
the Verify section below.

---

## How the pieces connect

```
  RoboClaw driver                    ZED 2i                    Hokuyo
  (low_level_control_pkg)            (submodule)              (submodule)
        │                               │                          │
        │ roboclaw/wheel_encoders       │ /zed/zed_node/odom        │ /scan
        │ (raw counts)                  │ (visual-inertial odom)    │
        ▼                               │                          │
  wheel_odometry_node                   │                          │
        │                               │                          │
        │ /wheel/odometry               │                          │
        ▼                               ▼                          │
     ┌───────────────────────────────────────┐                     │
     │   ekf_filter_node  (sensor_fusion)    │                     │
     └───────────────────────────────────────┘                     │
                        │                                          │
                        │ /odometry/filtered                       │
                        │ + TF odom → base_link                    │
                        ▼                                          ▼
              ═══════════ consumed by mapping_localization_pkg ═══════════
```

Two fusion decisions are worth stating up front, because they explain choices
you will see in the config:

**Only velocities are fused, never absolute positions.** Both odometry sources
report where they think the robot is *and* how fast it is moving. The position
estimates drift, which is unavoidable for dead reckoning. The velocity estimates
do not accumulate error. So the EKF takes velocities from both sources and
integrates them itself, which keeps a single consistent position estimate rather
than two competing ones.

**The IMU is not fused separately.** The ZED's `/odom` is already
visual-*inertial*: the SDK fuses camera and IMU internally. Feeding the raw IMU
into the EKF as well would count the same measurement twice, making the filter
overconfident. The raw IMU block in `ekf.yaml` is commented out for this reason.

---

## Hardware setup

Both sensors need one-time setup before any ROS command will work.

### Hokuyo UST-10LX (2D laser scanner)

The scanner talks over **Ethernet**, not USB. It ships at `192.168.0.10`, so
the Jetson's wired interface must be on the same `192.168.0.0/24` subnet.

```bash
ping 192.168.0.10      # must reply before you launch anything
```

If it does not reply: check the cable, then check your interface has an address
on that subnet (`ip addr`). Assign one if needed:

```bash
nmcli con up lidar-ethernet                  # the profile already on this machine
sudo ip addr add 192.168.0.1/24 dev end0     # or by hand
```

> The wired interface on this Jetson is **`end0`**, not `eth0`. Check for
> `NO-CARRIER` with `ip -brief link show end0` if the ping fails. That means
> the cable is unplugged or the scanner is unpowered.

Connection settings live in
[`lidar/custom_config/config/urg_node2.yaml`](lidar/custom_config/config/urg_node2.yaml)
(`ip_address`, `ip_port: 10940`, `frame_id: laser`), **not** in the submodule's
own `params_ether.yaml`, which upstream's launch file reads and ours does not.
`frame_id: laser` must stay as-is: it matches the frame name in
`helios_description/urdf/sensors.xacro`.

### ZED 2i (stereo depth camera)

1. Install the **ZED SDK** from Stereolabs, matching this JetPack/CUDA version.
   The ROS wrapper is only a thin layer over it and will not build or run
   without it.
2. Plug into a **USB 3.0** port (blue connector). USB 2.0 enumerates the camera
   but cannot carry the bandwidth.
3. Confirm the SDK sees it: `ZED_Explorer` (live view) or `ZED_Diagnostic`.

On the first launch after installing, the SDK optimises its AI depth models for
this GPU. That takes several minutes and only happens once; the camera appears
frozen while it runs. This is also why the EKF may log a one-off "failed to meet
update rate" warning at startup.

The vendor's camera settings live in `camera/zed-ros2-wrapper/zed_wrapper/config/`
(`common_stereo.yaml` shared, `zed2i.yaml` model-specific). **Read** those, but
do not edit them: they are inside the pinned submodule. Our changes go in
[`camera/zed_custom_tuning/config/zed_overrides.yaml`](camera/zed_custom_tuning/config/zed_overrides.yaml),
which bring-up passes to the wrapper as `ros_params_override_path` so it wins
over both. Note that a handful of parameters, `publish_tf` among them, are
applied from launch arguments *after* that file and so cannot be set in it; the
[`zed_custom_tuning` README](camera/zed_custom_tuning/README.md) lists them.

The setting that matters most for performance is `depth_mode`; it is currently
`NEURAL_LIGHT`, which is accurate but is the single largest CPU and GPU
consumer on the robot.

### Submodules

```bash
git submodule update --init --recursive     # if you cloned without --recurse-submodules
```

Both are pinned to upstream releases (ZED wrapper `v5.4.0`, `urg_node2`
`ver1.0.0-3`). Keep them clean: local patches here are invisible to this repo's
history and get silently reverted.

---

## Build

```bash
cd ~/helios_ws
colcon build --symlink-install
source install/setup.bash
```

To build only this layer:

```bash
colcon build --packages-select wheel_odometry sensor_fusion \
    custom_covariance zed_custom_tuning custom_config --symlink-install
```

The two drivers build from their submodule sources as part of the normal
workspace build. `--symlink-install` means edits to launch files, YAML, and
Python nodes take effect without rebuilding.

---

## Run

Normally you run the whole layer in one command:

```bash
ros2 launch sensor_fusion bringup.launch.py
```

That starts the robot model, wheel odometry, both drivers, and the EKF. Details
and arguments are in the [`sensor_fusion` README](sensor_fusion/README.md).

For bringing up one sensor on its own, useful when something is broken and you
want to know which half:

```bash
ros2 launch custom_config lidar.launch.py                          # laser only
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i   # camera only
```

Both are lifecycle nodes that configure and activate themselves on launch, so
no manual transition is needed.

Use `custom_config lidar.launch.py`, **not** `urg_node2 urg_node2.launch.py`.
The upstream one reads its parameters from inside the pinned submodule; ours
reads them from `lidar/custom_config/config/urg_node2.yaml`, which is where this
workspace's LiDAR settings actually live.

---

## Verify

**Laser is publishing:**

```bash
ros2 topic hz /scan            # ~40 Hz
ros2 topic echo /scan --once    # ranges[] mostly finite, frame_id: laser
rviz2 -d $(ros2 pkg prefix --share custom_config)/rviz/lidar_scan.rviz  # Fixed Frame: laser
```

In RViz the scan should trace the room outline. All-`inf` ranges usually means
the scanner is pointing at nothing within 10 m; a scan rotated relative to the
room points at `laser_mount_joint` in the URDF.

**Camera is publishing:**

```bash
ros2 topic hz /zed/zed_node/rgb/color/rect/image      # ~30 Hz
ros2 topic hz /zed/zed_node/odom                      # ~50 Hz
ros2 topic echo /zed/zed_node/pose/status --once      # tracking state
```

Main topics: `/rgb/color/rect/image` (rectified colour),
`/depth/depth_registered` (depth aligned to the colour image),
`/point_cloud/cloud_registered` (3D points), `/odom` (visual-inertial odometry),
`/imu/data`. Full list with `ros2 topic list`.

**Both together, geometrically consistent**, the real test of this layer:

```bash
rviz2 -d $(ros2 pkg prefix --share sensor_fusion)/rviz/visual_odometry_with_lidar.rviz
```

Fixed Frame is already `base_link` in that layout, and the rover model is drawn
at 60% opacity so scan and cloud points stay visible through it. The laser scan
and the camera point cloud should overlap on the same physical surfaces, and
both should sit correctly relative to the chassis. If a wall appears twice,
offset from itself, the sensor mount offsets in
`helios_description/urdf/sensors.xacro` are wrong, not the sensors.

**Fusion output:**

```bash
ros2 topic hz /odometry/filtered      # ~30 Hz
ros2 run tf2_tools view_frames        # one tree, odom → base_link → sensors
```

Push the rover forward by hand about a metre and watch
`ros2 topic echo /odometry/filtered`. `pose.position.x` should increase by
roughly that much. More detail in the [`sensor_fusion`
README](sensor_fusion/README.md).

---

## Common problems

| Symptom | Cause | Fix |
|---|---|---|
| `could not open ethernet port` | Scanner's TCP port not ready yet, or wrong subnet | `ping 192.168.0.10`, wait a few seconds, relaunch |
| Camera launches then exits | ZED SDK missing or JetPack mismatch | Reinstall the matching SDK; check with `ZED_Explorer` |
| Camera stalls for minutes on first run | SDK optimising AI models | One-off; wait it out |
| `Gravity alignment issues detected` | Rover moved during ZED startup | Keep it still ~5 s while it launches, then relaunch |
| Laser node appears as `/zed_node` | Launch-argument leak between includes | Already fixed in `bringup.launch.py`; see the comment there |
| Scan and cloud do not overlap | Sensor mount offsets in the URDF | Measure and update `sensors.xacro` |

---

## Related

- [`helios_description`](../helios_description/README.md): where the sensors
  are mounted
- [`low_level_control_pkg`](../low_level_control_pkg/README.md): publishes the
  encoder counts this layer consumes
- [`mapping_localization_pkg`](../mapping_localization_pkg/README.md): consumes
  this layer's output
