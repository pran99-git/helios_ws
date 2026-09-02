# sensor_fusion

Combines the wheel odometry and the camera's visual-inertial odometry into one
position estimate, and provides the launch file that brings up the entire
sensing layer.

This package is the boundary of the perception layer. It produces
`/odometry/filtered` and the `odom → base_link` transform, and stops there; it
starts no mapper.

**Read [the root README](../../../README.md) first** for the system overview.

---

## Why fuse at all

Two things on this robot independently estimate motion, and both are wrong in
different ways:

| Source | Good at | Bad at |
|---|---|---|
| Wheel encoders (`/wheel/odometry`) | Always available, smooth, accurate short-term | Slips, especially sideways on mecanum wheels |
| ZED visual-inertial (`/zed/zed_node/odom`) | No slip, low drift over distance | Fails in dark or featureless spaces; can lose tracking entirely |

An **EKF** (Extended Kalman Filter) merges them. The short version of how: it
keeps a running estimate of the robot's state plus a measure of how uncertain
that estimate is; each new measurement pulls the estimate toward itself in
proportion to how much that source is trusted. A source declared noisy moves the
estimate less. The result tracks better than either input alone, and degrades
gracefully when one input goes bad.

We use `robot_localization`, the standard ROS implementation. This package only
supplies its configuration.

### Two decisions that shape the config

**Only velocities are fused, never absolute positions.** Both sources report
position *and* velocity. Their positions drift, and fusing two independently
drifting positions produces a fight between them. Velocities do not accumulate
error. So the EKF takes `vx, vy, ωz` from both sources and integrates the
position itself: one consistent estimate rather than two competing ones.

**The IMU is deliberately not a separate input.** The ZED's `/odom` is already
visual-*inertial*: the SDK fuses camera and IMU internally. Adding the raw IMU
as a third input would count the same physical measurement twice, making the
filter overconfident and prone to overshoot. The IMU block in `ekf.yaml` is
present but commented out, with a note on the one condition under which it
should be enabled.

---

## Transform ownership

This is the rule that most often breaks a ROS robot, so it is worth being
explicit. Every transform must have exactly one publisher. Two publishers do not
produce an error; the position simply flickers between two answers and
everything downstream behaves strangely.

```
map ──[mapping layer]──► odom ──[this package's EKF]──► base_link ──[URDF]──► sensors
    (NOT owned here)
```

| Transform | Sole publisher |
|---|---|
| `map → odom` | `mapping_localization_pkg`: slam_toolbox **or** RTAB-Map, never both |
| `odom → base_link` | `ekf_filter_node`, from this package |
| `base_link → sensors, wheels` | `robot_state_publisher`, from `helios_description` |

Two nodes are capable of publishing `odom → base_link` and are actively
prevented from doing so:

- **`wheel_odometry_node`** is started with `publish_tf: False`.
- **The ZED wrapper** is started with `publish_tf:=false`.

Both still publish their measurements as *topics*, which is what the EKF
consumes. They just are not allowed to state where the robot is.

---

## Files

```
config/
  ekf.yaml               All EKF tuning. The substance of this package.
launch/
  bringup.launch.py      Starts the whole sensing layer. The one you run.
  ekf.launch.py          Starts only the EKF.
rviz/
  visual_odometry_with_lidar.rviz  Layout for camera + laser together.
CMakeLists.txt           Installs config/, launch/ and rviz/ into share/.
package.xml              Metadata; declares the dependency on robot_localization.
```

### Per-sensor content lives with its sensor, not here

Anything specific to one sensor sits beside that sensor's vendor submodule, so
everything for a given device is in one place:

| | where | what |
|---|---|---|
| Camera | [`camera/custom_covariance/`](../camera/custom_covariance/README.md) | Republishes the ZED's odometry with the twist covariance the wrapper never sets |
| LiDAR | [`lidar/custom_config/`](../lidar/custom_config/README.md) | Hokuyo driver parameters, its launch file, and the laser-only RViz layout |

Both are **ours**, deliberately placed *outside* the submodule they sit next
to, which is the entire point. Editing vendor content inside a pinned submodule
gets silently reverted by `git submodule update`.

`bringup.launch.py` pulls both in. What remains in this package is the fusion
layer itself: the EKF, its config, and the layout that shows both sensors at
once.

### `config/ekf.yaml`

This package has no source code: the EKF is `robot_localization`'s node, and
this file is what makes it a *mecanum rover* EKF rather than a generic one.

| Parameter | Value | Meaning |
|---|---|---|
| `frequency` | 30.0 | Output rate, Hz |
| `sensor_timeout` | 0.2 | Seconds before an input is treated as stale |
| `transform_time_offset` | 0.1 | Post-dates the published transform, see below |
| `two_d_mode` | true | Force z, roll, pitch to zero |
| `publish_tf` | true | This node owns `odom → base_link` |
| `world_frame` | `odom` | Makes this a *local* filter |

**`two_d_mode: true`** because the rover drives on flat floors and the laser
scanner sees a flat slice. Forcing z, roll and pitch to zero removes three
noise-only degrees of freedom rather than estimating quantities that cannot
usefully be observed.

**`world_frame: odom`** makes this a *local* filter: it produces a smooth,
continuous estimate that drifts slowly, which is what you want for control. The
non-drifting global estimate is the mapping layer's job.

**Input configuration.** Each source has a 15-element boolean mask over
`[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]`.
Both `odom0` (wheels) and `odom1` (ZED) enable exactly `vx`, `vy` and `vyaw`,
which is the "velocities only" decision made concrete.

`odom1_twist_rejection_threshold: 2.0` discards ZED velocity readings that jump
implausibly: the VIO occasionally resets and reports a large false jump, and
without this the filter would follow it.

**`transform_time_offset: 0.1`** post-dates the published transform by 100 ms.
The EKF stamps its transform with the time of the newest measurement it fused,
which lands roughly 42 ms in the past. ZED data is stamped *fresher* than that,
so a consumer looking up the transform at the camera's timestamp asks for a
moment newer than any EKF data exists for, and the lookup fails with
"extrapolation into the future", visible in RViz as
`Could not transform from [zed_left_camera_frame] to [map]`. This re-stamps the
transform rather than extrapolating the pose, so a consumer asking for time T
gets the pose from T−0.1s; at indoor speeds that is about 3 cm. slam_toolbox
does the same thing on its own `map → odom` edge.

Process and initial covariances are left at `robot_localization` defaults.

### `launch/bringup.launch.py`

The whole sensing layer in one command, in order: robot description → wheel
odometry → ZED → laser → EKF. Two of its settings are non-obvious and are
documented at length inside the file:

- **`enable_ipc:=false` on the ZED wrapper is required, not an optimisation.**
  With intra-process communication enabled, the wrapper cannot use a static
  transform broadcaster and instead republishes the camera's *geometrically
  fixed* internal frames as dynamic transforms at frame rate, from the same
  thread doing depth processing. Under load that thread slips, those transforms
  go stale, and every timestamped lookup into the camera chain fails, which was
  making RTAB-Map discard about a third of all frames.

- **`node_name` is passed explicitly to the laser driver.** Launch
  configurations leak between sibling includes in a single launch description,
  and the ZED include sets `node_name` first. Without this, the laser driver
  inherited it and came up as `/zed_node`, silently breaking anything
  addressing it by name.

Arguments: `camera:=false`, `lidar:=false`, `rviz:=true`.

### `launch/ekf.launch.py`

Just `ekf_filter_node` with `ekf.yaml`. Use it when the drivers are already
running and you want to restart only the filter, for instance after editing
tuning values. Included by `bringup.launch.py` as its final step.

---

## Build

```bash
cd ~/helios_ws
colcon build --packages-select sensor_fusion --symlink-install
source install/setup.bash
```

Requires `robot_localization`:

```bash
sudo apt install ros-jazzy-robot-localization
```

With `--symlink-install`, edits to `ekf.yaml` take effect on the next launch
with no rebuild.

## Run

```bash
# The whole sensing layer; this is the normal command
ros2 launch sensor_fusion bringup.launch.py

# Wheels only, no camera (useful when the ZED is unavailable)
ros2 launch sensor_fusion bringup.launch.py camera:=false

# Just the EKF, drivers already running
ros2 launch sensor_fusion ekf.launch.py
```

**Keep the rover still for the first ~5 seconds.** The ZED aligns its sense of
"down" against gravity at startup; moving during that logs a realignment warning
and starts tracking from a worse estimate.

A one-off "failed to meet update rate" warning at startup is normal: the ZED
SDK is loading its depth model onto the GPU and briefly starves the CPU.

To add mapping, run one of these separately on top:

```bash
ros2 launch mapping_localization_pkg slam_toolbox.launch.py   # 2D
ros2 launch mapping_localization_pkg rtabmap.launch.py        # 3D
```

---

## Verify

**1. Both inputs are arriving.** The EKF publishes output even when starved of
input, so check the inputs first:

```bash
ros2 topic hz /wheel/odometry          # matches encoder rate
ros2 topic hz /zed/zed_node/odom       # ~50 Hz
ros2 node info /ekf_filter_node        # both must appear as subscriptions
```

**2. Output is running:**

```bash
ros2 topic hz /odometry/filtered       # ~30 Hz, steady
```

**3. The transform tree is correct.** The single most important check:

```bash
ros2 run tf2_tools view_frames         # writes frames.pdf
```

Expect one connected tree: `odom → base_link → sensors and wheels` (plus
`map → odom` once a mapper runs). No frame may appear twice.

To confirm nobody else is publishing `odom → base_link`:

```bash
ros2 topic info /tf --verbose | grep "Node name"
```

`ekf_filter_node` should be the only odometry-related publisher. Seeing
`wheel_odometry` or a ZED node there means a `publish_tf` override did not take.

**4. It tracks real motion.** With the rover stationary, `/odometry/filtered`
velocities should read ~0 and the pose should not creep. Then push it forward
about a metre:

```bash
ros2 topic echo /odometry/filtered --field pose.pose.position
```

`x` should increase by roughly a metre. Rotating in place should change
orientation while position stays put.

**5. Fusion is actually helping.** Compare the two inputs against the output
during the same motion. Plot all three in RViz as *Odometry* displays. The
fused track should sit between the two inputs and be smoother than either. If it
follows one input exactly, the other is being rejected: check its covariances
and the rejection threshold.

**6. Visually,** with `rviz:=true`, set Fixed Frame to `odom`. The laser scan
should stay locked to the world as the robot moves, not swim. Scan points
sliding across static walls means the fused estimate disagrees with reality.

---

## Tuning

- **The ZED is unavailable:** `camera:=false` runs wheel-only. Expect faster
  drift, especially in heading.
- **`vy` looks noisy:** mecanum slips sideways. Its covariance in
  `wheel_odometry.yaml` is already 5× the longitudinal one for exactly this
  reason. Raise it further if the fused estimate still pulls off-axis when
  strafing.
- **Heading drifts in `odom`:** expected, and corrected by the mapping layer at
  the `map` level. Tightening it before SLAM means enabling the IMU block in
  `ekf.yaml`, which is only valid if you first disable the ZED's internal IMU
  fusion so `/zed/zed_node/odom` becomes purely visual.
- **Estimate is jumpy:** tune `process_noise_covariance`, currently at defaults.
  Raise it to trust measurements more, lower it to trust the model more.
- **RViz reports transform errors on camera data:** raise
  `transform_time_offset`. Lowering it toward 0.05 reduces the positional lag
  but leaves less margin.

---

## Related

- [`wheel_odometry`](../wheel_odometry/README.md): one of the two inputs
- [`perception_pkg`](../README.md): sensor hardware setup
- [`helios_description`](../../helios_description/README.md): provides
  `base_link` and below
- [`mapping_localization_pkg`](../../mapping_localization_pkg/README.md):
  consumes `/odometry/filtered`
