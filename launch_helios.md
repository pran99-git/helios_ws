# Launching Helios

The end-to-end runbook: every step to take the rover from powered-off to
mapping, in order, with what to check at each stage.

This is the operational document. For *why* any of it works the way it does,
read [the root README](README.md) and the package READMEs it links to.

---

## The short version

Four terminals to map, five to navigate. Order matters for the first one, and
navigation needs everything below it already running.

```bash
source ~/helios_ws/install/setup.bash        # in EVERY terminal

ros2 launch low_level_control_pkg roboclaw_driver.launch.py   # 1. motors
ros2 launch sensor_fusion bringup.launch.py                   # 2. sensors + EKF
ros2 launch low_level_control_pkg joy_teleop.launch.py        # 3. joystick
ros2 launch mapping_localization_pkg slam_toolbox.launch.py   # 4. a mapper
ros2 launch navigation_pkg navigation.launch.py               # 5. autonomy (optional)
```

Hold the **right shoulder button (R)** to drive. Everything below is the
detail: what to check first, what each terminal actually starts, and what to do
when one of them does not come up.

---

## How the stack stacks

Each layer consumes the one below it. Nothing skips a level.

```
  ┌─────────────────────────────────────────────────────────┐
  │  5. NAVIGATION      navigation_pkg (optional)           │
  │     goal in, /cmd_vel out. Publishes no TF.             │
  └───────────────────────────▲─────────────────────────────┘
                              │  /map  /scan  /odometry/filtered
  ┌───────────────────────────┴─────────────────────────────┐
  │  4. MAPPER          slam_toolbox | rtabmap | amcl       │
  │     owns map -> odom, exactly one at a time             │
  └───────────────────────────▲─────────────────────────────┘
                              │  /scan  /odometry/filtered
                              │  TF odom -> base_link
  ┌───────────────────────────┴─────────────────────────────┐
  │  2. PERCEPTION      sensor_fusion/bringup.launch.py     │
  │     robot model, ZED, Hokuyo, wheel odometry, EKF       │
  │     owns odom -> base_link                              │
  └───────────────────────────▲─────────────────────────────┘
                              │  roboclaw/wheel_encoders
  ┌───────────────────────────┴─────────────────────────────┐
  │  1. HARDWARE        roboclaw_driver.launch.py           │
  │     the ONLY process allowed to open the serial ports   │
  └───────────────────────────▲─────────────────────────────┘
                              │  /cmd_vel
  ┌───────────────────────────┴─────────────────────────────┐
  │  3. TELEOP          joy_teleop.launch.py                │
  │     start any time; publishes /cmd_vel and nothing else │
  └─────────────────────────────────────────────────────────┘
```

**Only terminal 1 has a hard ordering requirement.** It is the sole owner of
both RoboClaw serial ports, and RoboClaw's packet-serial protocol is half
duplex on a single UART. Two processes on one port interleave and corrupt each
other's packets. Sent to a motor controller, that is a safety problem, not a
dropped reading.

Terminal 3 can start whenever, since it only publishes `/cmd_vel`. Terminal 5
is the opposite: it needs all four layers below it, and it must NOT run
alongside terminal 3, because both drive `/cmd_vel`.

---

## Before the first run

One-time setup, each owned by a package README:

| Step | Where |
|---|---|
| ZED SDK, matching your JetPack version | [`perception_pkg`](src/perception_pkg/README.md) |
| Laser on the `192.168.0.0/24` subnet | [`perception_pkg`](src/perception_pkg/README.md) |
| udev rule pinning the RoboClaw ports | [`low_level_control_pkg`](src/low_level_control_pkg/README.md) |
| Gamepad paired, axis mapping confirmed | [`low_level_control_pkg`](src/low_level_control_pkg/README.md) |

Then build:

```bash
cd ~/helios_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` matters: with it, edits to launch files, YAML and Python
nodes take effect on the next launch with no rebuild.

---

## Pre-flight

Sixty seconds of checks that save a confusing failure later.

```bash
ping -c2 192.168.0.10              # laser scanner replies
ls -l /dev/roboclaw_left /dev/roboclaw_right   # udev symlinks exist
ls /dev/input/js0                  # gamepad is paired and awake
```

| Check fails | What it means |
|---|---|
| No ping reply | Cable, or the Jetson's `end0` has no address on that subnet. `ip -brief link show end0` shows `NO-CARRIER` if unplugged. |
| Missing `/dev/roboclaw_*` | udev rule not installed, or the controllers are unpowered. `ls /dev/ttyACM*` shows whether they enumerated at all. |
| Missing `/dev/input/js0` | Gamepad asleep or unpaired. Press Home to wake it. |

> **Every terminal needs `source ~/helios_ws/install/setup.bash`.** This catches
> everyone at least once. The symptom is `Package 'sensor_fusion' not found`.

---

## Terminal 1: motors

```bash
ros2 launch low_level_control_pkg roboclaw_driver.launch.py
```

Starts the sole owner of both serial ports. It subscribes to `/cmd_vel` and
publishes raw encoder counts on `roboclaw/wheel_encoders`.

No launch arguments. Everything is in
`low_level_control_pkg/config/roboclaw.yaml`.

**Expect on startup:** a line reporting each controller's QPPS. A value near
**7590** is correct. A wildly different number means the controller's EEPROM
was reflashed, and every speed command will be scaled wrong.

```bash
ros2 topic hz roboclaw/wheel_encoders     # ~30 Hz
```

---

## Terminal 2: sensors and fused odometry

```bash
ros2 launch sensor_fusion bringup.launch.py
```

**Keep the rover still for the first ~5 seconds.** The ZED aligns its internal
sense of "down" against gravity at startup. Moving during that prints
`Gravity alignment issues detected` and starts the run with a worse estimate.

This one launch file starts six things:

| What | Publishes |
|---|---|
| `robot_state_publisher` (helios_description) | `base_link` and every frame below it |
| `wheel_odometry_node` | `/wheel/odometry` |
| ZED 2i wrapper | RGB-D, `/zed/zed_node/odom`, IMU |
| `zed_odom_covariance_node` | `/zed/odom_with_cov` |
| Hokuyo `urg_node2` | `/scan` |
| `ekf_filter_node` | `/odometry/filtered` + TF `odom -> base_link` |

| Argument | Default | Use |
|---|---|---|
| `camera` | `true` | `false` to bring the rover up without the ZED |
| `lidar` | `true` | `false` to bring it up without the Hokuyo |
| `rviz` | `false` | `true` opens the sensor-check layout |

First launch after installing the SDK takes several minutes while the ZED
optimises its AI depth models for this GPU. It only happens once, and the
camera looks frozen while it runs.

```bash
ros2 topic hz /scan                  # ~40 Hz
ros2 topic hz /odometry/filtered     # ~30 Hz
ros2 run tf2_ros tf2_echo odom base_link
```

---

## Terminal 3: joystick

```bash
ros2 launch low_level_control_pkg joy_teleop.launch.py
```

| Argument | Default |
|---|---|
| `joy_dev` | `/dev/input/js0` |

**Hold the right shoulder button (R) to drive.** Release it and the rover
stops. That is a deadman switch and it is deliberate.

| Control | Motion |
|---|---|
| Left stick Y | forward / back |
| Left stick X | strafe left / right |
| Right stick X | rotate |

Full deflection is 0.40 m/s and 0.8 rad/s, about 27% of what the wheels can
actually do. That margin is deliberate traction headroom.

```bash
ros2 topic echo /joy      # sticks should move the numbers
ros2 topic echo /cmd_vel  # only nonzero while R is held
```

---

## Terminal 4: pick exactly one mapper

All three publish `map -> odom`. Running two means two nodes fighting over one
transform, which shows up as the robot appearing in two places in RViz.

| Goal | Command |
|---|---|
| Build a 2D floor plan | `ros2 launch mapping_localization_pkg slam_toolbox.launch.py` |
| Build a 3D coloured map | `ros2 launch mapping_localization_pkg rtabmap.launch.py` |
| Reuse a saved 2D map | `ros2 launch mapping_localization_pkg amcl_localization.launch.py map:=<path>.yaml` |

**The one documented exception:** `rtabmap.launch.py` defaults to
`publish_tf_map:=false`, so it can run *alongside* slam_toolbox to evaluate 3D
mapping without taking over the transform. Set `publish_tf_map:=true` only when
slam_toolbox is stopped.

### slam_toolbox

```bash
ros2 launch mapping_localization_pkg slam_toolbox.launch.py rviz:=true
```

| Argument | Default | Notes |
|---|---|---|
| `rviz` | `false` | Opens `slam.rviz` |
| `localization` | `false` | `true` reuses a `.posegraph` instead of mapping |
| `map_file_name` | `''` | Prefix, no extension. Required when localizing. |
| `map_start_pose` | `[0.0, 0.0, 0.0]` | Required when localizing, unless `map_start_at_dock` |
| `map_start_at_dock` | `false` | |

### rtabmap

```bash
ros2 launch mapping_localization_pkg rtabmap.launch.py rviz:=true
```

| Argument | Default | Notes |
|---|---|---|
| `publish_tf_map` | `false` | `true` makes it the TF authority. Stop slam_toolbox first. |
| `localization` | `false` | `true` localizes against `database_path` |
| `run_name` | timestamp | Names the run's database |
| `database_path` | `maps/rtabmap_<run_name>.db` | Absolute path. Overrides `run_name`. |
| `rviz` | `false` | Opens `rtabmap.rviz` |
| `rtabmap_viz` | `false` | **Leave off during a real run.** See below. |

> **Do not run `rtabmap_viz:=true` while mapping.** It was measured pushing
> load average to 6.9-10.6 and RTAB-Map's own frame time from 0.176 s to
> 0.374 s. Inspect the saved database afterwards with
> `rtabmap-databaseViewer` instead.

In RViz, watch **MapGraph**: neighbour links draw blue, global loop closures
draw **red**. Red links appearing when you revisit somewhere is the fastest
read on whether a run is healthy. A bare blue chain means nothing is closing.

### AMCL, to reuse a saved map

```bash
ros2 launch mapping_localization_pkg amcl_localization.launch.py \
    map:=$HOME/helios_ws/src/mapping_localization_pkg/slam_toolbox/maps/slam_toolbox_20260728_175429.yaml \
    rviz:=true
```

`map` is required and has no default. Saved maps live in
`src/mapping_localization_pkg/slam_toolbox/maps/`.

**After launching, the rover does not know where it is.** Give it a pose:

- RViz **2D Pose Estimate**: click the position, drag the heading.
- Or `ros2 service call /reinitialize_global_localization std_srvs/srv/Empty`
  to scatter particles over the whole map.

Then **drive**. AMCL only updates after 0.20 m or 0.20 rad of motion, so a
stationary rover never converges. Watch `/particlecloud` tighten as it goes.

Heading is less forgiving than position: the default yaw spread is about
±0.5 rad. Half a metre off recovers; 90° off usually does not.

---

## Terminal 5: autonomous navigation (optional)

Only if you want the rover to drive itself. Skip for pure mapping runs.

```bash
ros2 launch navigation_pkg navigation.launch.py rviz:=true
```

Starts five Nav2 servers plus a lifecycle manager. Set a goal with the **Nav2
Goal** tool in RViz.

| Argument | Default | Effect |
|---|---|---|
| `params_file` | the package's `nav2_params.yaml` | Alternative parameter set |
| `autostart` | `true` | `false` leaves servers unconfigured |
| `rviz` | `false` | Navigation layout. Costs CPU. |

> **Stop terminal 3 first.** `joy_teleop` and Nav2 both publish `/cmd_vel`, and
> the driver acts on whichever arrived last. Running both interleaves human and
> autonomous commands at 20 Hz.
>
> **There is no deadman button in autonomous mode.** Ctrl+C on this terminal
> stops new commands; the driver's `cmd_timeout` (0.5 s) then ramps to a stop.

Also know that **the laser plane sits 0.176 m above the ground**, so anything
shorter than that is invisible to the costmaps. Nav2 will drive into it.

```bash
ros2 lifecycle get /controller_server    # expect: active [3]
ros2 topic hz /local_costmap/costmap     # ~2 Hz
ros2 topic hz /cmd_vel                   # only while a goal is active
```

---

## Saving a run

### slam_toolbox

Both formats, all four files, one shared name:

```bash
S=~/helios_ws/src/mapping_localization_pkg/slam_toolbox/scripts
$S/save_slam.sh lab_corridor
```

| Output | Read by |
|---|---|
| `.pgm` + `.yaml` | AMCL, nav2 costmaps, any image viewer |
| `.posegraph` + `.data` | slam_toolbox, the only format you can keep mapping from |

Neither substitutes for the other, which is why the script writes both.
`--map-only` / `--graph-only` narrow it if you want just one.

### rtabmap

**The run is already saved.** Ctrl+C writes the whole session into the `.db`.
Use **Ctrl+C, never `kill -9`**: a SIGKILL skips the save and the run is gone.

The script only extracts formats other tools can open, and the two halves have
opposite timing requirements:

```bash
S=~/helios_ws/src/mapping_localization_pkg/rtabmap/scripts
$S/save_rtabmap.sh lab_run --map-only      # BEFORE Ctrl+C: .pgm + .yaml
$S/save_rtabmap.sh lab_run --cloud-only    # AFTER  Ctrl+C: _cloud.ply
```

The `.pgm` half is a live capture of `/rtabmap/map` and there is no offline
exporter for it. Miss it and you have to redrive the run.

---

## Verifying the whole stack

Run these with everything up.

```bash
ros2 topic hz /scan                  # laser        ~40 Hz
ros2 topic hz /odometry/filtered     # fused pose   ~30 Hz
ros2 topic hz /map                   # once a mapper is running
ros2 run tf2_tools view_frames       # writes frames.pdf
```

The transform tree must be **one connected tree** with a single owner per edge:

| Edge | Owner |
|---|---|
| `map -> odom` | the mapper in terminal 4 |
| `odom -> base_link` | `ekf_filter_node` |
| `base_link -> everything` | `robot_state_publisher` |

A frame with two parents, or one appearing twice, is the whole problem. List
publishers with `ros2 topic info /tf --verbose`.

**The end-to-end check:** push the rover forward about a metre by hand and
watch `ros2 topic echo /odometry/filtered`. `pose.position.x` should increase
by roughly that much.

**The map quality check:** drive a loop back to where you started. The corridor
you began in should land on itself, not beside itself.

---

## Shutting down

Reverse order, and Ctrl+C each terminal rather than closing the window.

1. **Terminal 5 first** if navigation is running, so no further `/cmd_vel`
   is issued while the rest comes down.
2. **Terminal 4**, especially rtabmap, which writes its database on Ctrl+C.
3. Terminal 3, the joystick.
4. Terminal 2, the sensors.
5. Terminal 1 last. It stops the motors and closes the serial ports on the way
   out.

Check nothing is left holding a port before relaunching:

```bash
pgrep -a -f "ros2 launch|roboclaw_driver_node|zed" || echo "clean"
```

---

## When something does not come up

| Symptom | Cause | Fix |
|---|---|---|
| `Package 'x' not found` | Terminal not sourced | `source ~/helios_ws/install/setup.bash` |
| `could not open ethernet port` | Laser's TCP port not ready yet | Wait a few seconds and relaunch. Confirm with `ping 192.168.0.10` |
| `Gravity alignment issues detected` | Rover moved during ZED startup | Keep it still ~5 s while terminal 2 starts, then relaunch |
| Rover will not move | Deadman not held, or gamepad asleep | Hold R. Check `ros2 topic echo /joy` responds |
| Wheels move wrong direction | Sign or axis mapping | `invert_*` in `roboclaw.yaml` fixes wheels; `invert_vx` etc. in `teleop.yaml` fixes sticks |
| Two robot positions in RViz | Two nodes publishing one transform | Only one terminal-4 mapper at a time |
| Laser node appears as `/zed_node` | Launch-argument leak between includes | Already handled in `bringup.launch.py`; if it recurs, check `node_name` is still passed explicitly |
| RTAB-Map drops frames, patchy map | Jetson overloaded | Turn off `rtabmap_viz`. Consider running RViz on a laptop instead |
| `/map` never publishes | slam_toolbox processed zero scans | It gates on translation, not rotation. Drive forward, do not only spin |
| Map is skewed after a loop | Odometry yaw drift | See `wheel_odometry.yaml`'s `yaw_scale` note |

---

## Quick reference

```bash
# every terminal
source ~/helios_ws/install/setup.bash

# 1  motors            (first, owns the serial ports)
ros2 launch low_level_control_pkg roboclaw_driver.launch.py

# 2  sensors + EKF     (hold still 5 s)
ros2 launch sensor_fusion bringup.launch.py

# 3  joystick          (hold R to drive)
ros2 launch low_level_control_pkg joy_teleop.launch.py

# 4  ONE of:
ros2 launch mapping_localization_pkg slam_toolbox.launch.py rviz:=true
ros2 launch mapping_localization_pkg rtabmap.launch.py rviz:=true
ros2 launch mapping_localization_pkg amcl_localization.launch.py map:=<path>.yaml

# 5  autonomy (optional; stop terminal 3 first, both publish /cmd_vel)
ros2 launch navigation_pkg navigation.launch.py rviz:=true

# save
~/helios_ws/src/mapping_localization_pkg/slam_toolbox/scripts/save_slam.sh <name>
~/helios_ws/src/mapping_localization_pkg/rtabmap/scripts/save_rtabmap.sh <name> --map-only
```

---

## Where to read next

- [Root README](README.md): what the system is and how the pieces fit
- [`helios_description`](src/helios_description/README.md): the robot model and mount offsets
- [`low_level_control_pkg`](src/low_level_control_pkg/README.md): motors, teleop, calibration
- [`perception_pkg`](src/perception_pkg/README.md): sensors and sensor fusion
- [`mapping_localization_pkg`](src/mapping_localization_pkg/README.md): the three mappers in depth
- [`navigation_pkg`](src/navigation_pkg/README.md): Nav2, and the mecanum-specific tuning
