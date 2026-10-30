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
| `sudo apt install ros-jazzy-twist-mux` | started with the driver; without it nothing publishes `/cmd_vel` |
| VLA virtualenv and checkpoint (only for terminal 6) | [`vla_pkg`](src/vla_pkg/README.md) |

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

Starts three things: the sole owner of both serial ports, `twist_mux`, and the
emergency stop. The driver subscribes to `/cmd_vel` and publishes raw encoder
counts on `roboclaw/wheel_encoders`.

`twist_mux` is the only publisher of `/cmd_vel`. Everything that wants to drive
goes through it by priority, so the joystick outranks autonomy and the e-stop
outranks both.

| Input | Priority |
|---|---|
| `/cmd_vel_estop` zeros, from `estop_node` | 250 |
| `/cmd_vel_lock` latched lock, from `estop_node` | lock at 200 |
| `/cmd_vel_joy` teleop | 100 |
| `/cmd_vel_smoothed` Nav2 and the VLA | 10 |

> **Press L on the gamepad to stop everything.** It latches: teleop and
> autonomy stay blocked until you hold **Start** for a second to clear it.
> `/estop/engaged` reports the state.
>
> This needs terminal 3 running, since the trigger is a gamepad button today.
> `/estop/request` works without it, and is where a GPIO button or an app will
> plug in later. Your battery cut switch remains the true last resort, at the
> cost of losing encoder counts and having to re-localise.

No launch arguments. Everything is in
`low_level_control_pkg/config/roboclaw.yaml`, `twist_mux.yaml` and
`estop.yaml`.

**Expect on startup:** a line reporting each controller's QPPS. A value near
**7590** is correct. A wildly different number means the controller's EEPROM
was reflashed, and every speed command will be scaled wrong.

```bash
ros2 topic hz roboclaw/wheel_encoders     # ~30 Hz
ros2 topic info /cmd_vel --verbose        # publisher should be twist_mux
ros2 topic echo /estop/engaged --once     # false
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
| ZED 2i wrapper | RGB-D at 30 Hz, `/zed/zed_node/odom`, IMU |
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
| Reuse a saved 2D map | `ros2 launch mapping_localization_pkg amcl_localization.launch.py map:=<path>.yaml` (self-relocalizes; add `recovery:=false` to disable) |

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

**You normally do not have to do any of the above.** `amcl_recovery` starts
with AMCL and re-localizes the rover by itself, at startup and whenever it goes
lost later. It spins, then drives short legs picked from Nav2's local costmap,
until the particle cloud tightens.

It also notices being **picked up and carried**: wheels still while the camera
reports motion is a signature nothing else produces, and it recovers once the
rover is set back down.

It needs **terminal 5 (Nav2) running**, since the local costmap and the
spin / drive_on_heading behaviors live there. Without Nav2 it stays passive and
the manual pose estimate above is still how you do it.

**The rover drives itself with no deadman.** To stop it, or to force it:

```bash
ros2 service call /amcl_recovery/abort std_srvs/srv/Trigger
ros2 service call /amcl_recovery/relocalize std_srvs/srv/Trigger
```

Turn it off with `recovery:=false` on the AMCL launch, or `enabled: false`
under `amcl_recovery` in `amcl.yaml`. Details in the
[`mapping_localization_pkg` README](src/mapping_localization_pkg/README.md).

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

> **Leave terminal 3 running.** `twist_mux` ranks the joystick above Nav2, so
> holding the deadman takes over mid-goal and releasing it hands control back.
> That is your fastest manual override.
>
> **There is still no deadman in autonomous mode.** Nothing has to be held for
> Nav2 to drive. Ctrl+C on this terminal stops new commands; the driver's
> `cmd_timeout` (0.5 s) then ramps to a stop.

Also know that **the laser plane sits 0.176 m above the ground**, so anything
shorter than that is invisible to the costmaps. Nav2 will drive into it.

```bash
ros2 lifecycle get /controller_server    # expect: active [3]
ros2 topic hz /local_costmap/costmap     # ~2 Hz
ros2 topic hz /cmd_vel                   # only while a goal is active
```

---

## Terminal 6: VLA navigation (optional)

Drive to a goal given as a **sentence**, a **photo**, or a **2D pose**, using a
learned policy instead of the global planner.

**Needs terminal 5 already running.** This bypasses Nav2's planner but still
hands its paths to `controller_server`, so MPPI and the costmaps are what keep
the rover off obstacles. Without them the policy drives blind.

One-time setup (virtualenv, 415 MB checkpoint, submodule) is in
[`src/vla_pkg/README.md`](src/vla_pkg/README.md).

```bash
ros2 launch vla_bridge vla_navigation.launch.py
```

Starts three processes: the policy server (outside ROS, in its own
virtualenv), `vla_bridge_node`, and `path_arbiter_node`.

Then give it a goal, exactly one kind at a time:

```bash
# language
ros2 topic pub --once /vla/goal_text std_msgs/msg/String "{data: 'the blue chair'}"

# clear it
ros2 topic pub --once /vla/goal_text std_msgs/msg/String "{data: ''}"
```

For a pose goal, point RViz's **2D Goal Pose** tool at `/vla/goal_pose`.

| Argument | Default | Effect |
|---|---|---|
| `start_server` | `true` | `false` to use an already-running policy server |
| `endpoint` | `http://127.0.0.1:8777/act` | A workstation IP also works |
| `params_file` | the package's `vla_bridge.yaml` | Alternative parameter set |
| `workspace` | `~/helios_ws` | Where `.venvs/` and `src/vla_pkg/` live |

> **Do not use the Nav2 Goal tool while this is running.** Both would command
> the same controller. That is why the pose goal here is `/vla/goal_pose` and
> not `/goal_pose`, which `bt_navigator` owns.
>
> **Leave terminal 3 running**, same as for terminal 5. The joystick outranks
> the policy, so holding the deadman is how you take a VLA run away from it.

**Run the ZED with depth off in this mode.** The policy needs RGB only, and
NEURAL_LIGHT depth is the documented cause of the EKF starvation described in
the rtabmap README. Pair this with AMCL rather than RTAB-Map.

The policy is **frozen and un-adapted to this rover**: it has never seen this
camera height or field of view, it will not strafe, and it knows nothing about
the 0.40 m/s cap. Treat its first outings as experiments.

```bash
curl -s localhost:8777/health            # {"ready":true,...}
ros2 topic hz   /vla/plan                # ~5 Hz once a goal is set
ros2 topic echo /vla/status --once       # ok modality=7 policy=95ms warm=True
ros2 topic echo /vla/arbiter_status --once
```

First time out, put a Path display on `/vla/plan` in RViz **with the motors
unplugged** and check the trajectory points where the camera is looking.

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
| Rover will not move, and R does nothing | The e-stop is latched | `ros2 topic echo /estop/engaged`. Hold **Start** for a second to clear |
| Rover rotates back and forth without arriving | `controller_server` aborting on the progress checker, so the behavior tree runs `Spin` and `BackUp` against the controller | Check the log for `Failed to make progress`. `progress_checker` must be `PoseProgressChecker`, which counts rotation as progress |
| Wheels move wrong direction | Sign or axis mapping | `invert_*` in `roboclaw.yaml` fixes wheels; `invert_vx` etc. in `teleop.yaml` fixes sticks |
| Two robot positions in RViz | Two nodes publishing one transform | Only one terminal-4 mapper at a time |
| Laser node appears as `/zed_node` | Launch-argument leak between includes | Already handled in `bringup.launch.py`; if it recurs, check `node_name` is still passed explicitly |
| RTAB-Map drops frames, patchy map | Jetson overloaded | Turn off `rtabmap_viz`. Consider running RViz on a laptop instead |
| `/map` never publishes | slam_toolbox processed zero scans | It gates on translation, not rotation. Drive forward, do not only spin |
| Map is skewed after a loop | Odometry yaw drift | See `wheel_odometry.yaml`'s `yaw_scale` note |
| `planner_server` stuck inactive | It blocks on `map -> odom`. Everything else still activates by design | Expected until AMCL localizes. Planning needs it; driving does not |
| Rover never self-relocalizes | Nav2 not running, or AMCL not active | `amcl_recovery` needs `/local_costmap/costmap` plus the `spin` and `drive_on_heading` servers, and polls `/amcl/get_state` before acting |
| Rover drives off on its own after launch | `amcl_recovery` doing its job | Expected. `ros2 service call /amcl_recovery/abort std_srvs/srv/Trigger`, or launch with `recovery:=false` |
| Map smears / walls double while turning | `transform_time_offset` too large | It is lag, not extrapolation. Keep it at 0.02 in `ekf.yaml` |
| `rviz2` over SSH: `qt.qpa.xcb: could not connect to display` | No X display on the SSH session | `ssh -X` (or `-Y`) from the laptop, or run RViz on the Jetson's own monitor. RViz on the Jetson also costs CPU the ZED needs |
| RTAB-Map cloud full of stray points in mid-air | Export filtering is off | Stereo flying pixels. Enable the radius outlier filter in `~/.ros/rtabmapGUI.ini` before exporting |

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

# 5  autonomy (optional; leave 3 running, the joystick outranks it)
ros2 launch navigation_pkg navigation.launch.py rviz:=true

# 6  VLA policy (optional; needs 5 running)
ros2 launch vla_bridge vla_navigation.launch.py
ros2 topic pub --once /vla/goal_text std_msgs/msg/String "{data: 'the blue chair'}"

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
- [`vla_pkg`](src/vla_pkg/README.md): the OmniVLA policy, and how it feeds Nav2
