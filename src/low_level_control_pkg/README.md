# low_level_control_pkg

The low-level-control subsystem for the Helios A4WD3 mecanum rover: owns the
RoboClaw hardware connection outright, and provides joystick teleop as its
first `/cmd_vel` source.

Two independent pieces:

- **`roboclaw_driver_node`** -- the sole owner of both RoboClaw serial ports.
  Reads all four wheel encoders and publishes them as a
  `sensor_msgs/JointState` on `roboclaw/wheel_encoders` (raw counts, not
  radians -- `perception_pkg`'s `wheel_odometry_node` does the counts->meters
  conversion and mecanum kinematics). Subscribes to `/cmd_vel`
  (`geometry_msgs/Twist`, body frame) and issues open-loop duty-cycle
  commands to the motors.
- **`teleop_joy_node`** -- 8BitDo SN30 Pro (Bluetooth) -> `sensor_msgs/Joy`
  (via `joy_node`) -> `/cmd_vel`, deadman-gated. This is the go-to node for
  driving the rover around manually to see how it behaves -- it has no idea
  what consumes `/cmd_vel`. A future path-planner-facing control node would
  be a second, independent `/cmd_vel` source (or publish somewhere upstream
  of a cmd_vel mux); it doesn't touch `roboclaw_driver_node`.

Control is **open-loop** (RoboClaw's raw duty-cycle command, not its
closed-loop Speed/QPPS mode) -- the RoboClaw's internal velocity PID and
QPPS limits aren't configured/verified on this hardware yet, so `vx`/`vy`/
`omega` here are normalized `[-1, 1]` commands, not true m/s or rad/s.

## Architecture: one process owns the hardware

RoboClaw's packet-serial protocol is half-duplex on a single UART -- two
independent processes issuing transactions on the same port would
interleave/corrupt each other's packets. That's not a style concern, it's a
hard constraint: corrupted packets to a motor controller are a real safety
issue, not just a dropped reading.

So `roboclaw_driver_node` is the **only** process that ever opens the
RoboClaw serial ports. Everything else -- `wheel_odometry_node` in
`perception_pkg` (needs encoder counts), teleop, and eventually a real
low-level control node driven by a path planner -- talks to it over ROS
topics, never the serial port directly. This is what makes it safe to run
wheel odometry and motor control at the same time: there's exactly one
hardware owner, full stop.

**Run `roboclaw_driver_node` exactly once** whenever the rover is powered
and connected. Running it twice (e.g. via both `roboclaw_driver.launch.py`
and `teleop.launch.py` at once) reintroduces the two-processes-one-port
problem.

## Launch files

- **`roboclaw_driver.launch.py`** -- just the hardware driver. Use this when
  running alongside the full stack (`sensor_fusion`'s `bringup.launch.py`,
  `mapping_localization_pkg`'s RTAB-Map, etc).
- **`joy_teleop.launch.py`** -- just `joy_node` + `teleop_joy_node`. No
  hardware access; safe to run alongside anything, as long as something
  else is running the driver.
- **`teleop.launch.py`** -- bundles both, for standalone bench-testing this
  package in isolation (e.g. wheels off the ground, no perception/SLAM
  stack up). Don't use this one if `roboclaw_driver.launch.py` is already
  running elsewhere -- use `joy_teleop.launch.py` alone in that case.

A full ground test (SLAM + drivable) looks like:
```
ros2 launch low_level_control_pkg roboclaw_driver.launch.py
ros2 launch sensor_fusion bringup.launch.py
ros2 launch low_level_control_pkg joy_teleop.launch.py
ros2 launch mapping_localization_pkg rtabmap.launch.py
```

## Setup

**1. Pair the SN30 Pro over Bluetooth** (put it in the appropriate pairing
mode for your desired profile -- X-input/D-input/Switch all report
differently to Linux, pick one and stay consistent). Confirm it shows up:
```
ls /dev/input/js*
jstest /dev/input/js0
```

**2. Verify the axis/button mapping before trusting the defaults.** The
indices in `config/teleop.yaml` were confirmed 2026-07-09 via
`ros2 topic echo /joy` against this specific SN30 Pro/pairing mode -- if the
controller, its pairing mode, or `joy_node`'s backend changes, re-verify:
```
ros2 run joy joy_node
ros2 topic echo /joy
```
Move each stick and press each button, note which index in `axes[]` /
`buttons[]` changes, and update `axis_vx`, `axis_vy`, `axis_omega`, and
`deadman_button` in `config/teleop.yaml` to match. This is safe to do live
-- nothing moves until you also hold the deadman button, regardless of
whether the axis mapping is right yet.

**3. Bench-test with wheels off the ground** before trusting direction/sign.
`max_linear_duty`/`max_angular_duty` in the config default to `0.5` (50%
duty) specifically so a wiring/sign mistake doesn't immediately mean full
power in the wrong direction. Flip the relevant `invert_*` flag in
`roboclaw_driver`'s config section if a wheel spins backwards from what you
commanded -- note these are a *separate* write path from
`wheel_odometry.yaml`'s `invert_*` flags (reading and writing can have
independent sign conventions), don't assume they match.

## Running (standalone bench test)

```
ros2 launch low_level_control_pkg teleop.launch.py
# or, if the joystick enumerated as a different device:
ros2 launch low_level_control_pkg teleop.launch.py joy_dev:=/dev/input/js1
```

## Safety mechanisms

- **Deadman button**: `teleop_joy` only publishes nonzero `/cmd_vel` while
  the configured deadman button is held; release it and the rover stops.
- **Joystick staleness watchdog** (`teleop_joy`, `joy_timeout`): if `/joy`
  stops arriving (controller powered off, Bluetooth dropped) for longer
  than `joy_timeout`, `teleop_joy` publishes zero `/cmd_vel` regardless of
  the last stick position it saw.
- **Command staleness watchdog** (`roboclaw_driver`, `cmd_timeout`): if
  `/cmd_vel` itself stops arriving (e.g. `teleop_joy` dies), `roboclaw_driver`
  zeros all four wheels independently of the above.
- **Shutdown stop**: `roboclaw_driver_node` sends a zero-duty command to both
  RoboClaws on clean shutdown (Ctrl+C).

None of these are a substitute for a physical kill switch / being ready to
cut power -- they cover software-level failure modes, not hardware ones.
