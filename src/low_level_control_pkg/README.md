# low_level_control_pkg

Everything that touches the motors. This package owns the connection to the two
RoboClaw motor controllers outright: it drives the wheels, reads the encoders,
and publishes those readings for the rest of the stack.

It also provides joystick teleop, the current source of drive commands and the
only way the rover moves today.

**Read [the root README](../../README.md) first** for the system overview.

---

## The one architectural rule

**`roboclaw_driver_node` is the only process that ever opens the RoboClaw serial
ports.** Everything else talks to it over ROS topics.

This is not a style preference. RoboClaw's packet-serial protocol is half-duplex
on a single UART: the host sends a command and reads the reply on the same wire.
Two OS processes issuing transactions on one port interleave their bytes and
corrupt each other's packets. Sent to a device that drives motors, a corrupted
packet is a safety problem, not a dropped reading.

So:

- `wheel_odometry_node` (in `perception_pkg`) needs encoder counts → it
  subscribes to `roboclaw/wheel_encoders`, and never opens a port.
- Teleop needs to drive → it publishes `/cmd_vel`, and never opens a port.
- The two direct-serial helper scripts here (`wheel_monitor`, `calibrate_qpps`)
  **require the driver node to be stopped first**. They say so on startup.

**Run `roboclaw_driver_node` exactly once.** Starting `roboclaw_driver.launch.py`
and `teleop.launch.py` together reintroduces the problem, because
`teleop.launch.py` bundles its own copy of the driver.

```
     joystick                    (future: path planner)
        │                                 │
        ▼                                 ▼
  teleop_joy_node ───── /cmd_vel ─────────┘
                            │
                            ▼
                   roboclaw_driver_node  ◄── sole owner of both serial ports
                       │           │
              motor commands   roboclaw/wheel_encoders
                       │           │
                       ▼           ▼
                  RoboClaws     wheel_odometry_node (perception_pkg)
```

---

## Closed-loop control

The driver uses the RoboClaw's **Speed/QPPS** command, not its raw duty-cycle
command. The difference matters:

- *Duty cycle* means "apply 40 % power", an open-loop request. A wheel that
  loses traction spins up freely.
- *Speed* means "turn at 3000 counts/second": the controller's own velocity PID
  holds that rate, throttling back a wheel that starts to run away.

**QPPS** (quadrature pulses per second) is the controller's top-speed reference,
which the PID scales against. It has to be measured per motor: gearbox and motor
variance means one shared value leaves some wheels under- or over-driven.
Measured on this rover, both controllers report 7590 QPPS ≈ 1.48 m/s at the
wheel.

The practical consequence is that `/cmd_vel` carries **true SI units**, m/s and
rad/s, not a normalised `[-1, 1]` range. Commanding 0.3 m/s gets you 0.3 m/s,
within traction limits.

Commands are also **acceleration-ramped** (`drive_accel`, counts/s²), which
removes the torque step that an unramped command applies and reduces wheel slip
on a hard start. The same ramp is the deceleration for every stop.

---

## Files

Three source folders, split by hardware boundary: what generates a command,
what talks to the controllers, and what measures them.

```
teleop/                     Generates /cmd_vel. Knows nothing about serial.
  teleop_joy_node.py        Joystick → /cmd_vel.
  speed_limits.py           Decides the QPPS ceiling. No ROS.
roboclaw/                   Everything that talks to or models the controllers.
  roboclaw_driver_node.py   The ROS node. Owns both serial ports.
  roboclaw_driver.py        Packet-serial protocol implementation. No ROS.
  mecanum_kinematics.py     Body twist → per-wheel commands. No ROS.
calibration/                Bench tools. Bypass ROS, talk to the hardware direct.
  calibrate_qpps.py         Measures each motor's top speed (QPPS).
  wheel_monitor.py          Live per-corner encoder table.
launch/
  roboclaw_driver.launch.py Driver alone. Use this with the full stack.
  joy_teleop.launch.py      Joystick alone. Safe alongside anything.
  teleop.launch.py          Both together, for standalone bench testing.
config/
  teleop.yaml               teleop_joy parameters.
  roboclaw.yaml             roboclaw_driver parameters.
test/
  test_*.py                 Unit tests. Pure pytest, no hardware.
setup.py / setup.cfg        Python package build; declares the executables.
package.xml                 Metadata and dependencies.
```

The dependency direction is one-way: `roboclaw/` imports nothing from
`teleop/` except `speed_limits`, `calibration/` imports from `roboclaw/`, and
nothing imports from `calibration/`. A node in `teleop/` never touches a serial
port.

### `roboclaw/roboclaw_driver_node.py`

The ROS node. Subscribes to `/cmd_vel`, converts the body twist into per-wheel
speeds in counts/second, and writes them to both controllers under an
acceleration ramp. Separately reads all four encoders and publishes them as a
`sensor_msgs/JointState` on `roboclaw/wheel_encoders` at
`encoder_publish_rate` (30 Hz).

It publishes **raw counts**, not metres or radians. The counts-to-metres
conversion and mecanum kinematics live in `wheel_odometry_node`; this node's
job is hardware access, not interpretation.

### `roboclaw/roboclaw_driver.py`

The wire protocol, with no ROS dependency: encoder reads, speed writes, the
velocity-PID/QPPS read, and a raw duty-cycle write kept only for the shutdown
stop. Every packet is `[address, command, payload…, crc16]`, and reads verify a
CRC computed over both what was sent and what came back.

Separated from the node so the protocol can be read and tested without a ROS
context.

### `roboclaw/mecanum_kinematics.py`

The inverse kinematics, body twist to four wheel commands:

```
fl = x - y - rot        fr = x + y + rot
rl = x + y - rot        rr = x - y + rot
```

This is the exact algebraic inverse of the forward kinematics in
`wheel_odometry_node`, which is what makes a wheel that *reads* positive when
driven forward also *command* positive. The two must stay inverses of each
other; `wheel_monitor.py` writes out the forward form inline for its read-out,
so a change here needs checking in both places.

No ROS dependency, so it is unit tested directly.

### `teleop/speed_limits.py`

Turns what the two controllers report into the single QPPS ceiling the rover is
allowed to command. Kept separate from the protocol layer because it is policy,
not wire format. Carries a measured fallback (7590) used only when a live read
fails.

### `teleop/teleop_joy_node.py`

Converts `sensor_msgs/Joy` into `/cmd_vel`. Axis and button indices come from
config rather than being hardcoded, because they depend on which mode the
gamepad is paired in and which backend `joy_node` picks up.

Stick deflection is scaled into SI units before publishing: full deflection maps
to `max_vx` / `max_vy` / `max_omega`.

This node knows nothing about what consumes `/cmd_vel`. A future path planner
would be a second, independent publisher of the same topic and would not touch
the driver.

### `calibration/wheel_monitor.py` and `calibration/calibrate_qpps.py`

Two calibration helpers that read the RoboClaws **directly, not through ROS**,
so the driver node must be stopped before running either.

`wheel_monitor` prints a live per-corner table of raw counts and per-cycle
deltas, plus the body twist the odometry kinematics would produce from them. Use
it to confirm wheel signs. It only reads; it never commands.

`calibrate_qpps` measures each motor's free-spin top speed by driving one motor
at a time. **Wheels must be off the ground.** It drives each channel directly
rather than through `/cmd_vel` because mecanum has only three controllable
degrees of freedom for four wheels, so no body-twist command can isolate a single
wheel.

Both have `ros2 run` entry points:

```bash
ros2 run low_level_control_pkg wheel_monitor
ros2 run low_level_control_pkg calibrate_qpps --duty 0.3   # cautious dry run
ros2 run low_level_control_pkg calibrate_qpps              # full 100% duty
```

### `config/teleop.yaml` and `config/roboclaw.yaml`

One file per node, each holding that node's `ros__parameters` block.

`teleop_joy` holds axis and button indices, and the velocity at full stick
deflection:

| Parameter | Value | Notes |
|---|---|---|
| `axis_vx` / `axis_vy` | 1 / 0 | Left stick Y / X |
| `axis_omega` | 3 | Right stick X |
| `deadman_button` | 5 | Right shoulder, must be held |
| `max_vx` / `max_vy` | 0.40 m/s | At full deflection |
| `max_omega` | 0.8 rad/s | |
| `deadzone` | 0.08 | Ignores stick centre drift |
| `joy_timeout` | 0.5 s | Stop if `/joy` goes silent |

`roboclaw_driver` holds ports, addresses, geometry and limits:

| Parameter | Value | Notes |
|---|---|---|
| `left_port` / `right_port` | `/dev/roboclaw_left` / `_right` | Stable names from the udev rule |
| `wheel_radius` / `counts_per_rev` | 0.076 m / 2448.0 | **Must match `wheel_odometry.yaml`** |
| `wheelbase` / `track_width` | 0.220 / 0.330 m | Same |
| `max_vx` / `max_vy` / `max_omega` | 0.40 / 0.40 / 0.8 | Clamp on commanded velocity |
| `drive_accel` | 5000 counts/s² | ~1.0 m/s²; also the deceleration for stops |
| `cmd_timeout` | 0.5 s | Ramped stop if `/cmd_vel` goes stale |
| `invert_*` (×4) | all `false` | Command-direction signs |

Three things to know about these values.

**Teleop's limits must not exceed the driver's.** If teleop can command more
than the driver allows, the stick goes dead over part of its travel because the
driver clamps it. Raise both together.

**The geometry must match `wheel_odometry.yaml`.** If they disagree, commanded
and reported velocity disagree by exactly that factor, and the EKF is fed a
consistent lie.

**The `invert_*` flags here are a separate write path from the ones in
`wheel_odometry.yaml`.** Reading and writing can have independent sign
conventions. Do not assume the two sets match; verify each on the bench.

**`drive_accel` is a real trade-off.** It is also the deceleration for every
stop, including watchdog trips and deadman release. At 5126 counts/m, 5000
counts/s² is ~0.98 m/s², so braking from `max_vx` (0.40 m/s) takes ~0.41 s and
~0.082 m. Drop it to 1000 and the same stop takes ~2.05 s and ~0.41 m.

The total stopping distance depends on *how* the stop was triggered:

| Trigger | What happens | Distance from 0.40 m/s |
|---|---|---|
| Deadman released, or joystick goes stale | `teleop_joy` keeps publishing at 20 Hz with zeroed values, so braking starts within ~50 ms | ~0.09 m |
| `/cmd_vel` stops entirely (teleop dies, comms drop) | The watchdog waits up to `cmd_timeout` (0.5 s) at full speed first | ~0.28 m |

Recompute these whenever `max_vx` or `drive_accel` changes. The numbers are in
`config/roboclaw.yaml`'s comments too, and both should be updated together.

### `launch/`

| File | Starts | When to use |
|---|---|---|
| `roboclaw_driver.launch.py` | Driver only | With the full stack. **The normal choice.** |
| `joy_teleop.launch.py` | `joy_node` + `teleop_joy_node` | Alongside anything; no hardware access |
| `teleop.launch.py` | Both of the above | Standalone bench test of this package alone |

`joy_teleop.launch.py` and `teleop.launch.py` accept `joy_dev:=/dev/input/js1`
if the gamepad enumerates elsewhere.

Do not run `teleop.launch.py` when `roboclaw_driver.launch.py` is already
running, or you get two drivers on one serial port.

### `test/`

Unit tests for `mecanum_kinematics`, `roboclaw_driver` and `speed_limits`. Pure
pytest: no ROS context, no serial hardware, so they run anywhere. `conftest.py`
puts the package root on the import path so `pytest` works without a sourced
install overlay.

---

## Setup

### 1. Stable serial port names

`/dev/ttyACM0` and `/dev/ttyACM1` renumber on replug or reboot, which silently
swaps left and right. A udev rule pins each controller to a fixed name.

The two RoboClaws are identical and expose **no unique USB serial number**, so
the rule matches the **physical USB port** instead:

```
left  controller → port 1-4.4 → /dev/roboclaw_left
right controller → port 1-4.3 → /dev/roboclaw_right
```

Install it (the file currently lives in `wheel_odometry`, for historical
reasons):

```bash
sudo cp src/perception_pkg/wheel_odometry/udev/99-roboclaw.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/roboclaw_*        # should symlink to the right ttyACM*
```

The rule also sets `MODE=0666`, which permanently fixes the serial permission
problem.

> Because matching is by physical port, **keep each RoboClaw in its current
> socket** (and the hub in the same Jetson port). If you move one, find its new
> port with `udevadm info -n /dev/ttyACM0 | grep DEVPATH`, take the `1-4.x`
> segment, and update `KERNELS==` in the rule.

If you would rather not use udev, add yourself to the `dialout` group instead
(`sudo usermod -aG dialout $USER`, then log out and back in), but you keep the
renumbering problem.

### 2. Pair the gamepad

Put the 8BitDo SN30 Pro into a pairing mode and pair it over Bluetooth. The
SN30 Pro reports differently in X-input, D-input and Switch modes, so pick one and
stay consistent, because the axis indices depend on it.

```bash
ls /dev/input/js*
jstest /dev/input/js0
```

### 3. Verify the axis mapping before trusting the defaults

The indices in `config/teleop.yaml` were confirmed against this specific gamepad
and pairing mode. If the controller, its mode, or `joy_node`'s backend changes,
re-check:

```bash
ros2 run joy joy_node
ros2 topic echo /joy
```

Move each stick and press each button, note which index in `axes[]` /
`buttons[]` changes, and update `axis_vx`, `axis_vy`, `axis_omega` and
`deadman_button` to match.

This is safe to do with the robot powered: nothing moves unless the deadman is
also held.

### 4. Bench-test with the wheels off the ground

Do this before trusting any sign. `max_vx` is 0.40 m/s, about 27 % of the
~1.48 m/s the wheels can reach, so a wiring mistake does not immediately mean
full speed in the wrong direction, but it will still be a surprise on the floor.

If a wheel spins backwards from what you commanded, flip its `invert_*` flag in
the `roboclaw_driver` block. Remember these are independent of
`wheel_odometry.yaml`'s flags.

---

## Build

```bash
cd ~/helios_ws
colcon build --packages-select low_level_control_pkg --symlink-install
source install/setup.bash
```

Needs `pyserial` and the `joy` package:

```bash
sudo apt install ros-jazzy-joy python3-serial
```

## Run

Alongside the full stack, the normal case:

```bash
ros2 launch low_level_control_pkg roboclaw_driver.launch.py   # first, always
ros2 launch low_level_control_pkg joy_teleop.launch.py
```

Standalone bench test of this package alone:

```bash
ros2 launch low_level_control_pkg teleop.launch.py
ros2 launch low_level_control_pkg teleop.launch.py joy_dev:=/dev/input/js1
```

**Hold the right shoulder button to drive.** Left stick moves and strafes, right
stick rotates.

## Test

```bash
cd ~/helios_ws
python3 -m pytest src/low_level_control_pkg/test -v
```

No hardware or ROS environment needed.

---

## Verify

**1. Both controllers are connected:**

```bash
ls -l /dev/roboclaw_*
ros2 node info /roboclaw_driver
```

The driver logs each controller's QPPS on startup; expect a value near 7590.
A failed read falls back to the stored default and logs a warning; that is worth
investigating, since it means the port opened but the protocol did not respond.

**2. Encoders are publishing:**

```bash
ros2 topic hz roboclaw/wheel_encoders          # ~30 Hz
ros2 topic echo roboclaw/wheel_encoders --once  # four names, four positions
```

Turn a wheel by hand and confirm its count changes.

**3. The joystick reaches the node:**

```bash
ros2 topic echo /joy            # sticks and buttons respond
ros2 topic echo /cmd_vel        # silent until the deadman is held
```

`/joy` responding but `/cmd_vel` staying silent while the deadman is held means
`deadman_button` is the wrong index.

**4. Command directions are right, wheels off the ground.** Hold the deadman
and push each stick:

| Input | Expect |
|---|---|
| Left stick forward | All four wheels forward |
| Left stick left | Diagonal pairs opposed (strafe pattern) |
| Right stick left | Left wheels back, right wheels forward |

Any single wheel going the wrong way is an `invert_*` flag. All four wrong
usually means a stick axis is inverted. Fix that in `invert_vx` etc.

**5. Commanded speed matches actual speed.** On the ground, mark a 2 m line,
command a steady 0.2 m/s, and time the run; expect about 10 seconds. A
consistent discrepancy means `wheel_radius` or `counts_per_rev` is wrong, and
the same error is corrupting the odometry.

**6. Every stop works.** Test each independently, wheels off the ground, while
driving:

- Release the deadman → wheels ramp down to a stop.
- Power off the gamepad → wheels stop within `joy_timeout` (0.5 s).
- Kill the teleop node (Ctrl+C) → wheels stop within `cmd_timeout` (0.5 s).

If any of these does not stop the wheels, do not put the rover on the floor.

---

## Safety mechanisms

Four layers, listed strongest first:

- **Deadman button.** `teleop_joy` publishes nonzero `/cmd_vel` only while the
  configured button is held. Release it and the rover stops.
- **Joystick staleness watchdog** (`joy_timeout`). If `/joy` stops arriving
  (controller off, Bluetooth dropped) `teleop_joy` publishes zero `/cmd_vel`
  regardless of the last stick position it saw.
- **Command staleness watchdog** (`cmd_timeout`). If `/cmd_vel` itself stops
  arriving (teleop crashed, for instance) the driver commands all four wheels
  to zero, independently of the above. This is re-issued every control cycle and
  warns if it is not acknowledged.
- **Shutdown stop.** On Ctrl+C the driver sends a zero-duty command to both
  controllers. This is the **weakest** stop here, not the strongest: it is
  unramped, attempted once, its result is not checked, and it drops the wheels
  out of closed-loop regulation entirely, so there is no holding torque and the
  rover can roll on a slope.

**None of these replaces a physical kill switch or being ready to cut power.**
They cover software failure modes only. A wedged process, a hung serial write,
or a controller fault is outside what any of them can catch.

---

## Related

- [`perception_pkg/wheel_odometry`](../perception_pkg/wheel_odometry/README.md):
  consumes `roboclaw/wheel_encoders`; holds the udev rule
- [`helios_description`](../helios_description/README.md): same geometry
  constants
