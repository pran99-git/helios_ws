# wheel_odometry

Turns raw wheel encoder counts into an estimate of where the robot is.

One node, `wheel_odometry_node`. It subscribes to `roboclaw/wheel_encoders`
(four raw counters), applies mecanum kinematics to recover how the robot body
moved, integrates that into a position, and publishes `nav_msgs/Odometry` on
`/wheel/odometry` for the EKF to fuse.

> **This node does not touch the motor controllers.** All RoboClaw serial I/O
> belongs to `roboclaw_driver_node` in
> [`low_level_control_pkg`](../../low_level_control_pkg/README.md): a single
> serial port cannot safely be shared between two processes. This node consumes
> that driver's topic and does everything downstream of it.

**Read [the root README](../../../README.md) first** for the system overview.

---

## What odometry is, and why mecanum makes it harder

**Odometry** is estimating how far you have moved by counting your own wheel
rotations, which is dead reckoning. Each wheel carries a quadrature
**encoder**: a sensor producing a counter that ticks up as the wheel turns
forward and down as it turns back. Knowing counts-per-revolution and wheel radius converts ticks
into metres travelled.

On a normal differential-drive robot, two wheel distances are enough to
reconstruct the motion. Mecanum wheels are different: the angled rollers mean
each wheel's rotation contributes to forward motion, sideways motion *and*
rotation at the same time. All four wheels must be read, and the body motion is
recovered by combining them.

With per-wheel linear displacements `d_fl, d_fr, d_rl, d_rr`, wheel radius `r`,
and `L = (wheelbase + track_width) / 2`:

```
forward     dx = ( d_fl + d_fr + d_rl + d_rr) / 4
sideways    dy = (-d_fl + d_fr + d_rl - d_rr) / 4
rotation    dθ = (-d_fl + d_fr - d_rl + d_rr) / (4·L)
```

Read those as difference patterns: all four the same → pure forward; diagonal
pairs opposed → strafe; left pair opposed to right pair → spin in place. This is
the standard X-roller mecanum model. The node integrates these increments into a
planar pose `(x, y, θ)` using the midpoint heading over each step, and handles
the encoders' signed 32-bit wraparound.

### Why this output is fused rather than trusted

Dead reckoning drifts: every slip and every rounding error is added to the total
permanently and never corrected. Mecanum wheels slip *more* than ordinary
wheels, and they slip most in the sideways direction, so `vy` is the noisiest
output here.

That is why the EKF fuses this with the ZED's visual-inertial odometry, and why
it fuses **only the velocities** from this node, not the absolute position. The
covariance values in the config tell the EKF how much to distrust each channel:
they are how "wheel `vy` is unreliable" gets expressed numerically.

---

## Interfaces

| Direction | Name | Type | Notes |
|---|---|---|---|
| Subscribes | `roboclaw/wheel_encoders` | `sensor_msgs/JointState` | `position[]` = raw counts per corner |
| Publishes | `/wheel/odometry` | `nav_msgs/Odometry` | twist has `linear.x`, `linear.y`, `angular.z` |
| Publishes | TF `odom → base_link` | `tf2` | **disabled in normal operation**, see below |

There is no publish rate parameter: one odometry message goes out per encoder
message received, so the rate is set by the driver's `encoder_publish_rate`.

### On TF publishing

The node *can* publish `odom → base_link`, and does by default when run standalone.

On the real robot it must not, because the EKF publishes that same transform and
two publishers of one transform makes the robot's position flicker between two
answers. `sensor_fusion/launch/bringup.launch.py` therefore starts this node
with `publish_tf: False` overriding the config file.

Leave `publish_tf: true` in the YAML; it makes the node usable on its own for
calibration, and the bring-up overrides it where it matters.

---

## Files

```
wheel_odometry/
  wheel_odometry_node.py    Thin ROS adapter. Parameters, pub/sub, conversion.
  mecanum_odometry.py       The maths. Forward kinematics + pose integration,
                            no ROS dependency.
  __init__.py               Marks the directory as a Python package.
config/
  wheel_odometry.yaml       Geometry, encoder counts, sign flags, covariances.
launch/
  wheel_odometry.launch.py  Starts the node with the config file.
udev/
  99-roboclaw.rules         Pins the RoboClaw USB ports to stable names.
setup.py / setup.cfg        Python package build; declares the executable.
package.xml                 Package metadata and dependencies.
```

### `wheel_odometry/mecanum_odometry.py`

The maths, with no ROS imports, so it can be read and tested on its own:
`meters_per_count`, `lever_arm`, `body_displacement` (the forward kinematics
above) and `PlanarPose` (midpoint-heading integration). It is the exact inverse
of `low_level_control_pkg`'s `mecanum_kinematics`; change one and check the
other.

### `wheel_odometry/wheel_odometry_node.py`

The ROS adapter. Declares parameters, subscribes to the encoder topic, converts
counts to per-wheel metres, hands them to `mecanum_odometry`, and publishes the
result as `nav_msgs/Odometry`.

One thing is hardcoded rather than configurable: which motor channel is which
corner. Left controller `M2` = front left, `M1` = rear left; right controller
`M1` = front right, `M2` = rear right. That mapping reflects the physical
wiring. If a wheel is rewired, this is the file to change. The config's
`invert_*` flags fix *sign*, not *identity*.

### `config/wheel_odometry.yaml`

| Parameter | Value | What it does |
|---|---|---|
| `encoder_topic` | `roboclaw/wheel_encoders` | Where counts come from |
| `wheel_radius` | 0.076 m | Counts → metres |
| `wheelbase` | 0.220 m | Front↔rear spacing; affects rotation scale |
| `track_width` | 0.330 m | Left↔right spacing; affects rotation scale |
| `counts_per_rev` | 2448.0 | 12 PPR × 4 (quadrature) × 51 (gearbox) |
| `invert_*` (×4) | all `false` | Flip if a wheel counts down when driven forward |
| `publish_tf` | `true` | Overridden to `false` by the bring-up |
| `pose_covariance_diagonal` | `[0.01, 0.05, 1e+6, 1e+6, 1e+6, 0.05]` | How much the EKF should trust each channel |
| `twist_covariance_diagonal` | same | Same, for velocities |

Three notes on those values.

**`counts_per_rev` is per wheel revolution, not per motor revolution.** The
encoder is on the motor shaft before the 51:1 gearbox, and quadrature decoding
gives four edges per pulse, hence `12 × 4 × 51`.

**The `1e+6` covariance entries mean "ignore this".** Order is
`[x, y, z, roll, pitch, yaw]`. A ground robot cannot observe z, roll or pitch
from wheel encoders, so those get an enormous variance and the EKF discards
them. The real values are x, y and yaw.

(Written `1e+6`, with the explicit `+`, on purpose. ROS 2's own YAML parser
reads `1e6` as a float either way, but strict YAML 1.1, which PyYAML
implements, requires a signed exponent and otherwise parses it as a *string*.
Any Python tooling that reads these configs would silently get `'1e6'`.)

**`y` is deliberately 5× `x`.** Both are observable on a mecanum rover, but they
are not equally trustworthy. Lateral motion is produced entirely by the free
rollers, and the per-wheel velocity PID has no body-level feedback: it holds
each wheel to its setpoint and cannot tell that the body went sideways instead
of straight. Ground tests confirmed the rover tracks well forward/back and in
yaw, and drifts off-axis only when strafing. The raised variance is what tells
the EKF to lean on the ZED's visual-inertial odometry for lateral velocity while
still trusting the wheels longitudinally. AMCL's `alpha5` is set high for the
same reason.

**The geometry constants also appear in `helios_description/urdf/base.xacro` and
in the RoboClaw driver.** They are not shared automatically. Re-measuring the
rover means editing all three.

### `launch/wheel_odometry.launch.py`

Starts the node with the config file. Accepts `config_file:=/path/to/params.yaml`
to point at an alternative, useful for testing calibration values without
editing the tracked config.

### `udev/99-roboclaw.rules`

A Linux rule giving each RoboClaw a stable device name. It lives in this package
for historical reasons; it is now used by
[`low_level_control_pkg`](../../low_level_control_pkg/README.md), which owns the
serial ports. Installation instructions are in that package's README.

---

## Build

```bash
cd ~/helios_ws
colcon build --packages-select wheel_odometry --symlink-install
source install/setup.bash
```

## Run

Normally started by the bring-up, not directly:

```bash
ros2 launch sensor_fusion bringup.launch.py
```

Standalone, for calibration (needs the RoboClaw driver running to supply counts):

```bash
ros2 launch low_level_control_pkg roboclaw_driver.launch.py   # terminal 1
ros2 launch wheel_odometry wheel_odometry.launch.py           # terminal 2
```

---

## Calibration

Do this once per rover, and again after changing wheels, gearboxes or wiring.
Each step has a clear pass condition.

### Step 1: wheel signs (wheels off the ground)

Every wheel must count *up* when driven forward. Use the monitor in
`low_level_control_pkg`, which prints all four corners live:

```bash
ros2 run low_level_control_pkg wheel_monitor
```

Turn each wheel by hand in its forward direction:

- Every corner should read **positive**. Any that reads negative → set that
  wheel's `invert_*` flag to `true` in `wheel_odometry.yaml`.
- If turning one wheel changes a *different* corner's row, the wiring does not
  match the hardcoded channel map. Fix the cabling or the node, not the flags.

Then check the derived twist: pushing the rover forward should give `vx` > 0,
strafing left `vy` > 0, rotating counter-clockwise `ωz` > 0.

### Step 2: linear scale (on the ground)

Mark a 1.0 m line. Zero the odometry by restarting the node, push the rover
along the line, and read the result:

```bash
ros2 topic echo /wheel/odometry --once
```

`pose.position.x` should read ≈ 1.0. If it is consistently off, scale
`counts_per_rev` by `reported / actual`. A ~2 % error is normal for mecanum on
a hard floor; ~10 % means a wrong constant, not slip.

### Step 3: rotation scale (on the ground)

Rotate the rover in place through a known angle (360° is easiest to judge by
eye) and compare the yaw in `/wheel/odometry`. If reported rotation is too
large, `L` is too small: increase `wheelbase` and/or `track_width`. Adjust these
only after step 2 passes, since linear scale error feeds into rotation error.

---

## Verify

**The node is receiving counts.** The most common failure is that it is not,
because the RoboClaw driver is not running:

```bash
ros2 topic hz roboclaw/wheel_encoders
ros2 node info /wheel_odometry            # confirm the subscription is connected
```

Silence on that topic means nothing is publishing counts. Odometry will sit at
zero and never move.

**Output is sane:**

```bash
ros2 topic hz /wheel/odometry             # matches the encoder rate
ros2 topic echo /wheel/odometry --once
```

With the rover stationary, all velocities should read ~0 and the pose should not
creep. Creeping while stationary means an encoder is reporting noise.

**It responds correctly to motion.** Push the rover by hand and watch:

```bash
ros2 topic echo /wheel/odometry --field twist.twist
```

Forward → `linear.x` positive. Strafe left → `linear.y` positive. Rotate
counter-clockwise → `angular.z` positive. Any sign that is backwards is an
`invert_*` flag.

**It is not publishing TF when it shouldn't be.** With the full stack running:

```bash
ros2 topic info /tf --verbose | grep -c "Node name: wheel_odometry"
```

Expect `0`. Anything else means `publish_tf` was not overridden and the robot's
position will fight with the EKF.

---

## Limitations

- Dead reckoning drifts without bound. This node is one input to the EKF, not a
  position source on its own.
- Mecanum wheels slip laterally more than they roll cleanly, so `vy` is the
  least trustworthy channel, reflected in its covariance, which is 5× the
  longitudinal one.
- Encoders only. This node never commands the motors.
- Reported motion assumes the wheels are on the ground and gripping; wheels-up
  testing shows motion that did not happen.

---

## Related

- [`low_level_control_pkg`](../../low_level_control_pkg/README.md): publishes
  the encoder counts, owns the serial ports, has `wheel_monitor`
- [`sensor_fusion`](../sensor_fusion/README.md): consumes `/wheel/odometry`
- [`helios_description`](../../helios_description/README.md): same geometry
  constants
