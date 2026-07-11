# wheel_odometry

Wheel odometry for the **Lynxmotion A4WD3 Mecanum** rover, driven by **two
RoboClaw** motor controllers — one per side (left RoboClaw → two left wheels,
right RoboClaw → two right wheels), each on its own USB port.

Mecanum wheels are **holonomic** (the rover can strafe sideways), so the node
reads **all four wheels independently** over the RoboClaw packet-serial
protocol, applies the mecanum forward-kinematics to recover the body twist
(`vx, vy, ωz`), integrates a full planar pose `(x, y, θ)`, and publishes
`nav_msgs/Odometry` (+ the `odom → base_link` TF) for sensor fusion
(`robot_localization`, or your own EKF fusing this with the ZED VIO and LiDAR).

## A4WD3 Mecanum hardware (from Lynxmotion / RobotShop)

| Spec | Value | Notes |
|------|-------|-------|
| Mecanum wheel diameter | 152 mm → radius **0.076 m** |
| Motors | 12 V DC + planetary gearbox |
| Encoder | magnetic, **12 PPR** at motor shaft, quadrature → **×4** |
| Gear ratio | **51:1** |
| `counts_per_rev` (at wheel) | `12 × 4 × 51 = 2448` |
| Wheelbase (front↔rear center) | **0.220 m** |
| Track width (left↔right center) | **0.330 m** |

## Mecanum model

With per-wheel distances `d_fl, d_fr, d_rl, d_rr` (m), wheel radius `r`, and
`L = lx + ly = (wheelbase + track_width) / 2`:

```
dx_body =  ( d_fl + d_fr + d_rl + d_rr) / 4
dy_body =  (-d_fl + d_fr + d_rl - d_rr) / 4
dθ      =  (-d_fl + d_fr - d_rl + d_rr) / (4·L)
```

Pose is integrated in the world frame with midpoint-heading; signed 32-bit
encoder wrap is handled in the driver.

## Published

| Name | Type | Notes |
|------|------|-------|
| `wheel/odometry` (configurable) | `nav_msgs/Odometry` | twist has `linear.x`, `linear.y`, `angular.z` |
| TF `odom → base_link` (optional) | `tf2` | disable with `publish_tf:=false` |

## Build

```bash
cd ~/helios_ws
pip3 install pyserial            # or: sudo apt install python3-serial
colcon build --packages-select wheel_odometry --symlink-install
source install/setup.bash
```

## Configure

Edit `config/wheel_odometry.yaml`:

1. **Ports** — defaults `/dev/ttyACM0` (left controller) and `/dev/ttyACM1`
   (right controller). These can swap on reboot; see *Stable port names* below.
2. **Signs** — wiring is hardcoded in the node: left controller (ACM0)
   `front_left`=M2, `rear_left`=M1; right controller (ACM1) `front_right`=M1,
   `rear_right`=M2. The `invert_*` flags make forward motion read positive —
   confirm with `wheel_monitor`.

## Run

```bash
ros2 launch wheel_odometry wheel_odometry.launch.py
# or a custom param file:
ros2 launch wheel_odometry wheel_odometry.launch.py config_file:=/path/to/params.yaml
```

### Step 1 — wheel signs (off the ground, with `wheel_monitor`)

Stop the node (the serial port is exclusive) and run the monitor, which reads
the RoboClaws directly and shows each corner live:

```bash
ros2 run wheel_odometry wheel_monitor
```

Spin one wheel at a time in its forward-drive direction:

- Every corner should read **positive** when driven forward; any that reads
  negative → set its `invert_*` flag `true` in the YAML.
- Check the computed twist line: forward → `vx+`, strafe-left → `vy+`,
  rotate CCW → `wz+`.

(If a wheel turns the *wrong* corner's row, the wiring isn't what the fixed
`ACM0=left/ACM1=right`, `M1=front/M2=rear` mapping assumes — re-check the cabling.)

### Step 2 — linear & rotation scale (on the ground, with the node)

3. **Linear scale** — drive a measured 1.0 m forward, compare `pose.position.x`.
   `counts_per_rev` is 2448 (51:1); fine-tune by `reported / actual` if needed.
4. **Rotation scale** — rotate in place a known angle; if reported yaw is off,
   adjust `wheelbase` + `track_width` (equivalently `L`).

## Stable port names

`/dev/ttyACM0` and `/dev/ttyACM1` can renumber on replug/reboot, swapping
left/right. The udev rule in [`udev/99-roboclaw.rules`](udev/99-roboclaw.rules)
pins each RoboClaw to a fixed name. The two units are identical and have **no
unique USB serial**, so the rule matches the **physical USB port**:

```
left  controller -> port 1-4.4 -> /dev/roboclaw_left
right controller -> port 1-4.3 -> /dev/roboclaw_right
```

Install it:

```bash
sudo cp src/perception_pkg/wheel_odometry/udev/99-roboclaw.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/roboclaw_*        # should symlink to the right ttyACM*
```

The config already points at `/dev/roboclaw_left` and `/dev/roboclaw_right`.

**Important:** because matching is by physical port, keep each RoboClaw in its
current socket (and the hub in the same Jetson port). If you move one, update the
`KERNELS==` value in the rule — find the new port with
`udevadm info -n /dev/ttyACM0 | grep DEVPATH` (the `1-4.x` segment is the port).
The rule also sets `MODE=0666`, so it permanently fixes the serial permission
issue too.

## Notes / limitations

- Serial access may need the `dialout` group:
  `sudo usermod -aG dialout $USER` then re-login.
- This is dead-reckoning — it drifts, and mecanum wheels slip more than
  standard wheels, so wheel `vy` is noisier than `vx`. That is exactly why it is
  fused with low-drift sensors (ZED VIO, LiDAR). Tune the covariance diagonals
  in the YAML to reflect this (looser on `y`).
- The node reads encoders only; it does **not** command the motors.
