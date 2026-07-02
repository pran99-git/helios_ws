# sensor_fusion

Sensor fusion for the Helios A4WD3 mecanum rover, combining **all four sources**
the right way (REP-105):

| Source | How it's used |
|--------|---------------|
| Wheel odometry (`/wheel/odometry`) | EKF input — body velocities `vx, vy, vyaw` |
| ZED 2i camera + IMU (`/zed/zed_node/odom`) | EKF input — VIO body velocities (camera **and** IMU enter here together) |
| Hokuyo LiDAR (`/scan`) | slam_toolbox — builds the map and the `map → odom` correction |

## Architecture & TF ownership

```
 map ──[slam_toolbox]──► odom ──[robot_localization EKF]──► base_link ──[robot_state_publisher]──► sensors/wheels
```

| Transform | Owner (only publisher) |
|-----------|------------------------|
| `map → odom`        | slam_toolbox |
| `odom → base_link`  | robot_localization EKF |
| `base_link → *`     | robot_state_publisher (URDF) |

## The two important design constraints — and how they're avoided here

1. **Duplicate `odom → base_link`.** The bring-up launches `wheel_odometry` with
   `publish_tf:=false` and the ZED wrapper with `publish_tf:=false`. They still
   publish their **topics** (consumed by the EKF) but not TF. The EKF is the sole
   owner of `odom → base_link`.
2. **IMU double-counting.** The ZED `/odom` is already visual-**inertial** (the SDK
   fuses camera + IMU). So the IMU enters via `odom1`, and the raw
   `/zed/.../imu/data` is **not** fused separately (its block in `ekf.yaml` is
   commented out, with a note on when to enable it).

## Install deps

```bash
sudo apt install ros-jazzy-robot-localization ros-jazzy-slam-toolbox
```

## Build & run

```bash
cd ~/helios_ws
colcon build --packages-select sensor_fusion
source install/setup.bash

# Everything: description + wheel odom + ZED + LiDAR + EKF + SLAM
ros2 launch sensor_fusion bringup.launch.py

# Just the fusion nodes (if drivers/topics are already up):
ros2 launch sensor_fusion fusion.launch.py
```
Toggles: `camera:=false`, `lidar:=false`, `rviz:=true`.

> slam_toolbox's async node is a lifecycle node that does **not** auto-activate
> on this build, so `fusion.launch.py` drives it through configure → activate
> automatically (LifecycleNode + event handlers). It comes up `active` with no
> manual `ros2 lifecycle set` needed. Confirm with
> `ros2 lifecycle get /slam_toolbox` → `active`.

## Verify

```bash
ros2 run tf2_tools view_frames          # expect map -> odom -> base_link -> sensors, NO duplicates
ros2 topic echo /odometry/filtered      # EKF output (fused odom)
ros2 topic hz /odometry/filtered        # ~30 Hz
ros2 node info /ekf_filter_node         # confirm it subscribes to /wheel/odometry and /zed/zed_node/odom
```
In RViz (`rviz:=true`): fixed frame `odom` or `map`; add Odometry on `/odometry/filtered`, the LaserScan, and the Map.

## Tuning notes

- Start simple: if the ZED isn't running yet, `camera:=false` runs wheel-only EKF.
- Mecanum slips sideways — if `vy` looks noisy, raise the wheel odometry `vy`
  covariance (in `wheel_odometry.yaml`) so the EKF trusts it less.
- Heading drift in `odom` is expected and corrected by slam_toolbox at the
  `map` level. If you need a tighter heading before SLAM, see the optional IMU
  block in `ekf.yaml` (only with the ZED's internal IMU fusion disabled).
- `process_noise_covariance` is left at defaults; tune after observing behavior.
