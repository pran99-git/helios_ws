# sensor_fusion

Sensor fusion for the Helios A4WD3 mecanum rover, combining **all four sources**
the right way (REP-105):

| Source | How it's used |
|--------|---------------|
| Wheel odometry (`/wheel/odometry`) | EKF input — body velocities `vx, vy, vyaw` |
| ZED 2i camera + IMU (`/zed/zed_node/odom`) | EKF input — VIO body velocities (camera **and** IMU enter here together) |
| Hokuyo LiDAR (`/scan`) | published here, consumed by the mapping layer (`mapping_localization_pkg`) |

## Architecture & TF ownership

```
 map ──[mapping layer]──► odom ──[robot_localization EKF]──► base_link ──[robot_state_publisher]──► sensors/wheels
     (NOT this package)
```

| Transform | Owner (only publisher) |
|-----------|------------------------|
| `map → odom`        | **not this package** — `mapping_localization_pkg` (slam_toolbox or RTAB-Map) |
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
sudo apt install ros-jazzy-robot-localization
```

## Build & run

```bash
cd ~/helios_ws
colcon build --packages-select sensor_fusion
source install/setup.bash

# Everything: description + wheel odom + ZED + LiDAR + EKF
ros2 launch sensor_fusion bringup.launch.py

# Just the EKF (if drivers/topics are already up):
ros2 launch sensor_fusion ekf.launch.py
```
Toggles: `camera:=false`, `lidar:=false`, `rviz:=true`.

> This package stops at `/odometry/filtered`. It does **not** start a mapper --
> `map -> odom` belongs to `mapping_localization_pkg`, which runs separately:
> ```
> ros2 launch mapping_localization_pkg slam_toolbox.launch.py   # 2D LiDAR SLAM
> ros2 launch mapping_localization_pkg rtabmap.launch.py        # 3D RGB-D SLAM
> ```

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
- Heading drift in `odom` is expected and corrected by the mapping layer at the
  `map` level. If you need a tighter heading before SLAM, see the optional IMU
  block in `ekf.yaml` (only with the ZED's internal IMU fusion disabled).
- `process_noise_covariance` is left at defaults; tune after observing behavior.
