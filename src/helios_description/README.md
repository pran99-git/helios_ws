# helios_description

URDF/xacro model of the **Lynxmotion A4WD3 Mecanum "Helios" rover**. This is a
**rough primitives** model (boxes + cylinders) — enough to give the correct
TF tree (`base_link` → wheels + sensors) for sensor fusion and RViz. Swap in CAD
meshes and tune the sensor mounts later.

## TF tree

```
base_link (root; odom -> base_link comes from the wheel_odometry node)
├─ base_footprint            (ground projection, z = -wheel_radius)
├─ front_left_wheel  … rear_right_wheel   (continuous joints, axis = y)
├─ zed_camera_link           (ZED 2i mount — TODO origin)
└─ laser                     (Hokuyo UST-10LX mount — TODO origin; matches urg_node2 frame_id)
```

Geometry (from the verified rover): wheel radius 0.076 m, wheelbase 0.220 m,
track 0.330 m. Convention REP-103 (x forward, y left, z up).

## Build & view

```bash
cd ~/helios_ws
colcon build --packages-select helios_description
source install/setup.bash
ros2 launch helios_description description.launch.py
```
RViz opens showing the model; the joint_state_publisher GUI gives wheel sliders.
Launch args: `gui:=false` (zero wheel states, no sliders), `rviz:=false`.

## To finish the model

1. **Sensor mounts** — the `<origin>` of `zed_mount_joint` and
   `laser_mount_joint` in [urdf/sensors.xacro](urdf/sensors.xacro) are rough
   placeholders. Measure the real offsets from the rover center and update them
   (verify visually in RViz).
2. **Meshes** — convert CAD (STEP/SLDPRT) to STL, drop in [meshes/](meshes/),
   and replace the primitive `<geometry>` blocks (see meshes/README.md). Keep
   `<collision>` as primitives.
3. **ZED frame** — this URDF defines `zed_camera_link`. Ensure the ZED wrapper
   does not *also* publish `base_link -> zed_camera_link` (would give the frame
   two parents). Let the wrapper publish only its internal camera frames.

## Next

Feed `base_link` into a `robot_localization` EKF fusing `wheel_odometry` +
ZED odom/IMU. The static sensor transforms here make the LiDAR `/scan` and ZED
cloud line up in `base_link`.
