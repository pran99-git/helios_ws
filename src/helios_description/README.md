# helios_description

URDF/xacro model of the **Lynxmotion A4WD3 Mecanum "Helios" rover**. This is a
model TF tree (`base_link` → wheels + sensors) for sensor fusion and RViz.

## TF tree

```
base_link (root; odom -> base_link comes from the wheel_odometry node)
├─ base_footprint            (ground projection, z = -wheel_radius)
├─ front_left_wheel  … rear_right_wheel   (continuous joints, axis = y)
├─ zed_camera_link           (ZED 2i mount)
└─ laser                     (Hokuyo UST-10LX mount; matches urg_node2 frame_id)
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
   `laser_mount_joint` in [urdf/sensors.xacro](urdf/sensors.xacro). 
   Measure the real offsets from the rover center and update them
   (verify visually in RViz).
2. **Meshes** — done: chassis, both A4WD3 wheel variants, the Hokuyo, and the
   ZED2i are wired in as `<visual>` mesh geometry (`<collision>` stays
   primitives per meshes/README.md). The per-mesh rotation/translation was
   *derived* from each STL's bounding box (matching known hardware dimensions),
   not measured from CAD — **verify in RViz**:
   - chassis/sensor orientation are correct now.
   - the two wheel mesh variants (`A4WD3-W-ME-A/B.STL`) are assigned to
     diagonal corners per the standard mecanum roller convention.
3. **ZED frame** — this URDF defines `zed_camera_link`. Ensure the ZED wrapper
   does not *also* publish `base_link -> zed_camera_link` (would give the frame
   two parents). Let the wrapper publish only its internal camera frames.

## Next

Feed `base_link` into a `robot_localization` EKF fusing `wheel_odometry` +
ZED odom/IMU. The static sensor transforms here make the LiDAR `/scan` and ZED
cloud line up in `base_link`.
