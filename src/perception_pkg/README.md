# perception_pkg

Perception stack for the Helios robot..
It bundles two sensor subsystems:

- **LiDAR** — Hokuyo **UST-10LX** 2D scanner over Ethernet (`urg_node2` driver)
- **Camera** — Stereolabs **ZED 2i** stereo camera (`zed-ros2-wrapper`)

Build the workspace before running anything below:

```bash
cd ~/helios_ws
colcon build --symlink-install
source install/setup.bash
```

> source `install/setup.bash` in **every** new terminal before launching a node.

---

# SETUP LIDAR

The Hokuyo UST-10LX talks over Ethernet on the `192.168.0.0/24` subnet. The sensor
defaults to `192.168.0.10`; the Jetson's wired interface must be on the same subnet.

- Verify the sensor is reachable before launching ROS:
  ```bash
  ping 192.168.0.10
  ```

Connection settings live in `LiDAR/urg_node2/config/params_ether.yaml`
(`ip_address: 192.168.0.10`, `ip_port: 10940`, `frame_id: laser`). Edit that file if
your sensor uses a different address.

# ROS_SETUP LIDAR

The driver is a **lifecycle node** that auto-starts (configures → activates) on launch.

- Launch the driver (Ethernet config):
  ```bash
  ros2 launch urg_node2 urg_node2.launch.py
  ```

- Useful launch arguments (defaults shown):
  - `auto_start:=true` — automatically transition to the Active (publishing) state
  - `node_name:=urg_node2` — node name
  - `scan_topic_name:=scan` — output topic

- Confirm scans are publishing:
  ```bash
  ros2 topic hz /scan
  ros2 topic echo /scan --once
  ```

- Visualize in RViz (fixed frame `laser`):
  ```bash
  rviz2 -d LiDAR/rviz/lidar_scan.rviz
  ```

---

# SETUP Camera

- Install the **ZED SDK** matching this JetPack/CUDA version and confirm the camera is
  detected (the SDK's `ZED_Diagnostic` / `ZED_Explorer` tools).
- Plug the ZED 2i into a USB 3.0 port (blue). On first launch the SDK optimizes the AI
  depth/detection models, which can take a few minutes.

# ROS_SETUP Camera

- Launch the ZED 2i wrapper:
  ```bash
  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
  ```

- Key topics published by the wrapper:
  - `/zed/zed_node/rgb/color/rect/image` — rectified RGB image (also `/compressed`, `/theora`, `/zstd`)
  - `/zed/zed_node/depth/depth_registered` — registered depth map
  - `/zed/zed_node/point_cloud/cloud_registered` — 3D point cloud
  - `/zed/zed_node/odom` — visual-inertial odometry
  - `/zed/zed_node/pose` — fused camera pose (`/pose/status` for tracking state)
  - `/zed/zed_node/imu/data` — IMU measurements
  - `/zed/zed_node/status/health`, `/status/heartbeat` — node health/liveness
  - `/tf`, `/tf_static`, `/zed/joint_states` — camera frame transforms

  List them live with `ros2 topic list`.

- Camera tuning lives in `Camera/zed-ros2-wrapper/zed_wrapper/config/`
  (`common_stereo.yaml` for shared settings, `zed2i.yaml` for model-specific ones).

---

# Visualize both sensors together

With the LiDAR and camera nodes running, open the combined RViz config:

```bash
rviz2 -d rviz/visual_odemetry_with_LiDAR.rviz
```
