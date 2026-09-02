# zed_custom_tuning

Workspace-owned parameter overrides for the ZED 2i.

It sits next to `zed-ros2-wrapper` so that everything camera-related lives in
`camera/`, mirroring `lidar/custom_config`. But **it is our content, not the
vendor's**: `zed-ros2-wrapper` is a git submodule pinned to `v5.4.0`, and
anything written inside it is reverted by `git submodule update`. This package
is outside that boundary, which is the entire reason it exists.

```
config/zed_overrides.yaml    every deliberate departure from Stereolabs' defaults
```

This is the home for ZED configuration changes going forward, not just the
ones in it today. Anything that would otherwise be a hand-edit to
`zed_wrapper/config/common_stereo.yaml` or `zed2i.yaml` belongs here instead.

---

## Why this is only a YAML, when `custom_config` is a whole launch file

Because the two vendors made different choices, and it is worth knowing which
kind of vendor you are dealing with before assuming you need to fork anything.

`urg_node2`'s launch file hardcodes its parameter path with no override, so the
only way to own the Hokuyo's parameters was to own its launch file too, hence
`custom_config/launch/lidar.launch.py`.

The ZED wrapper supports overrides directly. `zed_camera.launch.py` declares a
`ros_params_override_path` argument (line 518) and appends that file to the
node's parameter list, so we keep using upstream's launch file unmodified and
only supply the delta. **Don't** copy the wrapper's launch file into this
package to achieve the same thing: it is ~700 lines of container/NITROS/SVO
plumbing that would then need hand-merging on every submodule bump.

## Precedence: what this file can and cannot override

The wrapper stacks parameter sources in this order
(`zed_camera.launch.py:415-451`), later winning:

| # | Source | Ours? |
|---|---|---|
| 1 | `common_stereo.yaml` | vendor |
| 2 | `zed2i.yaml` | vendor |
| 3 | object-detection YAMLs | vendor |
| 4 | **`config/zed_overrides.yaml`** | **this package** |
| 5 | the launch-argument dict | beats us |
| 6 | inline `param_overrides:=k:=v` | highest |

Step 5 is the trap. These are written from launch arguments **after** this file
is read, so setting them here does nothing at all and fails silently:

```
pos_tracking.publish_tf        pos_tracking.publish_map_tf
sensors.publish_imu_tf         general.camera_model
general.camera_name            general.serial_number
general.camera_id              svo.svo_path
```

Those are set as launch arguments in `sensor_fusion/launch/bringup.launch.py`
instead. `publish_tf: 'false'` in particular, which is load-bearing for TF
ownership (the EKF owns `odom -> base_link`; see that file's comment).

## Why the file is keyed `/**:`

Same reason as `custom_config` and the vendor's own configs: it must match
whatever node name the wrapper ends up using. The wrapper's `node_name`
defaults to `zed_node` but is a launch argument, and launch configurations leak
between sibling includes in one `LaunchDescription`; this workspace has
already been bitten by exactly that (the LiDAR came up as `/zed_node` once).
A wildcard cannot be bitten by it.

## What's currently overridden

See the comments in `config/zed_overrides.yaml`; the reasoning lives beside
each value, not here, so it cannot drift out of sync with what is actually set.
In summary:

| Parameter | Vendor | Ours | Why |
|---|---|---|---|
| `general.grab_compute_capping_fps` | `0.0` (no cap) | `30.0` | Halve depth compute while the sensor keeps capturing at 60 fps, so exposure stays short and turns stay unblurred |
| `general.pub_frame_rate` | `0.0` (= grab rate, 60) | `15.0` | Nothing downstream reads images faster; stops serialising 60 image+depth pairs/s |

The file also records, as comments, the things that were **considered and
deliberately left alone** (`depth_mode`, `point_cloud_freq`, `area_memory`,
`mapping_enabled`, `od_enabled`) so re-opening one of those is a decision
rather than a rediscovery.

Only values we actually change are listed. Copying a vendor default in "for
reference" silently pins it, and a future submodule bump that improves that
default would be shadowed by a stale number nobody remembers typing.

---

## Build and use

```bash
colcon build --packages-select zed_custom_tuning --symlink-install
source install/setup.bash
```

It is wired into the normal bring-up already, no extra argument needed:

```bash
ros2 launch sensor_fusion bringup.launch.py
```

To use it when launching the camera on its own:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i \
    ros_params_override_path:=$(ros2 pkg prefix --share zed_custom_tuning)/config/zed_overrides.yaml
```

## Verify it actually took effect

The wrapper logs `Using ROS parameters override file: <path>` at startup
(`zed_camera.launch.py:285`). That only proves the path was passed, not that
the values landed. Read them back off the live node:

```bash
ros2 param get /zed/zed_node general.grab_compute_capping_fps   # 30.0
ros2 param get /zed/zed_node general.pub_frame_rate             # 15.0
```

Then confirm the intended effect, and the side effect:

```bash
ros2 topic hz /zed/zed_node/rgb/color/rect/image   # ~15 Hz, was ~30
ros2 topic hz /zed/zed_node/odom                   # ~30 Hz, was ~50  <-- the side effect
ros2 topic hz /odometry/filtered                   # must stay ~30 Hz
```

`/zed/zed_node/odom` dropping is expected: positional tracking runs off
processed grabs, so capping compute caps it too. The EKF runs at 30 Hz with
`sensor_timeout: 0.2`, so 30 Hz still clears it, but with one sample of margin
rather than two. If `ekf_filter_node` starts logging *failed to meet update
rate*, raise `grab_compute_capping_fps` to `40.0` before changing anything else.

---

## Related

- [`custom_covariance`](../custom_covariance/README.md): the other half of
  `camera/`; a running node, not config. Fixes the ZED's all-zero twist
  covariance before the EKF sees it.
- [`custom_config`](../../lidar/custom_config/README.md): the same
  own-it-outside-the-submodule pattern for the Hokuyo, done the harder way
  because that vendor offers no override hook.
- [`sensor_fusion`](../../sensor_fusion/README.md): where this file is passed
  to the wrapper, and where the TF-ownership launch arguments live.
