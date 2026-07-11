# mapping_localization_pkg

Mapping and localization approaches for Helios, beyond the 2D `slam_toolbox`
pipeline in `sensor_fusion`. Currently: RTAB-Map (RGB-D + LiDAR registration,
GTSAM pose-graph optimization). Each technique lives in its own top-level
folder (`rtabmap/` now).

## Saved maps: `maps/`

Every RTAB-Map run gets its own database instead of all runs overwriting a
single `~/.ros/rtabmap.db` -- `rtabmap.launch.py` defaults `database_path` to
`maps/rtabmap_<run_name>.db`, where `run_name` defaults to a timestamp
(override either with `run_name:=my_test` or `database_path:=/anywhere.db`
directly).

`maps/` contents are gitignored (large binary files, one per run) -- only the
folder itself is tracked.

**The `.db` file *is* the 3D map** -- it's the full SLAM session: every
keyframe's RGB-D data, the pose graph, loop closures. The occupancy grid and
point cloud are derived from what's in it.

**To also save a standalone 2D map** (`.pgm` + `.yaml`, e.g. for Nav2) for
the same run, while `rtabmap.launch.py` is still running:
```
~/helios_ws/src/mapping_localization_pkg/scripts/save_2d_map.sh <run_name>
```
Use the same `run_name` the mapping run was launched with, so the `.db`,
`.pgm`, and `.yaml` for a given run are grouped by filename.

## Running

```
ros2 launch mapping_localization_pkg rtabmap.launch.py
# custom run name:
ros2 launch mapping_localization_pkg rtabmap.launch.py run_name:=garage_test
```

Runs alongside `sensor_fusion`'s existing `slam_toolbox` + EKF stack rather
than replacing it (`publish_tf_map:=false` by default) -- see the launch
file's own docstring for the full TF-ownership rationale and all toggles
(`rviz`, `rtabmap_viz`, `localization`, `publish_tf_map`).
