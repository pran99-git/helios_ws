# mapping_localization_pkg

Mapping and localization for Helios. Everything that owns `map -> odom` lives
here; sensing and odometry fusion stay in `perception_pkg`.

Two approaches, each a self-contained folder, each runnable on its own:

| Folder | What it is | Publishes |
|---|---|---|
| `slam_toolbox/` | 2D LiDAR SLAM (Hokuyo UST-10LX) | `/map`, `map -> odom` |
| `rtabmap/` | 3D RGB-D + LiDAR SLAM, GTSAM pose-graph | `/rtabmap/*`, a `.db` |

Both consume the same inputs -- `/scan`, the ZED topics, and
`/odometry/filtered` -- and both expect the sensor layer to already be running:

```
ros2 launch sensor_fusion bringup.launch.py
```

`bringup.launch.py` starts sensors + EKF only. It does **not** start a mapper;
pick one (or run both) yourself.

## slam_toolbox -- 2D LiDAR SLAM

```
ros2 launch mapping_localization_pkg slam_toolbox.launch.py
ros2 launch mapping_localization_pkg slam_toolbox.launch.py rviz:=true
```

Config: `slam_toolbox/config/slam_toolbox.yaml`. The async node is a lifecycle
node that does not auto-activate on this build, so the launch file drives it
through configure -> activate automatically -- no manual `ros2 lifecycle set`.

**Save the current map** (while it is still running -- it captures a live topic):

```
~/helios_ws/src/mapping_localization_pkg/slam_toolbox/scripts/save_map.sh
~/helios_ws/src/mapping_localization_pkg/slam_toolbox/scripts/save_map.sh lab_corridor
```

Output lands in `slam_toolbox/maps/` as `slam_toolbox_<name>.pgm` + `.yaml`,
where `<name>` defaults to a timestamp. The script refuses to overwrite an
existing map.

## rtabmap -- 3D RGB-D SLAM

```
ros2 launch mapping_localization_pkg rtabmap.launch.py
ros2 launch mapping_localization_pkg rtabmap.launch.py run_name:=garage_test
```

Every run gets its own database instead of all runs overwriting a single
`~/.ros/rtabmap.db` -- `database_path` defaults to
`rtabmap/maps/rtabmap_<run_name>.db`,
with `run_name` defaulting to a timestamp (override either with
`run_name:=my_test` or `database_path:=/anywhere.db`).

**The `.db` file *is* the 3D map** -- the full SLAM session: every keyframe's
RGB-D data, the pose graph, loop closures. The occupancy grid and point cloud
are derived from what's in it. Inspect it offline with `rtabmap-databaseViewer`.

### Do not run rtabmap_viz live on the Jetson

`rtabmap_viz:=true` pushes the Jetson past real-time and makes RTAB-Map discard
frames. Inspect the saved `.db` offline instead. See the launch file's own
docstring for all toggles (`rviz`, `rtabmap_viz`, `localization`,
`publish_tf_map`).

## Running both at once

They coexist safely: `rtabmap.launch.py` defaults to `publish_tf_map:=false`,
so slam_toolbox stays the sole `map -> odom` authority. They are otherwise
independent -- neither subscribes to anything the other publishes.

To make RTAB-Map TF-authoritative instead, set `publish_tf_map:=true` **and**
stop running `slam_toolbox.launch.py`. Never let both publish `map -> odom`.

## Map storage

| Path | Holds | Tracked? |
|---|---|---|
| `slam_toolbox/maps/` | slam_toolbox `.pgm` + `.yaml` | folder only, contents gitignored |
| `rtabmap/maps/` | RTAB-Map `.db`, plus derived `.pgm`/`.yaml`/`.ply` | folder only, contents gitignored |
