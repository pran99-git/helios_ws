# mapping_localization_pkg

Builds the map. This layer takes the sensor data and the fused odometry from
`perception_pkg` and produces a map of the space the robot has driven through,
plus the correction that keeps the robot's position from drifting.

Two approaches live here, each in its own folder and each runnable on its own.

**Read [the root README](../../README.md) first** for the system overview.

---

## What SLAM is solving

**SLAM** — Simultaneous Localisation And Mapping — is the chicken-and-egg
problem of building a map while working out where you are in it. You need a map
to know where you are; you need to know where you are to add to the map.

The way out is that odometry gives a rough answer to "where am I" for a short
time. SLAM uses it to place each new sensor reading approximately, matches that
reading against what it has already mapped to refine the placement, and adds it.
The refinement accumulates into a correction between where odometry thinks the
robot is and where it actually is.

That correction is the `map → odom` transform, and publishing it is this layer's
core job:

```
map ──[this layer]──► odom ──[EKF]──► base_link ──[URDF]──► sensors
```

`odom → base_link` drifts smoothly and never jumps — good for control.
`map → odom` absorbs the drift, and jumps when the robot recognises a place it
has been before (**loop closure**). Together they give a position that is both
smooth short-term and correct long-term.

**Only one node may publish `map → odom`.** Running both mappers with both
publishing it means the robot's position fights between two answers.

---

## Which mapper should I use?

| | `slam_toolbox` | `rtabmap` |
|---|---|---|
| Sensors | Laser only | Camera (RGB-D) + laser |
| Output | 2D occupancy grid — a floor plan | 3D map with colour and depth |
| CPU/GPU cost | Light | Heavy |
| Result format | `.pgm` image + `.yaml` | A single `.db` database |
| Standard nav stacks accept it | Yes, directly | Needs a derived grid |
| Publishes `map → odom` | Yes, always | Only if you ask (`publish_tf_map:=true`) |

**Start with `slam_toolbox`.** It is cheaper, more reliable, produces the format
navigation tools expect, and a 2D floor plan is enough for a rover driving on
flat ground.

**Use `rtabmap`** when you need the third dimension — obstacles the laser's flat
slice misses, or a visual record of the space.

**Running both at once** is supported and is how RTAB-Map is normally evaluated:
`rtabmap.launch.py` defaults to `publish_tf_map:=false`, so slam_toolbox stays
the sole transform authority while RTAB-Map builds its database alongside. They
share inputs and neither subscribes to the other's output.

Both need the sensing layer running first:

```bash
ros2 launch sensor_fusion bringup.launch.py
```

---

## Files

```
slam_toolbox/
  launch/slam_toolbox.launch.py   Starts slam_toolbox, drives its lifecycle.
  config/slam_toolbox.yaml        All tuning: solver, scan matcher, loop closure.
  rviz/slam.rviz                  RViz layout for watching the map build.
  scripts/save_map.sh             Saves the live /map to disk.
  maps/                           Saved 2D maps. Contents gitignored.
rtabmap/
  launch/rtabmap.launch.py        Wraps the upstream launch file with our settings.
  maps/                           Run databases. Contents gitignored.
CMakeLists.txt                    Installs launch/, config/, rviz/ — but not maps/.
package.xml                       Metadata and dependencies.
```

There is no source code in this package. Both mappers are third-party ROS
packages; what lives here is the configuration and launch wiring that makes them
work with *this* robot. That is deliberate — the algorithms are well-tested
upstream, and the value is in the integration.

### `CMakeLists.txt`

Installs the two folders' `launch/`, `config/` and `rviz/` subdirectories
explicitly, one by one, rather than installing each folder wholesale.

The reason is `maps/`. RTAB-Map databases run to tens of gigabytes, and a plain
(non-symlink) build would copy every one of them into `install/` on every build.
The obvious fix — installing the whole folder with `PATTERN maps EXCLUDE` — is
**silently ignored** under `colcon build --symlink-install`, which symlinks whole
directories and never evaluates the exclusion. Listing subdirectories explicitly
is the only approach that works under both build modes.

Both launch files therefore read and write `maps/` in the **source tree**, not
in `install/` — run outputs must survive a rebuild.

---

## slam_toolbox — 2D laser SLAM

Consumes `/scan` and the EKF's `odom → base_link`. Produces `/map` (a
`nav_msgs/OccupancyGrid`) and `map → odom`.

### Run

```bash
ros2 launch mapping_localization_pkg slam_toolbox.launch.py
ros2 launch mapping_localization_pkg slam_toolbox.launch.py rviz:=true
```

### `launch/slam_toolbox.launch.py`

Starts `async_slam_toolbox_node` and drives it through its lifecycle.

A **lifecycle node** is a ROS node with explicit states — it starts
*unconfigured* and does nothing until told to *configure* and then *activate*.
On this build slam_toolbox does not auto-activate, so the launch file does it
with two event handlers: configure when the process starts, activate once
configuring completes. The result is a node that comes up publishing, with no
manual `ros2 lifecycle set` needed.

If `/map` is silent, this is the first thing to check — see Verify below.

`rviz:=true` also opens RViz with the layout in `rviz/slam.rviz`.

### `config/slam_toolbox.yaml`

The values that matter most day to day:

| Parameter | Value | Meaning |
|---|---|---|
| `mode` | `mapping` | Build a new map. `localization` reuses a saved one |
| `resolution` | 0.05 | Map cell size, metres — 5 cm per pixel |
| `max_laser_range` | 10.0 | Matches the UST-10LX's usable range |
| `map_update_interval` | 2.0 | Seconds between published map updates |
| `minimum_travel_distance` | 0.2 | Metres of motion before a new scan is added |
| `minimum_travel_heading` | 0.2 | Radians, same idea |
| `transform_timeout` | 0.5 | How long to wait for a transform |
| `scan_queue_size` | 20 | Scans buffered while waiting for transforms |
| `do_loop_closing` | true | Enable drift correction on revisit |
| `loop_search_maximum_distance` | 3.0 | Metres — how far away to look for a match |

Three notes.

**The `minimum_travel_*` values decide map density.** Scans are only added after
the robot has moved this far, which prevents thousands of near-identical scans
piling up while it sits still. Lower them for finer detail in tight spaces at
the cost of more computation.

**`scan_queue_size: 20` was a fix, not a default.** The symptom was
`Message Filter dropping message: frame 'laser' ... queue is full` in the log.
That message is a *capacity* eviction, governed by queue size — a different
mechanism from `transform_timeout`, which governs a scan timing out while
waiting. At ~40 Hz the default queue drained in well under 200 ms and overflowed
long before any timeout could apply. If you see that message return, raise this,
not the timeout.

**The solver and scan-matcher blocks** (Ceres settings, correlation search space,
penalties) are largely upstream defaults. They rarely need touching; if the map
is bad, the cause is almost always odometry quality or sensor mounting, not
these.

### Saving a map

```bash
~/helios_ws/src/mapping_localization_pkg/slam_toolbox/scripts/save_map.sh
~/helios_ws/src/mapping_localization_pkg/slam_toolbox/scripts/save_map.sh lab_corridor
```

Output lands in `slam_toolbox/maps/` as `slam_toolbox_<name>.pgm` (the map as a
greyscale image) plus `.yaml` (its resolution and origin). With no argument the
name is a timestamp, matching how RTAB-Map names its runs. The script refuses to
overwrite an existing map.

Two constraints:

- **Run it while slam_toolbox is still up.** It saves a live topic, not
  something read back from disk afterwards. Close the mapper first and the map
  is gone.
- **Run it from the source path shown above,** not via `ros2 run`. It locates
  `maps/` relative to its own file, which only resolves correctly in the source
  tree.

---

## rtabmap — 3D RGB-D SLAM

Consumes the ZED's RGB and depth images, `/scan`, and `/odometry/filtered`.
Produces a `.db` database and, optionally, `map → odom`.

### Run

```bash
ros2 launch mapping_localization_pkg rtabmap.launch.py
ros2 launch mapping_localization_pkg rtabmap.launch.py run_name:=garage_test
```

### `launch/rtabmap.launch.py`

Wraps the upstream `rtabmap_launch` file, supplying this robot's topics, frames
and algorithm settings.

**It reuses the EKF's odometry rather than computing its own** (`visual_odometry:
false`, `odom_topic: /odometry/filtered`). RTAB-Map can estimate visual odometry
itself, but running it alongside the EKF would mean two competing pose estimators
and twice the compute.

**Every run gets its own database.** `database_path` defaults to
`rtabmap/maps/rtabmap_<run_name>.db`, with `run_name` defaulting to a timestamp
— rather than all runs overwriting a single `~/.ros/rtabmap.db`. Override either
with `run_name:=my_test` or `database_path:=/anywhere.db`.

**The `.db` file *is* the 3D map.** It holds the whole SLAM session: every
keyframe's RGB-D data, the pose graph, and the loop closures. The point cloud
and occupancy grid you see are derived from it. Inspect one offline with
`rtabmap-databaseViewer`.

Arguments:

| Argument | Default | Effect |
|---|---|---|
| `publish_tf_map` | `false` | Own `map → odom`. Keep false while slam_toolbox runs |
| `localization` | `false` | Localise against a saved database instead of mapping |
| `run_name` | timestamp | Names this run's database |
| `database_path` | derived | Full override of the output path |
| `rtabmap_viz` | `false` | RTAB-Map's own GUI — **see the warning below** |
| `rviz` | `false` | RViz instead |
| `wait_for_transform` | 0.5 | Seconds to wait for a transform before giving up |
| `sync_queue_size` | 30 | Buffer for time-synchronising camera + laser + odometry |

The internal algorithm settings passed via `args`:

- `Reg/Strategy 2` — register scans using vision *and* laser, since both are
  available, rather than vision alone.
- `Optimizer/Strategy 2` — use GTSAM for pose-graph optimisation.
- `Optimizer/GravitySigma 0.3` — keep optimised poses gravity-aligned. Only
  meaningful because visual-inertial odometry feeds this, and it depends on the
  ZED's gravity alignment at startup being good.
- `RGBD/NeighborLinkRefining true` — refine sequential pose-graph edges instead
  of copying the raw odometry delta, which can carry multi-metre jumps if the
  visual odometry glitches.

### Do not run `rtabmap_viz` live

`rtabmap_viz:=true` pushes the Jetson past real time and makes RTAB-Map discard
frames. This was measured directly: with the GUI on, "Could not convert rgb/depth
msgs" errors fired continuously and per-frame processing took 0.374 s; with it
off, over the same route, zero errors and 0.176 s per frame.

Inspect the saved database afterwards instead:

```bash
rtabmap-databaseViewer ~/helios_ws/src/mapping_localization_pkg/rtabmap/maps/rtabmap_<name>.db
```

The camera's `depth_mode: NEURAL_LIGHT` alone is the largest single consumer on
the robot, which is why there is so little headroom.

---

## Build

```bash
cd ~/helios_ws
colcon build --packages-select mapping_localization_pkg --symlink-install
source install/setup.bash
```

Needs both mappers installed:

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-rtabmap-ros ros-jazzy-nav2-map-server
```

---

## Verify

### slam_toolbox

**1. It reached the `active` state.** The most common failure is a node that
started but never activated, which looks like silence rather than an error:

```bash
ros2 lifecycle get /slam_toolbox        # must print: active [3]
```

Anything else — `unconfigured`, `inactive` — means the lifecycle handlers did
not fire. Check the launch output for a configure error, usually a bad parameter.

**2. It is publishing:**

```bash
ros2 topic hz /map                                  # every ~2 s
ros2 run tf2_ros tf2_echo map odom                  # updating, not static
```

**3. Its inputs are arriving:**

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_link            # the EKF must be running
```

A map that never appears is nearly always a missing input, not a mapper fault.

**4. The map is actually good** — the check that matters. In RViz
(`rviz:=true`), Fixed Frame `map`, with a *Map* display on `/map` and a
*LaserScan* on `/scan`:

- Live scan points should land **on** the mapped walls, not beside them. A
  consistent offset means the laser mount offset in the URDF is wrong.
- Drive a loop back to the start. The corridor you return along should overlay
  the one you mapped on the way out. Two parallel copies of the same wall means
  loop closure did not fire — usually too much odometry drift, or
  `loop_search_maximum_distance` too small for the error.
- Straight walls should look straight. Curved walls that should be straight mean
  odometry heading drift the scan matcher could not absorb.

**5. Watch the log** for `Message Filter dropping message ... queue is full`.
Occasional occurrences are tolerable; continuous ones mean scans are being lost
and the map is missing data — raise `scan_queue_size`.

### rtabmap

**1. It is processing frames.** RTAB-Map logs its per-frame timing. What you do
*not* want to see:

```
Could not convert rgb/depth msgs! Aborting rtabmap update...
Did not receive data since 5 seconds!
```

The first means the transform chain could not be resolved at the image's
timestamp — the system is overloaded, or something is publishing transforms
late. The second means the camera, laser and odometry never synchronised: check
all three are publishing.

Count them after a run:

```bash
grep -c "Could not convert" <log>
```

**2. The database was written:**

```bash
ls -lh ~/helios_ws/src/mapping_localization_pkg/rtabmap/maps/
```

A database of a few hundred kilobytes after a real run means almost nothing was
added — go back to step 1.

**3. The pose graph is well connected.** This is the real quality measure and it
is checked offline. Open the database in `rtabmap-databaseViewer` and look at
the graph view: nodes should form a connected chain with loop-closure links
bridging revisited areas. A graph in disconnected fragments means frames were
dropped mid-run — the same overload problem as step 1.

**4. Loop closures happened.** In the database viewer, the loop-closure count
should be non-zero after driving any route that revisits a place. Zero means
either the route never revisited anywhere, or the visual matching is failing —
too few features, or motion blur from driving too fast.

---

## Map storage

| Path | Holds | Tracked in git? |
|---|---|---|
| `slam_toolbox/maps/` | `.pgm` + `.yaml` pairs | Folder only; contents ignored |
| `rtabmap/maps/` | `.db` databases, plus any derived exports | Folder only; contents ignored |

Both are run outputs — large, binary, and regenerable — so `.gitignore` keeps
the folders (via `.gitkeep`) but not their contents. They live in the source tree
rather than `install/` so they survive rebuilds.

---

## Related

- [`perception_pkg/sensor_fusion`](../perception_pkg/sensor_fusion/README.md) —
  provides `/odometry/filtered` and `odom → base_link`
- [`perception_pkg`](../perception_pkg/README.md) — the sensors feeding this
  layer
- [`helios_description`](../helios_description/README.md) — sensor mount offsets,
  which directly affect map quality
