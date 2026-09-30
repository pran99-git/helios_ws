# mapping_localization_pkg

Builds the map. This layer takes the sensor data and the fused odometry from
`perception_pkg` and produces a map of the space the robot has driven through,
plus the correction that keeps the robot's position from drifting.

Two approaches live here, each in its own folder and each runnable on its own.

**Read [the root README](../../README.md) first** for the system overview.

---

## What SLAM is solving

**SLAM**, Simultaneous Localisation And Mapping, is the chicken-and-egg
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

`odom → base_link` drifts smoothly and never jumps, which is what control needs.
`map → odom` absorbs the drift, and jumps when the robot recognises a place it
has been before (**loop closure**). Together they give a position that is both
smooth short-term and correct long-term.

**Only one node may publish `map → odom`.** Running both mappers with both
publishing it means the robot's position fights between two answers.

---

## Which one should I use?

First decide whether you are **building** a map or **reusing** one. If you
already have a `.pgm`/`.yaml` and only want to know where the rover is inside
it, you do not want a mapper at all. Jump to
[Localizing in an existing map](#localizing-in-an-existing-map).

For building:

| | `slam_toolbox` | `rtabmap` |
|---|---|---|
| Sensors | Laser only | Camera (RGB-D) + laser |
| Output | 2D occupancy grid, a floor plan | 3D map with colour and depth |
| CPU/GPU cost | Light | Heavy |
| Result format | `.pgm` image + `.yaml` | A single `.db` database |
| Standard nav stacks accept it | Yes, directly | Needs a derived grid |
| Publishes `map → odom` | Yes, always | Only if you ask (`publish_tf_map:=true`) |

**Start with `slam_toolbox`.** It is cheaper, more reliable, produces the format
navigation tools expect, and a 2D floor plan is enough for a rover driving on
flat ground.

**Use `rtabmap`** when you need the third dimension: obstacles the laser's flat
slice misses, or a visual record of the space.

**Running both at once** is supported and is how RTAB-Map is normally evaluated:
`rtabmap.launch.py` defaults to `publish_tf_map:=false`, so slam_toolbox stays
the sole transform authority while RTAB-Map builds its database alongside. They
share inputs and neither subscribes to the other's output.

All three (both mappers, and AMCL localization) need the sensing layer running
first:

```bash
ros2 launch sensor_fusion bringup.launch.py
```

And only one of them may run at a time, since all three publish `map → odom`.
The one exception is the `slam_toolbox` + `rtabmap` pairing described above,
which works precisely because RTAB-Map defaults to not publishing it.

---

## Files

```
slam_toolbox/
  launch/slam_toolbox.launch.py   Starts slam_toolbox, drives its lifecycle.
  config/slam_toolbox.yaml        All tuning: solver, scan matcher, loop closure.
  rviz/slam.rviz                  RViz layout for watching the map build.
  scripts/save_slam.sh            Saves a run in both formats: .pgm + .yaml
                                  and .posegraph + .data, under one name.
  maps/                           Saved 2D maps. Contents gitignored.
rtabmap/
  launch/rtabmap.launch.py        Wraps the upstream launch file with our settings.
  rviz/rtabmap.rviz               RViz layout for watching the 3D map build.
  scripts/save_rtabmap.sh         Exports a finished run to .pgm + .yaml and
                                  to _cloud.ply. Neither is automatic.
  maps/                           Run databases. Contents gitignored.
localization/
  launch/amcl_localization.launch.py  AMCL + auto-recovery over a saved map.
  config/amcl.yaml                Particle filter, motion model, and the
                                  recovery node's thresholds and limits.
  amcl_recovery.py                The maths. Costmap ray casting, heading
                                  choice, convergence test. No ROS.
  amcl_recovery_node.py           Thin ROS adapter around it.
CMakeLists.txt                    Installs launch/, config/, rviz/ and the two
                                  recovery files; not maps/, not scripts/.
package.xml                       Metadata and dependencies.
```

The three folders answer different questions:

| Folder | Question | Writes a map? | Owns `map → odom`? |
|---|---|---|---|
| `slam_toolbox/` | "What does this place look like?" (2D) | yes | yes |
| `rtabmap/` | "What does this place look like?" (3D) | yes | only if `publish_tf_map:=true` |
| `localization/` | "Where am I in a map I already have?" | no, read-only | yes |

Only **one** of them may run at a time, because all three publish `map → odom`
and would fight over it.

**Almost no source code lives here.** Both mappers are third-party ROS packages,
and what this package holds is the configuration and launch wiring that makes
them work with *this* robot. That is deliberate: the algorithms are well-tested
upstream, and the value is in the integration.

The one exception is `localization/amcl_recovery*.py`, which closes a loop no
upstream node closes: local costmap in, AMCL covariance out, Nav2 behaviors in
between.

### `CMakeLists.txt`

Installs each of the three folders' `launch/`, `config/` and `rviz/`
subdirectories explicitly, one by one, rather than installing a folder
wholesale. `scripts/` is deliberately left out: both save scripts locate
`maps/` relative to their own path and are meant to run straight from the
source tree.

The reason is `maps/`. RTAB-Map databases run to tens of gigabytes, and a plain
(non-symlink) build would copy every one of them into `install/` on every build.
The obvious fix, installing the whole folder with `PATTERN maps EXCLUDE`, is
**silently ignored** under `colcon build --symlink-install`, which symlinks whole
directories and never evaluates the exclusion. Listing subdirectories explicitly
is the only approach that works under both build modes.

Both launch files therefore read and write `maps/` in the **source tree**, not
in `install/`: run outputs must survive a rebuild.

---

## slam_toolbox: 2D laser SLAM

Consumes `/scan` and the EKF's `odom → base_link`. Produces `/map` (a
`nav_msgs/OccupancyGrid`) and `map → odom`.

### Run

```bash
ros2 launch mapping_localization_pkg slam_toolbox.launch.py
ros2 launch mapping_localization_pkg slam_toolbox.launch.py rviz:=true
```

### `launch/slam_toolbox.launch.py`

Starts `async_slam_toolbox_node` and drives it through its lifecycle.

A **lifecycle node** is a ROS node with explicit states. It starts
*unconfigured* and does nothing until told to *configure* and then *activate*.
On this build slam_toolbox does not auto-activate, so the launch file does it
with two event handlers: configure when the process starts, activate once
configuring completes. The result is a node that comes up publishing, with no
manual `ros2 lifecycle set` needed.

If `/map` is silent, this is the first thing to check; see Verify below.

`rviz:=true` also opens RViz with the layout in `rviz/slam.rviz`.

### `config/slam_toolbox.yaml`

The values that matter most day to day:

| Parameter | Value | Meaning |
|---|---|---|
| `mode` | `mapping` | Build a new map. `localization` reuses a saved one |
| `resolution` | 0.05 | Map cell size, metres. 5 cm per pixel |
| `max_laser_range` | 10.0 | Matches the UST-10LX's usable range |
| `map_update_interval` | 2.0 | Seconds between published map updates |
| `minimum_travel_distance` | 0.05 | Metres of motion before a new scan is processed |
| `minimum_travel_heading` | 0.2 | Radians of rotation before a node is added |
| `minimum_time_interval` | 0.2 | Seconds, a floor on the gap between processed scans |
| `transform_timeout` | 0.5 | How long to wait for a transform |
| `scan_queue_size` | 20 | Scans buffered while waiting for transforms |
| `do_loop_closing` | true | Enable drift correction on revisit |
| `loop_search_maximum_distance` | 3.0 | Metres, how far away to look for a match |

Three notes.

**The `minimum_travel_*` values decide map density,** and one of them has a trap
in it. Scans are only processed after the robot has moved this far, which
prevents thousands of near-identical scans piling up while it sits still.

`minimum_travel_distance` gates on **translation only**. At the upstream default
of 0.2 m, a rover rotating in place moved zero metres, processed zero scans, and
left `map -> odom` bit-frozen for the whole turn. Nothing errors: `/map` simply
stops updating and any drift measured across a spin reads 0.00 because it was
never measured. It is 0.05 here for that reason. `minimum_time_interval: 0.2`
is what keeps the cost bounded now that the distance gate is loose.

**`scan_queue_size: 20` was a fix, not a default.** The symptom was
`Message Filter dropping message: frame 'laser' ... queue is full` in the log.
That message is a *capacity* eviction, governed by queue size, a different
mechanism from `transform_timeout`, which governs a scan timing out while
waiting. At ~40 Hz the default queue drained in well under 200 ms and overflowed
long before any timeout could apply. If you see that message return, raise this,
not the timeout.

**The solver and scan-matcher blocks** (Ceres settings, correlation search space,
penalties) are largely upstream defaults. They rarely need touching; if the map
is bad, the cause is almost always odometry quality or sensor mounting, not
these.

### Saving a map

One script, both formats, all four files, under one shared name:

```bash
S=~/helios_ws/src/mapping_localization_pkg/slam_toolbox/scripts
$S/save_slam.sh lab_corridor
```

That writes `.pgm` + `.yaml` (for AMCL / nav2) **and** `.posegraph` + `.data`
(for slam_toolbox) into `slam_toolbox/maps/` as `slam_toolbox_<name>.*`. See
[Map storage](#two-different-slam_toolbox-artifacts-and-why-you-want-both) for
why neither pair substitutes for the other. With no argument the name is a
timestamp, matching how RTAB-Map names its runs.

`--map-only` / `--graph-only` narrow it to one pair if you deliberately want
just one. The script refuses to overwrite any file it is about to write, and
checks all of them up front so a collision never leaves a half-saved run.

### Reusing a map with slam_toolbox

```bash
ros2 launch mapping_localization_pkg slam_toolbox.launch.py \
    localization:=true \
    map_file_name:=<path>/slam_toolbox_lab_corridor \
    map_start_pose:="[0.0, 0.0, 0.0]"
```

`map_file_name` takes the prefix **without** the `.posegraph`/`.data`
extension, exactly as `save_slam.sh` prints it. Localization is a
different executable (`localization_slam_toolbox_node`), not just a parameter,
and it will not start without a starting pose. Pass `map_start_pose:="[x, y,
yaw]"` or `map_start_at_dock:=true`.

To localize against the `.pgm` instead, use AMCL; see
[Localizing in an existing map](#localizing-in-an-existing-map).

Two constraints on the save scripts:

- **Run it while slam_toolbox is still up.** It saves a live topic, not
  something read back from disk afterwards. Close the mapper first and the map
  is gone.
- **Run it from the source path shown above,** not via `ros2 run`. It locates
  `maps/` relative to its own file, which only resolves correctly in the source
  tree.

---

## rtabmap: 3D RGB-D SLAM

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
rather than all runs overwriting a single `~/.ros/rtabmap.db`. Override either
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
| `rtabmap_viz` | `false` | RTAB-Map's own GUI. **See the warning below** |
| `rviz` | `false` | RViz instead |
| `wait_for_transform` | 0.5 | Seconds to wait for a transform before giving up |
| `sync_queue_size` | 30 | Buffer for time-synchronising camera + laser + odometry |

The internal algorithm settings passed via `args`:

| Parameter | Value | What it does |
|---|---|---|
| `Optimizer/Strategy` | 2 | GTSAM for pose-graph optimisation |
| `Reg/Strategy` | 2 | VisIcp: register on vision *and* laser, not vision alone |
| `RGBD/NeighborLinkRefining` | true | Refine sequential edges with ICP instead of copying the raw odometry delta |
| `Optimizer/Robust` | false | Vertigo switchable constraints OFF |
| `RGBD/OptimizeMaxError` | 0 | Hard loop-closure rejection OFF |
| `Optimizer/GravitySigma` | 0.3 | Unary roll/pitch constraint per node from measured gravity |
| `Mem/SaveDepth16Format` | true | Store depth as 16 mm uint16 + RVL instead of falling back to PNG |

### Why these parameter values

Four of these look wrong until you see what was measured. The numbers below all
come from databases in `rtabmap/maps/`, not from reasoning.

**`Optimizer/Robust false` + `RGBD/OptimizeMaxError 0` disable every
graph-level outlier rejection, deliberately.** Both were tried ON and both
threw away good data. Measured on `rtabmap_20260825_150118.db`, a 16.8 m loop
returning to its start:

| Source | Says the rover ended | |
|---|---|---|
| Odometry | 37.99° rotated from node 1 | |
| 44 visual loop closures | ~1.1° | |
| Optimizer, with rejection on | 33.57° | applied 4° of the 36° demanded |
| LiDAR scan alignment (independent) | **+2.0°, 5 cm** | residual 0.0097 m |

Forcing 37.99° raises that residual to 0.2233 m, 23x worse. The vision was
right and the graph discarded it.

The optimizer was not malfunctioning. `RGBD/NeighborLinkRefining` gives every
odometry edge an ICP covariance derived from two nearly identical consecutive
scans, median `sigma_yaw` 0.10°. Bending 181 of those to absorb 36° costs
roughly `181 * (0.199/0.104)^2 ~= 660`, while switching off 44 loop links costs
Vertigo far less. It made a rational choice from dishonest inputs. RTAB-Map
already inflates proximity-link covariance for this exact reason
(`RGBD/ProximityMergedScanCovFactor` = 100); there is no equivalent knob for
neighbour links.

With both mechanisms off, the optimizer runs plain weighted least squares and
must distribute the error: 36° over 181 edges is 0.2° each. Outlier protection
is not lost, it moves entirely to the front end, where a closure only survives
PnP RANSAC with `Vis/MinInliers >= 20`. The `Not enough inliers 0/20` log lines
are that filter working. **If a false closure ever corrupts a map, raise
`Vis/MinInliers` before re-enabling either mechanism.**

**`Optimizer/GravitySigma` is inert without an IMU subscription.** It needs
`imu_topic` here *and* `sensors.publish_imu_tf` on the ZED side in
`sensor_fusion/bringup.launch.py`. Without both, the graph holds zero links of
type Gravity. It matters more than it looks: `ekf.yaml` sets `two_d_mode: true`,
which pins roll and pitch to zero as an *assumption*, and nothing else in the
graph can contradict it (neighbour links come from that flat odometry, and ICP
on a single-plane Hokuyo sweep observes x, y and yaw only). Meanwhile the visual
loop closures are full 6-DoF and keep asserting real roll and pitch. The
optimizer splits the difference and the clouds fan out.

**`Mem/SaveDepth16Format true`, paired with the camera's `openni_depth_mode`.**
The ZED's default depth topic is 32-bit float, which `.rvl` cannot carry, so
every frame silently fell back to PNG. That is why `Compressing_data` measured
37% of total frame time and a 448-node run wrote 1074 MB (2.4 MB/node). Setting
this alone fixed the storage (2.40 -> 0.79 MB/node) but left RTAB-Map converting
every incoming frame and logging *"depth type detected is 32FC1, use 16UC1 depth
format to avoid this conversion"*. `zed_custom_tuning` now sets
`depth.openni_depth_mode: true` so the wrapper publishes 16UC1 millimetres
directly and no conversion happens. Both settings must stay together. The cost
is 1 mm quantisation and a 65.535 m ceiling, meaningless on a 0.120 m stereo
baseline indoors where depth degrades past ~15 m.

**`rgbd_sync` is what fixes `Not enough inliers 0/20 (matches=94)`.** Zero
inliers from ~90 matches means RANSAC found no camera pose consistent with any
subset, so lowering `Vis/MinInliers` changes nothing. Appearance matching
succeeds and only the geometry fails, because the 3D points behind the
keypoints are wrong. Subscribing to both topics at once showed 88.1% of depth
frames share a bit-identical stamp with an RGB frame, but **11.9% have no RGB
partner at all**, and `ApproximateTime` pairs those with the nearest survivor
instead of dropping the set. The error is `yaw_rate x skew`, which is why
closures fail in bursts while turning: at 0.82 rad/s a ~100 ms mispair is 4.7°
of camera rotation, about 33 cm at 4 m, against a `Vis/PnPReprojError` budget of
2 pixels.

`approx_rgbd_sync` stays at its default `true`. Exact sync would be stronger,
but `camera_info` was measured publishing at a different rate than the images
(29.3 vs 10.0 Hz), so requiring all three to match exactly risks starving the
pipeline.

### Settings that were tried and rejected

Do not re-add these without re-measuring. The baseline for any comparison is
`rtabmap_20260827_134811.db`: 47.7 m at 0.131 m/s median, 1.27 LocalSpaceClosure
links per node, 0.0159 m / 0.23° optimizer error.

**`Grid/Sensor 2`** (build the occupancy grid from camera depth instead of the
laser). Disproven by reprocessing the good database with `rtabmap-reprocess`,
so the poses are identical and only the grid source changes:

| | `Sensor=0` | `Sensor=2` | `Sensor=2` + RayTracing |
|---|---|---|---|
| `ground_cells` | 0.00 MB | 2.81 MB | 0.00 MB |
| `obstacle_cells` | **3.43 MB** | 1.65 MB | 1.65 MB |
| `empty_cells` | **4.91 MB** | 0.00 MB | 2.99 MB |

Obstacles more than halve and free space drops ~40%, because a ~110° camera
cone replaces a 270° laser sweep. It becomes worth revisiting only when
navigation must see obstacles below the **0.176 m** laser plane (`base_link`
sits 0.076 m up at axle height, and `laser_mount_joint` adds 0.100 m). Even
then, `Grid/RangeMax` must be passed explicitly: its auto-set to unlimited is
conditional on `Grid/Sensor` being 0, so grid range silently drops 10 m to 5 m.

**`Icp/PointToPlane false`.** The theory was that a single-plane 2D sweep cannot
give well-conditioned normals. Wrong: `Icp/Strategy=1` (libpointmatcher) handles
the 2D case, and `Icp/PointToPlaneLowComplexityStrategy` is the safeguard for
degenerate geometry, which only applies when point-to-plane is **on**. Turning
it off made the `Variance is unknown!` warning near-constant and cost ~18% of
the lidar proximity closures (0.164 to 0.135 per node).

**A "drive faster" batch** (`Rtabmap/DetectionRate 2`,
`Icp/MaxCorrespondenceDistance 0.15`, `Vis/PnPReprojError 3`). Measured on
`rtabmap_20260828_120512.db`:

| | 0827 baseline | with the batch |
|---|---|---|
| Optimization error, median | 5.72 | 21.48 |
| Optimizer max angular error | 0.23° | 0.55° |
| Database size per metre driven | 7.7 MB/m | 21.8 MB/m |

Read that run with care: 76% of it was stationary (416 of 544 frames under
0.02 m/s), so the loop-closure count is inflated by a parked robot connecting
to itself, with a median endpoint separation of 1 mm. The optimizer residual and
the grid loss are real regardless.

### Saving a run

**The run is already saved.** Ctrl+C writes the whole SLAM session into the
`.db`, and that file holds both maps: the pose graph, every keyframe's RGB-D,
the laser scans, and the occupancy grid. Kill it with **Ctrl+C, never
`kill -9`**, because a SIGKILL skips the save and the run is gone.

`save_rtabmap.sh` exists only because a `.db` is not a format AMCL, nav2,
CloudCompare or Meshlab can open. It extracts the two standalone formats:

| Output | What reads it | When to run it |
|---|---|---|
| `.pgm` + `.yaml` | AMCL, nav2 costmaps | **Before** Ctrl+C. It is a live capture of `/rtabmap/map`; there is no offline exporter. |
| `_cloud.ply` | CloudCompare, Meshlab | **After** Ctrl+C, exported from the `.db`. |

The two halves have opposite timing requirements, so the script checks what is
running and tells you which half it can do:

```bash
S=~/helios_ws/src/mapping_localization_pkg/rtabmap/scripts
$S/save_rtabmap.sh lab_run --map-only      # 2D grid, rtabmap still RUNNING
$S/save_rtabmap.sh lab_run --cloud-only    # 3D cloud, rtabmap STOPPED
```

Run it twice with the same name to get both. `--db <path>` picks a specific
database instead of the newest in `maps/`. Output lands in `rtabmap/maps/` as
`rtabmap_<name>.{pgm,yaml}` and `rtabmap_<name>_cloud.ply`.

> **Check the export filter settings before trusting a cloud.** The `.ply` export
> reads its parameters from `~/.ros/rtabmapGUI.ini`, not from anything in this
> repo, and that file ships with every filter off (`filtering=false`,
> `filtering_radius=0`, `subtract=false`, `bilateral=false`). Stereo produces
> flying pixels at every depth discontinuity, so an unfiltered export carries all
> of them: measured, the radius-outlier filter took stray points from 2.23% of
> the cloud down to 0.11%, a 20x difference. Range is not the lever here, and was
> tested: 4 m gave 0.11%, 3 m 0.09%, 2.5 m 0.11%. The filter is.

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

## Localizing in an existing map

The other two folders build maps. This one reuses one: given a `.pgm`/`.yaml`
you saved earlier, work out where the rover is inside it. The map is
**read-only**; AMCL never writes to it.

```bash
# Terminal 1: sensors + fused odometry
ros2 launch sensor_fusion bringup.launch.py

# Terminal 2: localization
ros2 launch mapping_localization_pkg amcl_localization.launch.py \
    map:=$PWD/src/mapping_localization_pkg/slam_toolbox/maps/slam_toolbox_20260728_175429.yaml \
    rviz:=true
```

This starts three nodes. `nav2_bringup` is **not required**: the launch file
wires them up itself, so localization works whether or not it is installed.

| Node | Job |
|---|---|
| `map_server` | Serves the saved `.pgm` on `/map` |
| `amcl` | Particle filter; publishes `map → odom` and `/particlecloud` |
| `lifecycle_manager` | Drives both to `active` (neither self-activates) |

### Giving it a starting pose

On launch the rover does **not** know where it is. Supply a pose:

- **RViz "2D Pose Estimate"**: click the rover's position, drag in the
  direction it faces. Works from anywhere in the map; this is the normal path.
- **`ros2 service call /reinitialize_global_localization std_srvs/srv/Empty`**:
  scatter particles across the whole map and let it work the pose out
  unaided. Slower, and ambiguous in self-similar spaces like corridors.

Heading is far less forgiving than position. The default yaw spread is only
about ±0.5 rad, so a position off by half a metre recovers, while a heading off
by 90° usually does not.

`set_initial_pose: true` in `config/amcl.yaml` is **not** a claim that the rover
starts at the origin. It exists so `map -> odom` is published from the moment
AMCL activates, because without that transform the global costmap blocks,
`lifecycle_manager` aborts the Nav2 bringup, and it never retries, leaving
`planner_server` inactive forever.

`nav2_amcl` has no `initial_cov_*` parameter, so that seed comes with a
near-zero covariance: AMCL reports a tight cloud it has no evidence for.
`amcl_recovery`'s `scatter_on_startup` immediately replaces it with honest
uncertainty, which keeps the transform published while making the covariance
real. Set `scatter_on_startup: false` to trust the pose saved from the previous
session instead.

### Then drive

AMCL only runs a filter update after `update_min_d` (0.10 m) or `update_min_a`
(0.10 rad) of motion. **A stationary rover never converges.** Drive a few
metres past distinctive geometry and watch `/particlecloud` tighten in RViz.

Teleop is the normal way to do this by hand. Pushing the rover works just as
well: AMCL reads odometry, not commands.

**You normally do not have to.** `amcl_recovery` starts with AMCL and does all
of the above by itself, including the very first convergence at startup. See
[It re-localizes itself](#it-re-localizes-itself) below.

### Convergence tuning, and how to measure it

Recovery worked but converged slowly, so five parameters were retuned on
**2026-09-29**. All are in `config/amcl.yaml` with the old value recorded in a
comment beside each, so any one can be reverted alone.

| Parameter | Was | Now | Why |
|---|---|---|---|
| `z_hit` / `z_rand` | 0.5 / 0.5 | **0.8 / 0.2** | Nav2 defaults, never tuned. Half the model being uniform noise flattened the likelihood field, so wrong particles kept too much weight to be resampled away. **Free, and the biggest single win** |
| `update_min_d` / `_a` | 0.20 | **0.10** | A 2*pi survey bought only ~31 updates and a 1.5 m leg only ~7. Halving doubles the updates for the same driving |
| `alpha1-4` / `alpha5` | 0.2 / 0.4 | **0.1 / 0.2** | Every prediction blurred the cloud faster than the scan could re-tighten it. The 2:1 strafe ratio is preserved; it came from measured off-axis drift |
| `sigma_hit` | 0.2 | **0.15** | A 20 cm std let a particle 20 cm off still score well |
| `survey_yaw` | 2*pi | **pi** | Rotating does not move the viewpoint, so a survey constrains heading but barely helps position |
| `leg_speed` | 0.2 | **0.3** | Translation to new viewpoints is what disambiguates position |

**Revert order if something regresses.** `update_min_*` first: it is the only
one that costs CPU, on a Jetson that has already logged `ekf_filter_node:
Failed to meet update rate`. Then the alphas, whose symptom is a filter that
grows confident and then jumps when scan matching disagrees. `z_rand` last, and
only toward 0.3 if a cluttered room full of unmapped obstacles makes it diverge.

#### Measuring, instead of guessing

Five knobs moved at once. Only a measured number can say which earned its
place, so there is a tool:

```bash
# start it BEFORE AMCL, then drive or let recovery drive, then Ctrl+C
python3 src/mapping_localization_pkg/localization/scripts/amcl_convergence.py
python3 .../amcl_convergence.py --csv run_a.csv    # to plot two runs together
```

It watches `/amcl_pose` and reports **time to converge**, using the same
thresholds `amcl_recovery` uses to declare success, so the number matches the
moment recovery would stop driving.

```
    2.3s  update   10  std x= 0.15 y= 0.15 yaw= 0.06   <<< CONVERGED
--- summary ---
updates seen        : 21
TIME TO CONVERGE    : 2.3 s
```

Change one parameter, re-run, compare. If no `/amcl_pose` ever arrives, the
rover has not moved far enough to trigger an update.

### It re-localizes itself

`amcl_recovery` starts with AMCL and needs no trigger. It watches AMCL's
reported covariance and, when the rover has been lost for long enough, drives it
until the particle cloud tightens again. No RViz click, no teleop, no service
call.

It starts by discarding the pose it was seeded with, then watches two
independent detectors:

| Trigger | Fires on | Catches |
|---|---|---|
| **Cold start** | first cycle after AMCL and Nav2 are both up | The seeded pose, which is invented |
| **Covariance** | position std > 0.60 m or yaw std > 0.40 rad, held 5 s | Gradual divergence, and startup with no pose |
| **Lift** | wheels still while the camera reports motion, held 1 s | Being picked up and carried |

```
   cold start (seeded pose is not evidence)
   or covariance bad for lost_confirm_time (5 s)
   or no /amcl_pose at all for pose_timeout (30 s)
   or carried, then set down
            |
            v
   +---> survey: spin 360 deg in place
   |        |
   |        v
   |    converged?  --yes-->  back to monitoring
   |        | no
   |        v
   |    rank 36 headings by clearance in the LOCAL costmap
   |        |
   |        v
   |    turn onto the best one, drive one leg
   |        |
   +--------+   until max_legs, then escalate
                        |
                        v
              scatter globally, explore again
                        |
                        v
              still lost: wait out the cooldown, retry
```

### Why two detectors

The covariance test needs AMCL to *notice* it is lost. That needs a filter
update, which AMCL only runs after 0.20 m or 0.20 rad of odometry motion. Pick
the rover up and the wheels report nothing, so whether AMCL ever notices comes
down to the ZED tracking the carry.

The EKF helps here by accident. Carrying the rover, the wheels insist on zero
while the camera reports real motion, and inverse-variance weighting
(`0.01` against `0.02`) lands the fused estimate at roughly **a third** of true
carrying speed. Carrying it 3 m still registers ~1 m, well past the 0.20 m
threshold, so AMCL usually does update and its covariance does blow up.

**Usually** is the problem. Cover the lens, carry it through a dark stretch, or
move it fast enough to blur, and the ZED loses tracking, no update happens, and
the last `/amcl_pose` still reads converged. The kidnap is invisible.

The lift detector does not depend on AMCL at all. Wheels still + camera moving
is a signature nothing in normal operation produces:

| Situation | wheels | camera | Lift? |
|---|---|---|---|
| Parked | 0.000 | 0.00002 | no |
| Driving | 0.300 | 0.310 | no |
| Mecanum roller slip | 0.300 | 0.291 | no |
| Wheels spinning on a slick floor | 0.300 | 0.000 | no |
| **Carried** | **0.005** | **0.600** | **yes** |

Roller slip cannot reach it: slip is a scale error on a *turning* wheel
(measured 0.9686 of truth), not a still wheel under real motion.

It triggers on **touchdown**, not on the lift. Recovering while still in the air
would spin the wheels in someone's hands.

Still not caught: lifting the rover with the camera covered *and* the wheels
still. Nothing on the robot sees that; it would need the IMU.

**Exploring comes before scattering, except on a cold start.** AMCL's own
`recovery_alpha_slow` / `recovery_alpha_fast` injection is the designed
mechanism for recovering a lost filter, and it only acts when the robot moves.
A global scatter throws away whatever partial information AMCL still holds, so
for a rover that was localized and then got lost it is the second resort.

At startup there is nothing to preserve: the pose is a seed, not a measurement.
That case scatters first, then explores.

**Why the local costmap is safe to steer by while lost.** It is built in the
`odom` frame from the live laser, and its layers never consult `/map`:

| | Global costmap | Local costmap |
|---|---|---|
| Frame | `map` | `odom` |
| Layers | static + obstacle + inflation | obstacle + inflation |
| Reads `/map` | yes | **no** |
| Valid while AMCL is wrong | no | **yes** |

So the obstacle picture stays correct no matter how wrong the pose estimate is.
Steering by the global costmap would drive into walls it believes are elsewhere.

**Collision checking is Nav2's, not ours.** The node drives through the `spin`
and `drive_on_heading` actions on the behavior server, which already simulate
`simulate_ahead_time` (2.0 s) of motion against the local costmap and abort with
`COLLISION_AHEAD`. No new `/cmd_vel` publisher is introduced.

> **It needs Nav2 running.** Both the local costmap and those behaviors live
> there. Without Nav2 the node stays passive and logs why, so AMCL alone still
> works exactly as it did, with the RViz pose estimate.

| Parameter | Value | What it controls |
|---|---|---|
| `enabled` | `true` | `false` disables recovery without unwiring the node |
| `lost_position_std` | 0.60 m | Above this counts as lost |
| `lost_yaw_std` | 0.40 rad | Same for heading |
| `lost_confirm_time` | 5.0 s | How long it must stay lost before recovery starts |
| `pose_timeout` | 30.0 s | Silence from `/amcl_pose` that also counts as lost |
| `max_position_std` | 0.25 m | Tighter exit threshold, so it cannot oscillate |
| `max_yaw_std` | 0.15 rad | Same |
| `cooldown` | 30.0 s | Minimum gap between attempts |
| `scatter_on_startup` | `true` | Discard the seeded pose once, at startup |
| `lift_detection_enabled` | `true` | The second detector |
| `lift_max_wheel_speed` | 0.02 m/s | Wheels at or below this count as stationary |
| `lift_min_visual_speed` | 0.15 m/s | Camera must claim at least this much |
| `lift_confirm_time` | 1.0 s | Disagreement held this long counts as a lift |
| `leg_distance` | 1.5 m | Distance driven per leg |
| `leg_speed` | 0.2 m/s | Half the `roboclaw.yaml` ceiling |
| `safety_margin` | 0.45 m | Clearance needed beyond `leg_distance`. Matches `inflation_radius` |
| `retrace_penalty` | 1.5 m | Stops it ping-ponging along one corridor |

Entering recovery uses the loose `lost_*` thresholds and leaving it uses the
tight `max_*` ones. That hysteresis is what stops a pose hovering near one value
from flapping the node in and out.

**Rotation counts.** AMCL gates its filter updates on translation **or**
rotation (`update_min_d` 0.20 m, `update_min_a` 0.20 rad), so the survey spin
does drive updates. This is the opposite of slam_toolbox, which gates on
translation only and processes nothing during an in-place turn.

Translation is still what collapses a scattered cloud: different particle
hypotheses predict different scan changes as the rover moves, and only motion
across the map distinguishes them. The spin mostly resolves heading and fills
the 90 deg the 270 deg Hokuyo cannot see at rest.

> **The rover drives itself, with no deadman.** Two manual overrides exist:
> `ros2 service call /amcl_recovery/abort std_srvs/srv/Trigger` cancels a
> running recovery, and `/amcl_recovery/relocalize` forces one to start.
> `recovery:=false` on the launch file, or `enabled: false` in `amcl.yaml`,
> turns it off entirely.

### Why `OmniMotionModel`

`config/amcl.yaml` sets `robot_model_type: nav2_amcl::OmniMotionModel`, not the
more common `DifferentialMotionModel`. This rover is mecanum: it can translate
sideways without rotating, and the differential model assumes that is
impossible. Under that model a strafe looks like sensor noise, and the particle
cloud gets dragged badly.

`alpha5` (lateral odometry noise, omni-only) is deliberately the largest of the
alpha terms, for the same reason the `y` covariance is raised in
`wheel_odometry.yaml`: strafe is the least trustworthy thing this drivetrain
reports, so the filter is told to lean on scan matching to correct it.

---

## Build

```bash
cd ~/helios_ws
colcon build --packages-select mapping_localization_pkg --symlink-install
source install/setup.bash
```

`amcl_recovery_node` and its `amcl_recovery.py` both install into
`lib/mapping_localization_pkg/`. Python puts a script's own directory first on
`sys.path`, so the node imports its sibling with no package machinery.

Needs both mappers plus the nav2 localization pieces installed:

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-rtabmap-ros \
                 ros-jazzy-nav2-map-server ros-jazzy-nav2-amcl \
                 ros-jazzy-nav2-lifecycle-manager
```

`ros-jazzy-nav2-bringup` is deliberately **not** required.
`amcl_localization.launch.py` starts `map_server`, `amcl` and
`lifecycle_manager` itself, and `navigation_pkg` does the same for the
navigation servers. It happens to be installed on this machine as a dependency
of other nav2 packages; nothing here launches it.

---

## Verify

### slam_toolbox

**1. It reached the `active` state.** The most common failure is a node that
started but never activated, which looks like silence rather than an error:

```bash
ros2 lifecycle get /slam_toolbox        # must print: active [3]
```

Anything else (`unconfigured`, `inactive`) means the lifecycle handlers did
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

**4. The map is actually good.** The check that matters. In RViz
(`rviz:=true`), Fixed Frame `map`, with a *Map* display on `/map` and a
*LaserScan* on `/scan`:

- Live scan points should land **on** the mapped walls, not beside them. A
  consistent offset means the laser mount offset in the URDF is wrong.
- Drive a loop back to the start. The corridor you return along should overlay
  the one you mapped on the way out. Two parallel copies of the same wall means
  loop closure did not fire, usually from too much odometry drift, or
  `loop_search_maximum_distance` too small for the error.
- Straight walls should look straight. Curved walls that should be straight mean
  odometry heading drift the scan matcher could not absorb.

**5. Watch the log** for `Message Filter dropping message ... queue is full`.
Occasional occurrences are tolerable; continuous ones mean scans are being lost
and the map is missing data, raise `scan_queue_size`.

### rtabmap

**1. It is processing frames.** RTAB-Map logs its per-frame timing. What you do
*not* want to see:

```
Could not convert rgb/depth msgs! Aborting rtabmap update...
Did not receive data since 5 seconds!
```

The first means the transform chain could not be resolved at the image's
timestamp. The system is overloaded, or something is publishing transforms
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
added, go back to step 1.

**3. The pose graph is well connected.** This is the real quality measure and it
is checked offline. Open the database in `rtabmap-databaseViewer` and look at
the graph view: nodes should form a connected chain with loop-closure links
bridging revisited areas. A graph in disconnected fragments means frames were
dropped mid-run, the same overload problem as step 1.

**4. Loop closures happened.** In the database viewer, the loop-closure count
should be non-zero after driving any route that revisits a place. Zero means
either the route never revisited anywhere, or the visual matching is failing:
too few features, or motion blur from driving too fast.

---

## Map storage

| Path | Holds | Tracked in git? |
|---|---|---|
| `slam_toolbox/maps/` | `.pgm` + `.yaml` pairs, and `.posegraph` + `.data` pairs | Folder only; contents ignored |
| `rtabmap/maps/` | `.db` databases, plus any derived exports | Folder only; contents ignored |

### Two different slam_toolbox artifacts, and why you want both

A mapping run can be saved in two formats, and **neither can be regenerated
from the other**, which is why `save_slam.sh` writes both by default.

| Output | What it is | Who can read it |
|---|---|---|
| `.pgm` + `.yaml` | The rendered occupancy grid, an image plus its metadata | AMCL, nav2 costmaps, any map viewer |
| `.posegraph` + `.data` | The full SLAM session: nodes, scans, constraints, loop closures | slam_toolbox only |

The `.pgm` is lossy: it is the *result* of the pose graph, with no graph behind
it, so slam_toolbox cannot resume, extend, or localize from it. The pose graph
is the only format you can continue mapping from, but nav2 cannot read it.

Saving both in one call also keeps the two halves on the **same name**. Running
two separate scripts a minute apart left you with an occupancy grid and a pose
graph timestamped differently, with nothing recording that they came from the
same run.

Both halves are **live captures**, a service call and a topic subscription,
so this must be run while `slam_toolbox.launch.py` is still up:

```bash
src/mapping_localization_pkg/slam_toolbox/scripts/save_slam.sh lab_corridor
```

> Serializing before any scan has been processed produces an *empty* pose graph
> that reports success and then segfaults `localization_slam_toolbox_node` on
> load. The script checks the output size and refuses to leave one behind. The
> pose graph is written first for exactly this reason: an empty graph means an
> empty grid too, so it bails out before writing a misleading `.pgm`.

Both are run outputs (large, binary, regenerable) so `.gitignore` keeps
the folders (via `.gitkeep`) but not their contents. They live in the source tree
rather than `install/` so they survive rebuilds.

---

## Related

- [`perception_pkg/sensor_fusion`](../perception_pkg/sensor_fusion/README.md):
  provides `/odometry/filtered` and `odom → base_link`
- [`perception_pkg`](../perception_pkg/README.md): the sensors feeding this
  layer
- [`helios_description`](../helios_description/README.md): sensor mount offsets,
  which directly affect map quality
- [`navigation_pkg`](../navigation_pkg/README.md): consumes `/map` and the
  `map -> odom` this layer publishes
