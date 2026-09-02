# helios_description

The robot's physical model: how big Helios is, and where every wheel and sensor
sits relative to the chassis.

Nothing here moves the robot or reads a sensor. It exists so the rest of the
stack can answer one question: *given a laser reading 2 m ahead of the scanner,
where is that point relative to the robot?* Answering it requires knowing
exactly where the scanner is bolted on, which is what this package describes.

**Read [the root README](../../README.md) first** for the system overview.

---

## Why a robot needs a model

Sensors report data in their own coordinate frame. The laser says "obstacle at
2 m, bearing 30°" in the `laser` frame; the camera reports depth in its own
frame. To combine them, something must know the fixed offsets between those
frames and the robot's centre.

That description is a **URDF** (Unified Robot Description Format), an XML file
listing **links** (rigid parts: chassis, wheels, sensors) and **joints** (how
they connect). A node called `robot_state_publisher` reads it and continuously
broadcasts every `base_link → <part>` transform, so any node can convert between
frames.

We write it as **xacro** rather than raw URDF. Xacro is URDF plus variables,
maths, and macros, so the wheel radius is defined once as a property instead of
repeated in eight places, and the four wheels come from one macro called four
times. It expands to plain URDF at launch time.

### Frame conventions

Two ROS standards apply, and following them is why off-the-shelf ROS tools work
with this robot unmodified.

| Standard | What it fixes | This package's part |
|---|---|---|
| **REP-103** | Axes x forward, y left, z up; metres and radians | Every link and joint below |
| **REP-105** | Hierarchy `map → odom → base_link → everything else` | Only `base_link` and below |

`base_link` is the robot's reference point: the chassis geometric centre, at
wheel-axle height. Every other frame here is defined relative to it.

---

## The frame tree

```
        odom                      (owned by the EKF in sensor_fusion)
          │
     base_link                    root here; chassis centre, at axle height
          ├── base_link_inertia   mass/inertia holder, zero offset
          ├── base_footprint      ground projection, z = -0.076 m
          ├── front_left_wheel    ┐
          ├── front_right_wheel   │ continuous joints, rotate about y
          ├── rear_left_wheel     │
          ├── rear_right_wheel    ┘
          ├── zed_camera_link     ZED 2i mount point
          └── laser               Hokuyo UST-10LX mount point
```

Everything below `base_link` is a **fixed** joint except the four wheels, which
are **continuous** (free to rotate forever). The transform from `odom` into
`base_link` comes from the EKF in `sensor_fusion`, not from this package.

Two frames are worth explaining.

**`base_footprint`** is `base_link` projected onto the ground. Navigation tools
conventionally plan in this frame because it sits at z = 0. It is a *child* of
`base_link`, not a parent, so `base_link` keeps exactly one parent (`odom`). A
frame with two parents breaks the transform tree.

**`base_link_inertia`** works around a library limitation. KDL, the kinematics
library, cannot handle mass on a root link and warns if you try, so the chassis
mass and inertia live on this zero-offset child instead. Functionally identical,
warning gone.

Frame names are not arbitrary. `laser` matches `urg_node2`'s configured
`frame_id`, so incoming `/scan` messages land on this frame automatically.
`zed_camera_link` is the name the ZED wrapper expects as the root of its own
internal camera frames.

---

## Where everything sits

Side view, looking at the rover's left flank (x forward, z up, not to scale):

```
                      laser  z = +0.100
                        ▲
                        ●
             zed  z = +0.050
                  ▲     │
                  ●     │
    ┌─────────────┼─────┼──────────────┐   top plate, z ≈ +0.039
    │             │     │              │
 ───┼─────────────●─────┼──────────────┼───  base_link  z = 0
    │          x=+0.106 │              │
    └────────────────┬──┴──────────────┘   chassis box, 0.320 × 0.240 × 0.0785
        ( O )        │        ( O )
                     ▼
                base_footprint  z = -0.076   ground
    ◄── rear                        front ──►
```

Top view, wheel corners (x forward, y left):

```
        y
        ▲
 FL ●───┼───● FR          FL, RR use A4WD3-W-ME-B.STL
    │   │   │             FR, RL use A4WD3-W-ME-A.STL
    │   ⊗───┼──► x        ⊗ = base_link
    │       │
 RL ●───────● RR
       x = ±0.110 m,  y = ±0.165 m
```

Every offset in one place:

| Frame | Joint | Joint type | Origin xyz (m) | rpy | Source |
|---|---|---|---|---|---|
| `base_link_inertia` | `base_link_inertia_joint` | fixed | 0, 0, 0 | 0 0 0 | KDL workaround |
| `base_footprint` | `base_footprint_joint` | fixed | 0, 0, -0.076 | 0 0 0 | `-wheel_radius` |
| `front_left_wheel` | `front_left_wheel_joint` | continuous | +0.110, +0.165, 0 | 0 0 0 | measured |
| `front_right_wheel` | `front_right_wheel_joint` | continuous | +0.110, -0.165, 0 | 0 0 0 | measured |
| `rear_left_wheel` | `rear_left_wheel_joint` | continuous | -0.110, +0.165, 0 | 0 0 0 | measured |
| `rear_right_wheel` | `rear_right_wheel_joint` | continuous | -0.110, -0.165, 0 | 0 0 0 | measured |
| `zed_camera_link` | `zed_mount_joint` | fixed | +0.106, 0, +0.050 | 0 0 0 | hand-measured |
| `laser` | `laser_mount_joint` | fixed | +0.050, 0, +0.100 | 0 0 0 | hand-measured |

---

## Files

| Path | What it holds |
|---|---|
| `urdf/helios.urdf.xacro` | Top level. Includes the other three, nothing else. |
| `urdf/common.xacro` | Shared mesh conventions and materials. Included first. |
| `urdf/base.xacro` | Chassis, 4 mecanum wheels, all geometry constants. |
| `urdf/sensors.xacro` | ZED and laser mount frames. |
| `launch/description.launch.py` | Starts `robot_state_publisher` (plus RViz). |
| `rviz/view.rviz` | RViz layout for inspecting the model. |
| `meshes/*.STL` | CAD geometry for visual display. |
| `CMakeLists.txt` | Installs `urdf/`, `launch/`, `rviz/`, `meshes/` into `share/`. |
| `package.xml` | Package metadata and dependencies. |

### `urdf/helios.urdf.xacro`

The entry point, and deliberately almost empty. Splitting the description into
three files means chassis geometry and sensor mounting can be edited
independently. This is the file the launch file processes; the other three are
never loaded directly.

`common.xacro` **must be included first**, because the other two consume its
properties and materials. `base.xacro` and `sensors.xacro` do not depend on each
other and may be listed in either order.

### `urdf/common.xacro`

Holds only what both other files need: the mesh conventions (`mesh_scale`,
`mesh_rpy`) and the three material definitions.

It is included **once**, from the top level, rather than from both files.
Properties are idempotent under a repeated include, but `<material>` elements
are not: two identical definitions make `urdf_parser` fail outright with
`material '<name>' is not unique`.

### `urdf/base.xacro`

Defines `base_link` and the four wheels. The measured rover geometry lives at
the top as xacro properties.

| Property | Value | Meaning |
|---|---|---|
| `wheel_radius` | 0.076 m | 152 mm wheel diameter / 2 |
| `half_wheelbase` | 0.110 m | front↔rear axle spacing / 2 |
| `half_track` | 0.165 m | left↔right wheel spacing / 2 |

The same three measurements appear in two other places, in full rather than
halved:

| File | Keys | Used for |
|---|---|---|
| `perception_pkg/wheel_odometry/config/wheel_odometry.yaml` | `wheel_radius`, `wheelbase`, `track_width` | Encoder counts → distance and yaw |
| `low_level_control_pkg/config/teleop.yaml` | `wheel_radius`, `wheelbase`, `track_width` | m/s → encoder counts/sec |

**If you re-measure the rover, all three files must be updated together.** They
are not shared automatically.

The `wheel` macro is called four times with different corner positions and mesh
files. Mecanum wheels are *handed*: the diagonal pairs have mirrored roller
angles, which is what makes strafing work. That is why there are two mesh
variants (`A4WD3-W-ME-A/B.STL`) assigned to diagonal corners rather than one
mesh reused four times.

Visual geometry uses the STL meshes; collision geometry stays as simple boxes
and cylinders. Collision checking against a detailed mesh is far slower and buys
nothing for a rover this size.

### `urdf/sensors.xacro`

Defines `zed_camera_link` and `laser`, each attached to `base_link` by a fixed
joint whose `<origin>` is the physical mounting offset.

> **Both offsets are hand-measured against the real rover**, from the chassis
> centre at axle height (`base_link`) to the point each driver actually anchors
> to: the laser's rotating mirror, and the ZED's bottom screw hole. They are
> worth re-checking after any remount. An error here displaces the scan and
> cloud from where they truly are, which shows up as a map that is subtly
> skewed rather than obviously broken.

`laser_mount_joint` is slam_toolbox's scan-matching lever arm, so a translation
error `d` becomes a prediction error of `(R(φ) - I)·d`, reaching `2·d` at a
180° turn. Its error grows with rotation, which is why it is measured rather
than estimated.

The file also carries a warning worth repeating: the ZED wrapper must not
publish `base_link → zed_camera_link` itself. This URDF owns that transform. If
both publish it, the frame has two parents and the camera data jitters between
two positions. The wrapper is launched with `publish_tf:=false` in
`sensor_fusion/launch/bringup.launch.py` for exactly this reason.

### `launch/description.launch.py`

Runs `xacro` on the top-level file at launch time and feeds the result to
`robot_state_publisher` as the `robot_description` parameter. Because the xacro
is processed at every launch, edits take effect on the next launch with no
rebuild needed (given `--symlink-install`).

Continuous joints need someone to report their angle, so the launch file also
starts a joint state publisher.

| Argument | Default | Effect |
|---|---|---|
| `gui` | `true` | `joint_state_publisher_gui`, sliders to rotate the wheels by hand |
| `gui:=false` | | Plain `joint_state_publisher`, all angles zero |
| `rviz` | `true` | Opens RViz with `rviz/view.rviz` |

`gui:=false` is what the real robot uses. Nothing reads wheel angle, so zeros
are fine. `sensor_fusion/launch/bringup.launch.py` includes this launch file
with `gui:=false`, so on the real robot you never launch it directly.

### `meshes/`

STL exports from CAD, in millimetres, hence the `0.001` scale factor in the
xacro. Each mesh needed a rotation and offset to align its native CAD axes with
REP-103. Those constants (`mesh_rpy`, `chassis_mesh_xyz`, and the per-wheel
offsets) were derived from each STL's bounding box rather than read from CAD, so
they are correct-looking rather than certified.

`ZED2i.STL` is the exception: it does not follow the shared convention and
carries its own rotation on the `<visual>` element in `sensors.xacro`.

---

## Build

```bash
cd ~/helios_ws
colcon build --packages-select helios_description --symlink-install
source install/setup.bash
```

## Run

Standalone, to look at the model:

```bash
ros2 launch helios_description description.launch.py
```

RViz opens with the rover and slider controls for the wheels. Arguments:
`gui:=false` (no sliders, wheels at zero), `rviz:=false` (publish transforms
only).

On the real robot you do not run this. `sensor_fusion`'s bring-up includes it.

---

## Verify

**1. The xacro parses and produces valid URDF.** Do this after every edit; a
typo here fails at launch with an unhelpful error. The workspace must be sourced
first, because `$(find helios_description)` resolves through the ament index:

```bash
cd ~/helios_ws
source install/setup.bash
xacro src/helios_description/urdf/helios.urdf.xacro > /tmp/helios.urdf
check_urdf /tmp/helios.urdf
```

`check_urdf` should print `base_link` as root with 8 children and no warnings
about multiple parents.

**2. All frames are published.** With the launch file running:

```bash
ros2 run tf2_tools view_frames      # writes frames.pdf in the current directory
```

Expect one connected tree rooted at `base_link` containing all nine frames.

**3. A specific offset is what you intended.** This is how you check a mount
measurement after editing it:

```bash
ros2 run tf2_ros tf2_echo base_link laser
```

The printed translation must match the `<origin>` you set in `sensors.xacro`,
and the table above.

**4. It looks right.** In RViz, add a *RobotModel* display with Fixed Frame
`base_link`, and add *TF* to see the frame axes. Check that the wheels are at
the corners, the sensors sit on the top plate, and nothing is rotated 90° or
sunk into the chassis. Dragging the GUI sliders should spin the wheels about
their axles. If a wheel orbits the robot instead, its joint axis or origin is
wrong.

**5. On the real robot,** the practical test is whether the laser scan lines up
with reality. Drive up to a flat wall, and in RViz the `/scan` points should
form a straight line parallel to the wall at the correct distance. A scan that
is rotated or offset points back at `laser_mount_joint`.

---

## Related

- [`perception_pkg/sensor_fusion`](../perception_pkg/sensor_fusion/README.md):
  owns `odom → base_link`, includes this package's launch file
- [`perception_pkg/wheel_odometry`](../perception_pkg/wheel_odometry/README.md):
  uses the same geometry constants
- [`low_level_control_pkg`](../low_level_control_pkg/README.md): converts
  velocity commands using the same wheel radius
