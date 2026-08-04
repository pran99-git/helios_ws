# Helios

Helios is a four-wheeled **mecanum rover** that builds a map of an indoor space
while it drives through it, using a 2D laser scanner and a stereo camera. It is
driven by hand today (a Bluetooth game controller); the mapping runs
autonomously alongside, so that later it can navigate on its own.

The whole thing runs on a single onboard computer — an **NVIDIA Jetson AGX
Orin** — under **ROS 2 Jazzy**. There is no off-board machine: you SSH into the
rover, or plug a monitor into it, and everything runs there.

*New to this repo? Read this file top to bottom first. It explains what the
robot is and how the software fits together. The per-package READMEs linked at
the bottom are reference material — they assume you have read this.*

---

## What it does today

| Works | Status |
|---|---|
| Drive by joystick | Yes |
| Wheel odometry (position from wheel rotation) | Yes |
| Fused odometry (wheels + camera + IMU) | Yes |
| 2D map from the laser scanner | Yes |
| 3D map from the camera + laser | Yes, quality still being tuned |
| Drive itself to a goal | **Not yet** — no path planner or navigation stack |

So: Helios can be driven around and will produce a map. It cannot yet decide
where to go.

---

## Hardware

| Part | What it is | Why it's there |
|---|---|---|
| Lynxmotion A4WD3 chassis | 4 mecanum wheels, 12 V motors, 51:1 gearboxes | Mecanum wheels let it strafe sideways and rotate in place |
| 2× RoboClaw controllers | Motor drivers, one per side, USB serial | Run the motors and read the wheel encoders |
| Hokuyo UST-10LX | 2D laser scanner, Ethernet | Measures distance to walls in a flat slice — the main mapping sensor |
| Stereolabs ZED 2i | Stereo depth camera + IMU, USB 3 | Depth images and visual-inertial odometry |
| NVIDIA Jetson AGX Orin | Onboard computer, GPU | Runs everything |
| 3S LiPo, 11.1 V | Battery | Powers motors and electronics |
| 8BitDo SN30 Pro | Bluetooth gamepad | Manual driving |

### What is a mecanum wheel?

Each wheel has angled rollers around its rim. Because the rollers are free to
spin sideways, driving the four wheels at *different* speeds produces sideways
(strafing) motion, not just forward/back and turning. That makes the rover
**holonomic** — it can move in any direction without turning first.

The cost is that mecanum wheels slip more than normal ones, so wheel-based
position estimates drift faster — which is exactly why the camera and laser are
here.

---

## How the software fits together

Data flows in one direction: sensors produce raw measurements, those get fused
into a single position estimate, and that estimate feeds the mapper.

```
  HARDWARE                 SENSING                  FUSION            MAPPING
  ────────                 ───────                  ──────            ───────

  RoboClaw ──encoder counts──► wheel_odometry ─┐
  (wheels)                     (mecanum math)  │
                                               ├──► EKF ──► /odometry/filtered
  ZED 2i ────depth + IMU─────► zed_wrapper ────┘    (robot_localization)   │
                                    │                                      │
                                    └─────── RGB + depth images ───────────┤
                                                                           │
  Hokuyo ────laser scan──────► urg_node2 ──► /scan ────────────────────────┤
                                                                           ▼
                                                          slam_toolbox  or  RTAB-Map
                                                           (2D map)        (3D map)
```

Three ideas make the rest of the repo readable:

**1. Odometry means "how far have I moved since I started?"** Three separate
things answer that question here — the wheels, the camera, and the fused
estimate. Each is imperfect in a different way: wheels slip, the camera loses
tracking in featureless corridors. Fusing them gives a better answer than any
one alone. That fusion is an **EKF** (Extended Kalman Filter), and its output is
`/odometry/filtered`.

**2. Odometry always drifts; a map fixes it.** Odometry is smooth but slowly
wrong — errors accumulate forever. A map is the opposite: it can jump when the
robot recognises a place it has been before ("loop closure"), but it does not
drift. ROS keeps both by using two reference frames:

- `odom` — smooth, continuous, drifts. Safe for short-term control.
- `map` — corrected against the world, occasionally jumps. Correct long-term.

**3. Exactly one node may publish each coordinate transform.** This is the rule
that breaks a robot most often when violated, because two nodes publishing the
same transform makes the robot's position flicker between two answers, and
nothing errors out — it just behaves strangely.

| Transform | Meaning | Published by |
|---|---|---|
| `map → odom` | The drift correction | The mapper (slam_toolbox **or** RTAB-Map, never both) |
| `odom → base_link` | Where the robot is | The EKF, and **only** the EKF |
| `base_link → sensors, wheels` | Where parts are bolted on | `robot_state_publisher`, from the URDF model |

`base_link` is the robot's own reference point (centre of the chassis). This
layering is a ROS convention called REP-105 — the point of following it is that
any standard ROS navigation tool will work with this robot unmodified.

Because of rule 3, the wheel odometry node and the ZED wrapper both run with
their TF publishing **turned off**. They still publish their measurements as
topics, which the EKF consumes — they just are not allowed to state where the
robot is.

---

## Repo map

```
helios_ws/
├── src/
│   ├── helios_description/        The robot's physical model (URDF): sizes,
│   │                              where each sensor is bolted on
│   ├── low_level_control_pkg/     Motors: RoboClaw driver + joystick teleop
│   ├── perception_pkg/            Sensors and fusion  ← a folder, NOT a package
│   │   ├── sensor_fusion/           the EKF + the bring-up launch file
│   │   ├── wheel_odometry/          encoder counts → position estimate
│   │   ├── Camera/zed-ros2-wrapper/ ZED driver   (upstream submodule)
│   │   └── LiDAR/urg_node2/         Hokuyo driver (upstream submodule)
│   └── mapping_localization_pkg/  Map building
│       ├── slam_toolbox/            2D laser SLAM
│       └── rtabmap/                 3D camera + laser SLAM
├── build/  install/  log/         Generated by the build — not in git
└── README.md                      This file
```

Two things to know about this layout:

**`perception_pkg` is a plain directory, not a ROS package.** It groups the
sensing-related packages. The actual packages inside it are `sensor_fusion` and
`wheel_odometry`, and those are the names you use with `ros2 launch` and
`colcon build --packages-select`. Every other top-level folder *is* a package.

**The two driver folders are git submodules** pulled from Stereolabs and Hokuyo.
Do not edit them — changes there are not tracked by this repo and will be lost.
Clone with `git clone --recurse-submodules`, or run `git submodule update
--init --recursive` if you already cloned.

---

## First-time setup

Assumes Ubuntu 24.04 with ROS 2 Jazzy and JetPack already installed.

```bash
# 1. Get the code, including the driver submodules
git clone --recurse-submodules <repo-url> ~/helios_ws

# 2. Dependencies — reads them from each package's package.xml
cd ~/helios_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
cd ~/helios_ws
colcon build --symlink-install
source install/setup.bash
```

Then three hardware steps, each covered in the package README that owns it:

1. **ZED SDK** must be installed separately from Stereolabs, matching your
   JetPack version → [`perception_pkg`](src/perception_pkg/README.md)
2. **Laser scanner network** — it lives at `192.168.0.10`, so the Jetson's wired
   interface must be on that subnet. Check with `ping 192.168.0.10` →
   [`perception_pkg`](src/perception_pkg/README.md)
3. **Motor controller serial ports** — a udev rule pins them to stable names →
   [`low_level_control_pkg`](src/low_level_control_pkg/README.md)

> **Every new terminal needs `source ~/helios_ws/install/setup.bash`** before
> any `ros2` command will find this workspace. This catches everyone at least
> once: the symptom is `Package 'sensor_fusion' not found`.

---

## Running it

Four terminals, started in this order. Each one keeps running — do not close it.

```bash
# Terminal 1 — motors (must be first; owns the serial ports)
ros2 launch low_level_control_pkg roboclaw_driver.launch.py

# Terminal 2 — sensors + fused odometry
ros2 launch sensor_fusion bringup.launch.py

# Terminal 3 — joystick driving
ros2 launch low_level_control_pkg joy_teleop.launch.py

# Terminal 4 — pick ONE mapper
ros2 launch mapping_localization_pkg slam_toolbox.launch.py   # 2D
ros2 launch mapping_localization_pkg rtabmap.launch.py        # 3D
```

**Hold the right shoulder button (R) to drive.** Release it and the rover stops
— that is a deadman switch, and it is deliberate. Left stick moves, right stick
rotates.

**Keep the rover still for the first ~5 seconds** after terminal 2 starts. The
ZED aligns its internal sense of "down" against gravity at startup, and moving
during that makes it print a realignment warning and start with a worse
estimate.

The order matters in one place only: the motor driver must start before
anything else, because it is the only process allowed to open the RoboClaw
serial ports. Two processes on one serial port corrupt each other's packets —
sent to a motor controller, that is a safety problem, not a dropped reading.

### Checking it actually works

```bash
ros2 topic hz /scan                  # laser        ~40 Hz
ros2 topic hz /odometry/filtered     # fused pose   ~30 Hz
ros2 topic hz /map                   # the map, once a mapper is running
ros2 run tf2_tools view_frames       # writes a PDF of the transform tree
```

The transform tree should be a single connected tree:
`map → odom → base_link → sensors and wheels`. If a frame is missing or
appears twice, that is the problem — go back to rule 3 above.

---

## Things that commonly go wrong

| Symptom | Cause | Fix |
|---|---|---|
| `Package 'x' not found` | Terminal not sourced | `source ~/helios_ws/install/setup.bash` |
| `could not open ethernet port` | Laser's network port not ready yet | Wait a few seconds, relaunch. Confirm with `ping 192.168.0.10` |
| `Gravity alignment issues detected` | Rover moved during ZED startup | Keep it still ~5 s while terminal 2 starts, then relaunch |
| RTAB-Map drops frames, map is patchy | Jetson overloaded | Do not run `rtabmap_viz:=true` during a real run — inspect the saved database afterwards instead |
| Rover will not move | Deadman not held, or gamepad disconnected | Hold R. Check `ros2 topic echo /joy` responds to sticks |
| Two robot positions in RViz | Two nodes publishing one transform | See rule 3. `ros2 topic info /tf --verbose` lists publishers |

---

## Vocabulary

Terms used throughout this repo and the ROS documentation:

- **TF / transform** — the relationship between two coordinate frames ("the
  laser sits 20 cm forward of the chassis centre"). ROS maintains a tree of
  these so any sensor reading can be expressed relative to any other part.
- **Frame** — a named coordinate system. `base_link`, `odom`, `map`, `laser`.
- **Odometry** — an estimate of how far the robot has moved, from its own
  sensors.
- **VIO** (visual-inertial odometry) — odometry from a camera plus an IMU. What
  the ZED produces.
- **IMU** — accelerometer + gyroscope. Measures rotation and acceleration.
- **EKF** (Extended Kalman Filter) — the algorithm that merges several noisy
  estimates into one better estimate.
- **SLAM** — Simultaneous Localisation and Mapping: building a map while
  simultaneously working out where you are in it.
- **Loop closure** — recognising a previously visited place, which lets SLAM
  correct accumulated drift.
- **Occupancy grid** — a 2D map as a grid of cells: free, occupied, or unknown.
  What `/map` contains and what you see in RViz.
- **Node** — one running ROS program. **Topic** — a named stream of messages
  between nodes. **Launch file** — a script that starts several nodes together
  with their settings.
- **Lifecycle node** — a node that must be explicitly configured and activated
  before it does anything. The laser driver and slam_toolbox are both of these;
  the launch files handle the transitions.
- **URDF** — the XML format describing the robot's physical structure.
- **RViz** — the 3D viewer for ROS data.
- **QPPS** — quadrature pulses per second, the RoboClaw's speed unit.
- **REP-105** — the ROS convention defining the `map`/`odom`/`base_link`
  frame layering described above.

---

## Where to read next

In this order:

1. [`helios_description`](src/helios_description/README.md) — the robot's
   physical model and the full transform tree
2. [`perception_pkg`](src/perception_pkg/README.md) — sensor hardware setup
   - [`sensor_fusion`](src/perception_pkg/sensor_fusion/README.md) — the EKF and
     transform ownership in detail
   - [`wheel_odometry`](src/perception_pkg/wheel_odometry/README.md) — mecanum
     kinematics and calibration
3. [`low_level_control_pkg`](src/low_level_control_pkg/README.md) — motor
   control, teleop, and the safety mechanisms
4. [`mapping_localization_pkg`](src/mapping_localization_pkg/README.md) — both
   mapping approaches and how maps are saved
