# navigation_pkg

Autonomous navigation: give Helios a goal pose, and it plans a route and drives
there while avoiding what it sees.

This package contains **no algorithms**. Every node it launches is an upstream
Nav2 server. What lives here is the configuration and launch wiring that makes
Nav2 work with *this* robot: a mecanum base that can strafe, an EKF that already
owns `odom -> base_link`, and a Jetson that is already busy.

**Read [the root README](../../README.md) first** for the system overview, and
[launch_helios.md](../../launch_helios.md) for how to run the whole stack.

> **Status: configured, not yet validated on a full drive.** Every server
> launches and reaches `active`, and the parameters below are reasoned from this
> robot's measured geometry and limits. Nothing here has been confirmed against
> a real goal-to-goal run, so treat the tuning values as starting points.

---

## Where it sits

Nav2 is the top layer. It consumes everything below it and drives everything
above nothing, because there is nothing above it.

```
            goal pose (RViz, or a NavigateToPose action client)
                                │
                                ▼
   ┌────────────────────────────────────────────────────────┐
   │  navigation_pkg                                        │
   │    bt_navigator ──► planner_server ──► global path     │
   │          │                                             │
   │          └────────► controller_server (MPPI) ──────┐   │
   │                            │                       │   │
   │                     behavior_server            /cmd_vel_nav
   │                     (spin/backup/wait)             │   │
   │                                                    ▼   │
   │                                        velocity_smoother│
   └────────────────────────────────────────────┬───────────┘
             ▲              ▲                   │ /cmd_vel_smoothed
     /map    │      /scan   │  /odometry/filtered
             │              │                   ▼
   mapping_localization  perception_pkg   low_level_control_pkg
        (map -> odom)   (odom -> base_link)   (drives the wheels)
```

**Nav2 publishes no transforms.** TF ownership is unchanged: the EKF keeps
`odom -> base_link`, whichever mapper is running keeps `map -> odom`.

---

## Driving style: nose-first, not crabbing

Helios *can* strafe, and for a long time this config let it. The result was a
rover that slid sideways toward its goal rather than facing it. Retuned
**2026-10-29** to drive nose-first, with strafe demoted to a correction axis.

Worth knowing why it looked erratic rather than merely omnidirectional: the
config had drifted into a hybrid. `PreferForwardCritic` was enabled at 8.0 and
`PathAngleCritic.mode` was 0, both of which ask for forward-facing motion,
while `vy_max` stayed at 0.40 making a sideways slide free. The two pulled
against each other every cycle. The settings below are at least coherent.

| Setting | Was | Now | Why |
|---|---|---|---|
| `vy_max` / `vy_std` | 0.40 / 0.20 | **0.10 / 0.05** | Strafe still available to slide around an obstacle, never enough to carry a manoeuvre |
| `PreferForwardCritic` | 8.0 | **25.0** | A goal behind is reached by turning to face it, not by reversing |
| `PathAngleCritic` | 6.0, 0.35 rad | **12.0, 0.25 rad** | Decides rotate-then-go versus curve-while-going. Large heading error makes turning on the spot cheaper than driving |
| `PathAlignCritic` | 10.0 | **6.0** | SmacPlanner2D emits zigzagging 8-connected paths; at 10.0 the optimiser chased every jog |
| `use_path_orientations` | false | **true** | With nose-first driving the path's heading is information, not a free variable |
| `TwirlingCritic` | 10.0 | **6.0** | Still suppresses aimless spin, no longer fights a deliberate turn-on-the-spot |
| `GoalAngleCritic` | 3.0 @ 0.5 m | **6.0 @ 0.8 m** | Blends the final heading into the approach instead of spinning after stopping |
| `ax_max` / `ax_min` / `ay_max` / `az_max` | unset | **0.60 / -0.60 / 0.30 / 1.50** | Were defaulting to ~3 m/s², so MPPI could demand a near-instant stop. `ax_min` is the braking ramp: 0.40 m/s to rest over ~0.67 s and ~0.13 m |

`motion_model` stays `"Omni"`. `DiffDrive` would forbid `vy` outright; the goal
is to deprioritise strafe, not remove it.

All four acceleration limits sit inside `velocity_smoother`'s
`[0.80, 0.80, 2.00]` ceilings, so the smoother is never fighting the optimiser.

> **This trades away most of the mecanum advantage.** It is the right call for
> predictable motion and it suits the VLA later, whose training data is
> nonholonomic. To get omnidirectional driving back, restore `vy_max` 0.40 /
> `vy_std` 0.20 and drop `PreferForwardCritic` toward 5.0.

### Still open

Paths come from `SmacPlanner2D`, which is grid-based and jagged. A
`smoother_server` would clean them up, but it needs a custom behaviour tree to
invoke `SmoothPath`, so lowering `PathAlignCritic` is the cheap stand-in for
now.

---

## Files

| Path | What it holds |
|---|---|
| `config/nav2_params.yaml` | Every server's parameters, in one file |
| `launch/navigation.launch.py` | Starts 5 servers plus the lifecycle manager |
| `rviz/navigation.rviz` | Costmaps, plans, footprint, and the goal tool |
| `CMakeLists.txt` | Installs `config/`, `launch/`, `rviz/` into `share/` |
| `package.xml` | Metadata and dependencies |

### What gets launched, and what does not

`nav2_bringup` starts around nine servers. This package starts **five**, plus
the lifecycle manager:

| Node | Job |
|---|---|
| `controller_server` | MPPI local control, owns the local costmap |
| `planner_server` | SmacPlanner2D global planning, owns the global costmap |
| `behavior_server` | Recovery behaviors: spin, backup, drive_on_heading, wait |
| `bt_navigator` | Runs the behavior tree that sequences the above |
| `velocity_smoother` | Ramps `/cmd_vel_nav` into `/cmd_vel_smoothed` |
| `lifecycle_manager_navigation` | Drives all five configure to activate |

**Activation order matters and is not alphabetical.** The manager transitions
nodes in sequence and stops at the first one that will not activate, leaving
everything after it inactive, and it never retries.

```
controller_server   ┐
behavior_server     │ need only odom -> base_link, from the EKF
velocity_smoother   │
bt_navigator        ┘
planner_server      <- carries the GLOBAL costmap: blocks on map -> odom
```

`planner_server` is therefore last. Put it earlier and a rover that is not yet
localized loses the behaviors too, which is what `amcl_recovery` drives through
to localize itself: a deadlock the robot cannot leave unaided. Verified live
that `behavior_server` activates with no `map -> odom` at all, because `Spin`
and `DriveOnHeading` both work in `local_frame: odom`.

Omitted: `smoother_server`, `waypoint_follower`, `docking_server`,
`collision_monitor`. Nothing uses them yet, and this Jetson already runs the ZED
at around 155% CPU. Add them when there is a reason.

`nav2_bringup` is **not** a dependency. `navigation.launch.py` wires the
lifecycle itself, the same way `amcl_localization.launch.py` does.

---

## Topic wiring

The controller does not talk to the motors directly. It is routed through the
smoother so the driver only ever receives ramped commands:

```
controller_server ──► /cmd_vel_nav ──► velocity_smoother ──► /cmd_vel_smoothed
                                                                      │
                          teleop_joy_node ──► /cmd_vel_joy ──┐        │
                                              (priority 100) │        │ (10)
                                                             ▼        ▼
                                                          twist_mux
                                                              │ /cmd_vel
                                                              ▼
                                                    roboclaw_driver_node
```

`twist_mux` is started with the driver in `low_level_control_pkg` and is the
only publisher of `/cmd_vel`. Nav2 never writes that topic directly, which is
what lets the joystick override a goal in progress.

> ### Teleop outranks navigation, by design
>
> Both feed `twist_mux`, which is started with the driver and is the only
> publisher of `/cmd_vel`. The joystick sits at priority 100 and Nav2 at 10, so
> holding the deadman overrides a goal in progress and releasing it hands
> control back. Run them together.
>
> **There is also no deadman button in autonomous mode.** Teleop requires you to
> hold R for the rover to move. Nav2 does not. Keep a hand on the kill path and
> know that `Ctrl+C` on the navigation terminal stops new commands, after which
> the driver's own `cmd_timeout` (0.5 s) ramps the wheels to a stop.

---

## Key parameters

Geometry comes from `helios_description/urdf/base.xacro`. Re-measure there
first, then mirror it here.

| Parameter | Value | Source |
|---|---|---|
| `footprint` | 0.38 x 0.38 m square | Wheels reach x = ±0.186, y = ±0.1875 |
| `inflation_radius` | 0.45 m | ~1.7x the 0.27 m circumscribed radius |
| `vx_max` / `vy_max` / `wz_max` | 0.40 m/s, 0.40 m/s, 0.80 rad/s | Matches `roboclaw.yaml` ceilings |
| `max_accel` | 0.80 m/s² | `drive_accel` 5000 counts/s² is ~0.98 m/s² |
| `resolution` | 0.05 m/cell | Matches `slam_toolbox.yaml` |
| `always_send_full_costmap` | `true` (local) | Nav2's default sends the full grid once and only increments after, so a node subscribing later gets nothing at all |

A rectangular footprint is used rather than `robot_radius` because a
circumscribing circle on this base is 0.27 m and would refuse gaps the rover
actually fits through.

### CPU, and what to turn up first

`batch_size` and `time_steps` are **reduced from the Nav2 defaults** (1000/40
instead of 2000/56) because of the ZED's load on this Jetson. Cost scales with
`batch_size * time_steps`.

| If | Then |
|---|---|
| The controller misses 20 Hz | Lower `batch_size` first, then `time_steps` |
| There is CPU headroom | Raise `batch_size` toward 2000; path quality improves |
| Paths clip corners | Raise `time_steps` for a longer horizon |

`CostCritic.consider_footprint` is `false`. Setting it `true` is exact rather
than using the inscribed-circle approximation, but it is markedly slower. Turn
it on only if the rover clips obstacles in tight spaces and you have the CPU.

---

## Build

```bash
cd ~/helios_ws
colcon build --packages-select navigation_pkg --symlink-install
source install/setup.bash
```

This package installs config and launch files only, so the build itself needs
nothing. The servers it launches do:

```bash
sudo apt install ros-jazzy-nav2-mppi-controller ros-jazzy-nav2-smac-planner \
                 ros-jazzy-nav2-bt-navigator ros-jazzy-nav2-behaviors \
                 ros-jazzy-nav2-velocity-smoother ros-jazzy-nav2-lifecycle-manager
```

All six are already present on this Jetson.

## Run

Nav2 sits on top of a running stack. Start these first, in this order:

```bash
# 1. motors
ros2 launch low_level_control_pkg roboclaw_driver.launch.py
# 2. sensors + EKF
ros2 launch sensor_fusion bringup.launch.py
# 3. a map source, exactly ONE of:
ros2 launch mapping_localization_pkg slam_toolbox.launch.py
ros2 launch mapping_localization_pkg amcl_localization.launch.py map:=<path>.yaml
# 4. navigation
ros2 launch navigation_pkg navigation.launch.py rviz:=true
```

| Argument | Default | Effect |
|---|---|---|
| `params_file` | this package's `nav2_params.yaml` | Point at an alternative param set |
| `autostart` | `true` | `false` leaves the servers unconfigured for manual transitions |
| `rviz` | `false` | Opens the navigation layout. Costs CPU; prefer a laptop. |

Then set a goal: in RViz use **Nav2 Goal**, or call the action directly:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, \
   orientation: {w: 1.0}}}}"
```

---

## Verify

**1. All five servers reached active.** This is the single most useful check:

```bash
ros2 lifecycle get /controller_server   # expect: active [3]
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

If the lifecycle manager logs `Failed to bring up all requested nodes`, a server
could not configure. The usual cause is a missing transform: the costmaps block
until `map -> odom -> base_link` is complete.

**2. Costmaps are populated.**

```bash
ros2 topic hz /global_costmap/costmap     # ~1 Hz
ros2 topic hz /local_costmap/costmap      # ~2 Hz
```

In RViz the local costmap should show inflation around the walls the laser sees.
An empty costmap next to a visible `/scan` means the `sensor_frame` or the
`observation_sources` topic is wrong.

**3. A plan is produced.** Set a goal and watch:

```bash
ros2 topic echo /plan --once      # the global path
ros2 topic hz /local_plan         # MPPI's chosen trajectory, ~20 Hz
```

**4. Commands actually reach the driver.**

```bash
ros2 topic hz /cmd_vel_nav        # controller output
ros2 topic hz /cmd_vel_smoothed   # smoothed, into twist_mux
ros2 topic hz /cmd_vel            # twist_mux output, what the driver consumes
```

If `/cmd_vel_nav` publishes but `/cmd_vel_smoothed` does not, the smoother is
not active. If `/cmd_vel_smoothed` publishes but `/cmd_vel` does not, either
`twist_mux` is not running or the joystick is holding priority.

**5. It strafes.** Give it a goal directly to one side in an open space. A
correctly configured omni setup will translate sideways rather than turning,
driving forward, and turning back. If it always turns first, `motion_model` is
not `Omni` or `PreferForwardCritic` has crept back in.

---

## Known limitations

### Low obstacles, and what now covers them

The laser sweeps 270 deg but only at one height, so anything shorter than it
was simply invisible and the rover drove into it. The **local** costmap now
takes a second obstacle source from the ZED, which sits *lower* than the laser
and sees in 3D.

All heights are z in the costmap's `global_frame` (`odom`). The EKF runs
`two_d_mode`, so odom z = 0 is exactly the `base_link` plane, and `base_link`
sits at wheel-axle height:

| | z in `odom` | above real ground |
|---|---|---|
| Real ground | -0.076 m | 0 |
| ZED optical centre | +0.050 m | 0.126 m |
| Laser plane | +0.100 m | 0.176 m |

| Source | Arc | Height band | Range |
|---|---|---|---|
| `scan` | 270 deg | single plane at 0.176 m | 6 m |
| `zed_cloud` | ~110 deg forward | 0.036 m to 0.58 m above ground | 2.5 m |

`min_obstacle_height: -0.040` is the parameter to tune first. It sits 36 mm
above the floor, so objects taller than about 4 cm mark and the floor itself
does not. Raise it toward 0 if stereo noise starts painting the floor; lower it
toward -0.076 to catch flatter things, at that risk.

The short 2.5 m range is deliberate. Stereo depth error grows with the square
of distance, and further out the noise would mark phantom obstacles faster than
raytracing clears them.

> **This costs GPU that was previously free.** `publish_point_cloud` only
> advertises; the SDK computes nothing until something subscribes, and until
> now only RViz ever did. The costmap subscribes permanently. `point_cloud_res`
> is set to `REDUCED` to halve that, which loses nothing when everything ends
> up in 0.05 m cells anyway.

### What this still does not fix

| Limitation | Consequence |
|---|---|
| **The ZED is forward-only (~110 deg); the laser is 270 deg** | Low obstacles to the side and behind are still invisible, and **this rover strafes**. MPPI's Omni model will drive sideways into them |
| Stereo has a near blind zone (~0.25 m) | Nothing is seen right at the bumper. Costmap memory covers most of this, since you normally see an object before you get that close |
| **Global** costmap is still `scan` only | The planner routes through low obstacles and MPPI has to fight it locally. Add `zed_cloud` to the global `obstacle_layer` if that becomes a problem, but keep the range short: a non-rolling costmap keeps noise forever |
| Not in the slam_toolbox map | slam_toolbox consumes `/scan` only, so a saved map never contains low obstacles. For *permanent* low structure, paint it into the `.pgm` by hand so the planner sees it |
| No `collision_monitor` | There is no independent last-resort stop below the controller |
| Glass and low-contrast surfaces | Missed by the laser, and often by stereo too |

---

## Related

- [`mapping_localization_pkg`](../mapping_localization_pkg/README.md): supplies
  `/map` and owns `map -> odom`
- [`perception_pkg/sensor_fusion`](../perception_pkg/sensor_fusion/README.md):
  supplies `/odometry/filtered` and owns `odom -> base_link`
- [`low_level_control_pkg`](../low_level_control_pkg/README.md): consumes
  `/cmd_vel`, and the reason the velocity ceilings here match `roboclaw.yaml`
- [`helios_description`](../helios_description/README.md): the footprint and
  sensor heights this package's costmaps depend on
