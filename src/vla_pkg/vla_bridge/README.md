# vla_bridge

The ROS side of VLA navigation. Feeds the policy server an observation history
and a goal, turns the returned waypoints into a `nav_msgs/Path`, and hands that
to Nav2's existing MPPI controller.

Nothing here imports torch. The policy lives in
[`omnivla_server`](../omnivla_server/README.md).

## Three nodes

```
  route goal ──► subgoal_walker_node ──┐          (planner_server owns the route)
                        │              │ carrot every 1.5 m
                        │              ▼
  ZED RGB ──────────────┼──► vla_bridge_node ──/vla/plan──► path_arbiter_node
  TF ───────────────────┘                                          │
                        │                                          │ FollowPath
                        │ pure yaw, bearing > 0.90 rad             ▼
                        └──► /cmd_vel_walker ──► twist_mux ◄── controller_server
                                  (priority 20)      (10)    (MPPI + costmaps)
```

**`vla_bridge_node`** runs the cycle: newest frame into a 6-frame window, goal
resolved into the body frame through TF, one HTTP call, waypoints lifted from
`base_link` into `odom`, path published.

**`path_arbiter_node`** decides when that path is worth acting on. The policy
republishes at 5 Hz, but every preemption throws away MPPI's warm optimiser
state, so a path is only forwarded when it actually disagrees with the one
being followed, or when the active goal has gone stale.

**`subgoal_walker_node`** is optional and off by default (`walker:=true`). The
policy reaches about a metre per chunk and holds no map, so a distant goal
makes it drive at the bearing until an obstacle stops it. The walker asks
`planner_server` for a route and feeds the policy a carrot 1.5 m along it, so
route choice stays with the component that has the map.

### Turning before handing over

The walker is also the only thing that can turn the rover, and it has to be.

**The policy only ever proposes forward motion.** Measured over 2785 live
proposals, 99-100% of chunks pointed within +/-20 degrees of straight ahead. So
a carrot behind the rover yields a chunk aimed the wrong way.

`RotationShimController` cannot rescue that, which is the non-obvious part: it
sits **downstream** of the policy and aligns to the chunk, and the chunk already
reads as straight ahead, so the real heading error is never seen by anything.

Above `turn_first_angle` the walker therefore turns the rover itself, on its own
`twist_mux` input, and returns no carrot so the policy stays idle meanwhile.
Hysteresis at 40% of the threshold stops it chattering, and `turn_min_range`
suppresses turns when the carrot is nearly underneath the rover.

The turn is a slow **arc**, not a spin. `slam_toolbox`'s `shouldProcessScan`
gates on translation alone, so a pure spin is unobserved and its accumulated
yaw error arrives as a lurch when driving resumes. Forward speed is scaled by
`max(0, cos(bearing))`, so the rover creeps only once it substantially faces
where it is going, never into the space behind it. Measured translation per
turn: 7.6 cm at the threshold, 11.1 cm for a 172-degree turn, both clear of the
mapper's 5 cm gate.

## Interfaces

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/Image` | depth 1, best-effort. RGB only |
| in | `/vla/goal_pose` | `geometry_msgs/PoseStamped` | **not** `/goal_pose`, see below |
| in | `/vla/goal_text` | `std_msgs/String` | empty string clears |
| in | `/vla/goal_image` | `sensor_msgs/Image` | egocentric goal view |
| in | `/vla/route_goal` | `geometry_msgs/PoseStamped` | walker only: the final destination |
| in | TF `map` → `odom` → `base_link` | | read only |
| out | `/vla/plan` | `nav_msgs/Path` | 8 poses in `odom`, empty means retract |
| out | `/vla/route` | `nav_msgs/Path` | walker only: the planned route, for RViz |
| out | `/cmd_vel_walker` | `geometry_msgs/Twist` | walker only: turn-first, `twist_mux` priority 20 |
| out | `/vla/status`, `/vla/arbiter_status`, `/vla/walker_status` | `std_msgs/String` | 1 Hz |
| out | `follow_path` | `nav2_msgs/FollowPath` | action client |
| out | `compute_path_to_pose` | `nav2_msgs/ComputePathToPose` | walker only: action client |

A **non-finite position** on `/vla/goal_pose` or `/vla/route_goal` means
"clear". Without it a stale goal is pursued forever, and a producer that chooses
goals programmatically, such as `exploration/goal_selector`, has no way to stand
the rover down. `/vla/goal_image` still has no equivalent.

### Why not `/goal_pose`

Nav2's `bt_navigator` subscribes to `/goal_pose`. Sharing it would mean one
RViz goal click starts a `NavigateToPose` **and** a VLA run, two controllers
fighting over the same rover. Point RViz's "2D Goal Pose" tool at
`/vla/goal_pose` instead.

### On TF publishing

This package publishes **no transforms**, same as Nav2. The EKF owns
`odom` → `base_link` and the active mapper owns `map` → `odom`.

The path is stamped in **`odom`, never `map`**. `map` → `odom` jumps whenever
AMCL relocalises, and a 2.4 s path that teleports mid-execution makes the
controller lurch. Odometry drift over that horizon is negligible.

## Files

| File | What | ROS? |
|---|---|---|
| `vla_bridge/vla_bridge_node.py` | Parameters, pub/sub, TF, message conversion | yes |
| `vla_bridge/path_arbiter_node.py` | FollowPath action client | yes |
| `vla_bridge/chunk_decoding.py` | `(8,4)` → `Waypoint(x, y, yaw)` | no |
| `vla_bridge/goal_resolution.py` | `Pose2D`, `relative_pose`, `compose_pose`, angle helpers | no |
| `vla_bridge/context_buffer.py` | 6-frame window on a time stride | no |
| `vla_bridge/divergence.py` | When a new path justifies a preemption | no |
| `vla_bridge/policy_client.py` | HTTP and JPEG wire format | no |
| `vla_bridge/subgoal_walker_node.py` | Route goal in, carrot or turn out | yes |
| `vla_bridge/subgoal.py` | Route projection, carrot, turn maths, progress | no |
| `config/vla_bridge.yaml` | Every parameter, commented, with units | |
| `launch/vla_navigation.launch.py` | Server plus both nodes | |

Only the two `*_node.py` files import `rclpy`. Everything else is plain Python
with unit tests.

## Parameters worth knowing

Full list with units in `config/vla_bridge.yaml`.

| Parameter | Default | Why |
|---|---|---|
| `inference_rate` | 5.0 Hz | Half the measured 10.5 Hz ceiling, leaving GPU for the ZED |
| `context_stride` | 0.333 s | **A guess.** Training spacing is undocumented |
| `request_timeout` | 0.30 s | Must stay below `staleness_timeout` |
| `staleness_timeout` | 1.00 s | How long a plan stands after the policy goes quiet |
| `wire_size` | 224 px | Largest resolution the model consumes |
| `divergence_threshold` | 0.15 m | Movement at waypoint 4 that justifies preemption |
| `controller_id` | `FollowVLAPath` | **Must match** `navigation_pkg/config/nav2_params.yaml` |
| `lookahead_distance` | 1.5 m | Carrot distance. Beyond the policy's reach on purpose |
| `turn_first_angle` | 0.90 rad | Above this the walker turns instead of the policy |
| `turn_vx` | 0.10 m/s | Arc, not spin, so `slam_toolbox` still processes scans |
| `goal_tolerance` | 0.25 m | Looser than Nav2's 0.15: the policy approaches approximately |

A wrong `controller_id` is accepted by the action server and then fails with
`INVALID_CONTROLLER`, which is not an obvious symptom.

## Degradation

Every failure produces no path rather than a bad one.

| Situation | Behaviour |
|---|---|
| Policy slow or dead | plan stands until `staleness_timeout`, then retracted once |
| Empty path reaches the arbiter | active FollowPath goal is cancelled |
| No goal, or all three set | no request sent, reason in `/vla/status` |
| TF unavailable | no path, reason in `/vla/status` |
| `controller_server` down | logged, throttled, arbiter keeps running |
| Walker has no route, is stuck, or has arrived | carrot retracted, reason in `/vla/walker_status` |

Past that the roboclaw's `cmd_timeout` watchdog stops the rover.

## Build

```bash
colcon build --packages-select vla_bridge
source install/setup.bash
```

## Run

```bash
# Policy drives at a goal directly. Goals go to /vla/goal_pose.
ros2 launch vla_bridge vla_navigation.launch.py

# Planner routes, walker feeds carrots. Goals go to /vla/route_goal.
ros2 launch vla_bridge vla_navigation.launch.py walker:=true
```

Needs motors, sensors, a map source and `navigation_pkg` already up. See
[`../../../launch_helios.md`](../../../launch_helios.md).

For autonomous exploration, `walker:=true` plus
[`mapping_localization_pkg/exploration`](../../mapping_localization_pkg/README.md#exploring-an-unknown-space),
which chooses the goals.

## Verify

```bash
ros2 topic hz   /vla/plan                # ~5 Hz once a goal is set
ros2 topic echo /vla/status --once       # ok modality=7 policy=95ms warm=True
ros2 topic echo /vla/arbiter_status --once
ros2 topic echo /vla/walker_status --once   # walker:=true only
```

Add a Path display on `/vla/plan` in RViz. **With the motors unplugged**,
confirm the trajectory points where the camera is looking. Frames, image
encoding and action decoding can each be silently wrong, and this shows all
three at once.

## Tests

```bash
python3 -m pytest src/vla_pkg/vla_bridge/test -q
```

96 tests, pure pytest, no ROS context and no policy server. Three are worth
knowing about:

`test_displacements_pass_through_unchanged` pins the cumulative semantics.
Re-integrating dx/dy squares the trajectory's reach.

`test_rotation_is_applied_not_ignored` puts the robot at yaw π/2 with the goal
to the east and asserts it reads as 1 m to the **right**. A dropped or
sign-flipped rotation passes every test where the robot faces +x, then drives
confidently the wrong way.

`test_replanned_route_does_not_look_stalled` pins the fix for a real bug. The
walker's progress monitor originally measured arc travelled along the route, but
routes replan every second from the live pose, so the arc collapsed to zero each
time and the rover was declared stuck after 8 s of flawless driving. Progress is
now measured as distance still to run, which is anchored at the goal and so
survives replanning.

## Limitations

The policy is un-adapted to this rover. See [`../README.md`](../README.md).

The JPEG wire encoding costs roughly 0.2% positional deviation, about 2 mm over
a 1 m chunk. Raise `wire_size` or the encoder quality if that ever matters.

If the policy predicts a very short chunk, its last waypoint can fall inside
the controller's `xy_goal_tolerance` of 0.15 m and `FollowPath` succeeds
immediately. That is the policy asking to stop, but it looks like a stall.

**Terminal overshoot.** The walker correctly declares arrival inside its 0.25 m
tolerance and retracts the carrot, but the rover coasts 0.2 to 0.46 m further
while the bridge finishes its last chunk and the smoother ramps down. It
compounds across waypoints, so it matters more for exploration than for a
single goal.

**`FollowVLAPath` is a byte-identical copy of `FollowPath`.** Three of its
thresholds are wrong for a ~1 m chunk rather than a 10 m planner path, and the
most significant is that `PathFollowCritic` has `threshold_to_consider: 1.5`,
which exceeds the whole chunk, so MPPI's primary progress critic is inactive for
the entire path while `GoalCritic` brakes toward a carrot replaced every 0.2 s.
That is the likely cause of the 20% near-still samples measured on both long
runs. Not yet fixed, because it is only worth tuning once the policy has been
shown to beat the classical stack on the same route.

**`/vla/goal_image` cannot be cleared**, unlike the pose and text goals.

## Related

- [`../README.md`](../README.md)
- [`../omnivla_server/README.md`](../omnivla_server/README.md)
- [`../../navigation_pkg/README.md`](../../navigation_pkg/README.md)
