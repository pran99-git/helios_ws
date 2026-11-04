# vla_bridge

The ROS side of VLA navigation. Feeds the policy server an observation history
and a goal, turns the returned waypoints into a `nav_msgs/Path`, and hands that
to Nav2's existing MPPI controller.

Nothing here imports torch. The policy lives in
[`omnivla_server`](../omnivla_server/README.md).

## Two nodes

```
  ZED RGB ──┐
  goal ─────┼──> vla_bridge_node ──/vla/plan──> path_arbiter_node
  TF ───────┘                                          │
                                                       │ FollowPath
                                                       v
                                        controller_server (MPPI + costmaps)
```

**`vla_bridge_node`** runs the cycle: newest frame into a 6-frame window, goal
resolved into the body frame through TF, one HTTP call, waypoints lifted from
`base_link` into `odom`, path published.

**`path_arbiter_node`** decides when that path is worth acting on. The policy
republishes at 5 Hz, but every preemption throws away MPPI's warm optimiser
state, so a path is only forwarded when it actually disagrees with the one
being followed, or when the active goal has gone stale.

## Interfaces

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/Image` | depth 1, best-effort. RGB only |
| in | `/vla/goal_pose` | `geometry_msgs/PoseStamped` | **not** `/goal_pose`, see below |
| in | `/vla/goal_text` | `std_msgs/String` | empty string clears |
| in | `/vla/goal_image` | `sensor_msgs/Image` | egocentric goal view |
| in | TF `map` → `odom` → `base_link` | | read only |
| out | `/vla/plan` | `nav_msgs/Path` | 8 poses in `odom`, empty means retract |
| out | `/vla/status`, `/vla/arbiter_status` | `std_msgs/String` | 1 Hz |
| out | `follow_path` | `nav2_msgs/FollowPath` | action client |

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
| `controller_id` | `FollowPath` | **Must match** `navigation_pkg/config/nav2_params.yaml` |

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

Past that the roboclaw's `cmd_timeout` watchdog stops the rover.

## Build

```bash
colcon build --packages-select vla_bridge
source install/setup.bash
```

## Run

```bash
ros2 launch vla_bridge vla_navigation.launch.py
```

Needs motors, sensors, a map source and `navigation_pkg` already up. See
[`../../../launch_helios.md`](../../../launch_helios.md).

## Verify

```bash
ros2 topic hz   /vla/plan                # ~5 Hz once a goal is set
ros2 topic echo /vla/status --once       # ok modality=7 policy=95ms warm=True
ros2 topic echo /vla/arbiter_status --once
```

Add a Path display on `/vla/plan` in RViz. **With the motors unplugged**,
confirm the trajectory points where the camera is looking. Frames, image
encoding and action decoding can each be silently wrong, and this shows all
three at once.

## Tests

```bash
python3 -m pytest src/vla_pkg/vla_bridge/test -q
```

Pure pytest, no ROS context and no policy server. Two are worth knowing about:

`test_displacements_pass_through_unchanged` pins the cumulative semantics.
Re-integrating dx/dy squares the trajectory's reach.

`test_rotation_is_applied_not_ignored` puts the robot at yaw π/2 with the goal
to the east and asserts it reads as 1 m to the **right**. A dropped or
sign-flipped rotation passes every test where the robot faces +x, then drives
confidently the wrong way.

## Limitations

The policy is un-adapted to this rover. See [`../README.md`](../README.md).

The JPEG wire encoding costs roughly 0.2% positional deviation, about 2 mm over
a 1 m chunk. Raise `wire_size` or the encoder quality if that ever matters.

If the policy predicts a very short chunk, its last waypoint can fall inside
the controller's `xy_goal_tolerance` of 0.15 m and `FollowPath` succeeds
immediately. That is the policy asking to stop, but it looks like a stall.

## Related

- [`../README.md`](../README.md)
- [`../omnivla_server/README.md`](../omnivla_server/README.md)
- [`../../navigation_pkg/README.md`](../../navigation_pkg/README.md)
