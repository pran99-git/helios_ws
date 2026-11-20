# rl_bridge

The ROS side of `rl_pkg`. Runs the policy trained in `rl_sim` as the rover's
controller: it rebuilds the observation the policy was fitted to from live
sensors, runs the exported actor, and publishes a body twist.

The policy is the primary controller here and Nav2 is the fallback beneath
it, not a layer inside the loop. Nothing in this package hands a path to
`controller_server`.

## Architecture

```
  /scan  (1081 beams)  ─┐
  /odometry/filtered   ─┤                       ┌──────────────┐
  /map  (extent only)  ─┼──►  contract.py  ────►│ tracker.onnx │
  TF map -> base_link  ─┤     73 values         │  73 -> 3     │
  /rl/goal_pose ─► planner_server ─► route ─────┘ onnxruntime  │
                                                 └──────┬──────┘
                                      action in -1..1   │
                                                        v
                                    limits.py   scale + rate limit
                                                        │
                                                        v
     cmd_vel_estop (250) ─┐                       /cmd_vel_rl
     cmd_vel_joy   (100) ─┤                        priority 30
     cmd_vel_rl     (30) ─┼──► twist_mux ──► /cmd_vel ──► roboclaw_driver
     cmd_vel_walker (20) ─┤
     cmd_vel_smoothed(10)─┘      Nav2, the fallback
```

Priority 30 is the whole fallback mechanism. While this node publishes it
outranks Nav2; when it stops, `twist_mux` times its input out after 0.5 s and
Nav2 takes the rover back. The joystick and the e-stop both outrank it.

## The contract, and why it is shared code

A policy is a function of exactly one vector in exactly one order. Feed it a
vector assembled differently and it does not fail, it returns a confident
twist answering a question nobody asked.

So `contract.py` and `limits.py` import no ROS and **the simulator imports
them from here**, through `rl_sim/envs/deploy.py`. Training and deployment
run the same assembly function, and there is no second copy to drift.

| Channel | Values | Content | Normalised by |
|---|---|---|---|
| 0 to 59 | 60 | Scan, 1.0 is open space | `max_range` 10.0 m |
| 60 to 65 | 6 | Route carrots at 0.5 / 1.5 / 3.0 m, body frame | each by its own lookahead |
| 66 | 1 | Route left to run | the map's diagonal |
| 67 to 69 | 3 | Commanded twist, the node's own last output | `max_velocity` |
| 70 to 72 | 3 | Measured twist, from the EKF | `max_velocity` |

The gap between channels 67 to 69 and 70 to 72 is slip, load and latency. A
policy that cannot see it cannot compensate for it.

### Scan handling

The real `/scan` is 1081 beams; the policy takes 60. Three steps, in this
order:

| Step | What it does | Why the order matters |
|---|---|---|
| `clean` | No-return (65.533), NaN and sub-minimum values become 10.0 | The minimum of a bin holding a NaN is not a distance |
| `sector` | Keeps the central 270 degrees | A wider driver would otherwise be rebinned into wrong bearings |
| `downsample` | Min-pool to 60 beams | A mean would average a narrow obstacle away against the space beside it |

### Yaw-feedback blinding

`blind_yaw_feedback` zeroes channels 69 and 72 before inference, and is **on
by default**. The policy's learned response to its own yaw rate is a D-term
through the loop's lag, which sustains a limit cycle at about 1.7 Hz.

| | wz sign reversals/s | success |
|---|---|---|
| All channels live | 3.83 | 10/10 |
| Channels 69, 72 blinded | 2.43 | 10/10 |

## Interfaces

| Direction | Topic | Type |
|---|---|---|
| in | `/scan` | `sensor_msgs/LaserScan` |
| in | `/odometry/filtered` | `nav_msgs/Odometry` |
| in | `/map` | `nav_msgs/OccupancyGrid` (latched, extent only) |
| in | `/rl/goal_pose` | `geometry_msgs/PoseStamped` |
| out | `/cmd_vel_rl` | `geometry_msgs/Twist` |
| out | `/rl/status` | `std_msgs/String`, 1 Hz |
| out | `/rl/route` | `nav_msgs/Path` |
| action | `compute_path_to_pose` | `nav2_msgs/ComputePathToPose` |

A non-finite goal position means "stand down", the same sentinel
`subgoal_walker_node` accepts.

### Parameters worth knowing

Full list with units in `config/rl_bridge.yaml`.

| Parameter | Default | Note |
|---|---|---|
| `policy` | none | Path to an exported `.onnx`. Required. |
| `control_rate` | 20.83 Hz | The simulator's 48 ms period. Changing it changes what channels 67 to 69 mean. |
| `speed_scale` | 1.0 | Leash on the published twist. Scales output only, so the difference appears as slip in the measured channel. |
| `blind_yaw_feedback` | true | See above. |
| `sensor_timeout` | 0.30 s | Older than this holds the rover instead of driving on it. |
| `depth_scan_topic` | empty | See the gap below. |

### When it holds the rover

With a goal active and an input unusable, the node publishes zeros rather
than going quiet, because going quiet would hand a moving rover to whatever
Nav2 last wanted. With no goal it publishes nothing at all, leaving the rest
of the stack untouched.

| State | Published |
|---|---|
| No goal | nothing (one zero on the way out) |
| Waiting for `/map`, scan or odometry stale, no TF, no route | zeros |
| Goal reached | one zero, then nothing |
| Driving | the policy's twist |

Every number entering the observation is checked for finiteness, because this
input outranks Nav2 in twist_mux and only the joystick and the e-stop sit above
it. `clean` guards the scan; `finite` guards the pose, the measured twist, the
route and the command itself.

| Non-finite input | Effect |
|---|---|
| Odometry twist | left to go stale, so the staleness check holds the rover |
| TF pose | treated as no TF, so the rover holds |
| Any route pose | the route is refused |
| Goal position | the stand-down sentinel, so the node drops the goal |
| The policy's own command | logged as an error and the rover holds |

## Deployment pipeline

```
  tracker_v1.zip ──► export_onnx.py ──► tracker_v1.onnx ──► rl_bridge_node
   (SB3, 2 MB)        parity check        (345 kB)           0.05 ms/call
```

1. **Export and verify**, on the training side:

   ```bash
   cd ../rl_sim
   env -u PYTHONPATH ../../../.venvs/rl_train/bin/python \
       tools/export_onnx.py policies/tracker_v1.zip
   ```

   It exports the actor alone, then drives the simulator with the original
   `.zip` and replays every observation through the exported file. Measured
   agreement: 4e-07 on `cluttered`, 5e-07 on the lab map, against a 1e-05
   tolerance. A failure here exits non-zero and says not to deploy.

2. **Bench check** before the wheels turn. Run the stack, then watch
   `/rl/status` and `/cmd_vel_rl` with the rover on a stand.

3. **First live runs** with the joystick in hand as the deadman and
   `speed_scale` below 1.0.

## Known gaps

| Gap | Effect | Status |
|---|---|---|
| No fused depth scan | The policy trained on laser + ZED fused. Nothing publishes a depth-derived `LaserScan`, so it runs laser-only and is blind below the 0.176 m laser plane. | `depth_scan_topic` is ready for one |
| No obstacle avoidance | tracker_v1 follows the planned route and does not dodge. Replacing its whole scan with noise costs it nothing measurable. | The fallback is not optional behind it |
| Residual weave | 2.43 wz sign reversals/s with blinding on, against a 1.5 gate | Unsolved; see the policy notes |
| Replanning is new to it | The simulator plans once per episode, so channel 66 only ever falls. Here a route arrives every second and that value can step. | Raise `replan_period` if it shows |
| Sensing latency | 230 ms measured on hardware against the 1 to 5 step delay trained | Unmeasured live |
| It cannot see an override | While the joystick or e-stop holds the mux, this node keeps publishing and its commanded-twist channel claims a twist the wheels never got. The policy reads that as total slip, and resumes from a command the rover is not at. | A supervisor comparing `/cmd_vel_rl` against `/cmd_vel` would close it |
| No supervisor | Nothing demotes to Nav2 on loss of progress or a scan floor | Not built |

## Build

`onnxruntime` has no apt package or rosdep key on Ubuntu 24.04, so it is a
pip install rather than a `package.xml` dependency.

```bash
pip install --user --break-system-packages onnxruntime

cd ~/helios_ws
colcon build --packages-select rl_bridge --symlink-install
source install/setup.bash
```

## Run

Needs, in order: `roboclaw_driver`, `sensor_fusion` bringup, AMCL
localization, and `navigation_pkg` for `planner_server`. Full sequence in
`launch_helios.md`.

```bash
ros2 launch rl_bridge rl_bridge.launch.py \
    policy:=$HOME/helios_ws/src/rl_pkg/rl_sim/policies/tracker_v1.onnx \
    speed_scale:=0.6

# then send a goal, NOT to /goal_pose
ros2 topic pub --once /rl/goal_pose geometry_msgs/PoseStamped \
    '{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}'
```

Do not send goals to Nav2's `bt_navigator` at the same time. Both would
drive.

## Related

| Package | Relationship |
|---|---|
| `rl_sim` | Trains the policy, and imports `contract.py` and `limits.py` from here |
| `vla_bridge` | Supplies the route geometry both stacks steer by |
| `navigation_pkg` | `planner_server` for routes, and MPPI as the fallback |
| `low_level_control_pkg` | `twist_mux` priorities and the e-stop above this node |
