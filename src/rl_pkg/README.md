# rl_pkg

Reinforcement learning for the Helios rover. Train a driving policy in MuJoCo,
then deploy it behind the same `/cmd_vel` interface everything else on this
robot already uses.

This is an umbrella directory, like `perception_pkg` and `vla_pkg`. Only
`rl_bridge` is a ROS package.

## Why there are two halves

MuJoCo 4.0 pulls `numpy 2.5.4`, and the training stack wants `torch` and
`gymnasium` beside it. None of that belongs in the interpreter that runs ROS 2
Jazzy nodes, so the simulation and training code lives in its own virtualenv
outside ROS, exactly as `omnivla_server` does.

The policy crosses back as an **ONNX file**, not a live process. A driving
policy is a small MLP, so `onnxruntime` runs it inside the ROS node in well
under a millisecond. No HTTP boundary is needed, unlike OmniVLA.

## Architecture

```
   TRAINING (no ROS, .venvs/rl_train)
   ┌──────────────────────────────────────────┐
   │  rl_sim/                                 │
   │    assets/helios.xml   the rover         │
   │    envs/               gymnasium env     │
   │    train/              PPO              │
   │    eval/               scenario suite    │
   └───────────────────┬──────────────────────┘
                       │  policy.onnx
                       v
   DEPLOYMENT (ROS 2 Jazzy)
   ┌──────────────────────────────────────────┐
   │  rl_bridge/                              │
   │    rl_policy_node    /scan + goal in     │
   │                      /cmd_vel_rl out     │
   └───────────────────┬──────────────────────┘
                       │
             twist_mux ─┴─► /cmd_vel ─► roboclaw_driver
```

`/cmd_vel_rl` enters `twist_mux` **below** `cmd_vel_joy`, so the joystick stays
a hard override and the heartbeat e-stop keeps working unchanged.

## What's in here

| Path | What | ROS package? |
|---|---|---|
| `rl_sim/` | Simulation, environment, training, evaluation | no |
| `rl_bridge/` | The ROS adapter that drives the rover | yes |

## The physics backend: CPU, not GPU

Benchmarked on the AGX Orin with the rover model, 4 ms timestep:

| Backend | Config | Throughput |
|---|---|---|
| **MuJoCo CPU** | **12 threads** | **2,966,925 steps/s** |
| MuJoCo CPU | 1 thread | 198,615 steps/s |
| `mujoco-warp` | `cuda:0`, 4096 worlds | 22,745 steps/s |

The GPU loses by a wide margin and the reason is structural, not a
misconfiguration: batched GPU physics pays off when each world is expensive,
and this rover is 3 degrees of freedom with one colliding box. The Orin's
integrated GPU also shares memory bandwidth with the CPU.

`mujoco-warp` stays installed and importable for when the scene gets much
heavier or a discrete GPU appears. It is not the default.

## Train in your own map

A scenario can take a saved `slam_toolbox` map instead of generated obstacles,
so the policy learns in the room the rover actually drives in. Pass the
`map_server` `.yaml` as the scenario name:

```bash
env -u PYTHONPATH ../../../.venvs/rl_train/bin/python train/ppo.py \
    --name lab --timesteps 200000
# with run.scenario in the config set to an absolute path ending in .yaml
```

Occupied cells are merged into as few boxes as possible, and those go straight
onto the worldbody rather than each into its own body:

| | Lab map, 202x271 at 5 cm |
|---|---|
| Occupied cells | 23,341 |
| Merged rectangles | **698** |
| Geoms in the model | 707 |
| Rollout | 956 control steps/s |
| **Training, end to end** | **1,187 fps**, vs 1,430 synthetic |

**A real map costs only 17% of training speed**, even though its rollout rate
is 5x lower, because gradient updates dominate the loop.

Two findings behind those numbers:

- **A body per rectangle costs 19x.** With each wall box in its own body,
  `mj_kinematics` walks all 698 every step: 3,500 steps/s against 67,290 with
  static worldbody geoms. A static geom needs no body.
- **Heightfields lose, despite being one geom.** `mjGEOM_HFIELD` steps 12,549/s
  but raycasts at only 104 scans/s against the boxes' 1,732. Since the lidar
  fires once per control step, that nets 94 control steps/s versus 1,324.

Unknown cells are sealed by default, so a policy cannot escape into unmapped
space. Counterintuitively that is also *cheaper*: unknown regions are large
contiguous blobs, so sealing the lab map gives 698 rectangles where leaving it
open gives 840.

Starts and goals are drawn from cells with real footprint clearance, computed
with the same `distance_transform_edt` approach `exploration/` uses. Verified
over 200 resets with none starting in collision.

## Training throughput

Physics is not what limits training. Measured end to end:

| Stage | Rate | 5 M steps |
|---|---|---|
| Env stepping only, 1 env | 4,780 control steps/s | 17 min |
| Env stepping only, 12 workers | 7,409 control steps/s | 11 min |
| PPO end to end, cpu | 552 fps | 2.5 hours |
| PPO end to end, cuda, no realism | 1,312 fps | 64 min |
| **PPO end to end, full realism** | **803 fps** | **104 min** |

Two results worth keeping in mind:

- **12 workers beat 1 env by only 1.5x.** `SubprocVecEnv` pickles every
  observation across a pipe, so per-step IPC dominates, not physics. An
  in-process batched env is the fix if it ever matters.
- **The policy trains on the GPU, unlike the physics.** The usual advice to
  keep a small MLP on the CPU assumes a discrete card across PCIe. Here memory
  is unified, so the copy is nearly free and 12 Cortex-A78AE cores lose to the
  iGPU on batched matmuls.

## Route follower, not goal seeker

The policy is trained to follow a **planned route**, not to steer at a distant
goal. That matches how deployed systems are built, from Waymo to warehouse
AMRs: a planner finds the route over a map, and the learned part handles local
driving. It is also what this repo already concluded for the VLA, in
`feat(vla_pkg): walk a global route instead of a distant goal`.

The reason is not stylistic. A reactive policy steering at a straight-line
goal bearing drives into any dead end whose mouth faces the goal, and it
cannot learn otherwise, because "this is a dead end" is nowhere in its
observation.

Measured on a U-shaped trap, driving from the mouth toward the back wall:

| Rover moves | Straight-line distance left | Route arc left |
|---|---|---|
| x = -0.5 m | 2.50 m | 5.97 m |
| x = +0.6 m | **1.40 m** | **5.97 m** |

Straight-line progress pays **+1.10** for driving into the trap. Route progress
pays **zero**. That is the whole argument.

`envs/planner.py` supplies the route in sim, by Dijkstra over the occupancy
grid inflated by the footprint radius, standing in for Nav2's Smac planner.
About 6 ms per episode on the lab map, and routing costs 2.5% of throughput
(932 against 956 control steps/s).

### Borrowed geometry, not copied

`envs/route.py` imports `project`, `remaining`, `point_at_arc` and `carrot`
from **`vla_bridge/subgoal.py`** rather than reimplementing them. Both that
module and `goal_resolution.py` beside it are ROS-free by design, so they run
unchanged in the training venv.

One definition of route geometry serves the robot and the simulator, which
matters because the RL policy follows the same routes `subgoal_walker_node`
walks. The cost is that training depends on a ROS package's source layout.
That coupling lives only in `envs/route.py`, and `test_route.py` fails loudly
if the path moves or ROS leaks into those files.

## Hardware realism, all of it measured

Nothing in `envs/hardware.py` is a guess. Every constant came out of the 11
bags in `datasets/vla_runs/`:

| Quantity | Measured | How |
|---|---|---|
| `cmd_vel` to `odometry` lag, `vx` | **230 ms** | cross-correlation, r = 0.98-1.00, 9 runs |
| Same, `wz` | 300 ms | as above |
| Dead time to 25% of a step, `vx` | 160 ms median, p10 41, p90 263 | n = 38 step events |
| Settled twist noise sd | 0.0009, 0.0022, 0.0037 | n = 153,931 samples at rest |
| Steady-state tracking | odom/cmd = 1.04 | plateau comparison |
| EKF impulse spikes | to **8.63 m/s** on a 0.40 m/s rover | 3 of 11 runs, 0.03-0.06% of samples |
| Spike decay | 0.79 per 33 ms sample | 8.63 → 7.11 → 5.92 … |
| `map→odom` correction | 12 mm median every 0.27 s | 2,029 jumps |
| Worst `map→odom` jump | **6.14 m** | AMCL converging confidently wrong |

The simulator reproduces the **observation**, never the pipeline. There is no
EKF in sim: `TwistEstimator` reproduces its output characteristics instead,
which is one stage of calibrated modelling rather than three stages of
guessing at encoder slip and VIO tracking loss.

Latency is a **delay line, not a first-order lag**, because dead time has no
response at all for its duration. Each episode redraws it from 1 to 5 control
periods, since a policy tuned to one latency will not survive the measured
spread.

### Reward on truth, observation on the estimate

The policy sees the pose localisation would report, including its rare 6 m
jumps, but is rewarded for where it actually went. Rewarding the estimate
would teach it to exploit localisation error instead of driving well.

### Depth fused into the scan

Nav2's local costmap uses `observation_sources: scan zed_cloud`, so MPPI
already sees obstacles the laser cannot. A policy given only the laser would
be **blind to a class of obstacle its own baseline handles**, making any A/B
comparison meaningless.

`DepthFan` casts five elevations per azimuth bin from the camera site and
keeps hits in the costmap's own 0.036 to 0.50 m height band, out to 2.5 m,
across the ZED's 110 degrees. Verified against a box 1.2 m ahead:

| Obstacle top | Laser | Depth | Who sees it |
|---|---|---|---|
| 0.06 m | 3.90 m (the far wall) | 0.95 m | **depth only** |
| 0.10 m | 3.90 m | 0.80 m | **depth only** |
| 0.17 m | 3.90 m | 0.80 m | **depth only** |
| 0.25 m | 0.85 m | 0.80 m | both |
| 0.50 m | 0.85 m | 0.80 m | both |

The laser plane sits at 0.176 m, so anything shorter is invisible to it.
Scenarios grow `n_low` obstacles in the 0.06 to 0.15 m band for exactly this
reason, drawn in orange in the viewer.

Cost: throughput falls from 1,187 to **803 fps**, so 5M steps takes about 104
minutes rather than 64.

## The robot model

`rl_sim/assets/helios.xml` is the same rover RViz shows, in two layers, which is
how `helios_description`'s URDF is already organised:

| Layer | Geometry | Cost |
|---|---|---|
| Visual | The CAD meshes, `contype=0` | free, measured |
| Collision | One box, Nav2's 0.38 x 0.38 footprint | the only thing that collides |

**No wheels in the physics.** A mecanum wheel's motion comes from its rollers,
which the URDF does not describe, so four hinges without them would spin and
move nothing. `mecanum_kinematics.py` already converts body twist to wheel
commands downstream of `/cmd_vel`, so the sim takes that twist directly as
`slide x`, `slide y`, `hinge z`. The wheel meshes are decoration and do not
turn.

Every dimension traces back to `helios_description/urdf/`. That package stays
the one source of truth; a re-measurement there has to be copied across.

## Build and setup

`rl_sim` is not built by colcon. It carries `COLCON_IGNORE`.

```bash
cd ~/helios_ws

# 1. the virtualenv
python3 -m venv .venvs/rl_train
.venvs/rl_train/bin/pip install --upgrade pip
.venvs/rl_train/bin/pip install -r src/rl_pkg/rl_sim/requirements.txt

# 2. the meshes. REQUIRED: MuJoCo rejects any STL over 200,000 faces and the
#    CAD exports reach 1.66 M, so assets/helios.xml will not load without this.
cd src/rl_pkg/rl_sim
env -u PYTHONPATH ../../../.venvs/rl_train/bin/python tools/decimate_meshes.py

# 3. check it
env -u PYTHONPATH ../../../.venvs/rl_train/bin/python verify_install.py --bench
```

Step 2 writes `assets/meshes/`, which is **gitignored**. It is derived from
`helios_description`, so it is regenerated rather than tracked. 148.1 MB of CAD
becomes 11.7 MB.

### Always scrub PYTHONPATH

Sourcing a ROS workspace puts `/opt/ros/jazzy/lib/python3.12/site-packages`
**ahead** of the venv on `sys.path`. Nothing overlaps today, so it happens to
work, but that is luck and not isolation. `env -u PYTHONPATH` makes it
deterministic, and `verify_install.py` checks for the problem and says so.

## Look at the model

```bash
cd src/rl_pkg/rl_sim
env -u PYTHONPATH ../../../.venvs/rl_train/bin/python -m mujoco.viewer \
    --mjcf=assets/helios.xml
```

Needs the Jetson's own display, not a plain SSH session. Press `2` and `3` to
toggle the visual and collision layers; the orange box is the footprint, and
the red and blue dots are the lidar and camera mount sites.

To compare against RViz:

```bash
source ~/helios_ws/install/setup.bash
ros2 launch helios_description description.launch.py
```

## Train

```bash
cd src/rl_pkg/rl_sim

# a short run first, to confirm the loop works end to end
env -u PYTHONPATH ../../../.venvs/rl_train/bin/python train/ppo.py \
    --name smoke --timesteps 40000

# the real thing, about an hour
env -u PYTHONPATH ../../../.venvs/rl_train/bin/python train/ppo.py

# watch it, from another terminal
../../../.venvs/rl_train/bin/tensorboard --logdir runs/
```

Everything lands in `runs/<name>_<timestamp>/`, which is gitignored: the exact
config used, periodic checkpoints, the best model by eval reward, tensorboard
logs, and `final.zip`. Ctrl-C saves before exiting.

Hyperparameters live in `train/config/rover_ppo.yaml`, one commented line each.
`envs/reward.py` is the file to iterate on first; on a task this simple the
shaping matters more than the hyperparameters.

## Watch it drive

Training itself cannot be watched: it runs 12 subprocess workers, each with
its own model and episode, so there is no single thing to look at. Watch a
**checkpoint** instead, which can be done while the run is still going since
checkpoints land every 250 k steps.

```bash
cd src/rl_pkg/rl_sim

# on the Jetson's own display
env -u PYTHONPATH ../../../.venvs/rl_train/bin/python eval/play.py \
    runs/<run>/best/best_model.zip --scenario cluttered

# over SSH, where there is no GL context: write PNGs instead
env -u PYTHONPATH MUJOCO_GL=egl ../../../.venvs/rl_train/bin/python \
    eval/play.py runs/<run>/best/best_model.zip --frames /tmp/frames
```

Omit the policy argument to watch random actions, which is a quick way to
confirm a scenario looks right.

The view follows a chase camera on the rover, and the **planned route is drawn
in green with the goal as a magenta sphere**. Without that the rover just
appears to wander, since the thing it is rewarded for following is otherwise
invisible. Obstacle colour encodes which sensor sees it: **red** is tall
enough for the laser, **orange** is low enough that only the depth fan does.

Each episode prints its outcome, route remaining and cross-track error.

| Flag | |
|---|---|
| `--scenario` | Suite name or a map `.yaml`. Defaults to `cluttered` |
| `--speed` | Multiple of real time in the viewer |
| `--stride` | Write every Nth frame; 2 gives about 10 fps |
| `--camera` | Model camera name, or `-1` for a free camera |
| `--episodes` | How many to run |

## Related

- [`low_level_control_pkg`](../low_level_control_pkg/README.md): owns
  `twist_mux`, `/cmd_vel` and the e-stop the policy sits behind
- [`navigation_pkg`](../navigation_pkg/README.md): the velocity limits and
  footprint this model copies, and the baseline any policy has to beat
- [`vla_pkg`](../vla_pkg/README.md): the other learned policy on this robot,
  and the precedent for the venv split
