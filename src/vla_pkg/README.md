# vla_pkg

Vision-language-action navigation for Helios. Give the rover a goal as a
sentence, a photo, or a 2D pose, and a learned policy proposes where to drive
next. Nav2 still decides how to get there safely.

This is an umbrella directory, like `perception_pkg`. Only `vla_bridge` is a
ROS package.

## Why there are two processes

OmniVLA pins `torch==2.2.0` and a **git fork of transformers**, and no such
wheel exists for JetPack 7. Those pins cannot coexist with ROS 2 Jazzy on
Python 3.12 in one interpreter.

So the policy runs in its own virtualenv, outside ROS, behind a small HTTP
boundary. That constraint turned out to buy three other things:

| | |
|---|---|
| Domain logic never imports `rclpy` | the policy process physically cannot |
| Backend is swappable | edge today, 7.5B or TensorRT later, same contract |
| Off-board deployment | change one URL, no code change |

## Architecture

```
   goal: /vla/goal_text | /vla/goal_image | /vla/goal_pose
                      │
   ZED RGB 30 Hz ─────┤
   TF map->odom->base_link (read only)
                      v
        ┌─────────────────────────┐
        │  vla_bridge_node        │   ROS, Python 3.12
        │  6-frame window, TF,    │
        │  goal -> body frame     │
        └───────────┬─────────────┘
                    │  HTTP, 6 JPEGs out / 8x4 chunk back
                    v
        ┌─────────────────────────┐
        │  omnivla_server         │   venv, torch, NOT a ROS node
        │  OmniVLA-edge, 108 M    │   loopback or a workstation
        └───────────┬─────────────┘
                    │  waypoints
                    v
        ┌─────────────────────────┐
        │  path_arbiter_node      │   throttles preemption
        └───────────┬─────────────┘
                    │  nav2_msgs/FollowPath
                    v
        Nav2 controller_server (MPPI + costmaps)  ──> /cmd_vel
```

The policy proposes, MPPI disposes. Routing through `FollowPath` rather than
straight to `/cmd_vel` is the whole safety argument: the controller re-optimises
against the live costmap at 20 Hz and rejects trajectories its obstacle critics
dislike, which raw policy waypoints know nothing about.

## What's in here

| Path | What | ROS package? |
|---|---|---|
| `omnivla/` | Vendor submodule, [NHirose/OmniVLA](https://github.com/NHirose/OmniVLA). Never edited | no |
| `omnivla_server/` | Our policy server and backends | no |
| `vla_bridge/` | Our ROS adapter: bridge node, arbiter node, domain modules | yes |

## Measured on the AGX Orin

OmniVLA-edge, JetPack 7.2.1, CUDA 13.2, sm_87. Measured, not estimated.

| | |
|---|---|
| Policy parameters | 108.31 M |
| Inference | 95 ms, **10.5 Hz** |
| of which CLIP text encode | 14 ms, cached per goal |
| Model load | 5.0 s, once at server start |
| Checkpoint | 415 MB |
| Default inference rate | 5 Hz, half the ceiling, leaving GPU for the ZED |

The policy was designed for 3 Hz, so there is real headroom.

## Measured under load, 2026-10-29

The worry was that a 108 M policy and the ZED would fight over the GPU. They
do not. Staged on the bench, wheels up:

| Load | Policy latency | GPU | RAM | new EKF misses |
|---|---|---|---|---|
| idle GPU | 93.7 ms / 10.7 Hz | - | - | - |
| ZED + point cloud | 93.4 ms / 10.7 Hz | 91% | 6.38 GB | - |
| + lidar, EKF, AMCL, Nav2 (controller idle) | 98.2 ms / 10.2 Hz | 98-99% | 6.74 GB | 0 |
| **+ MPPI actively optimising** | **100.2 ms / 10.0 Hz** | 25% | 6.79 GB | **0** |

**7% penalty from idle to full load, still 2x the 5 Hz the bridge asks for.**
No new `ekf_filter_node` rate misses at any stage, and `controller_server`
held 19.992 Hz with 0.6 ms std dev while the policy ran beside it.

The last row matters: MPPI only optimises while following a path, so it was
driven with a synthetic 20 m path sent to the `follow_path` action, the same
route `path_arbiter_node` uses. Without that the largest CPU consumer in the
stack is idle and the number flatters itself.

> **The camera costs more than the policy.** The point cloud added for
> low-obstacle costmapping takes the GPU from 0% to 65% on its own. The policy
> adds less than the ZED already does.

Incidental confirmation of the nose-first MPPI retune: with the optimiser
tracking, the commanded twist was `vx 0.0006, vy -0.0017, wz -0.094`. Yaw
dominates strafe by roughly 50x, as intended.

## Goal modalities

The checkpoint defines a fixed set. Combinations outside it are refused with
HTTP 422 rather than silently guessed.

| pose | image | language | id |
|:---:|:---:|:---:|---|
| ✓ | | | 4 |
| | ✓ | | 6 |
| | | ✓ | 7 |
| ✓ | ✓ | | 5 |
| ✓ | | ✓ | 8 |
| | ✓ | ✓ | 9 |

All three at once, or none, is not a valid combination.

## Setup

The venv, the checkpoint and the submodule ignore marker are all per-machine.
None of them are in git.

```bash
# 1. submodule, and keep colcon out of it
git submodule update --init src/vla_pkg/omnivla
touch src/vla_pkg/omnivla/COLCON_IGNORE
echo "COLCON_IGNORE" >> .git/modules/src/vla_pkg/omnivla/info/exclude

# 2. virtualenv (torch for CUDA 13.2 / aarch64 / cp312 / sm_87)
python3 -m venv .venvs/omnivla_edge
source .venvs/omnivla_edge/bin/activate && unset PYTHONPATH
pip install -r src/vla_pkg/omnivla_server/requirements.txt
python -c "import torch; print(torch.cuda.get_device_capability())"   # (8, 7)

# 3. checkpoint (415 MB, needs git-lfs)
git clone https://huggingface.co/NHirose/omnivla-edge models/omnivla-edge

# 4. build
colcon build --packages-select vla_bridge
```

`colcon list` must still show exactly one new package. If it tries to build
OmniVLA, the `COLCON_IGNORE` in step 1 is missing.

## Build

```bash
colcon build --packages-select vla_bridge
source install/setup.bash
```

The submodule and `omnivla_server/` carry `COLCON_IGNORE`; neither is built.

## Run

See [`../../launch_helios.md`](../../launch_helios.md) for the full stack.
Short version, once motors, sensors, AMCL and Nav2 are up:

```bash
ros2 launch vla_bridge vla_navigation.launch.py
ros2 topic pub --once /vla/goal_text std_msgs/msg/String "{data: 'the blue chair'}"
```

## Limitations

**The edge checkpoint is frozen.** The vendor ships no training script for it,
only inference. It carries the nonholonomic prior of its training mix, so it
will not strafe and knows nothing about the 0.40 m/s cap. Fine-tuning requires
the 7.5B model, LoRA, and a 48 GB GPU.

**Frozen means un-adapted.** It has never seen this rover, this camera height,
or the ZED 2i's field of view.

**`context_stride` is a guess.** The checkpoint does not pin it and the vendor
demo feeds six identical frames, so the spacing used in training is unknown.

## Related

- [`vla_bridge/README.md`](vla_bridge/README.md), the ROS side
- [`omnivla_server/README.md`](omnivla_server/README.md), the policy side
- [`../navigation_pkg/README.md`](../navigation_pkg/README.md), the MPPI controller this feeds
- [OmniVLA paper](https://arxiv.org/abs/2509.19480)
