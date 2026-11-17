# policies

Trained policies worth keeping. `runs/` is gitignored and holds every
experiment; this folder holds only the ones that are the best known result
for something, so they survive a disk wipe and a `rm -rf runs/`.

A policy here must never be overwritten by a later run that scores lower.
Add a new name instead.

## What is here

| File | Source run | What it is good at |
|---|---|---|
| `tracker_v1.zip` | `rover_ppo_20261115_234708` | Route tracking |
| `tracker_v1_best.zip` | same, `best/` checkpoint | The eval-selected checkpoint |
| `tracker_v1.onnx` | exported from `tracker_v1.zip` | The same policy, for the robot |

The `.onnx` is the actor alone, with no critic, and is what `rl_bridge` runs.
Regenerate it with `tools/export_onnx.py`, which verifies the export against
the `.zip` before writing it. Measured agreement: 4e-07 on `cluttered` and
5e-07 on the lab map, against a 1e-05 tolerance.

### tracker_v1

Route following on the saved lab map
(`slam_toolbox_lab_indoor_run_with_VIO_feed`), over **500 episodes**:

| | |
|---|---|
| Success | **468/500, 93.6%** |
| 95% interval | 91.1% to 95.4% |
| Mean episode | 396 steps |
| Achievable ceiling | 94% |

The ceiling is what a pure-pursuit controller reaches using the same
estimated pose and the 0.5 m carrot. Ground-truth pure pursuit gets 98%,
but nothing driving from the estimate can reach that.

| scenario | success | episodes |
|---|---|---|
| lab map | 93.6% | 500 |
| cluttered | 100% | 100 |
| corridor | 96% | 100 |

**What it cannot do.** Obstacle avoidance. The planner hands it a
collision-free route and it follows that route; replacing its whole scan
with random noise costs nothing measurable. It is a tracker, not a
navigator, and the fallback controller is not optional behind it.

**It weaves.** 3.83 wz sign reversals/s, against a 1.5 gate. Blinding
observation channels 69 and 72 at inference brings that to 2.43 at no cost
in success, which is why `rl_bridge` does it by default.

## Reproducing

Trained with `cross_track` at -0.25, no `approach` term, and staleness off.

Two later changes each made it worse, both measured on the lab map:

| Change | Lab success | Episodes |
|---|---|---|
| tracker_v1 | 93.6% | 500 |
| plus four reward changes | 26% | 100 |
| plus the pose-jitter cadence fix alone | 77.2% | 500 |

The jitter fix is kept in the simulator regardless, because it is measured
truth about AMCL rather than a tuning choice. It means the next policy has
to re-earn this ground on a more honest simulator, not that this number was
wrong.
