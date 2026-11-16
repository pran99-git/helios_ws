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

### tracker_v1

Route following, measured over 100 episodes on the saved lab map
(`slam_toolbox_lab_indoor_run_with_VIO_feed`):

| | |
|---|---|
| Success | **94/100** |
| Mean cross-track | 0.057 m |
| Achievable ceiling | 94% |

The ceiling is what a pure-pursuit controller reaches using the same
estimated pose and the 0.5 m carrot. Ground-truth pure pursuit gets 98%,
but nothing driving from the estimate can reach that.

| scenario | success |
|---|---|
| lab map | 94% |
| cluttered | 100% |
| corridor | 96% |

**What it cannot do.** Obstacle avoidance. The planner hands it a
collision-free route and it follows that route; replacing its whole scan
with random noise costs nothing measurable. It is a tracker, not a
navigator, and the fallback controller is not optional behind it.

## Reproducing

Trained with `cross_track` at -0.25, no `approach` term, and staleness off.
Four reward changes after this run cost it 68 points on the lab map, so
check against the table above before trusting anything newer.
