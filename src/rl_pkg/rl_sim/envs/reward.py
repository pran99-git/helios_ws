"""Reward terms for point-goal navigation.

Kept separate from the environment and free of MuJoCo so the shaping can be
tested and changed without touching the simulation loop. Expect this file to
change more often than anything else in rl_sim: reward shaping, not
hyperparameters, is where most of the iteration goes.

Imports nothing from ROS or MuJoCo.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RewardWeights:
    """Coefficients for each reward term.

    Attributes:
        progress: Per metre of reduction in distance to the goal. The main
            driver; every other term is a correction to it.
        success: One-off bonus for reaching the goal.
        collision: BASE penalty for hitting something. The progress earned
            during the episode is forfeited on top of this, so a journey that
            ends in a crash earns nothing for having got close.
        time: Charged every control step, so dawdling costs. Deliberately
            small: a large per-step charge makes ending the episode early a
            way to save money, which rewards crashing.
        clearance: Penalty as the nearest obstacle approaches `danger`.
            Deliberately small. Its job is to smooth the gradient near
            obstacles, not to forbid them: the collision penalty already
            makes contact catastrophic, and a large clearance charge made the
            cheapest policy in a doorway the one that never entered it.
        jerk: Penalty on change in commanded twist, per unit change.
        strafe: Extra penalty on lateral velocity, since mecanum rollers
            slip far more sideways than forward and the real rover is
            configured to drive nose-first.
    """

    progress: float = 1.00
    success: float = 10.00
    collision: float = -20.00
    time: float = -0.004
    clearance: float = -0.10
    jerk: float = -0.02
    strafe: float = -0.05


#: The default coefficients, as a singleton so it is not rebuilt per call.
DEFAULT_WEIGHTS = RewardWeights()

#: Range at which the clearance penalty starts, in metres.
#:
#: 0.30 rather than the inflation layer's 0.45. The footprint's circumscribed
#: radius is 0.269 and a scan alongside a wall reads about 0.19, so 0.45 was
#: charging the rover for being in a corridor at all. With the old -0.50
#: weight a doorway traversal at 0.19 m cost -0.289 per step against progress
#: of +0.02, which made refusing the doorway the cheapest option. Now it is
#: -0.037 per step, about -1.5 for a two-second passage: noticeable, not
#: prohibitive. Measured scans back this up: median valid range is 1.61 m and
#: p5 is 0.50 m, so under 0.30 m really is close.
DANGER_RANGE: float = 0.30


def step_reward(
    previous_distance: float,
    distance: float,
    nearest_obstacle: float,
    twist: np.ndarray,
    previous_twist: np.ndarray,
    reached: bool,
    collided: bool,
    earned_progress: float = 0.0,
    weights: RewardWeights = DEFAULT_WEIGHTS,
    danger: float = DANGER_RANGE,
) -> tuple[float, dict[str, float]]:
    """Score one control step.

    Progress is measured as the REDUCTION in straight-line distance to the
    goal, not distance travelled: a rover circling covers ground without
    getting anywhere. This mirrors the fix already made in vla_bridge's
    progress monitor.

    A collision forfeits the progress earned so far on top of the base
    penalty. Without that, progress banks incrementally and a crash near the
    goal keeps its credit, which made crashing score better than stalling:
    -4.16 against -12.50 for a 10 m goal under the previous weights.

    Args:
        previous_distance: Distance to the goal before the step, in metres.
        distance: Distance to the goal after the step, in metres.
        nearest_obstacle: Shortest lidar range this step, in metres.
        twist: Twist applied this step, [vx, vy, wz].
        previous_twist: Twist applied the step before.
        reached: Whether the goal was reached this step.
        collided: Whether contact occurred this step.
        earned_progress: Total progress reward credited this episode so far,
            forfeited if `collided`. Ignored otherwise.
        weights: Term coefficients.
        danger: Range at which the clearance penalty begins, in metres.

    Returns:
        The total reward, and a breakdown per term for logging.
    """
    terms: dict[str, float] = {
        "progress": weights.progress * (previous_distance - distance),
        "time": weights.time,
    }

    # Linear ramp from zero at `danger` to one at contact, so the penalty
    # grows smoothly rather than switching on at a threshold.
    if nearest_obstacle < danger:
        terms["clearance"] = weights.clearance * (1.0 - nearest_obstacle / danger)

    terms["jerk"] = weights.jerk * float(np.linalg.norm(twist - previous_twist))
    terms["strafe"] = weights.strafe * abs(float(twist[1]))

    if reached:
        terms["success"] = weights.success
    if collided:
        terms["collision"] = weights.collision - max(earned_progress, 0.0)

    return float(sum(terms.values())), terms
