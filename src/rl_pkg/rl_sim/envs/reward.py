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
        collision: One-off penalty for hitting something.
        time: Charged every control step, so dawdling costs.
        clearance: Penalty as the nearest obstacle approaches `danger`.
        jerk: Penalty on change in commanded twist, per unit change.
        strafe: Extra penalty on lateral velocity, since mecanum rollers
            slip far more sideways than forward and the real rover is
            configured to drive nose-first.
    """

    progress: float = 1.00
    success: float = 10.00
    collision: float = -10.00
    time: float = -0.01
    clearance: float = -0.50
    jerk: float = -0.02
    strafe: float = -0.05


#: The default coefficients, as a singleton so it is not rebuilt per call.
DEFAULT_WEIGHTS = RewardWeights()

#: Range at which the clearance penalty starts, in metres. Chosen to match
#: the inflation layer's influence rather than the hard footprint.
DANGER_RANGE: float = 0.45


def step_reward(
    previous_distance: float,
    distance: float,
    nearest_obstacle: float,
    twist: np.ndarray,
    previous_twist: np.ndarray,
    reached: bool,
    collided: bool,
    weights: RewardWeights = DEFAULT_WEIGHTS,
    danger: float = DANGER_RANGE,
) -> tuple[float, dict[str, float]]:
    """Score one control step.

    Progress is measured as the REDUCTION in straight-line distance to the
    goal, not distance travelled: a rover circling covers ground without
    getting anywhere. This mirrors the fix already made in vla_bridge's
    progress monitor.

    Args:
        previous_distance: Distance to the goal before the step, in metres.
        distance: Distance to the goal after the step, in metres.
        nearest_obstacle: Shortest lidar range this step, in metres.
        twist: Twist applied this step, [vx, vy, wz].
        previous_twist: Twist applied the step before.
        reached: Whether the goal was reached this step.
        collided: Whether contact occurred this step.
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
        terms["collision"] = weights.collision

    return float(sum(terms.values())), terms
