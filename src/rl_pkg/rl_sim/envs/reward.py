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
        progress: Per metre of route closed. The main driver, and weighted
            so that an attempt which ends badly still beats refusing to move.
        success: One-off bonus for reaching the goal.
        collision: Penalty for hitting something. Deliberately smaller than
            the best achievable return rather than larger: see step_reward.
        time: Charged every control step, so dawdling costs. Deliberately
            small: a large per-step charge makes ending the episode early a
            way to save money, which rewards crashing.
        clearance: Penalty as the nearest obstacle approaches `danger`.
            Deliberately small. Its job is to smooth the gradient near
            obstacles, not to forbid them: the collision penalty already
            makes contact catastrophic, and a large clearance charge made the
            cheapest policy in a doorway the one that never entered it.
        jerk: Penalty on change in the RAW action, per unit change. Raw,
            not the rate-limited twist: the limiter already caps the twist's
            step at accel * dt, so measuring there hides most of the
            thrashing and the term cannot shape what the policy does.
        strafe: Penalty on lateral action, since mecanum rollers slip far
            more sideways than forward and the real rover is configured to
            drive nose-first. On the raw action for the same reason as jerk,
            which is also why the weight is far below the old twist-based
            one: the action spans -1..1 where the twist only spans vy_max.
    """

    progress: float = 2.00
    success: float = 10.00
    collision: float = -10.00
    time: float = -0.004
    clearance: float = -0.10
    jerk: float = -0.03
    strafe: float = -0.03


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
    action: np.ndarray,
    previous_action: np.ndarray,
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

    The weights are set so that TRYING BEATS STANDING STILL, which is not
    the same as making collisions maximally expensive. With collision at -20
    against a best achievable return of +12.75, a crash cost 1.6x the most
    the task could ever pay, and the optimal policy for anything short of
    expert was to creep. A 5 M step run did exactly that: it converted
    crashes into timeouts, drove at 0.09 m/s of a possible 0.40, and never
    beat a ten-line hand-written controller.

    So "a crash must cost more than a stall" is NOT enforced any more. On a
    4 m route, driving the whole way and then hitting something scores -3.66
    against -7.50 for refusing to move, and that ordering is deliberate: a
    rover that tries and occasionally bumps is what we want out of training,
    and a rover that never moves is not. The orderings that DO hold are the
    ones that matter:

        reaching the goal          +16.75   beats everything
        crashing at the goal        -3.66   more progress is always better
        stalling the full episode   -7.50
        crashing halfway            -7.66
        crashing immediately       -10.08   worst

    Progress is not clawed back on a collision either. Early in training
    nearly every episode ends in a crash, so refunding it deletes the dense
    signal in exactly the regime that needs it.

    Args:
        previous_distance: Distance to the goal before the step, in metres.
        distance: Distance to the goal after the step, in metres.
        nearest_obstacle: Shortest lidar range this step, in metres.
        twist: Twist applied this step, [vx, vy, wz]. Used for nothing but
            reporting; the smoothness terms read the action instead.
        action: Raw policy output this step, each axis in -1..1.
        previous_action: Raw policy output the step before.
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

    terms["jerk"] = weights.jerk * float(np.linalg.norm(action - previous_action))
    terms["strafe"] = weights.strafe * abs(float(action[1]))

    if reached:
        terms["success"] = weights.success
    if collided:
        terms["collision"] = weights.collision

    return float(sum(terms.values())), terms
