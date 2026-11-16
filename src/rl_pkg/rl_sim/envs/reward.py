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
        approach: Penalty for closing on something in the arc AHEAD,
            scaled by forward speed. This is the directional half, and the
            only term with a steering gradient: turning so an obstacle
            leaves the forward sector makes it fall, where `clearance`
            cannot tell left from right.

            Without it the policy's obstacle response was scalar. Feeding
            it pure noise instead of a real scan produced an identical
            slowdown, because `min(ranges)` discards direction before the
            reward ever sees it, so "slow down when something is close" was
            all the reward could teach.
        clearance: Penalty as the nearest obstacle approaches `danger`,
            SCALED BY FORWARD SPEED. Proximity alone does not bite: measured
            at the moment of collision the rover was doing a full 0.40 m/s
            with 0.15 m of margin and a 0.17 m tracking error. Charging for
            speed near an obstacle makes squeezing through slowly cheap and
            charging the gap expensive, which is what the gap actually
            needs. Penalising cross-track instead would fight the stale map,
            where leaving the route is the correct response.
            Deliberately small. Its job is to smooth the gradient near
            obstacles, not to forbid them: the collision penalty already
            makes contact catastrophic, and a large clearance charge made the
            cheapest policy in a doorway the one that never entered it.
        jerk: Penalty on change in the RAW action, per unit change. Raw,
            not the rate-limited twist: the limiter already caps the twist's
            step at accel * dt, so measuring there hides most of the
            thrashing and the term cannot shape what the policy does.
        cross_track: Penalty per metre of lateral offset from the route.
            This is Nav2's PathAlignCritic, which nav2_params weights at
            10.0 alongside CostCritic's 8.0: staying on the path and
            avoiding obstacles are competing terms, not alternatives, and
            the obstacle term wins where they genuinely conflict. Kept well
            below `clearance` here for that reason.

            Weighted comparably to `clearance`, because precision is
            worth as much here as caution. The observation carries carrots
            at 0.5, 1.5 and 3.0 m, and the policy had learned to steer by
            the far ones: its cross-track of 0.081 m matches a controller
            following the 1.5 m carrot (0.092 m), not the 0.5 m one
            (0.025 m). A longer lookahead drives faster and smoother and so
            earns more `progress`, right up until it clips a corner.

            Nothing else opposes that, since `progress` pays only for arc
            closed ALONG the route. At -0.05 the term carried 4% of the
            budget and corner cutting still paid better.
        align: Paid per radian of heading error closed toward the route.
            The missing half of `misalign`: without it, turning toward the
            route and standing still score identically, because a rotation
            closes no route arc and so earns nothing. The policy then has no
            reason to prefer turning, and measurably did not.

            Telescoping, so it cannot be farmed: summed over an episode it
            comes to |bearing at the start| - |bearing at the end|, bounded
            by pi whatever path is taken in between.
        misalign: Penalty for driving forward faster than the rover's
            alignment with the route allows. Without it, arcing around a
            corner always beats rotating to face it, because an arc earns
            route progress and a rotation earns none, so the policy learns
            tight high-speed arcs and never turns on the spot.
        strafe: Penalty on lateral action, since mecanum rollers slip far
            more sideways than forward and the real rover is configured to
            drive nose-first. Small, because vy_max of 0.02 already makes
            strafing nearly a no-op; this only stops the policy spending
            action range on it.
    """

    progress: float = 2.00
    success: float = 10.00
    collision: float = -10.00
    time: float = -0.004
    approach: float = -0.40
    clearance: float = -0.15
    cross_track: float = -0.25
    align: float = 1.00
    misalign: float = -0.10
    jerk: float = -0.015
    strafe: float = -0.005


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
#: Range at which the clearance penalty starts, metres. Derived, not
#: chosen: from full speed the rover covers 0.09 m during the 230 ms command
#: latency and another 0.16 m braking at 0.50 m/s^2, and the footprint
#: reaches 0.19 m ahead of the laser. A reading of 0.44 m is therefore the
#: last moment it can still stop. At the old 0.30 the bumper was already
#: 0.11 m out with 0.25 m of stopping distance needed.
DANGER_RANGE: float = 0.55

#: Forward ceiling from navigation_pkg, m/s. Normalises the clearance
#: penalty's speed term, and sets the stopping distance above.
MAX_FORWARD_SPEED: float = 0.40


def step_reward(
    previous_distance: float,
    distance: float,
    nearest_obstacle: float,
    forward_clearance: float,
    forward_speed: float,
    cross_track: float,
    twist: np.ndarray,
    action: np.ndarray,
    previous_action: np.ndarray,
    bearing: float,
    next_bearing: float,
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
        nearest_obstacle: Shortest lidar range this step, in metres, in
            any direction. Side scrapes matter too: the rover is 0.38 m
            wide, so an obstacle beside it is still a collision.
        forward_clearance: Shortest range inside the forward sector.
        forward_speed: Measured forward velocity, m/s. Scales the clearance
            penalty, so standing still next to a wall is free.
        cross_track: Signed lateral offset from the route, metres.
        twist: Twist applied this step, [vx, vy, wz]. Used for nothing but
            reporting; the smoothness terms read the action instead.
        action: Raw policy output this step, each axis in -1..1.
        previous_action: Raw policy output the step before.
        bearing: Signed angle from the rover's nose to the route carrot
            BEFORE this step, radians. That is the observation the policy
            chose this action from.
        next_bearing: The same angle after the step.
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
    if forward_clearance < danger:
        terms["approach"] = (
            weights.approach
            * (1.0 - forward_clearance / danger)
            * max(0.0, forward_speed)
            / MAX_FORWARD_SPEED
        )

    if nearest_obstacle < danger:
        proximity = 1.0 - nearest_obstacle / danger
        # Normalised against the forward ceiling so the weight keeps its
        # meaning: full speed at contact costs exactly `clearance`.
        pace = max(0.0, forward_speed) / MAX_FORWARD_SPEED
        terms["clearance"] = weights.clearance * proximity * pace

    terms["cross_track"] = weights.cross_track * abs(cross_track)

    # Closing heading error pays per radian, exactly as closing distance
    # pays per metre.
    terms["align"] = weights.align * (abs(bearing) - abs(next_bearing))

    # subgoal_walker's own law: forward speed is capped at cos(bearing) of
    # the maximum, so the rover creeps forward only as it comes into line and
    # drives hard only once it faces where it is going. Anything above that
    # cap is what makes an arc tight.
    allowed = max(0.0, float(np.cos(bearing)))
    excess = max(0.0, float(action[0]) - allowed)
    if excess > 0.0:
        terms["misalign"] = weights.misalign * excess

    terms["jerk"] = weights.jerk * float(np.linalg.norm(action - previous_action))
    terms["strafe"] = weights.strafe * abs(float(action[1]))

    if reached:
        terms["success"] = weights.success
    if collided:
        terms["collision"] = weights.collision

    return float(sum(terms.values())), terms
