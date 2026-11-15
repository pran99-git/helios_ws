"""Tests for the reward shaping."""

from __future__ import annotations

from itertools import pairwise

import numpy as np
import pytest
from envs.reward import DANGER_RANGE, RewardWeights, step_reward

STILL = np.zeros(3)


def _reward(**kwargs: object) -> tuple[float, dict[str, float]]:
    defaults: dict[str, object] = {
        "previous_distance": 2.0,
        "distance": 2.0,
        "nearest_obstacle": 10.0,
        "twist": STILL,
        "action": STILL,
        "previous_action": STILL,
        "bearing": 0.0,
        "next_bearing": 0.0,
        "reached": False,
        "collided": False,
    }
    defaults.update(kwargs)
    return step_reward(**defaults)  # type: ignore[arg-type]


def test_closing_on_the_goal_pays() -> None:
    total, terms = _reward(previous_distance=2.0, distance=1.5)
    assert terms["progress"] == pytest.approx(0.5 * RewardWeights().progress)
    assert total > 0.0


def test_retreating_costs() -> None:
    _, terms = _reward(previous_distance=1.5, distance=2.0)
    assert terms["progress"] == pytest.approx(-0.5 * RewardWeights().progress)


def test_circling_earns_nothing() -> None:
    """Progress is closing distance, so covering ground alone pays zero."""
    _, terms = _reward(previous_distance=2.0, distance=2.0)
    assert terms["progress"] == pytest.approx(0.0)


def test_standing_still_is_charged_for_time() -> None:
    total, _ = _reward()
    assert total < 0.0


def test_success_bonus_applied_only_when_reached() -> None:
    _, terms = _reward(reached=True)
    assert terms["success"] == pytest.approx(RewardWeights().success)
    _, terms = _reward(reached=False)
    assert "success" not in terms


def test_collision_penalty_applied_only_on_contact() -> None:
    _, terms = _reward(collided=True)
    assert terms["collision"] == pytest.approx(RewardWeights().collision)
    _, terms = _reward(collided=False)
    assert "collision" not in terms


def test_collision_penalty_is_flat() -> None:
    """Progress is NOT clawed back; see test_progress_pays_even_on_a_crash."""
    _, terms = _reward(collided=True, previous_distance=9.0, distance=1.0)
    assert terms["collision"] == pytest.approx(RewardWeights().collision)


def _episode(progress: float, seconds: float, outcome: str) -> float:
    """Total return for a journey, using the real weights."""
    w = RewardWeights()
    dt = 0.048
    value = w.progress * progress + w.time * int(seconds / dt)
    if outcome == "success":
        value += w.success
    if outcome == "crash":
        value += w.collision
    return value


def test_progress_pays_even_on_a_crash() -> None:
    """The signal PPO actually learns from, early on.

    Almost every early episode ends in a collision, so if progress is
    refunded at the end the landscape is flat and nothing is learnable. A
    3.5 M step run with the refund in place never once reached a goal and
    its action noise quadrupled.
    """
    assert _episode(3.0, 20.0, "crash") > _episode(0.0, 1.0, "crash")
    assert _episode(3.0, 20.0, "crash") > _episode(1.0, 8.0, "crash")


def test_trying_beats_standing_still() -> None:
    """The ordering the weights exist to produce.

    NOT "a crash costs more than a stall". Enforcing that with collision at
    -20, against a best achievable return of +12.75, made creeping optimal:
    a 5 M step run drove at 0.09 m/s of a possible 0.40 and turned crashes
    into timeouts rather than successes.
    """
    stall = _episode(0.0, 90.0, "none")
    assert _episode(4.0, 20.0, "crash") > stall, "a full attempt must beat refusing"
    assert _episode(0.0, 1.0, "crash") < stall, "but crashing at once must not"


def test_more_progress_always_scores_higher() -> None:
    assert _episode(4.0, 20.0, "crash") > _episode(2.0, 20.0, "crash")
    assert _episode(2.0, 20.0, "crash") > _episode(0.0, 1.0, "crash")


def test_the_expected_value_of_attempting_is_positive_early() -> None:
    """At a 2% success rate the optimum must already be to attempt.

    This is the bootstrap condition: a policy that cannot yet succeed has to
    be paid for getting closer, or it never gathers the experience to learn
    from.
    """
    best = _episode(4.0, 15.0, "success")
    typical_failure = _episode(2.0, 20.0, "crash")
    stall = _episode(0.0, 90.0, "none")
    for rate in (0.02, 0.10):
        assert rate * best + (1 - rate) * typical_failure > stall


def test_collision_costs_less_than_the_best_return() -> None:
    """A penalty larger than anything the task can pay teaches paralysis."""
    w = RewardWeights()
    assert abs(w.collision) < _episode(4.0, 15.0, "success")


def test_reaching_beats_every_alternative() -> None:
    best_failure = max(
        _episode(0.0, 90.0, "none"),
        _episode(3.0, 20.0, "crash"),
        _episode(3.7, 15.0, "crash"),
    )
    assert _episode(3.7, 15.0, "success") > best_failure


def test_clearance_penalty_is_off_beyond_danger_range() -> None:
    _, terms = _reward(nearest_obstacle=DANGER_RANGE + 0.01)
    assert "clearance" not in terms


def test_clearance_penalty_grows_as_the_obstacle_nears() -> None:
    _, far = _reward(nearest_obstacle=DANGER_RANGE * 0.9)
    _, near = _reward(nearest_obstacle=DANGER_RANGE * 0.1)
    assert near["clearance"] < far["clearance"] < 0.0


def test_clearance_penalty_is_continuous_at_the_boundary() -> None:
    _, terms = _reward(nearest_obstacle=DANGER_RANGE - 1e-9)
    assert terms["clearance"] == pytest.approx(0.0, abs=1e-6)


def test_jerk_penalises_changing_the_command() -> None:
    _, steady = _reward(
        action=np.array([0.4, 0, 0]), previous_action=np.array([0.4, 0, 0])
    )
    _, jumpy = _reward(
        action=np.array([0.4, 0, 0]), previous_action=np.array([-0.3, 0, 0])
    )
    assert jumpy["jerk"] < steady["jerk"]
    assert steady["jerk"] == pytest.approx(0.0)


def test_jerk_reads_the_raw_action_not_the_limited_twist() -> None:
    """The limiter caps the twist's step, so measuring there sees almost none
    of the thrashing: 0.062 against 0.376 per step on a trained policy."""
    _, terms = _reward(
        twist=STILL,
        action=np.array([1.0, 0.0, 0.0]),
        previous_action=np.array([-1.0, 0.0, 0.0]),
    )
    assert terms["jerk"] == pytest.approx(2.0 * RewardWeights().jerk)


def test_strafing_costs_more_than_driving_forward() -> None:
    _, forward = _reward(action=np.array([0.5, 0.0, 0.0]))
    _, sideways = _reward(action=np.array([0.0, 0.5, 0.0]))
    assert sideways["strafe"] < forward["strafe"]
    assert forward["strafe"] == pytest.approx(0.0)


def test_total_is_the_sum_of_terms() -> None:
    total, terms = _reward(
        previous_distance=3.0,
        distance=2.0,
        nearest_obstacle=0.2,
        action=np.array([0.2, 0.05, 0.1]),
        reached=True,
    )
    assert total == pytest.approx(sum(terms.values()))


# ---------------------------------------------------------- alignment ------


def test_driving_while_aligned_is_free() -> None:
    _, terms = _reward(bearing=0.0, action=np.array([1.0, 0.0, 0.0]))
    assert "misalign" not in terms


def test_driving_while_misaligned_costs() -> None:
    """The law subgoal_walker uses: speed is capped at cos(bearing)."""
    _, terms = _reward(bearing=np.pi / 2, action=np.array([1.0, 0.0, 0.0]))
    assert terms["misalign"] == pytest.approx(RewardWeights().misalign)


def test_misalignment_cost_grows_with_the_angle() -> None:
    cost = []
    for deg in (20, 50, 80):
        _, terms = _reward(bearing=np.deg2rad(deg), action=np.array([1.0, 0.0, 0.0]))
        cost.append(terms.get("misalign", 0.0))
    assert cost[0] > cost[1] > cost[2]


def test_turning_on_the_spot_is_never_penalised() -> None:
    """Rotating to face the carrot must stay free, or the policy will arc."""
    for deg in (0, 90, 180):
        _, terms = _reward(bearing=np.deg2rad(deg), action=np.array([0.0, 0.0, 1.0]))
        assert "misalign" not in terms


def test_creeping_within_the_cos_law_is_free() -> None:
    """At 60 degrees, cos is 0.5, so half speed is allowed."""
    _, terms = _reward(bearing=np.deg2rad(60), action=np.array([0.49, 0.0, 0.0]))
    assert "misalign" not in terms


# ------------------------------------------------------------- align ------


def test_turning_toward_the_route_pays() -> None:
    """The missing half: a rotation closes no route arc, so without this it
    scores the same as standing still and the policy will not turn."""
    _, terms = _reward(bearing=np.pi / 2, next_bearing=np.pi / 4)
    assert terms["align"] == pytest.approx(RewardWeights().align * np.pi / 4)


def test_turning_away_from_the_route_costs() -> None:
    _, terms = _reward(bearing=np.pi / 4, next_bearing=np.pi / 2)
    assert terms["align"] < 0.0


def test_holding_a_heading_earns_nothing() -> None:
    _, terms = _reward(bearing=1.0, next_bearing=1.0)
    assert terms["align"] == pytest.approx(0.0)


def test_sign_of_the_bearing_does_not_matter() -> None:
    _, left = _reward(bearing=-np.pi / 2, next_bearing=-np.pi / 4)
    _, right = _reward(bearing=np.pi / 2, next_bearing=np.pi / 4)
    assert left["align"] == pytest.approx(right["align"])


def test_align_cannot_be_farmed_by_cycling() -> None:
    """It telescopes, so a loop back to the starting heading nets zero and
    there is no free reward in oscillating."""
    w = RewardWeights()
    path = [0.5, 1.2, 0.3, 0.9, 0.5]
    total = sum(
        _reward(bearing=a, next_bearing=b)[1]["align"] for a, b in pairwise(path)
    )
    assert total == pytest.approx(0.0, abs=1e-9)
    assert w.align > 0.0
