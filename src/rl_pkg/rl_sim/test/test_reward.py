"""Tests for the reward shaping."""

from __future__ import annotations

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
        "previous_twist": STILL,
        "reached": False,
        "collided": False,
    }
    defaults.update(kwargs)
    return step_reward(**defaults)  # type: ignore[arg-type]


def test_closing_on_the_goal_pays() -> None:
    total, terms = _reward(previous_distance=2.0, distance=1.5)
    assert terms["progress"] == pytest.approx(0.5)
    assert total > 0.0


def test_retreating_costs() -> None:
    _, terms = _reward(previous_distance=1.5, distance=2.0)
    assert terms["progress"] == pytest.approx(-0.5)


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


def test_crashing_still_scores_worse_than_stalling() -> None:
    """What the large collision penalty is for, kept without the refund."""
    stall = _episode(0.0, 90.0, "none")
    assert _episode(3.0, 20.0, "crash") < stall
    assert _episode(0.0, 1.0, "crash") < stall
    assert stall < _episode(3.7, 15.0, "success")


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
        twist=np.array([0.4, 0, 0]), previous_twist=np.array([0.4, 0, 0])
    )
    _, jumpy = _reward(
        twist=np.array([0.4, 0, 0]), previous_twist=np.array([-0.3, 0, 0])
    )
    assert jumpy["jerk"] < steady["jerk"]
    assert steady["jerk"] == pytest.approx(0.0)


def test_strafing_costs_more_than_driving_forward() -> None:
    _, forward = _reward(twist=np.array([0.1, 0.0, 0.0]))
    _, sideways = _reward(twist=np.array([0.0, 0.1, 0.0]))
    assert sideways["strafe"] < forward["strafe"]


def test_total_is_the_sum_of_terms() -> None:
    total, terms = _reward(
        previous_distance=3.0,
        distance=2.0,
        nearest_obstacle=0.2,
        twist=np.array([0.2, 0.05, 0.1]),
        reached=True,
    )
    assert total == pytest.approx(sum(terms.values()))
