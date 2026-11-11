"""Tests for frontier scoring and the rejection memory."""

from __future__ import annotations

import math

import pytest
from goal_selection import (
    DEFAULT_REJECTION_HALF_LIFE,
    Candidate,
    RejectionMemory,
    nearest,
    score_candidates,
    wrap_angle,
)


def rank(candidates, x=0.0, y=0.0, yaw=0.0, **kwargs):
    """Score from a robot pose and return the candidates in order."""
    return [s.candidate for s in score_candidates(candidates, x, y, yaw, **kwargs)]


class TestInformationTerm:
    """A wider opening reveals more map, so it should win all else equal."""

    def test_wider_frontier_wins_at_equal_range(self):
        narrow = Candidate(5.0, 0.0, 0.0, edge=0.6)
        wide = Candidate(-5.0, 0.0, math.pi, edge=1.8)
        assert rank([narrow, wide])[0] is wide

    def test_information_saturates(self):
        """Past edge_cap, extra width stops breaking the tie, so the nearer
        frontier wins instead."""
        near = Candidate(2.0, 0.0, 0.0, edge=2.0)
        far = Candidate(9.0, 0.0, 0.0, edge=8.0)
        assert rank([far, near], edge_cap=2.0)[0] is near


class TestTravelTerm:
    """A nearer frontier is cheaper, and the term must actually bite."""

    def test_nearer_wins_at_equal_width(self):
        near = Candidate(1.5, 0.0, 0.0, edge=1.0)
        far = Candidate(8.0, 0.0, 0.0, edge=1.0)
        assert rank([far, near])[0] is near

    def test_travel_saturates_beyond_range_cap(self):
        """Two frontiers both past the cap are separated by width, not range."""
        far = Candidate(20.0, 0.0, 0.0, edge=1.8)
        further = Candidate(40.0, 0.0, 0.0, edge=1.8)
        scores = score_candidates([far, further], 0.0, 0.0, 0.0, range_cap=12.0)
        assert scores[0].score == pytest.approx(scores[1].score)

    def test_distance_is_reported(self):
        scored = score_candidates([Candidate(3.0, 4.0, 0.0, 1.0)], 0.0, 0.0, 0.0)
        assert scored[0].distance == pytest.approx(5.0)


class TestTurningTerm:
    """A frontier behind the rover costs a turn before any progress."""

    def test_frontier_ahead_beats_one_behind(self):
        ahead = Candidate(4.0, 0.0, 0.0, edge=1.0)
        behind = Candidate(-4.0, 0.0, math.pi, edge=1.0)
        assert rank([behind, ahead])[0] is ahead

    def test_bearing_is_relative_to_robot_heading(self):
        """Same frontier, rover turned around: the bearing flips."""
        candidate = [Candidate(4.0, 0.0, 0.0, 1.0)]
        facing = score_candidates(candidate, 0.0, 0.0, 0.0)[0]
        away = score_candidates(candidate, 0.0, 0.0, math.pi)[0]
        assert facing.bearing == pytest.approx(0.0)
        assert abs(away.bearing) == pytest.approx(math.pi)
        assert facing.score > away.score

    def test_turning_does_not_outrank_information(self):
        """Turning is a tiebreaker. A much wider frontier behind still wins."""
        ahead = Candidate(4.0, 0.0, 0.0, edge=0.5)
        behind = Candidate(-4.0, 0.0, math.pi, edge=2.0)
        assert rank([ahead, behind])[0] is behind


class TestSemanticHook:
    """Reserved for step 5; off by default so it cannot change behaviour yet."""

    def test_ignored_at_default_weight(self):
        """Same geometry, different semantic score: identical rank today."""
        plain = Candidate(4.0, 0.0, 0.0, edge=1.0, semantic=0.0)
        tagged = Candidate(4.0, 0.0, 0.0, edge=1.0, semantic=1.0)
        scores = score_candidates([plain, tagged], 0.0, 0.0, 0.0)
        assert scores[0].score == pytest.approx(scores[1].score)

    def test_breaks_ties_when_weighted(self):
        plain = Candidate(4.0, 0.0, 0.0, edge=1.0, semantic=0.0)
        tagged = Candidate(-4.0, 0.0, math.pi, edge=1.0, semantic=1.0)
        order = rank([plain, tagged], weights={"semantic": 2.0})
        assert order[0] is tagged


class TestRejectionMemory:
    """Decaying failure memory, never a hard blacklist."""

    def test_fresh_rejection_is_near_full(self):
        memory = RejectionMemory()
        memory.reject(1.0, 1.0, now=100.0)
        assert memory.penalty(1.0, 1.0, now=100.0) == pytest.approx(1.0)

    def test_halves_over_the_half_life(self):
        memory = RejectionMemory(half_life=60.0)
        memory.reject(0.0, 0.0, now=0.0)
        assert memory.penalty(0.0, 0.0, now=60.0) == pytest.approx(0.5)
        assert memory.penalty(0.0, 0.0, now=120.0) == pytest.approx(0.25)

    def test_falls_off_with_distance(self):
        memory = RejectionMemory(radius=1.0)
        memory.reject(0.0, 0.0, now=0.0)
        near = memory.penalty(0.2, 0.0, now=0.0)
        far = memory.penalty(3.0, 0.0, now=0.0)
        assert near > 0.9
        assert far < 0.01

    def test_bounded_by_one_however_many_failures(self):
        """Worst-of, not sum: an unbounded penalty could never decay away."""
        memory = RejectionMemory()
        for _ in range(20):
            memory.reject(0.0, 0.0, now=0.0)
        assert memory.penalty(0.0, 0.0, now=0.0) <= 1.0

    def test_nearby_rejection_replaces_rather_than_accumulates(self):
        memory = RejectionMemory(radius=1.0)
        memory.reject(0.0, 0.0, now=0.0)
        memory.reject(0.3, 0.0, now=10.0)
        assert len(memory) == 1

    def test_distant_rejections_coexist(self):
        memory = RejectionMemory(radius=1.0)
        memory.reject(0.0, 0.0, now=0.0)
        memory.reject(5.0, 0.0, now=0.0)
        assert len(memory) == 2

    def test_prune_forgets_stale_entries(self):
        memory = RejectionMemory(half_life=10.0)
        memory.reject(0.0, 0.0, now=0.0)
        memory.prune(now=1000.0)
        assert len(memory) == 0

    def test_prune_keeps_live_entries(self):
        memory = RejectionMemory(half_life=10.0)
        memory.reject(0.0, 0.0, now=0.0)
        memory.prune(now=10.0)
        assert len(memory) == 1

    def test_rejected_frontier_loses_to_an_alternative(self):
        memory = RejectionMemory()
        blocked = Candidate(2.0, 0.0, 0.0, edge=2.0)
        other = Candidate(-6.0, 0.0, math.pi, edge=0.8)
        assert rank([blocked, other], rejections=memory, now=0.0)[0] is blocked
        memory.reject(blocked.x, blocked.y, now=0.0)
        assert rank([blocked, other], rejections=memory, now=0.0)[0] is other

    def test_rejection_clears_with_enough_time(self):
        """The point of decay: the best frontier becomes available again."""
        memory = RejectionMemory()
        blocked = Candidate(2.0, 0.0, 0.0, edge=2.0)
        other = Candidate(-6.0, 0.0, math.pi, edge=0.8)
        memory.reject(blocked.x, blocked.y, now=0.0)
        later = DEFAULT_REJECTION_HALF_LIFE * 6
        assert rank([blocked, other], rejections=memory, now=later)[0] is blocked

    def test_rejects_bad_construction(self):
        with pytest.raises(ValueError):
            RejectionMemory(half_life=0.0)
        with pytest.raises(ValueError):
            RejectionMemory(radius=-1.0)


class TestNearest:
    """Used to decide whether a committed target still exists."""

    def test_finds_the_closest(self):
        candidates = [
            Candidate(0.0, 0.0, 0.0, 1.0),
            Candidate(5.0, 0.0, 0.0, 1.0),
            Candidate(5.2, 0.0, 0.0, 1.0),
        ]
        index, distance = nearest(candidates, 5.1, 0.0)
        assert index in (1, 2)
        assert distance == pytest.approx(0.1)

    def test_empty_returns_none(self):
        assert nearest([], 0.0, 0.0) is None


class TestDegenerate:
    """Inputs that must not raise."""

    def test_no_candidates(self):
        assert score_candidates([], 0.0, 0.0, 0.0) == []

    def test_frontier_on_top_of_the_robot(self):
        """Bearing is undefined at zero range; it must not blow up."""
        scored = score_candidates([Candidate(0.0, 0.0, 0.0, 1.0)], 0.0, 0.0, 0.0)
        assert scored[0].bearing == 0.0
        assert math.isfinite(scored[0].score)

    def test_rejects_bad_caps(self):
        with pytest.raises(ValueError):
            score_candidates([], 0.0, 0.0, 0.0, edge_cap=0.0)
        with pytest.raises(ValueError):
            score_candidates([], 0.0, 0.0, 0.0, range_cap=-1.0)

    @pytest.mark.parametrize(
        "angle", [0.0, 0.5, -0.5, math.pi, -math.pi, 3 * math.pi, -3 * math.pi, 10.0]
    )
    def test_wrap_angle_is_equivalent_and_bounded(self, angle):
        """The two properties callers rely on. The sign exactly at +/-pi is a
        floating-point knife edge and is deliberately not asserted."""
        wrapped = wrap_angle(angle)
        assert -math.pi - 1e-9 <= wrapped <= math.pi + 1e-9
        turns = (wrapped - angle) / (2 * math.pi)
        assert turns == pytest.approx(round(turns), abs=1e-9)
