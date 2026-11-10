"""Tests for the subgoal walker's geometry. No ROS context, no GPU."""

from __future__ import annotations

import math

import pytest
from vla_bridge.goal_resolution import Pose2D
from vla_bridge.subgoal import (
    ProgressMonitor,
    carrot,
    heading_error,
    point_at_arc,
    project,
    remaining,
    route_length,
)

#: A 10 m straight route east along y=0, at the planner's 5 cm spacing.
STRAIGHT = [Pose2D(i * 0.05, 0.0, 0.0) for i in range(201)]

#: 5 m east, then 5 m north: the corner case that catches tangent errors.
CORNER = [Pose2D(i * 0.05, 0.0, 0.0) for i in range(101)] + [
    Pose2D(5.0, i * 0.05, 0.0) for i in range(1, 101)
]


def test_route_length_matches_geometry():
    assert route_length(STRAIGHT) == pytest.approx(10.0)
    assert route_length(CORNER) == pytest.approx(10.0)


def test_project_a_robot_sitting_on_the_route():
    p = project(STRAIGHT, 3.0, 0.0)
    assert p.arc == pytest.approx(3.0, abs=1e-6)
    assert p.offset == pytest.approx(0.0, abs=1e-6)


def test_project_a_robot_beside_the_route():
    """Offset must be perpendicular distance, not distance to a vertex."""
    p = project(STRAIGHT, 3.0, 0.4)
    assert p.arc == pytest.approx(3.0, abs=1e-6)
    assert p.offset == pytest.approx(0.4, abs=1e-6)


def test_project_clamps_before_the_start_and_past_the_end():
    assert project(STRAIGHT, -2.0, 0.0).arc == pytest.approx(0.0, abs=1e-6)
    assert project(STRAIGHT, 99.0, 0.0).arc == pytest.approx(10.0, abs=1e-6)


def test_carrot_sits_one_lookahead_ahead():
    c = carrot(STRAIGHT, 2.0, 0.0, lookahead=1.5)
    assert c.x == pytest.approx(3.5, abs=1e-6)
    assert c.y == pytest.approx(0.0, abs=1e-6)


def test_carrot_slides_forward_as_the_robot_advances():
    """The carrot is recomputed from the live pose, so it must not stick."""
    first = carrot(STRAIGHT, 1.0, 0.0, lookahead=1.5).x
    later = carrot(STRAIGHT, 4.0, 0.0, lookahead=1.5).x
    assert later - first == pytest.approx(3.0, abs=1e-6)


def test_carrot_degrades_to_the_goal_near_the_end():
    """Running off the end must give the real goal, not fail."""
    c = carrot(STRAIGHT, 9.5, 0.0, lookahead=1.5)
    assert (c.x, c.y) == pytest.approx((10.0, 0.0), abs=1e-6)


def test_carrot_orientation_follows_the_route_tangent():
    """Past the corner the carrot must face north, not east."""
    before = carrot(CORNER, 2.0, 0.0, lookahead=1.5)
    after = carrot(CORNER, 5.0, 2.0, lookahead=1.5)
    assert before.yaw == pytest.approx(0.0, abs=1e-6)
    assert after.yaw == pytest.approx(math.pi / 2, abs=1e-6)


def test_point_at_arc_interpolates_within_a_segment():
    p = point_at_arc(STRAIGHT, 1.025)
    assert p.x == pytest.approx(1.025, abs=1e-6)


def test_remaining_shrinks_as_the_robot_advances():
    assert remaining(STRAIGHT, 0.0, 0.0) == pytest.approx(10.0, abs=1e-6)
    assert remaining(STRAIGHT, 7.0, 0.0) == pytest.approx(3.0, abs=1e-6)
    assert remaining(STRAIGHT, 99.0, 0.0) == pytest.approx(0.0, abs=1e-6)


def test_heading_error_wraps_the_short_way():
    """Facing just past pi must give a small negative error, not ~2pi."""
    c = Pose2D(0.0, 0.0, -math.pi + 0.1)
    assert heading_error(c, math.pi - 0.1) == pytest.approx(0.2, abs=1e-6)


def test_a_short_route_raises_rather_than_guessing():
    with pytest.raises(ValueError):
        project([Pose2D(0.0, 0.0, 0.0)], 0.0, 0.0)
    with pytest.raises(ValueError):
        point_at_arc([], 1.0)


def test_duplicate_points_do_not_break_projection():
    """SmacPlanner2D can emit repeated poses; they must be skipped, not divided by."""
    route = [Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0)]
    assert project(route, 0.5, 0.0).arc == pytest.approx(0.5, abs=1e-6)


class TestProgressMonitor:
    """Arc-length progress, which is what distinguishes hunting from advancing."""

    def test_advancing_is_never_stuck(self):
        m = ProgressMonitor(min_progress=0.15, window=8.0)
        for i in range(20):
            m.update(arc=i * 0.5, now=i * 1.0)
            assert not m.stuck(now=i * 1.0)

    def test_hunting_in_place_trips_the_window(self):
        """Covering ground without advancing along the route is the failure."""
        m = ProgressMonitor(min_progress=0.15, window=8.0)
        m.update(arc=1.0, now=0.0)
        m.update(arc=1.02, now=5.0)  # 2 cm: below min_progress
        assert not m.stuck(now=5.0)
        assert m.stuck(now=8.5)

    def test_a_late_advance_rearms_the_window(self):
        m = ProgressMonitor(min_progress=0.15, window=8.0)
        m.update(arc=1.0, now=0.0)
        m.update(arc=2.0, now=7.0)
        assert not m.stuck(now=14.0)
        assert m.stuck(now=15.5)

    def test_reset_clears_state(self):
        m = ProgressMonitor(min_progress=0.15, window=8.0)
        m.update(arc=1.0, now=0.0)
        assert m.stuck(now=20.0)
        m.reset()
        assert not m.stuck(now=20.0)

    def test_nonsense_configuration_raises(self):
        with pytest.raises(ValueError):
            ProgressMonitor(min_progress=0.0, window=8.0)
        with pytest.raises(ValueError):
            ProgressMonitor(min_progress=0.15, window=-1.0)


class TestGoalClearingContract:
    """The sentinel the walker and the bridge agree on.

    The bridge keeps its last pose goal indefinitely, so a producer that falls
    silent leaves the policy chasing a carrot already reached. These pin the
    non-finite-position convention both sides implement.
    """

    @staticmethod
    def _is_clear(x: float, y: float) -> bool:
        """Mirror of vla_bridge_node._on_goal_pose's clear test."""
        return not (math.isfinite(x) and math.isfinite(y))

    def test_a_real_pose_is_not_a_clear(self):
        assert not self._is_clear(2.09, 0.24)
        assert not self._is_clear(0.0, 0.0)
        assert not self._is_clear(-11.5, 4.072)

    def test_nan_is_a_clear(self):
        assert self._is_clear(math.nan, math.nan)
        assert self._is_clear(math.nan, 0.0)
        assert self._is_clear(0.0, math.nan)

    def test_infinity_is_also_a_clear(self):
        """An unreachable carrot is as useless as an absent one."""
        assert self._is_clear(math.inf, 0.0)
        assert self._is_clear(0.0, -math.inf)
