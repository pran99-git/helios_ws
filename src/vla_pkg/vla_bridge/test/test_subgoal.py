"""Tests for the subgoal walker's geometry. No ROS context, no GPU."""

from __future__ import annotations

import math

import pytest
from vla_bridge.goal_resolution import Pose2D
from vla_bridge.subgoal import (
    ProgressMonitor,
    bearing_to,
    carrot,
    heading_error,
    point_at_arc,
    project,
    remaining,
    route_length,
    turn_in_place,
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
    """Progress is measured as distance still to run, not distance covered."""

    def test_advancing_is_not_stuck(self):
        m = ProgressMonitor(min_progress=0.15, window=8.0)
        for i in range(20):
            m.update(10.0 - 0.4 * i, float(i))
            assert not m.stuck(float(i))

    def test_standing_still_trips_the_window(self):
        m = ProgressMonitor(min_progress=0.15, window=8.0)
        m.update(10.0, 0.0)
        assert not m.stuck(7.9)
        assert m.stuck(8.0)

    def test_hunting_without_closing_trips(self):
        """Motion that never reduces the remaining distance is not progress."""
        m = ProgressMonitor(min_progress=0.15, window=5.0)
        for i in range(60):
            t = i * 0.2
            m.update(6.0 + 0.05 * math.sin(t * 3.0), t)
        assert m.stuck(12.0)

    def test_replanned_route_does_not_look_stalled(self):
        """The bug this replaced: a route replanned from the live pose restarts
        its arc at zero, so arc-based progress froze while the rover drove."""
        m = ProgressMonitor(min_progress=0.15, window=8.0)
        left = 10.0
        for i in range(100):
            t = i * 0.2
            left -= 0.08  # 0.4 m/s at 5 Hz
            m.update(left, t)
            assert not m.stuck(t), f"declared stuck at t={t:.1f}, {left:.2f} m left"

    def test_reset_clears(self):
        m = ProgressMonitor(min_progress=0.15, window=2.0)
        m.update(5.0, 0.0)
        assert m.stuck(5.0)
        m.reset()
        assert not m.stuck(5.0)

    def test_rejects_bad_config(self):
        with pytest.raises(ValueError):
            ProgressMonitor(min_progress=0.0, window=1.0)
        with pytest.raises(ValueError):
            ProgressMonitor(min_progress=0.1, window=0.0)


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


class TestBearingTo:
    """Which way must the rover turn to point at the carrot.

    This is the quantity the walker needs and the rotation shim never sees:
    the shim aligns to the policy's chunk, which always reads as forward.
    """

    def test_target_straight_ahead_is_zero(self):
        robot = Pose2D(0.0, 0.0, 0.0)
        assert bearing_to(robot, Pose2D(2.0, 0.0, 0.0)) == pytest.approx(0.0, abs=1e-9)

    def test_target_to_the_left_is_positive(self):
        robot = Pose2D(0.0, 0.0, 0.0)
        assert bearing_to(robot, Pose2D(0.0, 2.0, 0.0)) == pytest.approx(math.pi / 2)

    def test_target_behind_is_near_pi(self):
        """The case that broke the 10 m run: carrot at -178 deg."""
        robot = Pose2D(1.33, 11.30, math.radians(60.0))
        carrot = Pose2D(0.34, 10.00, 0.0)
        assert abs(bearing_to(robot, carrot)) > math.radians(150.0)

    def test_it_ignores_the_target_orientation(self):
        """Direction OF the target, not alignment WITH it."""
        robot = Pose2D(0.0, 0.0, 0.0)
        a = bearing_to(robot, Pose2D(2.0, 0.0, 0.0))
        b = bearing_to(robot, Pose2D(2.0, 0.0, math.pi))
        assert a == pytest.approx(b)

    def test_robot_heading_is_subtracted(self):
        """Facing the target gives zero however the world is oriented."""
        for yaw in (0.0, 1.0, -2.5, math.pi):
            robot = Pose2D(0.0, 0.0, yaw)
            target = Pose2D(math.cos(yaw), math.sin(yaw), 0.0)
            assert bearing_to(robot, target) == pytest.approx(0.0, abs=1e-9)


class TestTurnInPlace:
    """Yaw rate that reaches the bearing without sailing past it."""

    RATE, ACCEL, DT = 0.80, 2.00, 0.2

    def test_a_large_error_commands_the_full_rate_eventually(self):
        w = 0.0
        for _ in range(20):
            w = turn_in_place(math.pi, self.RATE, self.ACCEL, w, self.DT)
        assert w == pytest.approx(self.RATE, abs=1e-6)

    def test_it_respects_the_acceleration_limit(self):
        """One step can change the rate by at most accel * dt."""
        w = turn_in_place(math.pi, self.RATE, self.ACCEL, 0.0, self.DT)
        assert w <= self.ACCEL * self.DT + 1e-9

    def test_it_slows_inside_the_stopping_distance(self):
        """At w^2/2a of remaining angle, the rate must already be falling."""
        stopping = self.RATE**2 / (2 * self.ACCEL)  # 0.16 rad at 0.8/2.0
        w = turn_in_place(stopping * 0.5, self.RATE, self.ACCEL, self.RATE, self.DT)
        assert w < self.RATE

    def test_a_tiny_error_commands_a_tiny_rate(self):
        w = turn_in_place(0.01, self.RATE, self.ACCEL, 0.0, self.DT)
        assert abs(w) < 0.25

    def test_sign_follows_the_error(self):
        assert turn_in_place(+1.0, self.RATE, self.ACCEL, 0.0, self.DT) > 0
        assert turn_in_place(-1.0, self.RATE, self.ACCEL, 0.0, self.DT) < 0

    def test_a_full_turn_converges_without_overshoot(self):
        """Integrate a 178 deg turn and check it lands, not oscillates."""
        err, w = math.radians(178.0), 0.0
        for _ in range(400):
            w = turn_in_place(err, self.RATE, self.ACCEL, w, self.DT)
            err -= w * self.DT
            if abs(err) < math.radians(2.0) and abs(w) < 0.05:
                break
        assert abs(err) < math.radians(5.0), f"left {math.degrees(err):.1f} deg"
        assert abs(w) < 0.1, f"still turning at {w:.3f}"

    def test_nonsense_configuration_raises(self):
        with pytest.raises(ValueError):
            turn_in_place(1.0, 0.0, self.ACCEL, 0.0, self.DT)
        with pytest.raises(ValueError):
            turn_in_place(1.0, self.RATE, self.ACCEL, 0.0, 0.0)
