"""Unit tests for the body-frame goal transform. Pure pytest, no ROS."""

import math

import pytest
from vla_bridge.goal_resolution import Pose2D, relative_pose, wrap_angle


def test_goal_ahead_of_an_unrotated_robot():
    goal = relative_pose(Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0))
    assert goal.x == pytest.approx(1.0)
    assert goal.y == pytest.approx(0.0)


def test_positive_y_is_left():
    """A goal north of a robot facing east must read as 1 m to the left."""
    goal = relative_pose(Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 1.0, 0.0))
    assert goal.y == pytest.approx(1.0)


def test_rotation_is_applied_not_ignored():
    """Robot facing north, goal to the east, so the goal is on its right.

    This is the test that catches a dropped or sign-flipped rotation, which is
    the failure that drives the rover confidently the wrong way.
    """
    goal = relative_pose(Pose2D(0.0, 0.0, math.pi / 2), Pose2D(1.0, 0.0, 0.0))
    assert goal.x == pytest.approx(0.0, abs=1e-9)
    assert goal.y == pytest.approx(-1.0)


def test_translation_and_rotation_together():
    goal = relative_pose(Pose2D(2.0, 3.0, math.pi), Pose2D(1.0, 3.0, math.pi))
    assert goal.x == pytest.approx(1.0)
    assert goal.y == pytest.approx(0.0, abs=1e-9)
    assert goal.yaw == pytest.approx(0.0)


def test_relative_heading_wraps():
    goal = relative_pose(Pose2D(0.0, 0.0, -3.0), Pose2D(0.0, 0.0, 3.0))
    assert abs(goal.yaw) <= math.pi
    assert goal.yaw == pytest.approx(wrap_angle(6.0))


@pytest.mark.parametrize("angle", [0.0, math.pi, -math.pi, 7.0, -7.0, 100.0])
def test_wrap_angle_stays_in_range(angle):
    assert -math.pi <= wrap_angle(angle) <= math.pi
