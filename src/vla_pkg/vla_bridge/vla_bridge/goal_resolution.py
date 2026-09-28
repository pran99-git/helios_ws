"""Express a goal in the robot's body frame.

The policy wants its goal as `(x, y, yaw)` in REP-103 body coordinates, which
is what `base_link` already is, so the only work is the change of frame from
whatever common frame the goal and the robot are both known in.
"""

from __future__ import annotations

import math
from typing import NamedTuple


class Pose2D(NamedTuple):
    """A planar pose. Frame is whatever the caller is working in."""

    x: float
    y: float
    yaw: float


def wrap_angle(angle: float) -> float:
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_pose(observer: Pose2D, target: Pose2D) -> Pose2D:
    """Express `target` in `observer`'s body frame.

    Both poses must be in the same frame. Rotating the world-frame offset by
    -observer.yaw gives forward/left components:

        x_body =  dx*cos(yaw) + dy*sin(yaw)
        y_body = -dx*sin(yaw) + dy*cos(yaw)

    Args:
        observer: The robot, in the common frame.
        target: The goal, in the same common frame.

    Returns:
        The goal as `(forward, left, relative heading)`.
    """
    dx = target.x - observer.x
    dy = target.y - observer.y
    cos_yaw = math.cos(observer.yaw)
    sin_yaw = math.sin(observer.yaw)
    return Pose2D(
        x=dx * cos_yaw + dy * sin_yaw,
        y=-dx * sin_yaw + dy * cos_yaw,
        yaw=wrap_angle(target.yaw - observer.yaw),
    )
