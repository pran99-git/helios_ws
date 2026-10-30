"""Decode the policy's waypoint chunk into planar poses.

The model emits `(N, 4)` rows of `[dx, dy, cos, sin]`. dx/dy are CUMULATIVE
displacements from the robot, already in metres by the time they reach here,
so nothing is integrated. cos/sin are unit-normalised by the model.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from itertools import pairwise
from typing import NamedTuple

#: Columns per waypoint row, set by the checkpoint's learn_angle=True.
ROW_WIDTH = 4


class Waypoint(NamedTuple):
    """A planar pose in the robot body frame (REP-103: x forward, y left)."""

    x: float
    y: float
    yaw: float


def decode_chunk(chunk: Sequence[Sequence[float]]) -> list[Waypoint]:
    """Convert a raw waypoint chunk into body-frame poses.

    Args:
        chunk: `(N, 4)` rows of `[dx, dy, cos, sin]`, dx/dy in metres.

    Returns:
        One `Waypoint` per row, in the same order.

    Raises:
        ValueError: If any row is not `ROW_WIDTH` wide.
    """
    poses = []
    for index, row in enumerate(chunk):
        if len(row) != ROW_WIDTH:
            raise ValueError(
                f"waypoint {index} has {len(row)} columns, expected {ROW_WIDTH}"
            )
        dx, dy, cos_yaw, sin_yaw = row
        poses.append(Waypoint(float(dx), float(dy), math.atan2(sin_yaw, cos_yaw)))
    return poses


#: Nav2's MPPI critics index into the path by integer offset, tuned for
#: SmacPlanner2D's 5 cm resolution. Match it.
DEFAULT_PATH_SPACING = 0.05


def _lerp_angle(a: float, b: float, fraction: float) -> float:
    """Interpolate between two angles the short way round."""
    delta = math.atan2(math.sin(b - a), math.cos(b - a))
    return a + delta * fraction


def densify(
    waypoints: list[Waypoint], spacing: float = DEFAULT_PATH_SPACING
) -> list[Waypoint]:
    """Resample a waypoint chunk to roughly `spacing` metres apart.

    The policy emits 8 waypoints over about a metre, which is far sparser than
    a planner path. Nav2's critics address the path by INDEX, not distance:
    PathFollowCritic reads `offset_from_furthest: 15` and PathAlignCritic 20.
    On an 8-pose path both indices fall off the end, so the critic whose whole
    job is forward progress contributes nothing, the rover sits still, and the
    progress checker aborts the goal with FAILED_TO_MAKE_PROGRESS.

    Resampling changes nothing about where the policy wants to go. It is the
    same curve, sampled at the density the controller was tuned for.

    Args:
        waypoints: Decoded chunk in the body frame, cumulative from the robot.
        spacing: Target distance between output points, metres.

    Returns:
        The path from the robot's origin through every waypoint, at roughly
        `spacing` intervals. Always includes the final waypoint exactly, so
        the goal position is never moved by rounding.

    Raises:
        ValueError: If `spacing` is not positive.
    """
    if spacing <= 0.0:
        raise ValueError("spacing must be positive")
    if not waypoints:
        return []

    # Start at the robot, heading toward the first waypoint, so the path is
    # anchored where the controller expects and is long enough to index into.
    first = waypoints[0]
    polyline = [Waypoint(0.0, 0.0, math.atan2(first.y, first.x)), *waypoints]

    out: list[Waypoint] = [polyline[0]]
    carry = 0.0
    for start, end in pairwise(polyline):
        segment = math.dist((start.x, start.y), (end.x, end.y))
        if segment < 1e-9:
            continue
        travelled = spacing - carry
        while travelled < segment:
            f = travelled / segment
            out.append(
                Waypoint(
                    start.x + (end.x - start.x) * f,
                    start.y + (end.y - start.y) * f,
                    _lerp_angle(start.yaw, end.yaw, f),
                )
            )
            travelled += spacing
        carry = segment - (travelled - spacing)

    if out[-1] != polyline[-1]:
        out.append(polyline[-1])
    return out
