"""Decode the policy's waypoint chunk into planar poses.

The model emits `(N, 4)` rows of `[dx, dy, cos, sin]`. dx/dy are CUMULATIVE
displacements from the robot, already in metres by the time they reach here,
so nothing is integrated. cos/sin are unit-normalised by the model.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
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
