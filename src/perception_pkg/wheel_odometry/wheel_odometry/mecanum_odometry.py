# Copyright 2026
#
# Mecanum forward kinematics and planar pose integration for the Helios A4WD3
# rover. No ROS dependency -- this is the maths on its own, so it can be tested
# without a node.
#
# X-roller configuration. This is the exact inverse of the mixing table in
# low_level_control_pkg's mecanum_kinematics, so a corner commanded positive
# also reads positive here. Change one and check the other.

import math
from collections.abc import Mapping
from typing import Final

CORNERS: Final[tuple[str, str, str, str]] = (
    "front_left",
    "front_right",
    "rear_left",
    "rear_right",
)


def validate_geometry(
    wheelbase: float, track_width: float, counts_per_rev: float
) -> None:
    """Rejects geometry that would make the odometry meaningless.

    Args:
        wheelbase: Front-to-rear wheel center distance, m.
        track_width: Left-to-right wheel center distance, m.
        counts_per_rev: Encoder counts per wheel revolution, gearing included.

    Raises:
        ValueError: If any value is not positive.
    """
    if wheelbase <= 0.0 or track_width <= 0.0 or counts_per_rev <= 0.0:
        raise ValueError(
            "wheelbase, track_width and counts_per_rev must be positive, got "
            f"wheelbase={wheelbase}, track_width={track_width}, "
            f"counts_per_rev={counts_per_rev}"
        )


def meters_per_count(wheel_radius: float, counts_per_rev: float) -> float:
    """Converts one encoder count into wheel travel.

    One revolution advances the wheel by its circumference 2*pi*r, so a single
    count is that divided by the counts in a revolution.

    Args:
        wheel_radius: Wheel radius, m.
        counts_per_rev: Encoder counts per wheel revolution, gearing included.

    Returns:
        Distance travelled per encoder count, m.
    """
    return (2.0 * math.pi * wheel_radius) / counts_per_rev


def lever_arm(wheelbase: float, track_width: float) -> float:
    """Computes the geometric rotation lever arm L = lx + ly.

    With lx = wheelbase/2 and ly = track_width/2, L is the moment arm the
    mecanum rotation term divides by.

    Args:
        wheelbase: Front-to-rear wheel center distance, m.
        track_width: Left-to-right wheel center distance, m.

    Returns:
        The lever arm L, m.
    """
    return 0.5 * (wheelbase + track_width)


def body_displacement(
    wheel_distances: Mapping[str, float], arm: float, yaw_scale: float = 1.0
) -> tuple[float, float, float]:
    """Converts per-wheel travel into a body-frame displacement.

    Mecanum forward kinematics for the X-roller layout, with per-wheel linear
    displacements d_* and L = lx + ly:

        dx_body = (d_fl + d_fr + d_rl + d_rr) / 4
        dy_body = (-d_fl + d_fr + d_rl - d_rr) / 4
        dtheta  = (-d_fl + d_fr - d_rl + d_rr) / (4 * L)

    Args:
        wheel_distances: Per-corner travel since the last cycle, m, keyed by
            the names in `CORNERS`.
        arm: The lever arm L from `lever_arm`, m.
        yaw_scale: Empirical correction on the yaw term only. Rollers slip
            laterally during rotation, so the effective lever arm is shorter
            than the geometric one and 4*L understates dtheta. 1.0 leaves the
            pure geometric model.

    Returns:
        (dx_body, dy_body, dtheta) in m, m and rad.
    """
    d = wheel_distances
    dx = 0.25 * (d["front_left"] + d["front_right"] + d["rear_left"] + d["rear_right"])
    dy = 0.25 * (-d["front_left"] + d["front_right"] + d["rear_left"] - d["rear_right"])
    dtheta = (
        (-d["front_left"] + d["front_right"] - d["rear_left"] + d["rear_right"])
        * yaw_scale
        / (4.0 * arm)
    )
    return dx, dy, dtheta


class PlanarPose:
    """Integrates body-frame displacements into a world-frame planar pose."""

    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """Seeds the pose.

        Args:
            x: Initial world x, m.
            y: Initial world y, m.
            theta: Initial heading, rad.
        """
        self.x = x
        self.y = y
        self.theta = theta

    def integrate(self, dx_body: float, dy_body: float, dtheta: float) -> None:
        """Advances the pose by one body-frame displacement.

        Rotation uses the MIDPOINT heading (theta + dtheta/2) rather than the
        heading at the start of the step. Over a step that both translates and
        turns, the start-of-step heading biases the arc consistently to one
        side, and that bias integrates rather than averaging out.

        Args:
            dx_body: Forward displacement in the body frame, m.
            dy_body: Left displacement in the body frame, m.
            dtheta: Heading change over the step, rad.
        """
        mid = self.theta + 0.5 * dtheta
        self.x += dx_body * math.cos(mid) - dy_body * math.sin(mid)
        self.y += dx_body * math.sin(mid) + dy_body * math.cos(mid)
        # atan2 of the sine/cosine pair rewraps theta into (-pi, pi] without a
        # loop, so the heading cannot drift out of range over a long run.
        self.theta = math.atan2(
            math.sin(self.theta + dtheta), math.cos(self.theta + dtheta)
        )


def yaw_to_quaternion_zw(yaw: float) -> tuple[float, float]:
    """Converts a planar heading into the only two nonzero quaternion terms.

    A rotation of `yaw` about z alone gives x = y = 0, z = sin(yaw/2) and
    w = cos(yaw/2).

    Args:
        yaw: Heading, rad.

    Returns:
        (z, w) of the unit quaternion.
    """
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)
