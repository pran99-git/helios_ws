# Copyright 2026
#
# Mecanum wheel mixing for the Helios A4WD3 rover. No ROS dependency -- this
# is the sign convention on its own, so it can be tested without a node.
#
# X-roller configuration. The convention here is the exact inverse of the
# forward kinematics in perception_pkg's wheel_odometry_node, so a corner that
# reads positive when driven forward is also commanded positive:
#
#   fl = x - y - rot      fr = x + y + rot
#   rl = x + y - rot      rr = x - y + rot
#
# mix_wheels takes already-scaled body components, NOT raw velocities: the
# caller applies any geometry scaling first -- for physical units that means
# passing L * wz as `rot`, where L = lx + ly = (wheelbase + track_width) / 2.
# The split keeps mix_wheels unit-agnostic, so a caller working in some other
# unit can reuse the same table; body_twist_to_wheel_counts is the SI-to-counts
# wrapper the driver actually uses.
#
# This is the one place the INVERSE table lives, but not the only place the
# convention appears: wheel_monitor.py writes out the forward form inline for
# its calibration read-out, and it has to stay the algebraic inverse of the
# table above. Change one and check the other.

import math
from collections.abc import Mapping
from typing import Final

CORNERS: Final[tuple[str, str, str, str]] = (
    "front_left",
    "front_right",
    "rear_left",
    "rear_right",
)


def mix_wheels(x: float, y: float, rot: float) -> dict[str, float]:
    """Mixes scaled body components into per-corner wheel commands.

    Args:
        x: Forward component, already scaled by the caller.
        y: Left-strafe component, already scaled by the caller.
        rot: Rotation component, already scaled by the caller. In physical
            units this is L * wz, not wz.

    Returns:
        Per-corner commands keyed by the names in `CORNERS`, in whatever unit
        the caller supplied.
    """
    return {
        "front_left": x - y - rot,
        "front_right": x + y + rot,
        "rear_left": x + y - rot,
        "rear_right": x - y + rot,
    }


def validate_geometry(wheel_radius: float, counts_per_rev: float) -> None:
    """Rejects geometry that would make the counts/sec conversion meaningless.

    Args:
        wheel_radius: Wheel radius, m.
        counts_per_rev: Encoder counts per wheel revolution, gearing included.

    Raises:
        ValueError: If either value is not positive.
    """
    if wheel_radius <= 0.0 or counts_per_rev <= 0.0:
        raise ValueError(
            "wheel_radius and counts_per_rev must be positive, got "
            f"{wheel_radius} and {counts_per_rev}"
        )


def body_twist_to_wheel_counts(
    vx: float,
    vy: float,
    wz: float,
    *,
    wheel_radius: float,
    counts_per_rev: float,
    l_sum: float,
) -> dict[str, float]:
    """Converts a body twist in SI units to per-wheel encoder counts/sec.

    This is what makes closed-loop control possible: the RoboClaw's velocity
    PID regulates counts/sec, so the commanded twist has to be expressed in
    the encoder's own units rather than as a duty fraction.

    Args:
        vx: Forward velocity, m/s.
        vy: Left-strafe velocity, m/s.
        wz: Yaw rate, rad/s, positive counter-clockwise.
        wheel_radius: Wheel radius, m.
        counts_per_rev: Encoder counts per WHEEL revolution, gearing included.
        l_sum: lx + ly = (wheelbase + track_width) / 2, m.

    Returns:
        Per-corner speeds in counts/sec, unrounded. The caller rounds and
        bounds them to the wire width.

    Raises:
        ValueError: If wheel_radius or counts_per_rev is not positive.
    """
    validate_geometry(wheel_radius, counts_per_rev)

    # Reciprocal of wheel_odometry_node's meters_per_count, by construction:
    # the command path and the odometry path must agree on scale.
    counts_per_meter = counts_per_rev / (math.tau * wheel_radius)
    wheels = mix_wheels(vx, vy, l_sum * wz)
    return {corner: speed * counts_per_meter for corner, speed in wheels.items()}


def scale_to_ceiling(wheels: Mapping[str, float], ceiling: float) -> dict[str, float]:
    """Scales every wheel down together if any one exceeds the ceiling.

    Uniform scaling preserves the wheel-speed ratios, and those ratios ARE the
    commanded direction. Clipping wheels independently would keep the rover
    moving but along the wrong heading, with the saturated wheels fighting the
    others -- which is scrubbing, the thing this drivetrain is trying to stop.

    Args:
        wheels: Per-corner speeds in counts/sec.
        ceiling: Largest magnitude any single wheel may command. Must be
            positive -- a negative ceiling would scale by a negative factor
            and reverse every wheel.

    Returns:
        The wheels unchanged if all fit, otherwise all scaled by one factor.

    Raises:
        ValueError: If ceiling is not positive.
    """
    if ceiling <= 0.0:
        raise ValueError(f"ceiling must be positive, got {ceiling}")

    # An all-zero command has peak 0, which takes the pass-through branch, so
    # the division below never sees a zero denominator.
    peak = max(abs(speed) for speed in wheels.values())
    if peak <= ceiling:
        return dict(wheels)
    factor = ceiling / peak
    return {corner: speed * factor for corner, speed in wheels.items()}


def apply_inversions(
    wheels: Mapping[str, float], inversions: Mapping[str, bool]
) -> dict[str, float]:
    """Flips the sign of any corner whose wiring runs backwards.

    Args:
        wheels: Per-corner commands, as returned by `mix_wheels`.
        inversions: Per-corner inversion flags, keyed the same way.

    Returns:
        A new mapping with inverted corners negated.
    """
    return {c: (-v if inversions[c] else v) for c, v in wheels.items()}
