# Copyright 2026
#
# Operating-limit policy for the Helios drivetrain. No ROS dependency.
#
# Kept separate from roboclaw_driver.py, which only speaks the wire protocol:
# this is the decision layer that turns what the two controllers report into
# the single ceiling the rover is allowed to command.

from typing import Final, NamedTuple

# Measured on this rover 2026-07-29: both controllers reported qpps 7590.
# That is ~1.48 m/s at the wheel -- 7590 counts/s over 2448 counts-per-rev is
# 3.10 wheel rev/s, times the 0.478 m circumference (2 * pi * 0.076 m radius).
#
# Used only when a live read fails or is non-positive. A wrong value here does
# not scale commanded velocity -- the physical kinematics do that -- but it
# loosens the secondary clamp and makes the startup log misleading, so it is a
# measurement rather than a guess. rover_control's
# drivetrain assumes 45000, which is 5.9x this hardware's real ceiling; do not
# copy that number back here.
DEFAULT_MAX_QPPS: Final[int] = 7590

# How far a reported ceiling may sit from the expected one before it is treated
# as suspect. Wide enough to tolerate a re-gear or a wheel swap, tight enough to
# catch an uncalibrated controller: rover_control's 45000 is 5.9x away, and a
# near-zero reading is far below. Only affects whether the operator is warned.
PLAUSIBLE_QPPS_FACTOR: Final[int] = 4


class QppsLimit(NamedTuple):
    """The resolved speed ceiling and how it was arrived at.

    Attributes:
        ceiling: Counts/sec that both controllers can honour.
        status: Human-readable explanation, for the startup log.
        ok: False when the operator should read the status -- a reading was
            missing or non-positive, the two controllers report different QPPS
            ceilings, or the ceiling is implausibly far from the expected one.
            PID gains are logged elsewhere but never compared here.
    """

    ceiling: int
    status: str
    ok: bool


def resolve_max_qpps(
    left: int | None,
    right: int | None,
    fallback: int = DEFAULT_MAX_QPPS,
) -> QppsLimit:
    """Picks the QPPS ceiling that both controllers can honour.

    Args:
        left: QPPS reported by the left controller, or None if the read failed.
        right: QPPS reported by the right controller, or None if it failed.
        fallback: Ceiling to assume when either reading is unusable. Also the
            reference point for the plausibility check.

    Returns:
        The resolved ceiling, a status message, and whether it is nominal.
    """
    if left is None or right is None or left <= 0 or right <= 0:
        return QppsLimit(
            ceiling=fallback,
            status=(
                f"QPPS unusable (left={left}, right={right}); assuming "
                f"{fallback} counts/s. The ceiling is assumed, not measured -- "
                "calibrate the velocity PID in Motion Studio."
            ),
            ok=False,
        )

    ceiling = min(left, right)
    concerns: list[str] = []

    if left != right:
        concerns.append(
            f"asymmetric QPPS (left={left}, right={right}), locked to the "
            "slower controller so neither side is over-commanded"
        )

    low = fallback // PLAUSIBLE_QPPS_FACTOR
    high = fallback * PLAUSIBLE_QPPS_FACTOR
    if not low <= ceiling <= high:
        concerns.append(
            f"outside the plausible band {low}-{high} around the {fallback} "
            "measured on this rover, so the controllers may be uncalibrated "
            "or re-geared"
        )

    if concerns:
        return QppsLimit(
            ceiling=ceiling,
            status=(
                f"QPPS ceiling {ceiling} counts/s: "
                + "; ".join(concerns)
                + ". Check Motion Studio."
            ),
            ok=False,
        )

    return QppsLimit(
        ceiling=ceiling,
        status=f"QPPS ceiling {ceiling} counts/s, both controllers agree.",
        ok=True,
    )
