"""Per-axis acceleration limiting, matching Nav2's velocity_smoother.

On the real rover the acceleration ceiling is imposed in software by
`velocity_smoother`, not by motors fighting inertia. Reproducing it here is
what makes the sim's step response match the hardware's, and it is also why
the model's mass can be approximate: under these limits a 2.4x mass error
moves the velocity step response by only about 10%.

Imports nothing from ROS or MuJoCo.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: Defaults from navigation_pkg/config/nav2_params.yaml, [vx, vy, wz].
DEFAULT_ACCEL: tuple[float, float, float] = (0.80, 0.80, 2.00)
DEFAULT_DECEL: tuple[float, float, float] = (-0.50, -0.50, -1.50)


@dataclass(frozen=True)
class VelocityLimits:
    """The rover's velocity and acceleration envelope.

    Attributes:
        max_velocity: Upper bound per axis, [vx, vy, wz] in m/s and rad/s.
        min_velocity: Lower bound per axis.

            vx has NO reverse. The scan has a 90 degree blind spot aft, so
            reversing is driving blind, and with it available the policy's
            answer to a carrot behind it was to back toward it rather than
            turn: measured commanding -1.00 while the bearing held at 131
            degrees. Denying reverse leaves rotating as the only way to
            reduce bearing, which is the turn-then-drive profile Nav2 gets
            from RotationShimController.

            vy is a fine correction, matching the intent of nav2_params'
            0.10 ceiling rather than its letter. At 0.10 the policy used
            half of it continuously as dither; forcing it to zero changed
            success by one episode in forty.
        accel: Largest positive change per second, per axis.
        decel: Largest negative change per second, per axis. Negative values.
    """

    max_velocity: tuple[float, float, float] = (0.40, 0.02, 0.80)
    min_velocity: tuple[float, float, float] = (0.0, -0.02, -0.80)
    accel: tuple[float, float, float] = DEFAULT_ACCEL
    decel: tuple[float, float, float] = DEFAULT_DECEL

    def __post_init__(self) -> None:
        if any(a <= 0.0 for a in self.accel):
            raise ValueError("accel must be positive")
        if any(d >= 0.0 for d in self.decel):
            raise ValueError("decel must be negative")
        if any(
            hi <= lo
            for hi, lo in zip(self.max_velocity, self.min_velocity, strict=True)
        ):
            raise ValueError("max_velocity must exceed min_velocity on every axis")


class RateLimiter:
    """Clamps how fast a commanded twist may change.

    The clamp is on the SIGNED change, which is what `velocity_smoother`
    does: the step is bounded to `[decel * dt, accel * dt]` regardless of the
    direction of travel. One consequence is worth knowing, because it is not
    symmetric: accelerating backwards from rest is bounded by `decel`, not by
    `accel`, since the velocity change is negative.
    """

    def __init__(self, limits: VelocityLimits, dt: float) -> None:
        """Initialise the limiter.

        Args:
            limits: The velocity and acceleration envelope to enforce.
            dt: Control period in seconds, the interval between commands.

        Raises:
            ValueError: If `dt` is not positive.
        """
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        self._lo = np.asarray(limits.min_velocity, dtype=np.float64)
        self._hi = np.asarray(limits.max_velocity, dtype=np.float64)
        self._up = np.asarray(limits.accel, dtype=np.float64) * dt
        self._down = np.asarray(limits.decel, dtype=np.float64) * dt
        self._current = np.zeros(3, dtype=np.float64)

    @property
    def current(self) -> np.ndarray:
        """The twist most recently emitted, as [vx, vy, wz]."""
        return self._current.copy()

    def reset(self, velocity: np.ndarray | None = None) -> None:
        """Set the held velocity, for the start of an episode.

        Args:
            velocity: Twist to start from. Defaults to stationary.
        """
        self._current = (
            np.zeros(3, dtype=np.float64)
            if velocity is None
            else np.clip(np.asarray(velocity, dtype=np.float64), self._lo, self._hi)
        )

    def step(self, command: np.ndarray) -> np.ndarray:
        """Advance one control period toward `command`.

        Args:
            command: Requested twist [vx, vy, wz]. Clipped to the envelope
                before rate limiting, so an out-of-range request is treated
                as a request for the limit rather than rejected.

        Returns:
            The twist to actually apply, already rate limited.
        """
        target = np.clip(np.asarray(command, dtype=np.float64), self._lo, self._hi)
        self._current = np.clip(
            target, self._current + self._down, self._current + self._up
        )
        return self._current.copy()
