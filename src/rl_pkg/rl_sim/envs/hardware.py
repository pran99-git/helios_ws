"""Models the rover's real command and estimation behaviour.

Every constant here was measured from the bags in `datasets/vla_runs/`, not
guessed. The simulator reproduces the OBSERVATION the policy sees, not the
pipeline that produces it, so none of this reimplements the EKF: it reproduces
the EKF's output characteristics.

The measurements, for the record:

| quantity                        | measured                          |
|---------------------------------|-----------------------------------|
| cmd_vel -> odometry lag, vx     | 230 ms (r = 0.98-1.00, 9 runs)    |
| cmd_vel -> odometry lag, wz     | 300 ms                            |
| dead time to 25% of a step, vx  | 160 ms median, p10 41, p90 263    |
| settled twist noise sd          | 0.0009, 0.0022, 0.0037            |
| steady-state tracking odom/cmd  | 1.04                              |
| EKF impulse spikes              | 0.03-0.06% of samples, to 8.6 m/s |
| spike decay                     | 0.79 per 33 ms sample             |
| map->odom correction            | 12 mm median every 0.27 s         |
| worst map->odom jump            | 6.14 m                            |

Imports nothing from ROS or MuJoCo.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: Settled noise on the fused twist, [vx, vy, wz], m/s and rad/s. Measured
#: 1 s after the last command over 153,931 samples, impulse spikes excluded.
TWIST_NOISE_SD: tuple[float, float, float] = (0.0009, 0.0022, 0.0037)

#: Odometry reports 4% more than commanded at steady state.
TRACKING_SCALE: float = 1.04


@dataclass(frozen=True)
class HardwareModel:
    """Randomisation ranges for one episode's hardware behaviour.

    Ranges rather than point values because the measurements spread widely
    and a policy that relies on one particular latency will not survive the
    others. Each episode draws once from these.

    Attributes:
        delay_steps: Inclusive range of control periods of dead time. The
            measured vx dead time of 41 to 263 ms is 1 to 5 periods at 48 ms.
        twist_noise_sd: Per-axis noise on the reported twist.
        tracking_scale: Range for the odometry's steady-state gain.
        spike_probability: Chance per sample of an EKF impulse starting.
        spike_magnitude: Range of the initial impulse, as a multiple of the
            axis limit. Measured peak was 21x on vx.
        spike_decay: Per-sample decay of an active impulse.
        pose_correction_period: How long one localisation correction
            stays in force, seconds. AMCL publishes a new map->odom about
            every 0.27 s, so the jitter is a step function at that cadence,
            NOT white noise per control step. Redrawing it every 48 ms
            multiplied its high-frequency content 5.6x and taught the
            policy to chase it: the wz command flipped sign three times a
            second and the rover visibly weaved down straight routes.
        pose_jitter_sd: Noise on the pose estimate, metres. The median
            map->odom correction, since AMCL nudges every 0.27 s and the
            error never integrates far.
        pose_jump_probability: Chance per control step of a relocalisation
            jump starting.
        pose_jump_magnitude: Range of that jump, metres. The tail reached
            6.14 m, which is AMCL converging confidently wrong.
        pose_jump_duration: Range of how long a jump persists, seconds.
    """

    delay_steps: tuple[int, int] = (1, 5)
    twist_noise_sd: tuple[float, float, float] = TWIST_NOISE_SD
    tracking_scale: tuple[float, float] = (1.00, 1.08)
    spike_probability: float = 5e-4
    spike_magnitude: tuple[float, float] = (2.0, 21.0)
    spike_decay: float = 0.79
    pose_correction_period: float = 0.27
    pose_jitter_sd: float = 0.012
    pose_jump_probability: float = 2e-4
    pose_jump_magnitude: tuple[float, float] = (0.20, 6.14)
    pose_jump_duration: tuple[float, float] = (0.5, 5.0)

    def __post_init__(self) -> None:
        if self.delay_steps[0] < 0 or self.delay_steps[1] < self.delay_steps[0]:
            raise ValueError("delay_steps must be a non-negative ascending range")
        if not 0.0 <= self.spike_probability <= 1.0:
            raise ValueError("spike_probability must be a probability")
        if not 0.0 < self.spike_decay < 1.0:
            raise ValueError("spike_decay must be in (0, 1)")


class CommandDelay:
    """Holds commands back by a whole number of control periods.

    The rover's 160 to 230 ms of dead time is transport delay through serial,
    the RoboClaw PID and the EKF. A delay line reproduces that; a first-order
    lag does not, because dead time has no response at all for its duration.
    """

    def __init__(self, steps: int, width: int = 3) -> None:
        """Initialise an empty delay line.

        Args:
            steps: Control periods of delay. Zero passes commands straight
                through.
            width: Length of each command vector.

        Raises:
            ValueError: If `steps` is negative.
        """
        if steps < 0:
            raise ValueError("steps must not be negative")
        self._steps = steps
        self._width = width
        self.reset()

    @property
    def steps(self) -> int:
        """Control periods of delay applied."""
        return self._steps

    def reset(self) -> None:
        """Clear the line, as if the rover had been stationary."""
        self._queue = [np.zeros(self._width) for _ in range(self._steps)]

    def push(self, command: np.ndarray) -> np.ndarray:
        """Submit a command and get the one that now takes effect.

        Args:
            command: The new command.

        Returns:
            The command issued `steps` periods ago, or `command` itself when
            `steps` is zero.
        """
        if self._steps == 0:
            return np.asarray(command, dtype=np.float64).copy()
        self._queue.append(np.asarray(command, dtype=np.float64).copy())
        return self._queue.pop(0)


class TwistEstimator:
    """Turns the true body twist into what the EKF would report.

    Three effects, in the order they occur physically: a steady-state gain
    error, additive noise, and the occasional impulse spike that decays away.
    """

    def __init__(
        self,
        model: HardwareModel,
        limits: np.ndarray,
        rng: np.random.Generator,
    ) -> None:
        """Draw this episode's gain and prepare the spike state.

        Args:
            model: Randomisation ranges.
            limits: Per-axis velocity magnitudes, used to scale spikes.
            rng: Source of randomness.
        """
        self._model = model
        self._limits = np.asarray(limits, dtype=np.float64)
        self._rng = rng
        self._scale = rng.uniform(*model.tracking_scale)
        self._spike = np.zeros(3)

    @property
    def scale(self) -> float:
        """This episode's steady-state gain error."""
        return float(self._scale)

    @property
    def spiking(self) -> bool:
        """Whether an impulse is currently decaying."""
        return bool(np.any(np.abs(self._spike) > 1e-6))

    def reset(self) -> None:
        """Clear any active spike and redraw the gain."""
        self._scale = self._rng.uniform(*self._model.tracking_scale)
        self._spike = np.zeros(3)

    def estimate(self, true_twist: np.ndarray) -> np.ndarray:
        """Report the twist for this control step.

        Args:
            true_twist: The simulator's exact [vx, vy, wz].

        Returns:
            The twist as the EKF would publish it.
        """
        m = self._model
        reported = self._scale * np.asarray(true_twist, dtype=np.float64)
        reported = reported + self._rng.normal(0.0, m.twist_noise_sd)

        # An impulse decays geometrically rather than ending abruptly, which
        # is what the bags show: 8.63 -> 7.11 -> 5.92 at 0.79 per sample.
        self._spike *= m.spike_decay
        if self._rng.random() < m.spike_probability:
            axis = self._rng.integers(3)
            sign = 1.0 if self._rng.random() < 0.5 else -1.0
            self._spike[axis] = (
                sign * self._rng.uniform(*m.spike_magnitude) * self._limits[axis]
            )
        return reported + self._spike


class PoseEstimator:
    """Turns the true pose into the localisation estimate.

    NOT a dead-reckoning integrator. AMCL corrects `map -> odom` every 0.27 s
    by about 12 mm, so odometry error never integrates far: the estimate
    tracks truth closely most of the time. What it does do, rarely, is jump
    by up to 6 m and stay wrong for a while. That is the failure mode worth
    training against, and Gaussian noise does not represent it.
    """

    def __init__(self, model: HardwareModel, dt: float, rng: np.random.Generator):
        """Prepare the estimator.

        Args:
            model: Randomisation ranges.
            dt: Control period in seconds.
            rng: Source of randomness.
        """
        self._model = model
        self._dt = dt
        self._rng = rng
        self._offset = np.zeros(2)
        self._jump_left = 0.0
        self._jitter = np.zeros(2)
        self._since_correction = float("inf")

    @property
    def lost(self) -> bool:
        """Whether a relocalisation jump is currently in force."""
        return self._jump_left > 0.0

    @property
    def offset(self) -> np.ndarray:
        """The current pose error being applied, metres."""
        return self._offset.copy()

    def reset(self) -> None:
        """Clear any active jump and force a fresh correction."""
        self._offset = np.zeros(2)
        self._jump_left = 0.0
        self._jitter = np.zeros(2)
        self._since_correction = float("inf")

    def estimate(self, true_pose: np.ndarray) -> np.ndarray:
        """Report the pose for this control step.

        Args:
            true_pose: The simulator's exact [x, y, yaw].

        Returns:
            The pose as localisation would report it. Yaw is passed through:
            the measured corrections are dominated by translation.
        """
        m = self._model
        if self._jump_left > 0.0:
            self._jump_left -= self._dt
            if self._jump_left <= 0.0:
                self._offset = np.zeros(2)
        elif self._rng.random() < m.pose_jump_probability:
            bearing = self._rng.uniform(-np.pi, np.pi)
            size = self._rng.uniform(*m.pose_jump_magnitude)
            self._offset = size * np.array([np.cos(bearing), np.sin(bearing)])
            self._jump_left = self._rng.uniform(*m.pose_jump_duration)

        # One jitter sample per localisation correction, held in between:
        # the real map->odom error is a step function at the correction
        # cadence, and per-step redraws made it white noise the steering
        # loop then chased.
        if self._since_correction >= m.pose_correction_period:
            self._jitter = self._rng.normal(0.0, m.pose_jitter_sd, 2)
            self._since_correction = 0.0
        self._since_correction += self._dt

        reported = np.asarray(true_pose, dtype=np.float64).copy()
        reported[:2] += self._offset + self._jitter
        return reported
