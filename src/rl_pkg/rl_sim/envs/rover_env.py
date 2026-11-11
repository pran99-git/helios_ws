"""Gymnasium environment: drive the Helios rover to a point goal.

The action is a body twist, the same quantity `/cmd_vel` carries, so a trained
policy deploys without reinterpreting its own output. Commands pass through
the same acceleration limiter Nav2's `velocity_smoother` applies on hardware.

Imports nothing from ROS. The only MuJoCo use is the simulation itself.
"""

from __future__ import annotations

from typing import Any, ClassVar

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from envs import scenarios
from envs.lidar import Lidar, LidarSpec
from envs.rate_limit import RateLimiter, VelocityLimits
from envs.reward import RewardWeights, step_reward

#: Physics steps per control step. The model runs at 4 ms, so 12 gives a
#: 48 ms control period, close to the 20 Hz the real controller runs at.
DEFAULT_DECIMATION: int = 12


class RoverEnv(gym.Env[np.ndarray, np.ndarray]):
    """Point-goal navigation with a downsampled laser scan.

    Observation is one flat vector, which is what SB3's MlpPolicy expects:

        [ lidar / max_range          beams values, 1.0 is open space ]
        [ goal range / diagonal      1 value                         ]
        [ sin, cos of goal bearing   2 values, continuous at +-pi    ]
        [ last twist, normalised     3 values                        ]

    Bearing is given as sin and cos rather than an angle, because a raw angle
    jumps between -pi and +pi for an arbitrarily small turn and the policy
    then sees a discontinuity where the world has none.

    Action is [vx, vy, wz] in -1..1, scaled to the rover's real envelope.
    """

    metadata: ClassVar[dict[str, list[str]]] = {"render_modes": []}

    def __init__(
        self,
        scenario: scenarios.Scenario | None = None,
        lidar_spec: LidarSpec | None = None,
        limits: VelocityLimits | None = None,
        weights: RewardWeights | None = None,
        decimation: int = DEFAULT_DECIMATION,
        mass_randomisation: float = 0.3,
        seed: int | None = None,
    ) -> None:
        """Build the environment.

        Args:
            scenario: Task definition. Defaults to an empty 10x10 m room.
            lidar_spec: Scan geometry. Defaults to 60 beams over 270 degrees.
            limits: Velocity envelope. Defaults to the rover's real limits.
            weights: Reward coefficients.
            decimation: Physics steps per control step.
            mass_randomisation: Fractional spread applied to body mass each
                reset. The model's mass is an estimate, so this converts that
                uncertainty into robustness instead of a fixed bias.
            seed: Seed for the episode generator.

        Raises:
            ValueError: If `decimation` is below 1.
        """
        if decimation < 1:
            raise ValueError("decimation must be at least 1")

        self._scenario = scenario or scenarios.Scenario()
        self._spec = lidar_spec or LidarSpec()
        self._limits = limits or VelocityLimits()
        self._weights = weights or RewardWeights()
        self._decimation = decimation
        self._mass_randomisation = mass_randomisation
        self._rng = np.random.default_rng(seed)

        # Compiled once with the scenario's fixed layout, to learn the model's
        # dimensions and timing. Reset recompiles only when the layout changes.
        layout = scenarios.sample_obstacles(self._scenario, self._rng)
        self._model = scenarios.build(self._scenario, layout)
        self._data = mujoco.MjData(self._model)
        self._lidar = Lidar(self._model, "laser", "base_link", self._spec)
        self._nominal_mass = float(self._model.body_mass[1])

        self._dt = self._model.opt.timestep * decimation
        self._rate = RateLimiter(self._limits, self._dt)
        self._hi = np.asarray(self._limits.max_velocity, dtype=np.float64)
        self._lo = np.asarray(self._limits.min_velocity, dtype=np.float64)

        self._goal = np.zeros(2)
        self._previous_distance = 0.0
        self._previous_twist = np.zeros(3)
        self._elapsed = 0.0
        self._diagonal = float(np.linalg.norm(self._scenario.bounds)) * 2.0

        self.action_space = spaces.Box(-1.0, 1.0, (3,), dtype=np.float32)
        self.observation_space = spaces.Box(
            -1.0, 1.0, (self._spec.beams + 6,), dtype=np.float32
        )

    @property
    def dt(self) -> float:
        """Control period in seconds."""
        return self._dt

    def _scale(self, action: np.ndarray) -> np.ndarray:
        """Map a -1..1 action onto the rover's asymmetric velocity envelope."""
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        # Forward and reverse limits differ, so each side scales separately
        # rather than by a single symmetric gain.
        return np.where(a >= 0.0, a * self._hi, -a * self._lo)

    def _observe(self, ranges: np.ndarray) -> np.ndarray:
        """Assemble the observation vector at the current state.

        Takes the scan rather than casting it, because the caller also needs
        the nearest range for the reward and one cast per control step is
        enough.

        Args:
            ranges: Scan in metres, as returned by `Lidar.scan`.

        Returns:
            The observation vector.
        """
        scan = (ranges / self._spec.max_range).astype(np.float32)
        pose = self._data.qpos
        offset = self._goal - pose[:2]
        distance = float(np.linalg.norm(offset))
        # Goal bearing in the body frame: world bearing minus rover yaw.
        bearing = float(np.arctan2(offset[1], offset[0]) - pose[2])
        twist = self._rate.current
        normalised_twist = twist / np.maximum(np.abs(self._hi), np.abs(self._lo))
        return np.concatenate(
            [
                scan,
                [min(distance / self._diagonal, 1.0)],
                [np.sin(bearing), np.cos(bearing)],
                normalised_twist,
            ]
        ).astype(np.float32)

    def _collided(self) -> bool:
        """Whether the footprint is touching anything this step."""
        return self._data.ncon > 0

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        """Start a new episode with a fresh layout, start pose and goal.

        Args:
            seed: Reseeds the episode generator when given.
            options: Unused, present for the Gymnasium API.

        Returns:
            The first observation and an info dict.
        """
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        layout = scenarios.sample_obstacles(self._scenario, self._rng)
        if self._scenario.n_random:
            # Only recompile when the layout actually varies; a fixed scenario
            # reuses the model and resets state, which is far cheaper.
            self._model = scenarios.build(self._scenario, layout)
            self._data = mujoco.MjData(self._model)
            self._lidar = Lidar(self._model, "laser", "base_link", self._spec)
        else:
            mujoco.mj_resetData(self._model, self._data)

        if self._mass_randomisation > 0.0:
            spread = self._mass_randomisation
            self._model.body_mass[1] = self._nominal_mass * float(
                self._rng.uniform(1.0 - spread, 1.0 + spread)
            )

        start = scenarios.sample_pose(self._scenario, layout, self._rng)
        for _ in range(100):
            self._goal = scenarios.sample_pose(self._scenario, layout, self._rng)
            if np.linalg.norm(self._goal - start) >= self._scenario.min_goal_distance:
                break

        self._data.qpos[:] = [start[0], start[1], self._rng.uniform(-np.pi, np.pi)]
        self._data.qvel[:] = 0.0
        self._rate.reset()
        self._previous_twist = np.zeros(3)
        self._elapsed = 0.0
        mujoco.mj_forward(self._model, self._data)

        self._previous_distance = float(np.linalg.norm(self._goal - start))
        ranges = self._lidar.scan(self._model, self._data)
        return self._observe(ranges), {"goal": self._goal.copy()}

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        """Apply one control step.

        Args:
            action: Desired twist in -1..1 as [vx, vy, wz].

        Returns:
            Observation, reward, terminated, truncated, info.
        """
        twist = self._rate.step(self._scale(action))
        self._data.ctrl[:] = twist

        collided = False
        for _ in range(self._decimation):
            mujoco.mj_step(self._model, self._data)
            # Checked every physics step, not just the last: a glancing
            # contact can start and clear inside one control period.
            collided = collided or self._collided()

        self._elapsed += self._dt
        distance = float(np.linalg.norm(self._goal - self._data.qpos[:2]))
        ranges = self._lidar.scan(self._model, self._data)
        nearest = float(ranges.min())
        reached = distance <= self._scenario.goal_tolerance

        reward, terms = step_reward(
            previous_distance=self._previous_distance,
            distance=distance,
            nearest_obstacle=nearest,
            twist=twist,
            previous_twist=self._previous_twist,
            reached=reached,
            collided=collided,
            weights=self._weights,
        )

        self._previous_distance = distance
        self._previous_twist = twist
        truncated = self._elapsed >= self._scenario.time_limit

        info: dict[str, Any] = {
            "distance": distance,
            "nearest_obstacle": nearest,
            "is_success": reached,
            "collided": collided,
            "reward_terms": terms,
        }
        return self._observe(ranges), reward, reached or collided, truncated, info
