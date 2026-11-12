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
from envs.hardware import (
    CommandDelay,
    HardwareModel,
    PoseEstimator,
    TwistEstimator,
)
from envs.lidar import (
    UST10LX_DROPOUT,
    DepthFan,
    Lidar,
    LidarSpec,
    apply_dropout,
    fuse,
)
from envs.rate_limit import RateLimiter, VelocityLimits
from envs.reward import RewardWeights, step_reward
from envs.route import Pose2D, point_at_arc, project, route_length

#: Physics steps per control step. The model runs at 4 ms, so 12 gives a
#: 48 ms control period, close to the 20 Hz the real controller runs at.
DEFAULT_DECIMATION: int = 12

#: Arc lengths ahead of the rover's projection at which the route is sampled,
#: metres. Three rather than one so the policy can see the route's CURVATURE
#: and start turning before a corner, not just its immediate direction. The
#: 1.5 m middle value matches subgoal_walker's DEFAULT_LOOKAHEAD, so the
#: policy and the VLA walker read the same route the same way.
LOOKAHEADS: tuple[float, ...] = (0.5, 1.5, 3.0)


class RoverEnv(gym.Env[np.ndarray, np.ndarray]):
    """Point-goal navigation with a downsampled laser scan.

    Observation is one flat vector, which is what SB3's MlpPolicy expects:

        [ fused scan / max_range   beams values, 1.0 is open space    ]
        [ route carrots            2 per lookahead, body frame, /L    ]
        [ remaining route / diag   1 value                            ]
        [ commanded twist          3 values, normalised               ]
        [ measured twist           3 values, normalised               ]

    THE POLICY SEES A ROUTE, NOT A GOAL. A distant goal bearing points through
    walls, and a policy steering by it drives into dead ends with no way to
    learn otherwise, because "this is a dead end" is not in the observation.
    Carrots along a planned route are what Nav2 will hand the real node, and
    what `subgoal_walker_node` already feeds the VLA policy.

    Each carrot is the body-frame offset to the route point that far ahead,
    divided by its own lookahead, so every component stays in -1..1 and a
    carrot never dominates the vector by being distant.

    The scan FUSES laser and depth, because Nav2's costmap does. Commanded
    and measured twist are both present: the gap between them is slip, load
    and latency, and a policy that cannot see it cannot compensate for it.

    REWARD IS COMPUTED ON TRUTH, OBSERVATION ON THE ESTIMATE. The policy sees
    the pose localisation would report, including its rare 6 m jumps, but is
    rewarded for where it actually went. Rewarding the estimate would teach it
    to exploit localisation error.

    Action is [vx, vy, wz] in -1..1, scaled to the rover's real envelope, then
    rate limited and held back by the measured transport delay.
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
        hardware: HardwareModel | None = None,
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
            hardware: Latency, noise and localisation-failure model. Defaults
                to the values measured from datasets/vla_runs.
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
        self._hardware = hardware or HardwareModel()
        self._rng = np.random.default_rng(seed)

        # Compiled once with the scenario's fixed layout, to learn the model's
        # dimensions and timing. Reset recompiles only when the layout changes.
        self._from_map = self._scenario.map_yaml is not None
        layout = scenarios.sample_obstacles(self._scenario, self._rng)
        self._model = self._compile(layout)
        self._data = mujoco.MjData(self._model)
        self._lidar = Lidar(self._model, "laser", "base_link", self._spec)
        self._depth = DepthFan(self._model, "zed_mount", "base_link", self._spec)
        self._nominal_mass = float(self._model.body_mass[1])

        self._dt = self._model.opt.timestep * decimation
        self._rate = RateLimiter(self._limits, self._dt)
        self._delay = CommandDelay(self._hardware.delay_steps[0])
        self._twist_estimator = TwistEstimator(
            self._hardware,
            np.abs(np.asarray(self._limits.max_velocity)),
            self._rng,
        )
        self._pose_estimator = PoseEstimator(self._hardware, self._dt, self._rng)
        self._hi = np.asarray(self._limits.max_velocity, dtype=np.float64)
        self._lo = np.asarray(self._limits.min_velocity, dtype=np.float64)

        self._goal = np.zeros(2)
        self._measured_twist = np.zeros(3)
        self._route: list[Pose2D] = []
        self._route_length = 0.0
        self._previous_remaining = 0.0
        self._previous_twist = np.zeros(3)
        self._earned_progress = 0.0
        self._elapsed = 0.0
        # Normaliser for the goal-distance observation: the longest journey
        # possible in this world.
        extent = (
            scenarios.grid_for(self._scenario).extent
            if self._from_map
            else tuple(2.0 * b for b in self._scenario.bounds)
        )
        self._diagonal = float(np.linalg.norm(extent))

        self.action_space = spaces.Box(-1.0, 1.0, (3,), dtype=np.float32)
        self.observation_space = spaces.Box(
            -1.0,
            1.0,
            (self._spec.beams + 2 * len(LOOKAHEADS) + 7,),
            dtype=np.float32,
        )

    def _compile(self, layout: tuple[scenarios.Obstacle, ...]) -> mujoco.MjModel:
        """Compile the world, from a saved map or from generated obstacles.

        Args:
            layout: Obstacles, ignored when the scenario carries a map.

        Returns:
            The compiled model.
        """
        if self._from_map:
            return scenarios.build_from_map(self._scenario)
        return scenarios.build(self._scenario, layout)

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

    def _sense(self) -> np.ndarray:
        """Produce the fused scan the policy sees.

        Laser and depth are merged by taking the nearer per bin, matching the
        local costmap's `observation_sources: scan zed_cloud`. Dropout is
        applied after fusion, since a dropped laser beam can still be covered
        by the camera.

        Returns:
            Fused ranges in metres, one per beam.
        """
        laser = self._lidar.scan(self._model, self._data)
        depth = self._depth.ranges(self._model, self._data)
        merged = fuse(laser, depth, self._depth.bins)
        return apply_dropout(merged, UST10LX_DROPOUT, self._spec.max_range, self._rng)

    def _observe(self, ranges: np.ndarray, pose: np.ndarray) -> np.ndarray:
        """Assemble the observation vector.

        Args:
            ranges: Fused scan in metres.
            pose: The ESTIMATED pose [x, y, yaw], not ground truth. The policy
                navigates on what localisation tells it.

        Returns:
            The observation vector.
        """
        scan = (ranges / self._spec.max_range).astype(np.float32)
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        arc = project(self._route, x, y).arc
        # World to body is a rotation by -yaw.
        cos_yaw, sin_yaw = np.cos(-yaw), np.sin(-yaw)

        carrots: list[float] = []
        for lookahead in LOOKAHEADS:
            point = point_at_arc(self._route, arc + lookahead)
            dx, dy = point.x - x, point.y - y
            body_x = cos_yaw * dx - sin_yaw * dy
            body_y = sin_yaw * dx + cos_yaw * dy
            # Divided by its own lookahead so each carrot lands in -1..1 and
            # the far one does not swamp the near one.
            carrots.extend((body_x / lookahead, body_y / lookahead))

        left = max(0.0, self._route_length - arc)
        norm = np.maximum(np.abs(self._hi), np.abs(self._lo))
        return np.concatenate(
            [
                scan,
                np.clip(carrots, -1.0, 1.0),
                [min(left / self._diagonal, 1.0)],
                self._rate.current / norm,
                np.clip(self._measured_twist / norm, -1.0, 1.0),
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
        if self._scenario.n_random and not self._from_map:
            # Only recompile when the layout actually varies; a fixed scenario
            # reuses the model and resets state, which is far cheaper.
            self._model = self._compile(layout)
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

        # The route is what the policy is rewarded for following, so a
        # scenario the planner cannot solve is not a usable episode. Redraw
        # rather than train on an impossible one.
        planner = scenarios.planner_for(self._scenario, layout)
        for _ in range(20):
            try:
                self._route = planner.plan(start, self._goal)
                break
            except RuntimeError:
                self._goal = scenarios.sample_pose(self._scenario, layout, self._rng)
        else:
            raise RuntimeError(
                f"no reachable goal in {self._scenario.name} after 20 draws"
            )
        self._route_length = route_length(self._route)

        self._data.qpos[:] = [start[0], start[1], self._rng.uniform(-np.pi, np.pi)]
        self._data.qvel[:] = 0.0
        self._rate.reset()
        # Transport delay is redrawn per episode: a policy tuned to one
        # latency will not survive the spread measured on hardware.
        self._delay = CommandDelay(
            int(
                self._rng.integers(
                    self._hardware.delay_steps[0], self._hardware.delay_steps[1] + 1
                )
            )
        )
        self._twist_estimator.reset()
        self._pose_estimator.reset()
        self._measured_twist = np.zeros(3)
        self._previous_twist = np.zeros(3)
        self._earned_progress = 0.0
        self._elapsed = 0.0
        mujoco.mj_forward(self._model, self._data)

        arc = project(self._route, float(start[0]), float(start[1])).arc
        self._previous_remaining = max(0.0, self._route_length - arc)
        estimated = self._pose_estimator.estimate(np.asarray(self._data.qpos))
        return self._observe(self._sense(), estimated), {
            "goal": self._goal.copy(),
            "route_length": self._route_length,
            "delay_steps": self._delay.steps,
            "tracking_scale": self._twist_estimator.scale,
        }

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        """Apply one control step.

        Args:
            action: Desired twist in -1..1 as [vx, vy, wz].

        Returns:
            Observation, reward, terminated, truncated, info.
        """
        # Order matches the real path: the smoother runs on the Jetson, then
        # the command crosses serial and the RoboClaw PID, which is the delay.
        twist = self._rate.step(self._scale(action))
        self._data.ctrl[:] = self._delay.push(twist)

        collided = False
        for _ in range(self._decimation):
            mujoco.mj_step(self._model, self._data)
            # Checked every physics step, not just the last: a glancing
            # contact can start and clear inside one control period.
            collided = collided or self._collided()

        self._elapsed += self._dt
        self._measured_twist = self._twist_estimator.estimate(
            np.asarray(self._data.qvel)
        )
        estimated = self._pose_estimator.estimate(np.asarray(self._data.qpos))

        # Truth, for the reward.
        x, y = float(self._data.qpos[0]), float(self._data.qpos[1])
        distance = float(np.linalg.norm(self._goal - self._data.qpos[:2]))
        ranges = self._sense()
        nearest = float(ranges.min())
        reached = distance <= self._scenario.goal_tolerance

        # Progress is arc length closed ALONG the route, not straight-line
        # distance to the goal. Straight-line progress rewards driving into
        # a dead end whose mouth faces the goal; route progress does not,
        # because the route goes around it.
        projection = project(self._route, x, y)
        left = max(0.0, self._route_length - projection.arc)

        reward, terms = step_reward(
            previous_distance=self._previous_remaining,
            distance=left,
            nearest_obstacle=nearest,
            twist=twist,
            previous_twist=self._previous_twist,
            reached=reached,
            collided=collided,
            earned_progress=self._earned_progress,
            weights=self._weights,
        )

        # Tracked so a collision can forfeit it. Only the positive part
        # accumulates: retreating already costs through the progress term and
        # should not also buy a cheaper crash later.
        self._earned_progress += max(terms["progress"], 0.0)
        self._previous_remaining = left
        self._previous_twist = twist
        truncated = self._elapsed >= self._scenario.time_limit

        info: dict[str, Any] = {
            "distance": distance,
            "route_remaining": left,
            "cross_track": projection.offset,
            "nearest_obstacle": nearest,
            "is_success": reached,
            "collided": collided,
            "localisation_lost": self._pose_estimator.lost,
            "pose_error": float(np.linalg.norm(estimated[:2] - self._data.qpos[:2])),
            "reward_terms": terms,
        }
        return (
            self._observe(ranges, estimated),
            reward,
            reached or collided,
            truncated,
            info,
        )
