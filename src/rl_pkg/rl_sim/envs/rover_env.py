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

        # Compiled ONCE for the whole lifetime of the env. Reset repositions
        # obstacles in place rather than rebuilding, so every handle below
        # stays valid and a viewer can follow the env across episodes.
        self._from_map = self._scenario.map_yaml is not None
        layout = scenarios.sample_obstacles(self._scenario, self._rng)
        self._model = self._compile(layout)
        self._data = mujoco.MjData(self._model)
        self._lidar = Lidar(self._model, "laser", "base_link", self._spec)
        self._depth = DepthFan(self._model, "zed_mount", "base_link", self._spec)
        self._nominal_mass = float(self._model.body_mass[1])

        # Looked up by name, never assumed: the joint order in helios.xml is
        # load-bearing and has changed once already.
        self._adr = {
            name: int(
                self._model.jnt_qposadr[
                    mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
                ]
            )
            for name in ("slide_x", "slide_y", "hinge_z")
        }
        self._base = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, "base_link"
        )
        # Obstacle handles, cached once. A scenario's obstacle COUNT is fixed
        # (scenario.obstacles + n_random + n_low), so only their poses and
        # sizes vary per episode, and both are mutable on a compiled model.
        self._obstacle_bodies = [
            mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, f"obstacle_{i}")
            for i in range(len(layout))
        ]
        self._obstacle_geoms = [
            mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_GEOM, f"obstacle_{i}_geom"
            )
            for i in range(len(layout))
        ]

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
        self._carrot_bearing = 0.0
        self._measured_twist = np.zeros(3)
        self._route: list[Pose2D] = []
        self._route_length = 0.0
        self._previous_remaining = 0.0
        self._previous_action = np.zeros(3)
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

    def _place_obstacles(self, layout: tuple[scenarios.Obstacle, ...]) -> None:
        """Move this episode's obstacles without recompiling the model.

        Writing `body_pos` and `geom_size` is enough: the obstacles are
        static bodies, so `mj_forward` derives their world poses from those
        fields. Mutating `geom_pos` instead does nothing, because a geom's
        position is relative to its body.

        Recompiling cost 35 ms per reset and, worse, handed back a NEW
        MjModel and MjData every episode, which no viewer can follow. This
        costs nothing measurable and keeps one model alive for the whole run.

        Args:
            layout: This episode's obstacles, same length as at construction.
        """
        for body, geom, ob in zip(
            self._obstacle_bodies, self._obstacle_geoms, layout, strict=True
        ):
            half_z = ob.top / 2.0
            self._model.body_pos[body] = [ob.pos[0], ob.pos[1], half_z]
            self._model.geom_size[geom] = [
                ob.half_extent[0],
                ob.half_extent[1],
                half_z,
            ]

    def _draw_free_pose(
        self, planner: scenarios.GridPlanner, truth: tuple[scenarios.Obstacle, ...]
    ) -> np.ndarray:
        """Draw a pose the planner can route from that is also really clear.

        Args:
            planner: Supplies the believed-reachable cells.
            truth: The obstacles actually present.

        Returns:
            A position clear of the true obstacles, in metres.
        """
        need = scenarios.FOOTPRINT_RADIUS + 0.10
        for _ in range(200):
            point = scenarios.sample_pose(planner, self._rng)
            if all(ob.clearance_to(point) >= need for ob in truth):
                return point
        # Every draw was blocked by something unmapped, which means the
        # staleness is set too high for this layout to be startable.
        return point

    def _true_pose(self) -> np.ndarray:
        """Ground-truth world [x, y, yaw].

        Position comes from `xpos`, NOT from the slide joints. With the hinge
        ahead of them the slides translate along a ROTATED axis, so their
        qpos values are body-frame displacements and only agree with world
        coordinates at zero yaw. Reading them as world position sends the
        rover apparently off the map while it is really driving in circles.
        """
        return np.array(
            [
                self._data.xpos[self._base][0],
                self._data.xpos[self._base][1],
                self._data.qpos[self._adr["hinge_z"]],
            ]
        )

    def _true_twist(self) -> np.ndarray:
        """Ground-truth BODY twist [vx, vy, wz].

        The slides are world-frame, so their velocities are rotated back
        through the current yaw. Body frame is what `/cmd_vel` carries and
        what the observation needs.
        """
        v = self._data.qvel
        world = np.array([v[self._adr["slide_x"]], v[self._adr["slide_y"]]])
        yaw = float(self._data.qpos[self._adr["hinge_z"]])
        cos_yaw, sin_yaw = np.cos(-yaw), np.sin(-yaw)
        return np.array(
            [
                cos_yaw * world[0] - sin_yaw * world[1],
                sin_yaw * world[0] + cos_yaw * world[1],
                v[self._adr["hinge_z"]],
            ]
        )

    def _to_world(self, body_twist: np.ndarray) -> np.ndarray:
        """Rotate a body twist into the world frame the slides act in.

        Args:
            body_twist: [vx, vy, wz] in the rover's own frame.

        Returns:
            The same command with its linear part expressed in world axes.
        """
        yaw = float(self._data.qpos[self._adr["hinge_z"]])
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        return np.array(
            [
                cos_yaw * body_twist[0] - sin_yaw * body_twist[1],
                sin_yaw * body_twist[0] + cos_yaw * body_twist[1],
                body_twist[2],
            ]
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

    @property
    def model(self) -> mujoco.MjModel:
        """The compiled model, for a viewer to render.

        Recompiled on reset when the layout varies, so a viewer must re-read
        this after every reset rather than holding the first one.
        """
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        """Current simulation state, for a viewer to render."""
        return self._data

    @property
    def route(self) -> list[Pose2D]:
        """This episode's planned route, for a viewer to draw."""
        return list(self._route)

    @property
    def goal(self) -> np.ndarray:
        """This episode's goal as (x, y) in metres."""
        return self._goal.copy()

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
        self._carrot_bearing = 0.0
        for lookahead in LOOKAHEADS:
            point = point_at_arc(self._route, arc + lookahead)
            dx, dy = point.x - x, point.y - y
            body_x = cos_yaw * dx - sin_yaw * dy
            body_y = sin_yaw * dx + cos_yaw * dy
            # Divided by its own lookahead so each carrot lands in -1..1 and
            # the far one does not swamp the near one.
            carrots.extend((body_x / lookahead, body_y / lookahead))
            # The middle lookahead is the one subgoal_walker steers by, so
            # the reward's alignment law reads the same point.
            if lookahead == LOOKAHEADS[len(LOOKAHEADS) // 2]:
                self._carrot_bearing = float(np.arctan2(body_y, body_x))

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
            # Reseeded IN PLACE, not rebound. The twist and pose estimators
            # hold a reference to this generator from construction, so
            # assigning a new one here left their noise unseeded and the
            # same seed produced different episodes.
            self._rng.bit_generator.state = np.random.default_rng(
                seed
            ).bit_generator.state

        layout = scenarios.sample_obstacles(self._scenario, self._rng)
        mujoco.mj_resetData(self._model, self._data)
        if self._obstacle_bodies:
            self._place_obstacles(layout)

        if self._mass_randomisation > 0.0:
            spread = self._mass_randomisation
            self._model.body_mass[1] = self._nominal_mass * float(
                self._rng.uniform(1.0 - spread, 1.0 + spread)
            )

        # The planner gets the map's BELIEF, which may be stale; MuJoCo has
        # the truth. The gap between them is the policy's job.
        believed = scenarios.stale_layout(layout, self._scenario, self._rng)
        planner = scenarios.planner_for(self._scenario, believed)

        # Poses come from the planner's reachable set so a route always
        # exists, but must also be clear in TRUTH: a believed-free cell can
        # be where an unmapped obstacle actually sits.
        start = self._draw_free_pose(planner, layout)
        for _ in range(100):
            self._goal = self._draw_free_pose(planner, layout)
            if np.linalg.norm(self._goal - start) >= self._scenario.min_goal_distance:
                break

        # The route is what the policy is rewarded for following, so a
        # scenario the planner cannot solve is not a usable episode. Redraw
        # rather than train on an impossible one.
        self._route = planner.plan(start, self._goal)
        self._route_length = route_length(self._route)

        # The slides are world axes, so a start pose is written directly.
        self._data.qpos[self._adr["hinge_z"]] = self._rng.uniform(-np.pi, np.pi)
        self._data.qpos[self._adr["slide_x"]] = start[0]
        self._data.qpos[self._adr["slide_y"]] = start[1]
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
        self._previous_action = np.zeros(3)
        self._elapsed = 0.0
        mujoco.mj_forward(self._model, self._data)

        arc = project(self._route, float(start[0]), float(start[1])).arc
        self._previous_remaining = max(0.0, self._route_length - arc)
        estimated = self._pose_estimator.estimate(self._true_pose())
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
        clipped = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        twist = self._rate.step(self._scale(clipped))
        # Rate limiting and the transport delay both act on the BODY command,
        # as they do on the real rover; only the final actuator write is in
        # world axes, because that is the frame the slides move in.
        self._data.ctrl[:] = self._to_world(self._delay.push(twist))

        collided = False
        for _ in range(self._decimation):
            mujoco.mj_step(self._model, self._data)
            # Checked every physics step, not just the last: a glancing
            # contact can start and clear inside one control period.
            collided = collided or self._collided()

        self._elapsed += self._dt
        self._measured_twist = self._twist_estimator.estimate(self._true_twist())
        truth = self._true_pose()
        estimated = self._pose_estimator.estimate(truth)

        # Truth, for the reward.
        x, y = float(truth[0]), float(truth[1])
        distance = float(np.linalg.norm(self._goal - truth[:2]))
        ranges = self._sense()
        nearest = float(ranges.min())
        reached = distance <= self._scenario.goal_tolerance

        # Progress is arc length closed ALONG the route, not straight-line
        # distance to the goal. Straight-line progress rewards driving into
        # a dead end whose mouth faces the goal; route progress does not,
        # because the route goes around it.
        projection = project(self._route, x, y)
        left = max(0.0, self._route_length - projection.arc)

        # Observed before the reward is scored: the alignment term needs
        # the bearing on both sides of the step, and _observe is what
        # recomputes it.
        previous_bearing = self._carrot_bearing
        observation = self._observe(ranges, estimated)

        reward, terms = step_reward(
            previous_distance=self._previous_remaining,
            distance=left,
            nearest_obstacle=nearest,
            twist=twist,
            action=clipped,
            previous_action=self._previous_action,
            # Last step's bearing: that is the observation the policy
            # actually chose this action from, so it is what the action
            # should be judged against.
            bearing=previous_bearing,
            next_bearing=self._carrot_bearing,
            reached=reached,
            collided=collided,
            weights=self._weights,
        )

        self._previous_remaining = left
        self._previous_action = clipped
        truncated = self._elapsed >= self._scenario.time_limit

        info: dict[str, Any] = {
            "distance": distance,
            "route_remaining": left,
            "cross_track": projection.offset,
            "nearest_obstacle": nearest,
            "is_success": reached,
            "collided": collided,
            "localisation_lost": self._pose_estimator.lost,
            "pose_error": float(np.linalg.norm(estimated[:2] - truth[:2])),
            "reward_terms": terms,
        }
        return observation, reward, reached or collided, truncated, info
