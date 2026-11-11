"""Builds training and evaluation worlds around the rover model.

Obstacles are added through MuJoCo's spec API rather than by templating MJCF
text, so `assets/helios.xml` stays the one definition of the robot and its
`meshdir` keeps resolving.

A scenario is DATA, not code: the same definitions drive randomised training
episodes and the fixed evaluation suite, so an eval score means the same
thing as a training reward.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

MODEL = Path(__file__).resolve().parent.parent / "assets" / "helios.xml"

#: Half the diagonal of the 0.38 x 0.38 footprint. Anything closer than this
#: to an obstacle centre is already in contact, so starts and goals are kept
#: outside it with a margin.
FOOTPRINT_RADIUS: float = 0.269


@dataclass(frozen=True)
class Obstacle:
    """A box obstacle, axis aligned.

    Attributes:
        pos: Centre as (x, y) in metres. Height is fixed to the rover's.
        half_extent: Half width and half depth as (x, y) in metres.
    """

    pos: tuple[float, float]
    half_extent: tuple[float, float]

    def clearance_to(self, point: np.ndarray) -> float:
        """Distance from a point to this box, zero inside it.

        Args:
            point: An (x, y) position in metres.

        Returns:
            Shortest distance to the box surface, in metres.
        """
        # Distance to an axis-aligned box is the norm of the per-axis overshoot
        # beyond the box's half extents, clamped at zero.
        delta = np.abs(point[:2] - np.asarray(self.pos)) - np.asarray(self.half_extent)
        return float(np.linalg.norm(np.maximum(delta, 0.0)))


@dataclass(frozen=True)
class Scenario:
    """One navigation task definition.

    Attributes:
        name: Identifier used in eval output.
        bounds: Half extents of the walled room as (x, y) in metres.
        obstacles: Fixed obstacles. Empty means an open room.
        n_random: Random obstacles added per reset, on top of `obstacles`.
        random_half_extent: Half extent used for random obstacles.
        goal_tolerance: Distance at which the goal counts as reached.
        min_goal_distance: Shortest start-to-goal distance worth training on.
        time_limit: Episode cap in seconds.
    """

    name: str = "open_room"
    bounds: tuple[float, float] = (5.0, 5.0)
    obstacles: tuple[Obstacle, ...] = field(default_factory=tuple)
    n_random: int = 0
    random_half_extent: tuple[float, float] = (0.25, 0.25)
    goal_tolerance: float = 0.25
    min_goal_distance: float = 1.5
    time_limit: float = 60.0


def build(scenario: Scenario, obstacles: tuple[Obstacle, ...]) -> mujoco.MjModel:
    """Compile the rover model inside a walled room with obstacles.

    Args:
        scenario: Supplies the room size.
        obstacles: Every obstacle to place, fixed and random together.

    Returns:
        A compiled model ready for `MjData`.
    """
    spec = mujoco.MjSpec.from_file(str(MODEL))
    half_x, half_y = scenario.bounds
    # Height is cosmetic: the footprint box spans 0 to 0.152 m, so an
    # obstacle only has to overlap that band to be collidable.
    height = 0.25

    for i, ob in enumerate(obstacles):
        body = spec.worldbody.add_body(
            name=f"obstacle_{i}", pos=[ob.pos[0], ob.pos[1], height]
        )
        body.add_geom(
            name=f"obstacle_{i}_geom",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[ob.half_extent[0], ob.half_extent[1], height],
            rgba=[0.75, 0.25, 0.20, 1.0],
        )

    # Four walls, so a policy cannot escape the room and wander forever.
    for i, (pos, size) in enumerate(
        (
            ((half_x, 0.0), (0.05, half_y)),
            ((-half_x, 0.0), (0.05, half_y)),
            ((0.0, half_y), (half_x, 0.05)),
            ((0.0, -half_y), (half_x, 0.05)),
        )
    ):
        body = spec.worldbody.add_body(name=f"wall_{i}", pos=[pos[0], pos[1], height])
        body.add_geom(
            name=f"wall_{i}_geom",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[size[0], size[1], height],
            rgba=[0.55, 0.55, 0.58, 1.0],
        )

    return spec.compile()


def sample_obstacles(
    scenario: Scenario, rng: np.random.Generator
) -> tuple[Obstacle, ...]:
    """Draw this episode's obstacle layout.

    Args:
        scenario: Supplies the fixed obstacles and how many to add.
        rng: Source of randomness, so an episode is reproducible by seed.

    Returns:
        The fixed obstacles followed by the randomly placed ones.
    """
    half_x, half_y = scenario.bounds
    margin = max(scenario.random_half_extent) + FOOTPRINT_RADIUS
    extra: list[Obstacle] = []
    for _ in range(scenario.n_random):
        extra.append(
            Obstacle(
                pos=(
                    float(rng.uniform(-half_x + margin, half_x - margin)),
                    float(rng.uniform(-half_y + margin, half_y - margin)),
                ),
                half_extent=scenario.random_half_extent,
            )
        )
    return (*scenario.obstacles, *extra)


def sample_pose(
    scenario: Scenario,
    obstacles: tuple[Obstacle, ...],
    rng: np.random.Generator,
    clearance: float = 0.15,
    attempts: int = 200,
) -> np.ndarray:
    """Draw a collision-free (x, y) inside the room.

    Args:
        scenario: Supplies the room bounds.
        obstacles: Obstacles to stay clear of.
        rng: Source of randomness.
        clearance: Margin beyond the footprint radius, in metres.
        attempts: Draws to try before giving up.

    Returns:
        A free position as (x, y).

    Raises:
        RuntimeError: If no free position is found, which means the room is
            too crowded for the scenario to be solvable.
    """
    half_x, half_y = scenario.bounds
    edge = FOOTPRINT_RADIUS + clearance
    need = FOOTPRINT_RADIUS + clearance
    for _ in range(attempts):
        point = np.array(
            [
                rng.uniform(-half_x + edge, half_x - edge),
                rng.uniform(-half_y + edge, half_y - edge),
            ]
        )
        if all(ob.clearance_to(point) >= need for ob in obstacles):
            return point
    raise RuntimeError(
        f"no free pose in {scenario.name} after {attempts} attempts; "
        "reduce n_random or enlarge bounds"
    )


#: The evaluation suite. Fixed definitions, so scores are comparable between
#: runs and against the Nav2 baseline.
SUITE: tuple[Scenario, ...] = (
    Scenario(name="open_room", n_random=0),
    Scenario(name="sparse", n_random=3),
    Scenario(name="cluttered", n_random=8, time_limit=90.0),
    Scenario(
        name="corridor",
        bounds=(5.0, 2.0),
        obstacles=(
            Obstacle(pos=(0.0, 1.0), half_extent=(0.6, 0.6)),
            Obstacle(pos=(2.0, -1.0), half_extent=(0.6, 0.6)),
            Obstacle(pos=(-2.0, -1.0), half_extent=(0.6, 0.6)),
        ),
    ),
)
