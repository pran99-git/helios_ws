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

from envs.occupancy import OccupancyGrid, from_obstacles, load_ros_map
from envs.planner import GridPlanner

MODEL = Path(__file__).resolve().parent.parent / "assets" / "helios.xml"

#: Half the diagonal of the 0.38 x 0.38 footprint. At exactly this distance
#: the rover is already touching, whatever its heading.
FOOTPRINT_RADIUS: float = 0.269

#: Clearance the planner and the start/goal sampler both keep, in metres.
#:
#: navigation_pkg's inflation_radius, 1.7x the circumscribed radius. Planning
#: at the bare FOOTPRINT_RADIUS leaves ZERO tracking margin: a route may pass
#: exactly as close as the rover is wide, so any cross-track error at all is a
#: collision. Measured with a pure-pursuit baseline, which drifted 0.6 m off
#: route and hit a wall every time.
PLANNING_CLEARANCE: float = 0.45


@dataclass(frozen=True)
class Obstacle:
    """A box obstacle, axis aligned.

    Attributes:
        pos: Centre as (x, y) in metres.
        half_extent: Half width and half depth as (x, y) in metres.
        top: Height of the obstacle's top surface above the floor, metres.
            Matters because the laser plane is at 0.176 m: anything shorter is
            invisible to it and only the ZED depth fan sees it. Everything
            still collides, since the footprint box spans 0 to 0.152 m.
    """

    pos: tuple[float, float]
    half_extent: tuple[float, float]
    top: float = 0.50

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
        n_low: Random LOW obstacles added per reset, short enough that the
            laser misses them and only the depth fan sees them.
        low_half_extent: Half extent used for low obstacles.
        low_top: Range of top heights for low obstacles, metres. Kept between
            the costmap's 0.036 m floor threshold and the 0.176 m laser plane,
            so they are exactly the class of obstacle the ZED exists to catch.
        goal_tolerance: Distance at which the goal counts as reached.
        min_goal_distance: Shortest start-to-goal distance worth training on.
        time_limit: Episode cap in seconds. Short on purpose: a 4 m route
            at 0.40 m/s takes about 10 s, and at 90 s the 7% of episodes
            that stalled ran to 1,875 steps and consumed roughly half of
            all simulated steps.
        stale_moved: Fraction of obstacles the planner's map places at a
            STALE position: furniture shifted since the map was built. The
            route goes cleanly around where each one used to be, and the
            rover meets it somewhere else.
        stale_move_range: How far a stale obstacle has moved, metres.
        stale_unmapped: Fraction of obstacles missing from the planner's map
            altogether: objects that arrived after mapping.

            Together these decide how much of the avoidance the POLICY has
            to do. At zero the planner routes around everything and
            following the route blindly succeeds, which is what made the
            scan decorative: feeding it noise instead cost nothing.
        map_yaml: A saved slam_toolbox map to use as the world instead of
            `bounds` and `obstacles`. When set, geometry comes from the real
            room and starts and goals are drawn from its free cells.
    """

    name: str = "open_room"
    bounds: tuple[float, float] = (5.0, 5.0)
    obstacles: tuple[Obstacle, ...] = field(default_factory=tuple)
    n_random: int = 0
    random_half_extent: tuple[float, float] = (0.25, 0.25)
    n_low: int = 0
    low_half_extent: tuple[float, float] = (0.18, 0.18)
    low_top: tuple[float, float] = (0.06, 0.15)
    goal_tolerance: float = 0.25
    min_goal_distance: float = 1.5
    time_limit: float = 40.0
    stale_moved: float = 0.0
    stale_move_range: tuple[float, float] = (0.5, 1.2)
    stale_unmapped: float = 0.0
    map_yaml: Path | None = None


_GRIDS: dict[Path, OccupancyGrid] = {}
_PLANNERS: dict[tuple[Path, float], GridPlanner] = {}


def grid_for(scenario: Scenario) -> OccupancyGrid:
    """Load and cache the scenario's occupancy grid.

    Args:
        scenario: Must have `map_yaml` set.

    Returns:
        The grid, parsed once per path per process.

    Raises:
        ValueError: If the scenario has no map.
    """
    if scenario.map_yaml is None:
        raise ValueError(f"scenario {scenario.name!r} has no map_yaml")
    if scenario.map_yaml not in _GRIDS:
        _GRIDS[scenario.map_yaml] = load_ros_map(scenario.map_yaml)
    return _GRIDS[scenario.map_yaml]


def stale_layout(
    layout: tuple[Obstacle, ...],
    scenario: Scenario,
    rng: np.random.Generator,
) -> tuple[Obstacle, ...]:
    """What the planner BELIEVES is there, which is not what is there.

    The planner is never wrong about its own map: it routes around every
    obstacle it knows, inflated, exactly as Smac2D does. It is the MAP that
    is out of date. That keeps the route a good guide rather than sending it
    through the middle of a box, which would put the progress reward and
    collision avoidance in direct conflict.

    Args:
        layout: The obstacles actually present.
        scenario: Supplies the staleness fractions.
        rng: Source of randomness.

    Returns:
        The obstacles as the planner's map has them: some displaced, some
        absent, the rest correct.
    """
    if not layout or (scenario.stale_moved <= 0.0 and scenario.stale_unmapped <= 0.0):
        return layout

    n = len(layout)
    order = rng.permutation(n)
    n_unmapped = round(scenario.stale_unmapped * n)
    n_moved = round(scenario.stale_moved * n)
    unmapped = set(order[:n_unmapped].tolist())
    moved = set(order[n_unmapped : n_unmapped + n_moved].tolist())

    believed: list[Obstacle] = []
    for i, ob in enumerate(layout):
        if i in unmapped:
            continue
        if i in moved:
            bearing = float(rng.uniform(-np.pi, np.pi))
            distance = float(rng.uniform(*scenario.stale_move_range))
            believed.append(
                Obstacle(
                    pos=(
                        ob.pos[0] + distance * np.cos(bearing),
                        ob.pos[1] + distance * np.sin(bearing),
                    ),
                    half_extent=ob.half_extent,
                    top=ob.top,
                )
            )
        else:
            believed.append(ob)
    return tuple(believed)


def planner_for(
    scenario: Scenario,
    obstacles: tuple[Obstacle, ...],
    clearance: float = PLANNING_CLEARANCE,
) -> GridPlanner:
    """Build the route planner for this episode's layout.

    Cached for map scenarios, whose geometry never changes. A synthetic
    scenario with random obstacles has to rebuild, since the layout is new
    every reset.

    Args:
        scenario: The task definition.
        obstacles: This episode's obstacles, ignored for a map scenario.
        clearance: Distance the rover's centre keeps from walls, in metres.

    Returns:
        A planner over the traversable cells.
    """
    if scenario.map_yaml is not None:
        key = (scenario.map_yaml, clearance)
        if key not in _PLANNERS:
            _PLANNERS[key] = GridPlanner(grid_for(scenario), clearance)
        return _PLANNERS[key]

    boxes = [
        (ob.pos[0], ob.pos[1], ob.half_extent[0], ob.half_extent[1]) for ob in obstacles
    ]
    return GridPlanner(from_obstacles(scenario.bounds, boxes), clearance)


def build_from_map(scenario: Scenario) -> mujoco.MjModel:
    """Compile the rover inside a saved map's geometry.

    Wall boxes go straight onto the worldbody rather than each into its own
    body. A static geom needs no body, and one body per rectangle makes
    mj_kinematics walk all 849 of them every step: measured 3,500 steps/s
    that way against 67,290 like this.

    Args:
        scenario: Must have `map_yaml` set.

    Returns:
        A compiled model ready for `MjData`.
    """
    grid = grid_for(scenario)
    spec = mujoco.MjSpec.from_file(str(MODEL))
    height = 0.25
    for cx, cy, hx, hy in grid.boxes():
        spec.worldbody.add_geom(
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[hx, hy, height],
            pos=[cx, cy, height],
            rgba=[0.55, 0.55, 0.58, 1.0],
        )
    return spec.compile()


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
    height = 0.25

    for i, ob in enumerate(obstacles):
        # Boxes sit ON the floor, so a top of t means half-height t/2 centred
        # at t/2. Obstacle height is NOT cosmetic any more: the laser plane at
        # 0.176 m sees over anything shorter.
        half_z = ob.top / 2.0
        body = spec.worldbody.add_body(
            name=f"obstacle_{i}", pos=[ob.pos[0], ob.pos[1], half_z]
        )
        body.add_geom(
            name=f"obstacle_{i}_geom",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[ob.half_extent[0], ob.half_extent[1], half_z],
            # Low obstacles drawn differently, so the viewer shows at a glance
            # which ones only the camera can see.
            rgba=[0.90, 0.55, 0.10, 1.0] if ob.top < 0.176 else [0.75, 0.25, 0.20, 1.0],
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
    extra: list[Obstacle] = []

    def draw(half_extent: tuple[float, float], top: float) -> Obstacle:
        margin = max(half_extent) + FOOTPRINT_RADIUS
        return Obstacle(
            pos=(
                float(rng.uniform(-half_x + margin, half_x - margin)),
                float(rng.uniform(-half_y + margin, half_y - margin)),
            ),
            half_extent=half_extent,
            top=top,
        )

    for _ in range(scenario.n_random):
        extra.append(draw(scenario.random_half_extent, 0.50))
    for _ in range(scenario.n_low):
        extra.append(
            draw(scenario.low_half_extent, float(rng.uniform(*scenario.low_top)))
        )
    return (*scenario.obstacles, *extra)


def sample_pose(planner: GridPlanner, rng: np.random.Generator) -> np.ndarray:
    """Draw a reachable (x, y) the rover fits at.

    Drawn from the planner's own reachable set rather than from raw
    clearance, so a start and a goal are always connected by a route. At a
    realistic clearance the clear cells fragment into pockets, and sampling
    across two of them produced episodes with no plannable route at all.

    Args:
        planner: Supplies the reachable cells for this layout.
        rng: Source of randomness.

    Returns:
        A reachable position as (x, y).

    Raises:
        RuntimeError: If the layout has no reachable cell, which means the
            rover does not fit anywhere in it.
    """
    free = planner.free_world
    if len(free) == 0:
        raise RuntimeError("no reachable cell in this layout; the rover does not fit")
    return np.asarray(free[rng.integers(len(free))], dtype=np.float64)


def map_scenario(
    yaml_path: Path,
    name: str | None = None,
    time_limit: float = 60.0,
    min_goal_distance: float = 2.0,
) -> Scenario:
    """Build a scenario from a saved slam_toolbox map.

    Not in SUITE because the maps are gitignored run outputs: a hardcoded
    path would break on a fresh clone. Pass the path explicitly instead.

    Args:
        yaml_path: The map_server `.yaml`, with its `.pgm` beside it.
        name: Scenario name. Defaults to the map's filename stem.
        time_limit: Episode cap in seconds. Short on purpose: a 4 m route
            at 0.40 m/s takes about 10 s, and at 90 s the 7% of episodes
            that stalled ran to 1,875 steps and consumed roughly half of
            all simulated steps. Longer than the synthetic
            scenarios because a real room is bigger than a 10 m box.
        min_goal_distance: Shortest start-to-goal distance worth training on.

    Returns:
        A scenario whose geometry is the real room.
    """
    return Scenario(
        name=name or yaml_path.stem,
        map_yaml=yaml_path,
        time_limit=time_limit,
        min_goal_distance=min_goal_distance,
    )


#: The evaluation suite. Fixed definitions, so scores are comparable between
#: runs and against the Nav2 baseline.
SUITE: tuple[Scenario, ...] = (
    Scenario(name="open_room", n_random=0),
    Scenario(name="sparse", n_random=3, n_low=1, stale_moved=0.5, stale_unmapped=0.3),
    Scenario(
        name="cluttered",
        n_random=8,
        n_low=3,
        time_limit=60.0,
        stale_moved=0.5,
        stale_unmapped=0.3,
    ),
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
