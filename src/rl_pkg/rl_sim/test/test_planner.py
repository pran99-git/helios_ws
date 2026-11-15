"""Tests for the route planner and its dead-end behaviour."""

from __future__ import annotations

import numpy as np
import pytest
from envs.occupancy import from_obstacles
from envs.planner import GridPlanner
from envs.route import point_at_arc, project, route_length

FOOTPRINT = 0.269

#: A U opening toward the goal: a back wall at x=1 with arms top and bottom.
#: Straight-line progress rewards diving in here, route progress does not.
U_TRAP = [(1.0, 0.0, 0.1, 1.2), (0.0, 1.2, 1.1, 0.1), (0.0, -1.2, 1.1, 0.1)]


@pytest.fixture
def open_room() -> GridPlanner:
    return GridPlanner(from_obstacles((3.0, 3.0), []), FOOTPRINT)


@pytest.fixture
def trap() -> GridPlanner:
    return GridPlanner(from_obstacles((3.0, 3.0), U_TRAP), FOOTPRINT)


def test_open_room_route_is_near_straight(open_room: GridPlanner) -> None:
    start, goal = np.array([-2.0, 0.0]), np.array([2.0, 0.0])
    route = open_room.plan(start, goal)
    assert route_length(route) == pytest.approx(4.0, abs=0.25)


def test_route_starts_at_start_and_ends_at_goal(open_room: GridPlanner) -> None:
    start, goal = np.array([-1.0, -1.0]), np.array([1.5, 0.5])
    route = open_room.plan(start, goal)
    assert (route[0].x, route[0].y) == pytest.approx((-1.0, -1.0))
    assert (route[-1].x, route[-1].y) == pytest.approx((1.5, 0.5))


def test_trap_route_goes_around_not_through(trap: GridPlanner) -> None:
    start, goal = np.array([-0.5, 0.0]), np.array([2.0, 0.0])
    route = trap.plan(start, goal)
    straight = float(np.linalg.norm(goal - start))
    assert route_length(route) > 1.5 * straight


def test_driving_into_the_trap_earns_no_route_progress(trap: GridPlanner) -> None:
    """The whole point of fix 3.

    Moving from x=-0.5 to x=+0.6 along y=0 heads into the dead end. Straight
    line distance to the goal falls by 1.1 m, which the old reward paid for.
    Arc length along the route must not move.
    """
    route = trap.plan(np.array([-0.5, 0.0]), np.array([2.0, 0.0]))
    total = route_length(route)
    before = total - project(route, -0.5, 0.0).arc
    after = total - project(route, 0.6, 0.0).arc
    assert after >= before - 1e-6

    straight_before = abs(2.0 - -0.5)
    straight_after = abs(2.0 - 0.6)
    assert straight_after < straight_before - 1.0


def test_sealed_region_is_excluded_from_the_reachable_set() -> None:
    """Rather than failing to plan, a walled-off pocket is never sampled.

    The planner keeps only the largest connected component, so a start and a
    goal drawn from `free_world` always have a route. Before that filter,
    episodes were being generated with no plannable route at all and reset
    raised after twenty redraws.
    """
    walls = [
        (2.0, 0.0, 0.1, 0.8),
        (1.0, 0.0, 0.1, 0.8),
        (1.5, 0.8, 0.6, 0.1),
        (1.5, -0.8, 0.6, 0.1),
    ]
    planner = GridPlanner(from_obstacles((3.0, 3.0), walls), FOOTPRINT)
    inside = np.array([1.5, 0.0])
    assert len(planner.free_world) > 0
    nearest = np.min(np.linalg.norm(planner.free_world - inside, axis=1))
    assert nearest > 0.3, "the sealed pocket must not be offered as a pose"


def test_every_sampled_pair_is_plannable() -> None:
    """The property the component filter exists to guarantee."""
    planner = GridPlanner(from_obstacles((3.0, 3.0), U_TRAP), FOOTPRINT)
    rng = np.random.default_rng(0)
    free = planner.free_world
    for _ in range(25):
        a = free[rng.integers(len(free))]
        b = free[rng.integers(len(free))]
        assert len(planner.plan(a, b)) >= 2


def test_route_keeps_clear_of_obstacles(trap: GridPlanner) -> None:
    """Every vertex must be somewhere the footprint actually fits."""
    route = trap.plan(np.array([-0.5, 0.0]), np.array([2.0, 0.0]))
    for pose in route[1:-1]:
        for cx, cy, hx, hy in U_TRAP:
            delta = np.array([abs(pose.x - cx) - hx, abs(pose.y - cy) - hy])
            assert float(np.linalg.norm(np.maximum(delta, 0.0))) >= FOOTPRINT - 0.08


def test_traversable_excludes_cells_too_close_to_walls(trap: GridPlanner) -> None:
    assert trap.traversable.any()
    assert not trap.traversable.all()


def test_thinning_removes_collinear_vertices(open_room: GridPlanner) -> None:
    """A cell-by-cell route would carry a vertex every 5 cm."""
    route = open_room.plan(np.array([-2.5, 0.0]), np.array([2.5, 0.0]))
    assert len(route) < 10


def test_same_cell_start_and_goal_is_a_two_point_route(
    open_room: GridPlanner,
) -> None:
    point = np.array([0.0, 0.0])
    route = open_room.plan(point, point)
    assert len(route) == 2


def test_route_is_usable_by_project(open_room: GridPlanner) -> None:
    """Guards the contract with vla_bridge's geometry."""
    route = open_room.plan(np.array([-2.0, 0.0]), np.array([2.0, 0.0]))
    projection = project(route, -2.0, 0.0)
    assert projection.arc == pytest.approx(0.0, abs=0.1)
    assert projection.offset == pytest.approx(0.0, abs=0.1)


def test_thinning_preserves_the_shape_of_the_route(trap: GridPlanner) -> None:
    """Thinning must keep the detour, not collapse it to a straight line.

    Comparing each candidate only against its next dense neighbour leaves a
    deviation of at most a cell width, which never reaches the tolerance, so
    the whole route collapsed to its two endpoints. An 88 vertex, 5.63 m path
    around obstacles became a 2 vertex straight line through them.
    """
    start, goal = np.array([-0.5, 0.0]), np.array([2.0, 0.0])
    route = trap.plan(start, goal)
    straight = float(np.linalg.norm(goal - start))
    assert len(route) > 2, "a route around a trap cannot be two points"
    assert route_length(route) > 1.3 * straight


def test_thinned_route_keeps_its_clearance(trap: GridPlanner) -> None:
    """Sampled along its length, not just at the vertices.

    A straightened segment cuts the corner by up to the thinning tolerance,
    so the whole path has to be checked, not the kept points.
    """
    route = trap.plan(np.array([-0.5, 0.0]), np.array([2.0, 0.0]))
    total = route_length(route)
    worst = 99.0
    for arc in np.arange(0.0, total, 0.05):
        p = point_at_arc(route, float(arc))
        for cx, cy, hx, hy in U_TRAP:
            delta = np.array([abs(p.x - cx) - hx, abs(p.y - cy) - hy])
            worst = min(worst, float(np.linalg.norm(np.maximum(delta, 0.0))))
    assert worst > FOOTPRINT - 0.05
