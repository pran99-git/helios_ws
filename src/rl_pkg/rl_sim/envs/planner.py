"""Plans the global route the policy is trained to follow.

On the rover this job belongs to Nav2's Smac planner. The simulator needs its
own, because the policy is rewarded for progress ALONG a route and there is no
route without a planner.

Dijkstra over the occupancy grid, on cells that clear the footprint. Grid
graphs are built once per layout and cached, so a reset only pays for the
search itself.
"""

from __future__ import annotations

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from envs.occupancy import OccupancyGrid
from envs.route import Pose2D

#: 8-connected neighbour offsets with their edge costs in cells. Diagonals
#: cost sqrt(2) so the route does not prefer staircases to straight lines.
_NEIGHBOURS: tuple[tuple[int, int, float], ...] = (
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, np.sqrt(2.0)),
    (-1, 1, np.sqrt(2.0)),
    (1, -1, np.sqrt(2.0)),
    (1, 1, np.sqrt(2.0)),
)


class GridPlanner:
    """Shortest paths over one layout's traversable cells."""

    def __init__(self, grid: OccupancyGrid, clearance: float) -> None:
        """Build the traversability mask and the grid graph.

        Args:
            grid: The layout to plan over.
            clearance: Distance the rover's centre must keep from any
                occupied cell, in metres. Pass the footprint's circumscribed
                radius so the route fits the body, not just its centre.
        """
        self._grid = grid
        rows, cols = grid.shape
        # Padded so the array border reads as wall; see OccupancyGrid for why.
        padded = np.pad(grid.occupied, 1, constant_values=True)
        distance = np.asarray(distance_transform_edt(~padded))[1:-1, 1:-1]
        self._free = distance * grid.resolution >= clearance

        index = -np.ones((rows, cols), dtype=np.int64)
        free_rc = np.argwhere(self._free)
        index[self._free] = np.arange(len(free_rc))
        self._index = index
        self._cells = free_rc
        self._n = len(free_rc)
        # World coordinates of every traversable cell, vectorised once.
        # to_world in a loop per lookup cost 85 ms a plan on the lab map.
        self._world = np.column_stack(
            (
                grid.origin[0] + free_rc[:, 1] * grid.resolution,
                grid.origin[1] + (rows - 1 - free_rc[:, 0]) * grid.resolution,
            )
        )

        rows_i: list[np.ndarray] = []
        cols_i: list[np.ndarray] = []
        costs: list[np.ndarray] = []
        for dr, dc, cost in _NEIGHBOURS:
            source = index[
                max(0, -dr) : rows - max(0, dr), max(0, -dc) : cols - max(0, dc)
            ]
            target = index[
                max(0, dr) : rows - max(0, -dr), max(0, dc) : cols - max(0, -dc)
            ]
            both = (source >= 0) & (target >= 0)
            rows_i.append(source[both])
            cols_i.append(target[both])
            costs.append(np.full(int(both.sum()), cost * grid.resolution))

        self._graph = coo_matrix(
            (
                np.concatenate(costs),
                (np.concatenate(rows_i), np.concatenate(cols_i)),
            ),
            shape=(self._n, self._n),
        ).tocsr()

    @property
    def traversable(self) -> np.ndarray:
        """Boolean grid of cells whose centre the rover can occupy."""
        return self._free

    def _nearest_node(self, point: np.ndarray) -> int:
        """Index of the traversable cell closest to a world position."""
        if self._n == 0:
            raise RuntimeError("no traversable cells in this layout")
        return int(np.argmin(np.linalg.norm(self._world - point[:2], axis=1)))

    def plan(self, start: np.ndarray, goal: np.ndarray) -> list[Pose2D]:
        """Shortest traversable route between two world positions.

        Args:
            start: Start as (x, y) in metres.
            goal: Goal as (x, y) in metres.

        Returns:
            The route as world-frame poses, start first. A two-point straight
            line when start and goal share a cell.

        Raises:
            RuntimeError: If no route exists, which means the goal is walled
                off from the start.
        """
        source = self._nearest_node(goal)
        target = self._nearest_node(start)
        if source == target:
            return [
                Pose2D(float(start[0]), float(start[1]), 0.0),
                Pose2D(float(goal[0]), float(goal[1]), 0.0),
            ]

        # Searched FROM the goal, so predecessors point along the direction of
        # travel and the route is read off by walking from the start.
        _, predecessors = dijkstra(
            self._graph, directed=False, indices=source, return_predecessors=True
        )
        if predecessors[target] < 0:
            raise RuntimeError("goal is unreachable from the start")

        poses = [Pose2D(float(start[0]), float(start[1]), 0.0)]
        node = target
        while node != source:
            r, c = self._cells[node]
            poses.append(Pose2D(*self._grid.to_world(int(r), int(c)), 0.0))
            node = predecessors[node]
            if node < 0:  # pragma: no cover - guarded by the check above
                raise RuntimeError("route trace broke")
        poses.append(Pose2D(float(goal[0]), float(goal[1]), 0.0))
        return _thin(poses)


def _thin(poses: list[Pose2D], tolerance: float = 0.05) -> list[Pose2D]:
    """Drop points that lie on the straight line between their neighbours.

    A cell-by-cell route has a vertex every 5 cm, which makes projection and
    arc-length needlessly expensive and gives the observation nothing extra.

    Args:
        poses: The dense route.
        tolerance: Largest perpendicular deviation to treat as collinear, in
            metres.

    Returns:
        The route with interior collinear points removed. Ends are kept.
    """
    if len(poses) <= 2:
        return poses
    out = [poses[0]]
    for i in range(1, len(poses) - 1):
        # Measured against the last KEPT point, not the previous dense one, so
        # a long gentle curve accumulates deviation instead of being thinned
        # away one imperceptible step at a time.
        anchor, current, following = out[-1], poses[i], poses[i + 1]
        span = np.array([following.x - anchor.x, following.y - anchor.y])
        length = float(np.linalg.norm(span))
        if length < 1e-9:
            continue
        offset = np.array([current.x - anchor.x, current.y - anchor.y])
        # Perpendicular distance from `current` to the anchor-following line:
        # the 2D cross product's magnitude over the span length. Written out
        # because numpy 2 dropped cross() for 2-vectors.
        deviation = abs(span[0] * offset[1] - span[1] * offset[0]) / length
        if deviation > tolerance:
            out.append(current)
    out.append(poses[-1])
    return out
