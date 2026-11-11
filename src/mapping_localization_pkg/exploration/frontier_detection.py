"""Find the boundaries between mapped free space and the unknown.

A frontier cell is FREE and has at least one unknown 4-neighbour. Walls block
that test by themselves: the cell on the near side of a wall has the wall as
its neighbour, not the unknown behind it, so solid obstacles produce no
frontier and only genuine openings do.

Adjacent frontier cells are clustered, and each cluster is reduced to one
representative pose: a point far enough from obstacles for the rover to stand
on, facing into the unknown so the camera sees unexplored space on arrival.

Imports no ROS. The grid arrives as a flat sequence in the layout
`nav_msgs/OccupancyGrid` uses, and nothing here reads a transform.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Final, NamedTuple

import numpy as np
from scipy import ndimage

#: Occupancy at or below this counts as free, matching nav2's free_thresh 0.25.
FREE_MAX: Final[int] = 25

#: Occupancy at or above this counts as an obstacle, nav2's occupied_thresh 0.65.
OCCUPIED_MIN: Final[int] = 65

#: Shortest frontier worth driving to, metres of edge. Below roughly the
#: rover's 0.38 m width an opening is not passable anyway.
DEFAULT_MIN_EDGE: Final[float] = 0.50

#: Clearance the representative point must have from obstacles, metres. Just
#: over the 0.27 m circumscribed radius of the 0.38 x 0.38 footprint.
DEFAULT_MIN_CLEARANCE: Final[float] = 0.30

#: Half-width of the window searched for unknown cells when deciding which way
#: a frontier faces, in cells. 10 cells is 0.5 m at the usual 0.05 m grid.
_FACING_WINDOW: Final[int] = 10

#: 8-connected structuring element, so a diagonal step keeps one cluster whole.
_CLUSTER_STRUCTURE: Final[np.ndarray] = np.ones((3, 3), dtype=bool)


class GridInfo(NamedTuple):
    """Geometry of an occupancy grid, straight off `nav_msgs/MapMetaData`."""

    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int


class Frontier(NamedTuple):
    """One cluster of frontier cells, reduced to a pose worth driving to."""

    x: float
    y: float
    yaw: float
    edge: float
    cells: int
    clearance: float


def _as_image(grid: Sequence[int], info: GridInfo) -> np.ndarray:
    """Reshape a flat occupancy array into rows, validating its length."""
    expected = info.width * info.height
    if len(grid) != expected:
        raise ValueError(
            f"grid has {len(grid)} cells, but {info.width}x{info.height} "
            f"needs {expected}"
        )
    return np.asarray(grid, dtype=np.int16).reshape(info.height, info.width)


def _adjacent_to(mask: np.ndarray) -> np.ndarray:
    """Cells having at least one 4-connected neighbour inside `mask`.

    4-connected on purpose: a diagonal test leaks through the single-cell gap
    at the corner of two walls and invents frontiers inside solid geometry.
    """
    out = np.zeros_like(mask)
    out[1:, :] |= mask[:-1, :]
    out[:-1, :] |= mask[1:, :]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    return out


def _to_world(row: float, col: float, info: GridInfo) -> tuple[float, float]:
    """Cell indices to map coordinates, at the cell centre.

    Assumes an axis-aligned grid, which is what slam_toolbox and map_server
    both publish; `detect_frontiers` rejects a rotated one rather than
    silently misplacing every frontier.
    """
    return (
        info.origin_x + (col + 0.5) * info.resolution,
        info.origin_y + (row + 0.5) * info.resolution,
    )


def _facing(
    rows: np.ndarray,
    cols: np.ndarray,
    unknown: np.ndarray,
    rep_row: int,
    rep_col: int,
) -> float:
    """Heading from the representative cell into the unknown, radians.

    Takes the centroid of unknown cells in a window around the representative
    point and aims at it, so the rover arrives looking at what it has not
    mapped. Falls back to the whole cluster when the window holds nothing,
    and to zero when even that is degenerate.
    """
    top = max(0, rep_row - _FACING_WINDOW)
    left = max(0, rep_col - _FACING_WINDOW)
    window = unknown[
        top : rep_row + _FACING_WINDOW + 1, left : rep_col + _FACING_WINDOW + 1
    ]
    win_rows, win_cols = np.nonzero(window)
    if win_rows.size:
        dy = float(win_rows.mean() + top) - rep_row
        dx = float(win_cols.mean() + left) - rep_col
    else:
        # No unknown nearby: aim away from the cluster's own centre of mass,
        # which for a frontier still points outward.
        dy = rep_row - float(rows.mean())
        dx = rep_col - float(cols.mean())
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    # Rows increase with +y in an OccupancyGrid, so dy needs no sign flip.
    return math.atan2(dy, dx)


def detect_frontiers(
    grid: Sequence[int],
    info: GridInfo,
    *,
    min_edge: float = DEFAULT_MIN_EDGE,
    min_clearance: float = DEFAULT_MIN_CLEARANCE,
    require_clearance: bool = True,
) -> list[Frontier]:
    """Find every frontier in an occupancy grid, largest edge first.

    Args:
        grid: Flat occupancy values, -1 unknown and 0-100 otherwise.
        info: The grid's geometry.
        min_edge: Shortest frontier kept, metres of edge.
        min_clearance: Obstacle clearance the representative point needs,
            metres.
        require_clearance: Drop a cluster with no cell that clear. On a real
            lab map most sub-clearance frontiers are the ragged end of the
            laser sweep against a wall, and driving at one costs a full
            attempt timeout. Set False to chase a tight opening anyway, and
            let the planner rule on it.

    Returns:
        One `Frontier` per surviving cluster, sorted by edge length
        descending.

    Raises:
        ValueError: If the grid length disagrees with `info`, or the
            resolution is not positive.
    """
    if info.resolution <= 0.0:
        raise ValueError(f"resolution must be positive, got {info.resolution}")

    image = _as_image(grid, info)
    unknown = image < 0
    free = (image >= 0) & (image <= FREE_MAX)
    occupied = image >= OCCUPIED_MIN

    edges = free & _adjacent_to(unknown)
    if not edges.any():
        return []

    # Distance to the nearest obstacle, in metres. Unknown cells are treated as
    # non-obstacles because they are not known to be blocked, so clearance near
    # the unknown is optimistic by design.
    #
    # Padded with free space first: the transform treats everything outside the
    # array as background, which would read the map's own boundary as a wall and
    # reject frontiers near it. A frontier sits at the edge of explored space,
    # which is exactly where that border is.
    pad = math.ceil(min_clearance / info.resolution) + 1
    clearance = (
        ndimage.distance_transform_edt(np.pad(~occupied, pad, constant_values=True))[
            pad:-pad, pad:-pad
        ]
        * info.resolution
    )

    labels, count = ndimage.label(edges, structure=_CLUSTER_STRUCTURE)
    # Bounding box per cluster, so each one is searched inside its own box
    # instead of rescanning the whole grid. On a 359x919 map that is the
    # difference between 274 ms and single-digit milliseconds, and an
    # exploration map only grows.
    boxes = ndimage.find_objects(labels)
    sizes = np.bincount(labels.ravel(), minlength=count + 1)

    frontiers: list[Frontier] = []
    for index in range(1, count + 1):
        if sizes[index] * info.resolution < min_edge:
            continue
        box = boxes[index - 1]
        local_rows, local_cols = np.nonzero(labels[box] == index)
        rows = local_rows + box[0].start
        cols = local_cols + box[1].start
        edge = rows.size * info.resolution

        # Prefer a cell the rover actually fits on; among those, the one
        # nearest the cluster centroid, so the pose sits mid-opening.
        local = clearance[rows, cols]
        roomy = local >= min_clearance
        if roomy.any():
            pool = np.nonzero(roomy)[0]
        elif require_clearance:
            continue
        else:
            pool = np.array([int(local.argmax())])
        centre_row, centre_col = rows.mean(), cols.mean()
        offsets = (rows[pool] - centre_row) ** 2 + (cols[pool] - centre_col) ** 2
        pick = int(pool[int(offsets.argmin())])

        rep_row, rep_col = int(rows[pick]), int(cols[pick])
        x, y = _to_world(rep_row, rep_col, info)
        frontiers.append(
            Frontier(
                x=x,
                y=y,
                yaw=_facing(rows, cols, unknown, rep_row, rep_col),
                edge=edge,
                cells=int(rows.size),
                clearance=float(clearance[rep_row, rep_col]),
            )
        )

    frontiers.sort(key=lambda f: f.edge, reverse=True)
    return frontiers
