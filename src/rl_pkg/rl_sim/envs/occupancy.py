"""Turns a saved slam_toolbox map into a MuJoCo-ready world.

Training in the room the rover actually drives in removes a whole class of
sim-to-real gap: the geometry, the corridor widths and the clutter are the real
ones, not a guess at them.

Reads the ROS map_server pair that `slam_toolbox` writes, a `.pgm` image beside
a `.yaml` of metadata, and reduces the occupied cells to as few axis-aligned
rectangles as possible. One box geom per cell would be 4,805 geoms for the lab
map; merging gets that to 849.

Imports nothing from ROS or MuJoCo.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt


@dataclass(frozen=True)
class OccupancyGrid:
    """A thresholded occupancy grid with its world placement.

    Attributes:
        occupied: Boolean grid, True where the rover cannot go. Row 0 is the
            TOP of the image, which is MAXIMUM y in world coordinates.
        resolution: Metres per cell.
        origin: World position of the grid's bottom-left corner, (x, y).
    """

    occupied: np.ndarray
    resolution: float
    origin: tuple[float, float]

    @property
    def shape(self) -> tuple[int, int]:
        """Grid size as (rows, cols)."""
        return (int(self.occupied.shape[0]), int(self.occupied.shape[1]))

    @property
    def extent(self) -> tuple[float, float]:
        """Grid size in metres as (x, y)."""
        rows, cols = self.shape
        return (cols * self.resolution, rows * self.resolution)

    def to_world(self, row: int, col: int) -> tuple[float, float]:
        """Convert a cell index to the world position of its centre.

        Args:
            row: Grid row, 0 at the top of the image.
            col: Grid column, 0 at the left.

        Returns:
            The cell centre as (x, y) in metres.
        """
        rows, _ = self.shape
        # Image rows increase downward while world y increases upward, so the
        # row index is mirrored. Getting this wrong flips the map vertically,
        # which is not obvious from a symmetric room.
        return (
            self.origin[0] + col * self.resolution,
            self.origin[1] + (rows - 1 - row) * self.resolution,
        )

    def rectangles(self) -> list[tuple[int, int, int, int]]:
        """Cover the occupied cells with few axis-aligned rectangles.

        Greedy, scanning row-major: grow each seed cell as far right as it
        goes, then down for as long as the whole span stays occupied, then
        consume it. Not minimal, but it runs in milliseconds and gets roughly
        a 6x reduction on real maps, which is what matters.

        Returns:
            Rectangles as (row0, col0, row1, col1), inclusive on both ends.
        """
        remaining = self.occupied.copy()
        rows, cols = self.shape
        out: list[tuple[int, int, int, int]] = []
        for r in range(rows):
            c = 0
            while c < cols:
                if not remaining[r, c]:
                    c += 1
                    continue
                c2 = c
                while c2 + 1 < cols and remaining[r, c2 + 1]:
                    c2 += 1
                r2 = r
                while r2 + 1 < rows and remaining[r2 + 1, c : c2 + 1].all():
                    r2 += 1
                remaining[r : r2 + 1, c : c2 + 1] = False
                out.append((r, c, r2, c2))
                c = c2 + 1
        return out

    def boxes(self) -> list[tuple[float, float, float, float]]:
        """The rectangles as world-frame boxes.

        Returns:
            One (centre_x, centre_y, half_x, half_y) per rectangle, in metres.
            Half extents include a half cell on each side, so a box covers its
            cells' full area rather than just their centres.
        """
        half = self.resolution / 2.0
        out: list[tuple[float, float, float, float]] = []
        for r, c, r2, c2 in self.rectangles():
            x0, y1 = self.to_world(r, c)
            x1, y0 = self.to_world(r2, c2)
            out.append(
                (
                    (x0 + x1) / 2.0,
                    (y0 + y1) / 2.0,
                    abs(x1 - x0) / 2.0 + half,
                    abs(y1 - y0) / 2.0 + half,
                )
            )
        return out

    def free_positions(self, clearance: float) -> np.ndarray:
        """World positions where the rover fits, for sampling starts and goals.

        Args:
            clearance: Required distance from any occupied cell, in metres.
                Pass the footprint's circumscribed radius plus a margin.

        Returns:
            An (n, 2) array of world (x, y). Empty if nothing is clear enough.
        """
        # The grid is padded with occupied cells before the transform, because
        # distance_transform_edt treats everything outside the array as free
        # and would otherwise report the map's own border as wide open.
        padded = np.pad(self.occupied, 1, constant_values=True)
        distance = distance_transform_edt(~padded)[1:-1, 1:-1]
        rows, cols = np.nonzero(np.asarray(distance) * self.resolution >= clearance)
        if rows.size == 0:
            return np.empty((0, 2))
        return np.array(
            [self.to_world(int(r), int(c)) for r, c in zip(rows, cols, strict=True)]
        )


def load_ros_map(path: Path, unknown_is_occupied: bool = True) -> OccupancyGrid:
    """Read a map_server `.yaml` and its image.

    Args:
        path: The `.yaml` file. The image is resolved beside it.
        unknown_is_occupied: Whether never-observed cells block the rover.
            True seals the room, which is usually what you want: the rover
            cannot see into unmapped space, so it should not drive there.
            False leaves the map open at its edges.

    Returns:
        The thresholded grid.

    Raises:
        FileNotFoundError: If the yaml or its image is missing.
        ValueError: If the yaml lacks a field this needs.
    """
    if not path.is_file():
        raise FileNotFoundError(f"no map yaml at {path}")
    meta = yaml.safe_load(path.read_text())
    for key in ("image", "resolution", "origin", "occupied_thresh", "free_thresh"):
        if key not in meta:
            raise ValueError(f"{path.name} has no {key!r}")

    image_path = path.parent / meta["image"]
    if not image_path.is_file():
        raise FileNotFoundError(f"map image {image_path} referenced by {path.name}")

    pixels = np.asarray(Image.open(image_path).convert("L")).astype(np.float64)
    # map_server's convention: a DARK pixel is occupied, so occupancy is the
    # inverted intensity. negate flips that.
    probability = pixels / 255.0 if meta.get("negate", 0) else (255.0 - pixels) / 255.0
    occupied = probability >= float(meta["occupied_thresh"])
    if unknown_is_occupied:
        free = probability <= float(meta["free_thresh"])
        occupied = occupied | ~(occupied | free)

    return OccupancyGrid(
        occupied=occupied,
        resolution=float(meta["resolution"]),
        origin=(float(meta["origin"][0]), float(meta["origin"][1])),
    )
