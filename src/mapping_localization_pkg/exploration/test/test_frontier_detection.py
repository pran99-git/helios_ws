"""Tests for frontier detection.

Grids are written as text art so the geometry under test is visible:
`.` free, `#` occupied, `?` unknown. Row 0 is the bottom row in map
coordinates, matching how an OccupancyGrid is laid out.
"""

from __future__ import annotations

import math

import pytest
from frontier_detection import (
    FREE_MAX,
    OCCUPIED_MIN,
    Frontier,
    GridInfo,
    detect_frontiers,
)

_SYMBOLS = {".": 0, "#": 100, "?": -1}


def grid_from(art: list[str], resolution: float = 0.05, origin: tuple = (0.0, 0.0)):
    """Build a flat occupancy array and its GridInfo from text art.

    Args:
        art: Rows of `.`, `#` and `?`, row 0 first (the bottom row).
        resolution: Metres per cell.
        origin: Map coordinates of the grid's lower-left corner.

    Returns:
        The flat grid and its `GridInfo`.
    """
    width = len(art[0])
    assert all(len(row) == width for row in art), "rows must be equal length"
    cells = [_SYMBOLS[c] for row in art for c in row]
    return cells, GridInfo(resolution, origin[0], origin[1], width, len(art))


class TestThresholds:
    """Occupancy banding, which decides what counts as free at all."""

    def test_free_band_is_inclusive(self):
        """A cell at exactly FREE_MAX is free, so it can be a frontier."""
        cells, info = grid_from(["..?"])
        cells[0] = FREE_MAX
        assert detect_frontiers(cells, info, min_edge=0.0)

    def test_inflation_gradient_is_not_frontier(self):
        """A mid-cost cell is neither free nor an obstacle, so it is skipped."""
        cells, info = grid_from([".?"])
        cells[0] = (FREE_MAX + OCCUPIED_MIN) // 2
        assert detect_frontiers(cells, info, min_edge=0.0) == []


class TestWallsBlockFrontiers:
    """The property the whole approach rests on."""

    def test_unknown_behind_a_wall_is_not_a_frontier(self):
        cells, info = grid_from(
            [
                "...#???",
                "...#???",
                "...#???",
            ]
        )
        assert detect_frontiers(cells, info, min_edge=0.0) == []

    def test_a_gap_in_the_wall_is_a_frontier(self):
        cells, info = grid_from(
            [
                "...#???",
                "....???",
                "...#???",
            ]
        )
        found = detect_frontiers(cells, info, min_edge=0.0, min_clearance=0.0)
        assert len(found) == 1
        assert found[0].cells == 1

    def test_diagonal_corner_does_not_leak(self):
        """The free cell touches the unknown only diagonally, through the gap
        between two walls. Counting that invents frontiers inside geometry."""
        cells, info = grid_from(
            [
                ".#",
                "#?",
            ]
        )
        assert detect_frontiers(cells, info, min_edge=0.0, min_clearance=0.0) == []


class TestClustering:
    """Adjacent cells must collapse to one frontier, separated ones must not."""

    def test_one_opening_is_one_frontier(self):
        cells, info = grid_from(["." * 6, "?" * 6])
        found = detect_frontiers(cells, info, min_edge=0.0, min_clearance=0.0)
        assert len(found) == 1
        assert found[0].cells == 6

    def test_two_openings_stay_separate(self):
        cells, info = grid_from(
            [
                "..........",
                "??###??###",
            ]
        )
        found = detect_frontiers(cells, info, min_edge=0.0, min_clearance=0.0)
        assert len(found) == 2

    def test_diagonal_cells_join_one_cluster(self):
        """Both free cells are frontier and touch only diagonally. Clustering
        is 8-connected, so they are one opening, not two."""
        cells, info = grid_from(
            [
                ".?",
                "?.",
            ]
        )
        found = detect_frontiers(cells, info, min_edge=0.0, min_clearance=0.0)
        assert len(found) == 1
        assert found[0].cells == 2

    def test_sorted_by_edge_descending(self):
        cells, info = grid_from(
            [
                "..........",
                "?####?????",
            ]
        )
        found = detect_frontiers(cells, info, min_edge=0.0, min_clearance=0.0)
        assert [f.cells for f in found] == sorted(
            [f.cells for f in found], reverse=True
        )
        assert found[0].cells > found[-1].cells


class TestFiltering:
    """min_edge and min_clearance, the two rejection knobs."""

    def test_short_frontier_is_dropped(self):
        cells, info = grid_from(["....", "?###"])
        assert detect_frontiers(cells, info, min_edge=0.10, min_clearance=0.0) == []

    def test_edge_length_is_cells_times_resolution(self):
        cells, info = grid_from(["." * 8, "?" * 8], resolution=0.10)
        found = detect_frontiers(cells, info, min_edge=0.0, min_clearance=0.0)
        assert found[0].edge == pytest.approx(0.80)

    def test_tight_opening_dropped_by_default(self):
        """No cell meets the clearance, so the rover cannot stand there.

        On a real lab map these are the ragged end of the laser sweep against
        a wall, and driving at one costs a whole attempt timeout.
        """
        cells, info = grid_from(
            [
                "#.#",
                "#?#",
            ]
        )
        assert detect_frontiers(cells, info, min_edge=0.0, min_clearance=5.0) == []

    def test_tight_opening_kept_when_clearance_not_required(self):
        """Opt back in to chasing a tight gap and let the planner rule on it."""
        cells, info = grid_from(
            [
                "#.#",
                "#?#",
            ]
        )
        found = detect_frontiers(
            cells, info, min_edge=0.0, min_clearance=5.0, require_clearance=False
        )
        assert len(found) == 1
        assert found[0].clearance < 5.0

    def test_map_border_is_not_treated_as_a_wall(self):
        """The transform reads outside the array as background, so without
        padding the map's own edge would reject every frontier near it."""
        cells, info = grid_from(["." * 4, "?" * 4], resolution=0.05)
        found = detect_frontiers(cells, info, min_edge=0.0, min_clearance=0.30)
        assert len(found) == 1, "frontier rejected by the array boundary"

    def test_clearance_prefers_the_roomy_cell(self):
        """A pillar sits at the middle of a five-cell frontier, which is where
        the centroid lands. The pick must step away from it."""
        cells, info = grid_from(
            [
                "?????",
                ".....",
                "..#..",
            ],
            resolution=1.0,
        )
        found = detect_frontiers(cells, info, min_edge=0.0, min_clearance=2.0)
        assert len(found) == 1
        assert found[0].clearance >= 2.0
        assert found[0].x != pytest.approx(2.5), "stayed on the centroid cell"


class TestGeometry:
    """Cell-to-world conversion, the easy thing to get silently wrong."""

    def test_representative_point_is_a_cell_centre(self):
        cells, info = grid_from([".", "?"], resolution=0.20, origin=(0.0, 0.0))
        found = detect_frontiers(cells, info, min_edge=0.0, min_clearance=0.0)
        assert found[0].x == pytest.approx(0.10)
        assert found[0].y == pytest.approx(0.10)

    def test_origin_offset_is_applied(self):
        cells, info = grid_from([".", "?"], resolution=0.20, origin=(-3.0, 7.5))
        found = detect_frontiers(cells, info, min_edge=0.0, min_clearance=0.0)
        assert found[0].x == pytest.approx(-2.90)
        assert found[0].y == pytest.approx(7.60)

    def test_row_zero_is_the_low_y_row(self):
        """Unknown in row 1 sits at higher y, so the frontier faces +y."""
        cells, info = grid_from([".", "?"], resolution=1.0)
        found = detect_frontiers(cells, info, min_edge=0.0, min_clearance=0.0)
        assert found[0].yaw == pytest.approx(math.pi / 2, abs=1e-6)


class TestFacing:
    """The pose must look INTO the unknown, so arriving gives the camera a view."""

    @pytest.mark.parametrize(
        "art, expected",
        [
            ([".", "?"], math.pi / 2),
            (["?", "."], -math.pi / 2),
            ([".?"], 0.0),
            (["?."], math.pi),
        ],
    )
    def test_cardinal_directions(self, art, expected):
        cells, info = grid_from(art, resolution=1.0)
        found = detect_frontiers(cells, info, min_edge=0.0, min_clearance=0.0)
        got = found[0].yaw
        assert math.isclose(
            math.atan2(math.sin(got - expected), math.cos(got - expected)),
            0.0,
            abs_tol=1e-6,
        ), f"{math.degrees(got):.1f} deg, expected {math.degrees(expected):.1f}"

    def test_faces_the_bulk_of_the_unknown(self):
        """A frontier at the mouth of a corridor aims down it, not sideways."""
        cells, info = grid_from(
            [
                "#####",
                "##.##",
                "##?##",
                "##?##",
                "##?##",
            ],
            resolution=0.10,
        )
        found = detect_frontiers(cells, info, min_edge=0.0, min_clearance=0.0)
        assert len(found) == 1
        assert found[0].yaw == pytest.approx(math.pi / 2, abs=1e-6)


class TestDegenerate:
    """Inputs that must not raise or hang."""

    def test_fully_unknown_grid_has_no_frontier(self):
        cells, info = grid_from(["???", "???"])
        assert detect_frontiers(cells, info, min_edge=0.0) == []

    def test_fully_free_grid_has_no_frontier(self):
        cells, info = grid_from(["...", "..."])
        assert detect_frontiers(cells, info, min_edge=0.0) == []

    def test_wrong_length_is_rejected(self):
        cells, info = grid_from(["..."])
        with pytest.raises(ValueError, match="needs"):
            detect_frontiers(cells[:-1], info)

    def test_non_positive_resolution_is_rejected(self):
        cells, _ = grid_from(["..."])
        with pytest.raises(ValueError, match="resolution"):
            detect_frontiers(cells, GridInfo(0.0, 0.0, 0.0, 3, 1))

    def test_frontier_is_a_named_tuple(self):
        """Callers unpack these; keep the shape stable."""
        cells, info = grid_from([".", "?"])
        found = detect_frontiers(cells, info, min_edge=0.0, min_clearance=0.0)
        assert isinstance(found[0], Frontier)
        assert found[0]._fields == (
            "x",
            "y",
            "yaw",
            "edge",
            "cells",
            "clearance",
        )
