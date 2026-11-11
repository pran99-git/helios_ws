"""Tests for occupancy grid import and rectangle merging."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml
from envs.occupancy import OccupancyGrid, load_ros_map
from PIL import Image


def grid(occupied: np.ndarray, resolution: float = 0.05) -> OccupancyGrid:
    return OccupancyGrid(occupied=occupied, resolution=resolution, origin=(0.0, 0.0))


def write_map(
    tmp_path: Path,
    pixels: np.ndarray,
    *,
    negate: int = 0,
    resolution: float = 0.05,
    origin: tuple[float, float] = (0.0, 0.0),
) -> Path:
    """Write a map_server pair and return the yaml path."""
    Image.fromarray(pixels.astype(np.uint8), mode="L").save(tmp_path / "m.pgm")
    y = tmp_path / "m.yaml"
    y.write_text(
        yaml.safe_dump(
            {
                "image": "m.pgm",
                "resolution": resolution,
                "origin": [origin[0], origin[1], 0.0],
                "negate": negate,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }
        )
    )
    return y


# --------------------------------------------------------------- geometry ---


def test_row_zero_is_maximum_y() -> None:
    """Image rows run downward while world y runs up; a flip here is silent."""
    g = grid(np.zeros((3, 2), bool))
    top = g.to_world(0, 0)
    bottom = g.to_world(2, 0)
    assert top[1] > bottom[1]
    assert bottom == pytest.approx((0.0, 0.0))
    assert top == pytest.approx((0.0, 0.10))


def test_column_increases_x() -> None:
    g = grid(np.zeros((2, 3), bool))
    assert g.to_world(0, 2)[0] == pytest.approx(0.10)


def test_extent_is_metres() -> None:
    assert grid(np.zeros((10, 20), bool)).extent == pytest.approx((1.0, 0.5))


# ------------------------------------------------------------ rectangles ---


def test_rectangles_cover_every_occupied_cell_exactly_once() -> None:
    rng = np.random.default_rng(0)
    occ = rng.random((20, 25)) < 0.3
    g = grid(occ)
    counts = np.zeros_like(occ, dtype=int)
    for r, c, r2, c2 in g.rectangles():
        counts[r : r2 + 1, c : c2 + 1] += 1
    assert np.array_equal(counts > 0, occ)
    assert counts.max() <= 1


def test_rectangles_never_cover_free_cells() -> None:
    occ = np.zeros((6, 6), bool)
    occ[2:4, 1:5] = True
    g = grid(occ)
    for r, c, r2, c2 in g.rectangles():
        assert occ[r : r2 + 1, c : c2 + 1].all()


def test_a_solid_block_becomes_one_rectangle() -> None:
    occ = np.zeros((5, 5), bool)
    occ[1:4, 1:4] = True
    assert len(grid(occ).rectangles()) == 1


def test_empty_grid_has_no_rectangles() -> None:
    assert grid(np.zeros((4, 4), bool)).rectangles() == []


def test_merging_beats_one_box_per_cell() -> None:
    """A wall is a line of cells and must not become a box per cell."""
    occ = np.zeros((20, 20), bool)
    occ[0, :] = True
    occ[:, 0] = True
    assert len(grid(occ).rectangles()) < occ.sum() / 5


def test_boxes_cover_the_full_cell_area() -> None:
    occ = np.zeros((3, 3), bool)
    occ[1, 1] = True
    (cx, cy, hx, hy) = grid(occ).boxes()[0]
    # One cell at 0.05 m gives a half extent of 0.025, not zero.
    assert hx == pytest.approx(0.025)
    assert hy == pytest.approx(0.025)
    assert (cx, cy) == pytest.approx((0.05, 0.05))


# --------------------------------------------------------------- sampling ---


def test_free_positions_respects_clearance() -> None:
    occ = np.zeros((40, 40), bool)
    occ[20, 20] = True
    g = grid(occ)
    wall = np.array(g.to_world(20, 20))
    for point in g.free_positions(0.30):
        assert np.linalg.norm(point - wall) >= 0.30 - 1e-9


def test_free_positions_treats_the_border_as_wall() -> None:
    """Without padding, the transform reads the map edge as open space."""
    g = grid(np.zeros((20, 20), bool))
    free = g.free_positions(0.20)
    corner = np.array(g.to_world(0, 0))
    assert all(np.linalg.norm(p - corner) >= 0.20 - 1e-9 for p in free)


def test_free_positions_empty_when_nothing_fits() -> None:
    assert len(grid(np.zeros((5, 5), bool)).free_positions(10.0)) == 0


# ---------------------------------------------------------------- loading ---


def test_dark_pixels_are_occupied(tmp_path: Path) -> None:
    pixels = np.full((4, 4), 254, np.uint8)
    pixels[1, 1] = 0
    g = load_ros_map(write_map(tmp_path, pixels), unknown_is_occupied=False)
    assert g.occupied.sum() == 1
    assert g.occupied[1, 1]


def test_negate_inverts_the_reading(tmp_path: Path) -> None:
    pixels = np.full((4, 4), 254, np.uint8)
    pixels[1, 1] = 0
    g = load_ros_map(write_map(tmp_path, pixels, negate=1), unknown_is_occupied=False)
    assert not g.occupied[1, 1]
    assert g.occupied.sum() == 15


def test_unknown_cells_can_be_sealed(tmp_path: Path) -> None:
    """Mid-grey is unknown: sealing it stops a policy escaping the map."""
    pixels = np.full((4, 4), 128, np.uint8)
    pixels[0, 0] = 254
    assert load_ros_map(write_map(tmp_path, pixels)).occupied.sum() == 15
    open_map = load_ros_map(write_map(tmp_path, pixels), unknown_is_occupied=False)
    assert open_map.occupied.sum() == 0


def test_metadata_is_carried_through(tmp_path: Path) -> None:
    y = write_map(
        tmp_path, np.full((4, 4), 254, np.uint8), resolution=0.1, origin=(-1.5, 2.0)
    )
    g = load_ros_map(y)
    assert g.resolution == pytest.approx(0.1)
    assert g.origin == pytest.approx((-1.5, 2.0))


def test_missing_yaml_raises(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="no map yaml"):
        load_ros_map(tmp_path / "absent.yaml")


def test_missing_image_raises(tmp_path: Path) -> None:
    y = write_map(tmp_path, np.full((4, 4), 254, np.uint8))
    (tmp_path / "m.pgm").unlink()
    with pytest.raises(FileNotFoundError, match="map image"):
        load_ros_map(y)


def test_incomplete_yaml_raises(tmp_path: Path) -> None:
    y = tmp_path / "bad.yaml"
    y.write_text(yaml.safe_dump({"image": "m.pgm", "resolution": 0.05}))
    with pytest.raises(ValueError, match="has no 'origin'"):
        load_ros_map(y)
