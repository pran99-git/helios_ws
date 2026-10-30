"""Unit tests for waypoint chunk decoding. Pure pytest, no ROS."""

import math
from itertools import pairwise

import pytest
from vla_bridge.chunk_decoding import ROW_WIDTH, Waypoint, decode_chunk, densify


def test_displacements_pass_through_unchanged():
    """dx/dy are cumulative from the model; decoding must not integrate them.

    Re-integrating here would square the trajectory's reach, which shows up as
    the rover massively overshooting. Pinning it so that cannot regress.
    """
    chunk = [[1.0, 0.0, 1.0, 0.0], [2.0, 0.0, 1.0, 0.0], [3.0, 0.0, 1.0, 0.0]]
    assert [w.x for w in decode_chunk(chunk)] == [1.0, 2.0, 3.0]


def test_yaw_comes_from_atan2_of_sin_over_cos():
    chunk = [[0.0, 0.0, 0.0, 1.0]]
    assert decode_chunk(chunk)[0].yaw == pytest.approx(math.pi / 2)


def test_left_is_positive_y():
    """REP-103, matching base_link and the model's own convention."""
    assert decode_chunk([[0.0, 0.5, 1.0, 0.0]])[0].y == pytest.approx(0.5)


def test_real_chunk_from_the_checkpoint():
    """Regression fixture: the first and last rows the live model produced.

    Captured from omnivla-edge against the vendor's sample frame, already
    scaled to metres by the server.
    """
    chunk = [
        [0.12248, 0.00146, 0.99963, 0.02702],
        [1.05336, -0.21686, 0.97116, -0.23841],
    ]
    first, last = decode_chunk(chunk)
    assert first.x == pytest.approx(0.12248)
    assert last.y == pytest.approx(-0.21686)
    assert last.yaw == pytest.approx(math.atan2(-0.23841, 0.97116))


def test_wrong_row_width_is_rejected():
    with pytest.raises(ValueError, match=f"expected {ROW_WIDTH}"):
        decode_chunk([[1.0, 2.0]])


# --- densify ------------------------------------------------------------------

REAL_CHUNK = [
    Waypoint(0.137, 0.002, 0.0),
    Waypoint(0.248, 0.010, 0.0),
    Waypoint(0.364, 0.021, 0.0),
    Waypoint(0.476, 0.035, 0.0),
    Waypoint(0.581, 0.046, 0.0),
    Waypoint(0.685, 0.064, 0.0),
    Waypoint(0.799, 0.085, 0.0),
    Waypoint(1.042, 0.177, 0.0),
]


def test_densify_reaches_the_indices_nav2_critics_address():
    """PathFollowCritic reads path index 15 and PathAlignCritic index 20.

    A raw 8-pose chunk has neither, so the critic scoring forward progress
    contributes nothing and the goal aborts with FAILED_TO_MAKE_PROGRESS.
    """
    dense = densify(REAL_CHUNK, 0.05)
    assert len(dense) > 20


def test_densify_anchors_at_the_robot():
    """Nav2 expects a path starting where the robot is, not 0.14 m ahead."""
    assert densify(REAL_CHUNK, 0.05)[0] == Waypoint(
        0.0, 0.0, pytest.approx(0.0146, abs=1e-3)
    )


def test_densify_preserves_the_final_waypoint_exactly():
    """Rounding must never move the goal the policy actually chose."""
    assert densify(REAL_CHUNK, 0.05)[-1] == REAL_CHUNK[-1]


def test_densify_respects_the_spacing():
    dense = densify(REAL_CHUNK, 0.05)
    gaps = [math.dist((a.x, a.y), (b.x, b.y)) for a, b in pairwise(dense)]
    assert max(gaps) <= 0.05 + 1e-6


def test_densify_does_not_move_the_curve():
    """Every original waypoint must still lie on the resampled polyline."""
    dense = densify(REAL_CHUNK, 0.05)
    for wp in REAL_CHUNK:
        assert min(math.dist((wp.x, wp.y), (d.x, d.y)) for d in dense) < 0.026


def test_densify_handles_a_single_waypoint():
    assert len(densify([Waypoint(0.5, 0.0, 0.0)], 0.05)) >= 2


def test_densify_rejects_nonpositive_spacing():
    with pytest.raises(ValueError, match="spacing must be positive"):
        densify(REAL_CHUNK, 0.0)
