"""Unit tests for waypoint chunk decoding. Pure pytest, no ROS."""

import math

import pytest
from vla_bridge.chunk_decoding import ROW_WIDTH, decode_chunk


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
