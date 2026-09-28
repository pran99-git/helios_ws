"""Unit tests for path preemption. Pure pytest, no ROS."""

import pytest
from vla_bridge.divergence import DEFAULT_INDEX, should_reissue

STRAIGHT = [(float(i) * 0.25, 0.0) for i in range(8)]


def test_nothing_active_always_reissues():
    assert should_reissue([], STRAIGHT) is True


def test_identical_paths_do_not_reissue():
    """The policy republishes several times a second; most predictions repeat."""
    assert should_reissue(STRAIGHT, list(STRAIGHT)) is False


def test_small_drift_is_tolerated():
    nudged = [(x, y + 0.05) for x, y in STRAIGHT]
    assert should_reissue(STRAIGHT, nudged) is False


def test_real_divergence_reissues():
    swerved = [(x, y + 0.40) for x, y in STRAIGHT]
    assert should_reissue(STRAIGHT, swerved) is True


def test_only_the_comparison_waypoint_matters():
    """A path that agrees mid-chunk but not at the tail is not worth preempting."""
    tail_only = list(STRAIGHT)
    tail_only[-1] = (99.0, 99.0)
    assert should_reissue(STRAIGHT, tail_only, index=DEFAULT_INDEX) is False


def test_index_is_clamped_to_the_shorter_path():
    short = [(0.0, 0.0), (0.25, 0.0)]
    assert should_reissue(short, short, index=DEFAULT_INDEX) is False
    assert should_reissue(short, [(0.0, 0.0), (0.25, 5.0)], index=DEFAULT_INDEX) is True


def test_threshold_is_respected():
    moved = [(x, y + 0.20) for x, y in STRAIGHT]
    assert should_reissue(STRAIGHT, moved, threshold=0.30) is False
    assert should_reissue(STRAIGHT, moved, threshold=0.10) is True


def test_empty_candidate_is_a_caller_error():
    """An empty path means retract, which is a cancel, not a reissue."""
    with pytest.raises(ValueError, match="cancel instead"):
        should_reissue(STRAIGHT, [])
