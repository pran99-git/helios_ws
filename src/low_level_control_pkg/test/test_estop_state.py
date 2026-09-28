"""Unit tests for the latching emergency stop. Pure pytest, no ROS."""

import pytest
from estop.estop_state import EstopState


def test_starts_disengaged():
    assert EstopState().engaged is False


def test_a_single_sample_engages():
    """The button may be read once between /joy messages; one sample is enough."""
    assert EstopState().update(engage=True, clear=False, now=0.0) is True


def test_it_latches_after_the_button_is_released():
    """This is the whole point: releasing the button must not re-arm the rover."""
    s = EstopState()
    s.update(engage=True, clear=False, now=0.0)
    assert s.update(engage=False, clear=False, now=0.1) is True
    assert s.update(engage=False, clear=False, now=99.0) is True


def test_clearing_requires_the_full_hold():
    s = EstopState(clear_hold=1.0)
    s.update(engage=True, clear=False, now=0.0)
    assert s.update(engage=False, clear=True, now=1.0) is True  # hold starts
    assert s.update(engage=False, clear=True, now=1.5) is True  # not yet
    assert s.update(engage=False, clear=True, now=2.0) is False  # released


def test_releasing_the_clear_button_restarts_the_hold():
    """A bounce or a brush must not accumulate toward a clear."""
    s = EstopState(clear_hold=1.0)
    s.update(engage=True, clear=False, now=0.0)
    s.update(engage=False, clear=True, now=1.0)
    s.update(engage=False, clear=True, now=1.9)
    s.update(engage=False, clear=False, now=1.95)  # let go just short
    s.update(engage=False, clear=True, now=2.0)  # start over
    assert s.update(engage=False, clear=True, now=2.9) is True


def test_engage_beats_a_clear_in_progress():
    """Pressing stop while clearing must stop, not race."""
    s = EstopState(clear_hold=1.0)
    s.update(engage=True, clear=False, now=0.0)
    s.update(engage=False, clear=True, now=1.0)
    s.update(engage=True, clear=True, now=1.5)
    assert s.update(engage=False, clear=True, now=2.6) is True


def test_clearing_when_not_engaged_does_nothing():
    s = EstopState(clear_hold=1.0)
    assert s.update(engage=False, clear=True, now=0.0) is False
    assert s.update(engage=False, clear=True, now=5.0) is False


def test_both_buttons_at_once_engages():
    s = EstopState()
    assert s.update(engage=True, clear=True, now=0.0) is True


def test_clear_progress_reports_the_hold():
    s = EstopState(clear_hold=2.0)
    s.update(engage=True, clear=False, now=0.0)
    assert s.clear_progress(0.0) == 0.0
    s.update(engage=False, clear=True, now=10.0)
    assert s.clear_progress(11.0) == pytest.approx(0.5)
    assert s.clear_progress(99.0) == 1.0


def test_instant_clear_is_rejected():
    with pytest.raises(ValueError, match="clear_hold must be positive"):
        EstopState(clear_hold=0.0)
