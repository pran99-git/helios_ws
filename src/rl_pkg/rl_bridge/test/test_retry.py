"""Tests for the planner-retry backoff.

The node asks `planner_server` for a route on every tick it has none, which
at the control rate measured 966 requests in 46.3 s against one unreachable
goal. These tests pin the schedule that replaces that.
"""

from __future__ import annotations

import pytest
from rl_bridge.retry import Backoff


def test_starts_ready() -> None:
    """A first attempt is never delayed."""
    backoff = Backoff(0.2, 2.0)
    assert backoff.ready(0.0)
    assert backoff.wait == 0.0
    assert backoff.remaining(0.0) == 0.0


def test_doubles_to_the_ceiling() -> None:
    """Each failure doubles the wait until it saturates."""
    backoff = Backoff(0.2, 2.0)
    schedule = []
    for _ in range(6):
        backoff.failed(0.0)
        schedule.append(round(backoff.wait, 3))
    assert schedule == [0.2, 0.4, 0.8, 1.6, 2.0, 2.0]


def test_gates_until_the_wait_elapses() -> None:
    """Ready only once the full wait has passed since the failure."""
    backoff = Backoff(0.2, 2.0)
    backoff.failed(100.0)
    assert not backoff.ready(100.0)
    assert backoff.remaining(100.05) == pytest.approx(0.15)
    assert not backoff.ready(100.19)
    assert backoff.ready(100.2)


def test_success_clears_the_wait() -> None:
    """A working caller is never gated, and the next failure restarts low."""
    backoff = Backoff(0.2, 2.0)
    for _ in range(4):
        backoff.failed(100.0)
    assert backoff.wait == pytest.approx(1.6)

    backoff.succeeded()
    assert backoff.wait == 0.0
    assert backoff.ready(100.0)

    backoff.failed(100.0)
    assert backoff.wait == pytest.approx(0.2)


def test_ceiling_below_the_first_wait_still_holds() -> None:
    """A misconfigured floor above the ceiling cannot exceed the ceiling."""
    backoff = Backoff(5.0, 2.0)
    backoff.failed(0.0)
    assert backoff.wait == pytest.approx(2.0)


def test_zero_disables_it() -> None:
    """Zeros leave every attempt ungated, the documented escape hatch."""
    backoff = Backoff(0.0, 0.0)
    for _ in range(5):
        backoff.failed(0.0)
    assert backoff.wait == 0.0
    assert backoff.ready(0.0)


@pytest.mark.parametrize(("first", "ceiling"), [(-0.1, 2.0), (0.2, -1.0)])
def test_rejects_negative_waits(first: float, ceiling: float) -> None:
    with pytest.raises(ValueError, match="non-negative"):
        Backoff(first, ceiling)
