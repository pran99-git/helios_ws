"""Tests for the velocity_smoother replica."""

from __future__ import annotations

import numpy as np
import pytest
from envs.deploy import RateLimiter, VelocityLimits


@pytest.fixture
def limiter() -> RateLimiter:
    return RateLimiter(VelocityLimits(), dt=0.048)


def test_starts_stationary(limiter: RateLimiter) -> None:
    assert np.allclose(limiter.current, 0.0)


def test_single_step_is_capped_by_accel(limiter: RateLimiter) -> None:
    out = limiter.step(np.array([0.40, 0.0, 0.0]))
    # 0.80 m/s^2 over 48 ms is 0.0384 m/s, well short of the 0.40 requested.
    assert out[0] == pytest.approx(0.80 * 0.048)


def test_reaches_the_command_eventually(limiter: RateLimiter) -> None:
    for _ in range(100):
        out = limiter.step(np.array([0.40, 0.0, 0.0]))
    assert out[0] == pytest.approx(0.40)


def test_ramp_duration_matches_the_acceleration(limiter: RateLimiter) -> None:
    steps = 0
    while limiter.current[0] < 0.40 - 1e-9:
        limiter.step(np.array([0.40, 0.0, 0.0]))
        steps += 1
    # 0.40 m/s at 0.80 m/s^2 is 0.5 s, which is 10.4 control periods.
    assert steps * 0.048 == pytest.approx(0.5, abs=0.048)


def test_command_beyond_the_envelope_is_clipped_not_rejected(
    limiter: RateLimiter,
) -> None:
    for _ in range(200):
        out = limiter.step(np.array([9.0, 9.0, 9.0]))
    assert out[0] == pytest.approx(0.40)
    assert out[1] == pytest.approx(0.02)
    assert out[2] == pytest.approx(0.80)


def test_deceleration_uses_the_decel_limit(limiter: RateLimiter) -> None:
    limiter.reset(np.array([0.40, 0.0, 0.0]))
    out = limiter.step(np.zeros(3))
    # Slowing is bounded by -0.50 m/s^2, softer than the 0.80 for speeding up.
    assert out[0] == pytest.approx(0.40 - 0.50 * 0.048)


def test_forward_axis_has_no_reverse(limiter: RateLimiter) -> None:
    """Reversing is driving into the scan's 90 degree aft blind spot, and
    with it available the policy backed toward a carrot instead of turning.

    A -0.05 crawl was tried to avoid the flat action region this creates
    and learned worse, so the hard zero stands.
    """
    for _ in range(200):
        out = limiter.step(np.array([-1.0, 0.0, 0.0]))
    assert out[0] == pytest.approx(0.0)


def test_lateral_reverse_still_works(limiter: RateLimiter) -> None:
    """vy is clamped small, not removed: it is a fine correction."""
    for _ in range(200):
        out = limiter.step(np.array([0.0, -1.0, 0.0]))
    assert out[1] == pytest.approx(-0.02)


def test_reset_clips_into_the_envelope(limiter: RateLimiter) -> None:
    limiter.reset(np.array([99.0, 0.0, 0.0]))
    assert limiter.current[0] == pytest.approx(0.40)


def test_current_is_a_copy(limiter: RateLimiter) -> None:
    held = limiter.current
    held[0] = 123.0
    assert limiter.current[0] == 0.0


@pytest.mark.parametrize("dt", [0.0, -0.1])
def test_bad_dt_rejected(dt: float) -> None:
    with pytest.raises(ValueError, match="dt must be positive"):
        RateLimiter(VelocityLimits(), dt=dt)


def test_positive_decel_rejected() -> None:
    with pytest.raises(ValueError, match="decel must be negative"):
        VelocityLimits(decel=(0.5, 0.5, 0.5))


def test_inverted_envelope_rejected() -> None:
    with pytest.raises(ValueError, match="max_velocity must exceed"):
        VelocityLimits(max_velocity=(-1.0, 0.1, 0.8))
