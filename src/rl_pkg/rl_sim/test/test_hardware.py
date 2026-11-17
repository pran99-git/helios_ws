"""Tests for the measured hardware models."""

from __future__ import annotations

from itertools import pairwise

import numpy as np
import pytest
from envs.hardware import (
    TRACKING_SCALE,
    TWIST_NOISE_SD,
    CommandDelay,
    HardwareModel,
    PoseEstimator,
    TwistEstimator,
)

LIMITS = np.array([0.40, 0.10, 0.80])


def rng(seed: int = 0) -> np.random.Generator:
    return np.random.default_rng(seed)


# ------------------------------------------------------------------ delay ---


def test_delay_withholds_for_exactly_n_steps() -> None:
    d = CommandDelay(3)
    issued = [np.array([float(i), 0.0, 0.0]) for i in range(1, 6)]
    seen = [d.push(c)[0] for c in issued]
    # Three zeros first, then the commands in order.
    assert seen == [0.0, 0.0, 0.0, 1.0, 2.0]


def test_zero_delay_is_a_passthrough() -> None:
    d = CommandDelay(0)
    out = d.push(np.array([0.4, 0.0, 0.8]))
    assert out == pytest.approx([0.4, 0.0, 0.8])


def test_delay_starts_as_if_stationary() -> None:
    """A fresh episode must not inherit the previous one's commands."""
    d = CommandDelay(2)
    d.push(np.array([0.4, 0.0, 0.0]))
    d.reset()
    assert d.push(np.array([0.0, 0.0, 0.0])) == pytest.approx([0.0, 0.0, 0.0])


def test_delay_does_not_alias_the_caller_array() -> None:
    d = CommandDelay(1)
    command = np.array([0.4, 0.0, 0.0])
    d.push(command)
    command[0] = 99.0
    assert d.push(np.zeros(3))[0] == pytest.approx(0.4)


def test_negative_delay_rejected() -> None:
    with pytest.raises(ValueError, match="must not be negative"):
        CommandDelay(-1)


def test_measured_latency_is_within_the_configured_range() -> None:
    """The range must bracket what the bags measured, at a 48 ms period.

    vx dead time ran 41 ms (p10) to 263 ms (p90) with a 160 ms median, and
    the cross-correlation lag was 230 ms.
    """
    lo, hi = HardwareModel().delay_steps
    assert lo * 0.048 <= 0.160  # reaches down past the median
    assert hi * 0.048 >= 0.230  # reaches up past the correlation lag


# ------------------------------------------------------------------ twist ---


def test_twist_tracks_truth_apart_from_gain_and_noise() -> None:
    e = TwistEstimator(HardwareModel(spike_probability=0.0), LIMITS, rng())
    truth = np.array([0.30, 0.05, 0.40])
    out = np.array([e.estimate(truth) for _ in range(400)])
    assert np.allclose(out.mean(axis=0), e.scale * truth, atol=0.01)


def test_twist_noise_matches_the_measured_spread() -> None:
    e = TwistEstimator(HardwareModel(spike_probability=0.0), LIMITS, rng(1))
    out = np.array([e.estimate(np.zeros(3)) for _ in range(20000)])
    for axis in range(3):
        assert out[:, axis].std() == pytest.approx(TWIST_NOISE_SD[axis], rel=0.1)


def test_tracking_scale_brackets_the_measured_value() -> None:
    lo, hi = HardwareModel().tracking_scale
    assert lo <= TRACKING_SCALE <= hi


def test_spikes_exceed_the_physical_limit() -> None:
    """The bags show 8.63 m/s on a 0.40 m/s rover. That must be reachable."""
    e = TwistEstimator(HardwareModel(spike_probability=1.0), LIMITS, rng(2))
    worst = max(abs(e.estimate(np.zeros(3))).max() for _ in range(50))
    assert worst > 2.0


def test_spikes_decay_rather_than_switching_off() -> None:
    model = HardwareModel(spike_probability=1.0, spike_decay=0.79)
    e = TwistEstimator(model, LIMITS, rng(3))
    e.estimate(np.zeros(3))
    first = np.abs(e.estimate(np.zeros(3))).max()
    model_quiet = HardwareModel(spike_probability=0.0, spike_decay=0.79)
    e._model = model_quiet  # stop new spikes, watch the existing one decay
    trail = [np.abs(e.estimate(np.zeros(3))).max() for _ in range(5)]
    assert trail[0] < first
    assert all(b < a for a, b in pairwise(trail))


def test_no_spikes_when_probability_is_zero() -> None:
    e = TwistEstimator(HardwareModel(spike_probability=0.0), LIMITS, rng(4))
    for _ in range(2000):
        e.estimate(np.zeros(3))
    assert not e.spiking


def test_bad_spike_parameters_rejected() -> None:
    with pytest.raises(ValueError, match="spike_probability"):
        HardwareModel(spike_probability=1.5)
    with pytest.raises(ValueError, match="spike_probability"):
        HardwareModel(spike_probability=-0.1)
    with pytest.raises(ValueError, match="spike_decay"):
        HardwareModel(spike_decay=1.0)


# ------------------------------------------------------------------- pose ---


def test_pose_tracks_truth_closely_when_localised() -> None:
    """AMCL corrects every 0.27 s, so error does not integrate away."""
    p = PoseEstimator(HardwareModel(pose_jump_probability=0.0), 0.048, rng(5))
    truth = np.array([3.0, -2.0, 0.5])
    out = np.array([p.estimate(truth) for _ in range(5000)])
    error = np.linalg.norm(out[:, :2] - truth[:2], axis=1)
    assert error.mean() < 0.05
    assert not p.lost


def test_pose_jitter_matches_the_measured_correction() -> None:
    model = HardwareModel(pose_jump_probability=0.0)
    p = PoseEstimator(model, 0.048, rng(6))
    out = np.array([p.estimate(np.zeros(3)) for _ in range(20000)])
    assert out[:, 0].std() == pytest.approx(model.pose_jitter_sd, rel=0.1)


def test_yaw_is_passed_through() -> None:
    p = PoseEstimator(HardwareModel(), 0.048, rng(7))
    assert p.estimate(np.array([0.0, 0.0, 1.234]))[2] == pytest.approx(1.234)


def test_a_jump_displaces_the_estimate_and_persists() -> None:
    """The 6.14 m worst case: AMCL converging confidently wrong."""
    model = HardwareModel(pose_jump_probability=1.0, pose_jitter_sd=0.0)
    p = PoseEstimator(model, 0.048, rng(8))
    truth = np.zeros(3)
    first = p.estimate(truth)
    assert np.linalg.norm(first[:2]) >= model.pose_jump_magnitude[0]
    assert p.lost
    # still displaced on the next step, not a one-sample glitch
    assert np.linalg.norm(p.estimate(truth)[:2]) > 0.1


def test_a_jump_eventually_recovers() -> None:
    model = HardwareModel(
        pose_jump_probability=1.0,
        pose_jitter_sd=0.0,
        pose_jump_duration=(0.1, 0.1),
    )
    p = PoseEstimator(model, 0.048, rng(9))
    p.estimate(np.zeros(3))
    p._model = HardwareModel(pose_jump_probability=0.0, pose_jitter_sd=0.0)
    for _ in range(10):
        p.estimate(np.zeros(3))
    assert not p.lost
    assert np.allclose(p.offset, 0.0)


def test_reset_clears_a_jump() -> None:
    p = PoseEstimator(HardwareModel(pose_jump_probability=1.0), 0.048, rng(10))
    p.estimate(np.zeros(3))
    p.reset()
    assert not p.lost
    assert np.allclose(p.offset, 0.0)


def test_pose_jitter_holds_between_corrections() -> None:
    """One jitter sample per localisation correction, held in between.

    The real map->odom error is a step function at AMCL's 0.27 s cadence.
    Redrawing it every 48 ms control step made it white noise, and the
    steering loop chased it: the wz command flipped sign three times a
    second and the rover weaved down straight routes.
    """
    model = HardwareModel()
    dt = 0.048
    estimator = PoseEstimator(model, dt, np.random.default_rng(0))
    truth = np.array([1.0, 2.0, 0.5])
    steps_held = round(model.pose_correction_period / dt)

    first = estimator.estimate(truth)
    for _ in range(steps_held - 2):
        assert estimator.estimate(truth) == pytest.approx(first)
    later = [estimator.estimate(truth) for _ in range(3)]
    assert any(abs(p - first).max() > 1e-12 for p in later), (
        "a new correction must eventually arrive"
    )
