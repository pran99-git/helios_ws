"""Tests for scan geometry and the downsampling the ROS node must copy."""

from __future__ import annotations

import numpy as np
import pytest
from envs.lidar import UST10LX_FOV, LidarSpec, downsample


def test_angles_span_the_fov() -> None:
    spec = LidarSpec(beams=5, fov=np.pi)
    assert spec.angles[0] == pytest.approx(-np.pi / 2)
    assert spec.angles[-1] == pytest.approx(np.pi / 2)


def test_angles_are_ascending() -> None:
    assert np.all(np.diff(LidarSpec().angles) > 0.0)


def test_default_fov_is_270_degrees() -> None:
    """The UST-10LX does not sweep a full circle; the rear quadrant is dead."""
    assert UST10LX_FOV == pytest.approx(np.deg2rad(270.0))


def test_beam_count_floor() -> None:
    with pytest.raises(ValueError, match="beams must be at least 2"):
        LidarSpec(beams=1)


@pytest.mark.parametrize("fov", [0.0, -1.0, 7.0])
def test_bad_fov_rejected(fov: float) -> None:
    with pytest.raises(ValueError, match=r"fov must be in \(0, 2\*pi\]"):
        LidarSpec(fov=fov)


def test_inverted_range_rejected() -> None:
    with pytest.raises(ValueError, match="min_range must be below max_range"):
        LidarSpec(min_range=5.0, max_range=1.0)


def test_downsample_takes_the_minimum_not_the_mean() -> None:
    """A narrow obstacle must survive reduction, which a mean would erase."""
    ranges = np.full(100, 10.0)
    ranges[42] = 0.5
    reduced = downsample(ranges, 10)
    assert reduced.min() == pytest.approx(0.5)
    assert np.count_nonzero(reduced < 10.0) == 1


def test_downsample_preserves_bearing_order() -> None:
    ranges = np.linspace(1.0, 10.0, 90)
    reduced = downsample(ranges, 9)
    assert np.all(np.diff(reduced) > 0.0)


def test_downsample_length() -> None:
    assert len(downsample(np.ones(1081), 60)) == 60


def test_downsample_identity_when_sizes_match() -> None:
    ranges = np.linspace(1.0, 2.0, 16)
    assert np.allclose(downsample(ranges, 16), ranges)


def test_downsample_cannot_upsample() -> None:
    with pytest.raises(ValueError, match="cannot upsample"):
        downsample(np.ones(10), 20)


def test_downsample_rejects_zero_beams() -> None:
    with pytest.raises(ValueError, match="beams must be at least 1"):
        downsample(np.ones(10), 0)
