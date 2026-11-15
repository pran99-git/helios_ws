"""Tests for scan geometry and the downsampling the ROS node must copy."""

from __future__ import annotations

import numpy as np
import pytest
from envs.lidar import (
    ENCODE_NEAR,
    UST10LX_DROPOUT,
    UST10LX_FOV,
    ZED_FOV,
    ZED_MAX_HEIGHT,
    ZED_MIN_HEIGHT,
    LidarSpec,
    apply_dropout,
    downsample,
    encode,
    fuse,
)


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


# ------------------------------------------------------- depth and fusion ---


def test_fuse_takes_the_nearer_source() -> None:
    """Either sensor seeing an obstacle is enough, as in a costmap cell."""
    scan = np.full(10, 5.0)
    depth = np.array([2.0, 9.0])
    out = fuse(scan, depth, np.array([3, 4]))
    assert out[3] == pytest.approx(2.0)
    assert out[4] == pytest.approx(5.0)


def test_fuse_leaves_uncovered_bins_alone() -> None:
    scan = np.arange(10.0)
    out = fuse(scan, np.zeros(2), np.array([0, 1]))
    assert np.allclose(out[2:], scan[2:])


def test_fuse_does_not_modify_its_input() -> None:
    scan = np.full(5, 5.0)
    fuse(scan, np.zeros(2), np.array([0, 1]))
    assert np.allclose(scan, 5.0)


def test_zed_covers_less_than_the_laser() -> None:
    """110 degrees of the laser's 270, so the rear stays laser-only."""
    assert ZED_FOV < UST10LX_FOV
    assert ZED_FOV == pytest.approx(np.deg2rad(110.0))


def test_zed_height_band_matches_the_costmap() -> None:
    """36 mm above the floor to 0.50 m, from nav2_params.yaml."""
    assert ZED_MIN_HEIGHT == pytest.approx(0.036)
    assert ZED_MAX_HEIGHT == pytest.approx(0.50)
    # The laser plane sits inside the band, which is why fusion adds anything.
    assert ZED_MIN_HEIGHT < 0.176 < ZED_MAX_HEIGHT


# ---------------------------------------------------------------- dropout ---


def test_dropout_blanks_to_max_range_not_zero() -> None:
    """A real no-return reads 65.533 m, so a dropped beam is open space."""
    ranges = np.full(1000, 2.0)
    out = apply_dropout(ranges, 0.5, 10.0, np.random.default_rng(0))
    assert set(np.unique(out)) <= {2.0, 10.0}
    assert out.min() == pytest.approx(2.0)


def test_dropout_rate_matches_the_measurement() -> None:
    ranges = np.full(200_000, 1.0)
    out = apply_dropout(ranges, UST10LX_DROPOUT, 10.0, np.random.default_rng(1))
    assert (out == 10.0).mean() == pytest.approx(UST10LX_DROPOUT, rel=0.05)


def test_zero_dropout_is_a_passthrough() -> None:
    ranges = np.full(10, 3.0)
    assert apply_dropout(ranges, 0.0, 10.0, np.random.default_rng(2)) is ranges


# --------------------------------------------------------------- encode ---


def test_encode_spans_zero_to_one() -> None:
    spec = LidarSpec()
    assert encode(np.array([ENCODE_NEAR]), spec)[0] == pytest.approx(0.0)
    assert encode(np.array([spec.max_range]), spec)[0] == pytest.approx(1.0)


def test_encode_floors_at_contact_range() -> None:
    """Below the footprint radius there is nothing left to steer out of."""
    spec = LidarSpec()
    assert encode(np.array([0.10]), spec)[0] == pytest.approx(0.0)
    assert ENCODE_NEAR < 0.269


def test_encode_is_monotonic() -> None:
    spec = LidarSpec()
    out = encode(np.linspace(ENCODE_NEAR, spec.max_range, 200), spec)
    assert np.all(np.diff(out) > 0)


def test_encode_clamps_out_of_range_input() -> None:
    spec = LidarSpec()
    out = encode(np.array([-5.0, 0.0, 65.533, 1e6]), spec)
    assert np.all(out >= 0.0)
    assert np.all(out <= 1.0)


def test_encode_spends_its_range_where_decisions_are() -> None:
    """The point of the log scale.

    Linear normalisation gave the 0.20-0.60 m band 4% of the input range
    against 70% for 3-10 m, so closing 5 cm at 0.30 m moved the network's
    input exactly as much as closing 5 cm at 6 m. Compared here per
    centimetre, which is what the ratio actually is.
    """
    spec = LidarSpec()

    def e(r: float) -> float:
        return float(encode(np.array([r]), spec)[0])

    def linear(r: float) -> float:
        return min(max(r, spec.min_range), spec.max_range) / spec.max_range

    # resolution at the distance a collision is decided at
    near_log = e(0.35) - e(0.30)
    near_lin = linear(0.35) - linear(0.30)
    assert near_log > 5 * near_lin

    # and the same 5 cm far away, where it does not matter, moves it less
    assert near_log > 5 * (e(6.05) - e(6.00))

    # the collision band gets several times the range it had
    band_log = e(0.60) - e(0.25)
    band_lin = linear(0.60) - linear(0.25)
    assert band_log > 4 * band_lin
