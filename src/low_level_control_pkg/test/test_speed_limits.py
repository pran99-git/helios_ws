"""Unit tests for the QPPS ceiling policy.

Pure pytest -- no ROS context and no serial hardware.
"""

from teleop.speed_limits import (
    DEFAULT_MAX_QPPS,
    PLAUSIBLE_QPPS_FACTOR,
    resolve_max_qpps,
)


def test_default_matches_the_measured_hardware():
    """Pins the fallback to what this rover actually reported.

    Measured 2026-07-29. rover_control assumes 45000, which is 5.9x too high
    for these controllers; this test exists so that value cannot drift back in.
    """
    assert DEFAULT_MAX_QPPS == 7590


def test_matching_controllers_resolve_cleanly():
    """The nominal case: both agree, so there is nothing to warn about."""
    limit = resolve_max_qpps(7590, 7590)
    assert limit.ceiling == 7590
    assert limit.ok is True
    assert "7590" in limit.status


def test_asymmetric_pair_locks_to_the_slower_controller():
    """Over-commanding the slower side is what the min-lock prevents."""
    limit = resolve_max_qpps(7590, 6000)
    assert limit.ceiling == 6000
    assert limit.ok is False
    assert "7590" in limit.status and "6000" in limit.status


def test_asymmetry_is_detected_when_the_left_side_is_slower():
    """Detection must not depend on which argument happens to be smaller.

    Asserts the flag and the status, not just the ceiling: a branch written as
    `left > right` instead of `left != right` would still return 6000 here
    while silently reporting the pair as nominal.
    """
    limit = resolve_max_qpps(6000, 7590)
    assert limit.ceiling == 6000
    assert limit.ok is False
    assert "asymmetric" in limit.status
    assert "6000" in limit.status and "7590" in limit.status


def test_failed_read_falls_back_and_flags_it():
    """A None reading means the read failed, not that the motor is stopped."""
    for left, right in ((None, 7590), (7590, None), (None, None)):
        limit = resolve_max_qpps(left, right)
        assert limit.ceiling == DEFAULT_MAX_QPPS
        assert limit.ok is False
        # "unusable" is unique to this branch; "Motion Studio" is not.
        assert "unusable" in limit.status


def test_symmetric_but_implausible_ceiling_is_flagged():
    """Both controllers agreeing does not make the value trustworthy.

    QPPS is read unsigned, so garbage arrives as a large positive number, not a
    negative one. rover_control's 45000 assumption is the concrete case: 5.9x
    this hardware, and without this check it would log as nominal at info.
    """
    limit = resolve_max_qpps(45000, 45000)
    assert limit.ceiling == 45000
    assert limit.ok is False
    assert "plausible band" in limit.status


def test_implausibly_low_ceiling_is_flagged():
    """A barely-nonzero ceiling passes the sign check but is not usable."""
    limit = resolve_max_qpps(3, 3)
    assert limit.ok is False
    assert "plausible band" in limit.status


def test_plausibility_band_edges_are_inclusive():
    """The band boundaries themselves are acceptable, not suspect."""
    for ceiling in (
        DEFAULT_MAX_QPPS // PLAUSIBLE_QPPS_FACTOR,
        DEFAULT_MAX_QPPS * PLAUSIBLE_QPPS_FACTOR,
    ):
        assert resolve_max_qpps(ceiling, ceiling).ok is True


def test_plausibility_is_measured_against_the_supplied_fallback():
    """A re-geared rover passes its own reference in, not the default."""
    limit = resolve_max_qpps(45000, 45000, fallback=45000)
    assert limit.ok is True


def test_zero_qpps_falls_back_rather_than_locking_to_zero():
    """An uncalibrated controller reports 0; locking to it would disable drive."""
    limit = resolve_max_qpps(0, 7590)
    assert limit.ceiling == DEFAULT_MAX_QPPS
    assert limit.ok is False


def test_negative_qpps_falls_back():
    """QPPS is a rate ceiling; a negative value is garbage, not a limit."""
    limit = resolve_max_qpps(-1, 7590)
    assert limit.ceiling == DEFAULT_MAX_QPPS
    assert limit.ok is False


def test_explicit_fallback_overrides_the_default():
    """The caller can supply its own assumption for a differently tuned rover."""
    limit = resolve_max_qpps(None, None, fallback=12000)
    assert limit.ceiling == 12000
    assert "12000" in limit.status


def test_every_branch_names_the_resolved_ceiling():
    """The node logs the status verbatim, so it must state the number in force."""
    for args, expected in (
        ((7590, 7590), 7590),
        ((7590, 6000), 6000),
        ((None, 7590), DEFAULT_MAX_QPPS),
        ((0, 0), DEFAULT_MAX_QPPS),
    ):
        limit = resolve_max_qpps(*args)
        assert limit.ceiling == expected
        assert str(expected) in limit.status
