"""Unit tests for the mecanum wheel mixing.

Pure pytest -- no ROS context and no serial hardware.
"""

import math

import pytest
from teleop.mecanum_kinematics import (
    CORNERS,
    apply_inversions,
    body_twist_to_wheel_counts,
    mix_wheels,
    scale_to_ceiling,
)

# Geometry as configured for this rover (wheel_odometry.yaml).
WHEELBASE = 0.220
TRACK_WIDTH = 0.330
L_SUM = 0.5 * (WHEELBASE + TRACK_WIDTH)
WHEEL_RADIUS = 0.076
COUNTS_PER_REV = 2448.0

# Measured ceiling on this rover: 7590 counts/s.
MAX_QPPS = 7590

GEOMETRY = {
    "wheel_radius": WHEEL_RADIUS,
    "counts_per_rev": COUNTS_PER_REV,
    "l_sum": L_SUM,
}


def forward_kinematics(
    wheels: dict[str, float], l_sum: float
) -> tuple[float, float, float]:
    """Recovers the body twist, transcribed from wheel_odometry_node.

    A hand copy of `wheel_odometry_node.py:158-164`, which cannot be imported
    here because that module imports rclpy. It pins the sign convention as
    transcribed, so it will not notice if the odometry node itself changes --
    re-check the two by hand if either side is edited.
    """
    fl = wheels["front_left"]
    fr = wheels["front_right"]
    rl = wheels["rear_left"]
    rr = wheels["rear_right"]
    vx = 0.25 * (fl + fr + rl + rr)
    vy = 0.25 * (-fl + fr + rl - rr)
    wz = (-fl + fr - rl + rr) / (4.0 * l_sum)
    return vx, vy, wz


def test_corner_names_are_stable():
    """The corner names are a wire contract with the encoder JointState."""
    assert CORNERS == ("front_left", "front_right", "rear_left", "rear_right")


def test_pure_forward_drives_all_corners_equally():
    """Forward motion needs no differential across the four wheels."""
    wheels = mix_wheels(1.0, 0.0, 0.0)
    assert wheels == {
        "front_left": 1.0,
        "front_right": 1.0,
        "rear_left": 1.0,
        "rear_right": 1.0,
    }


def test_pure_strafe_splits_along_the_diagonals():
    """Left strafe drives the two diagonals in opposite directions."""
    wheels = mix_wheels(0.0, 1.0, 0.0)
    assert wheels == {
        "front_left": -1.0,
        "front_right": 1.0,
        "rear_left": 1.0,
        "rear_right": -1.0,
    }


def test_pure_rotation_splits_left_from_right():
    """Positive rotation is CCW: the left side reverses, the right advances."""
    wheels = mix_wheels(0.0, 0.0, 1.0)
    assert wheels == {
        "front_left": -1.0,
        "front_right": 1.0,
        "rear_left": -1.0,
        "rear_right": 1.0,
    }


def test_mixing_is_the_inverse_of_the_odometry_kinematics():
    """A mixed twist must survive a round trip through the odometry formula.

    This is the test that actually pins the sign convention: if any corner's
    sign flips, the commanded twist and the reported twist disagree and the
    rover fights itself.
    """
    for vx, vy, wz in (
        (0.2, -0.1, 0.3),
        (-0.15, 0.25, -0.4),
        (0.0, 0.0, 0.5),
        (0.25, 0.25, 0.0),
    ):
        wheels = mix_wheels(vx, vy, L_SUM * wz)
        got_vx, got_vy, got_wz = forward_kinematics(wheels, L_SUM)
        assert math.isclose(got_vx, vx, abs_tol=1e-12)
        assert math.isclose(got_vy, vy, abs_tol=1e-12)
        assert math.isclose(got_wz, wz, abs_tol=1e-12)


def test_combined_command_matches_the_sign_table():
    """Pins the four corner expressions at one explicit set of inputs."""
    wheels = mix_wheels(0.4, 0.3, 0.2)
    assert math.isclose(wheels["front_left"], 0.4 - 0.3 - 0.2)
    assert math.isclose(wheels["front_right"], 0.4 + 0.3 + 0.2)
    assert math.isclose(wheels["rear_left"], 0.4 + 0.3 - 0.2)
    assert math.isclose(wheels["rear_right"], 0.4 - 0.3 + 0.2)


def test_mixing_is_additive_across_axes():
    """Mixing must stay linear: no cross terms between the three axes.

    Asserted as a property rather than by restating the implementation, so a
    stray product term cannot slip through on the specific inputs above.
    """
    a = (0.4, 0.3, 0.2)
    b = (-0.1, 0.15, -0.35)
    combined = mix_wheels(*(ai + bi for ai, bi in zip(a, b, strict=True)))
    separate = mix_wheels(*a), mix_wheels(*b)
    for corner in CORNERS:
        # abs_tol is required: front_left sums to ~1e-17 here, and isclose
        # defaults to abs_tol=0, which no two distinct near-zero floats pass.
        assert math.isclose(
            combined[corner],
            separate[0][corner] + separate[1][corner],
            abs_tol=1e-12,
        )


def test_inversions_negate_only_the_flagged_corners():
    """A backwards-wired corner is corrected without touching the others."""
    wheels = mix_wheels(1.0, 0.0, 0.0)
    inversions = {
        "front_left": True,
        "front_right": False,
        "rear_left": False,
        "rear_right": True,
    }
    assert apply_inversions(wheels, inversions) == {
        "front_left": -1.0,
        "front_right": 1.0,
        "rear_left": 1.0,
        "rear_right": -1.0,
    }


def test_inversions_leave_the_input_untouched():
    """The node keeps no per-cycle state, so mixing must not mutate in place."""
    wheels = mix_wheels(1.0, 0.0, 0.0)
    apply_inversions(wheels, dict.fromkeys(CORNERS, True))
    assert wheels["front_left"] == 1.0


def test_forward_velocity_converts_to_the_expected_counts():
    """0.25 m/s is 1281.6 counts/s on this geometry.

    Worked from the geometry, not from the implementation: the wheel
    circumference is 2*pi*0.076 = 0.4775221 m, so 0.25 m/s is 0.5235360 wheel
    rev/s, times 2448 counts/rev = 1281.616 counts/s. That is ~17% of the
    measured 7590 ceiling.
    """
    counts = body_twist_to_wheel_counts(0.25, 0.0, 0.0, **GEOMETRY)
    for corner in CORNERS:
        assert math.isclose(counts[corner], 1281.616, abs_tol=0.001)


def test_rotation_converts_through_the_geometry_term():
    """Yaw rate scales by L before the wheel conversion, not after.

    0.5 rad/s * L 0.275 m = 0.1375 m/s at the wheel -> 0.1375 / 0.4775221 *
    2448 = 704.889 counts/s, left side negative for a CCW turn.
    """
    counts = body_twist_to_wheel_counts(0.0, 0.0, 0.5, **GEOMETRY)
    assert math.isclose(counts["front_right"], 704.889, abs_tol=0.001)
    assert math.isclose(counts["rear_right"], 704.889, abs_tol=0.001)
    assert math.isclose(counts["front_left"], -704.889, abs_tol=0.001)
    assert math.isclose(counts["rear_left"], -704.889, abs_tol=0.001)


def test_conversion_is_the_inverse_of_the_odometry_scale():
    """counts_per_meter must be the reciprocal of meters_per_count.

    wheel_odometry_node uses meters_per_count = (2*pi*r)/counts_per_rev. If the
    two disagree, commanded and reported velocity differ by that ratio and no
    amount of PID tuning fixes it.
    """
    meters_per_count = (math.tau * WHEEL_RADIUS) / COUNTS_PER_REV
    counts = body_twist_to_wheel_counts(0.2, -0.1, 0.3, **GEOMETRY)
    recovered = forward_kinematics(
        {c: counts[c] * meters_per_count for c in CORNERS}, L_SUM
    )
    assert math.isclose(recovered[0], 0.2, abs_tol=1e-12)
    assert math.isclose(recovered[1], -0.1, abs_tol=1e-12)
    assert math.isclose(recovered[2], 0.3, abs_tol=1e-12)


def test_zero_twist_converts_to_zero_counts():
    """A stop command must be exactly zero, not a small residual."""
    counts = body_twist_to_wheel_counts(0.0, 0.0, 0.0, **GEOMETRY)
    assert all(counts[c] == 0.0 for c in CORNERS)


@pytest.mark.parametrize(
    ("wheel_radius", "counts_per_rev"),
    [(0.0, 2448.0), (0.076, 0.0), (-0.076, 2448.0), (0.076, -1.0)],
)
def test_nonpositive_geometry_is_rejected(wheel_radius, counts_per_rev):
    """Bad geometry must fail loudly, not divide by zero at 20 Hz."""
    with pytest.raises(ValueError, match="must be positive"):
        body_twist_to_wheel_counts(
            0.1,
            0.0,
            0.0,
            wheel_radius=wheel_radius,
            counts_per_rev=counts_per_rev,
            l_sum=L_SUM,
        )


def test_commands_within_the_ceiling_pass_through_untouched():
    """The default limits never saturate, so scaling must be a no-op there."""
    counts = body_twist_to_wheel_counts(0.25, 0.25, 0.5, **GEOMETRY)
    assert scale_to_ceiling(counts, MAX_QPPS) == counts
    assert max(abs(v) for v in counts.values()) < MAX_QPPS


def test_saturation_scales_every_wheel_by_one_factor():
    """Ratios are the commanded direction, so they must survive scaling.

    Clipping wheels independently is what distorts a mecanum solution into a
    scrub; this is the guard against reintroducing that.
    """
    wheels = {
        "front_left": 2000.0,
        "front_right": 10000.0,
        "rear_left": -4000.0,
        "rear_right": 0.0,
    }
    scaled = scale_to_ceiling(wheels, MAX_QPPS)

    assert math.isclose(max(abs(v) for v in scaled.values()), MAX_QPPS)
    factor = MAX_QPPS / 10000.0
    for corner, original in wheels.items():
        assert math.isclose(scaled[corner], original * factor)


def test_saturation_preserves_sign_for_negative_peaks():
    """The peak may be a reversing wheel; scaling must not flip anything."""
    wheels = dict.fromkeys(CORNERS, 0.0) | {"rear_left": -20000.0}
    scaled = scale_to_ceiling(wheels, MAX_QPPS)
    assert math.isclose(scaled["rear_left"], -MAX_QPPS)


def test_scaling_an_all_zero_command_is_a_no_op():
    """A stop command has peak 0, which takes the pass-through branch."""
    wheels = dict.fromkeys(CORNERS, 0.0)
    assert scale_to_ceiling(wheels, MAX_QPPS) == wheels


def test_nonpositive_ceiling_is_rejected():
    """A negative ceiling would scale by a negative factor and reverse a wheel."""
    wheels = dict.fromkeys(CORNERS, 100.0)
    for ceiling in (0.0, -1.0, -MAX_QPPS):
        with pytest.raises(ValueError, match="ceiling must be positive"):
            scale_to_ceiling(wheels, ceiling)


def test_default_limits_leave_headroom_before_saturation():
    """Pins where the two limit layers start to interact.

    The velocity clamps are the primary limit and the QPPS ceiling is a
    backstop. At the shipped defaults the worst case is 43% of the ceiling, so
    scaling never engages; whoever raises the defaults needs to know it starts
    biting somewhere above 2x.
    """
    worst_case = body_twist_to_wheel_counts(0.25, 0.25, 0.5, **GEOMETRY)
    peak = max(abs(v) for v in worst_case.values())
    assert math.isclose(peak, 3268.12, abs_tol=0.01)
    assert math.isclose(peak / MAX_QPPS, 0.4306, abs_tol=0.0001)

    doubled = body_twist_to_wheel_counts(0.5, 0.5, 1.0, **GEOMETRY)
    assert scale_to_ceiling(doubled, MAX_QPPS) == doubled

    tripled = body_twist_to_wheel_counts(0.75, 0.75, 1.5, **GEOMETRY)
    assert scale_to_ceiling(tripled, MAX_QPPS) != tripled
