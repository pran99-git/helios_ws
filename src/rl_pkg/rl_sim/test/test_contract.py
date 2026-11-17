"""Tests for the observation and action contract shared with the ROS node.

The layout assertions here are deliberately literal. `policies/tracker_v1.zip`
is a function of exactly 73 values in exactly this order, so a change that
renumbers a channel does not produce an error at deployment, it produces a
policy confidently answering a different question. These tests are what makes
that change loud.
"""

from __future__ import annotations

import numpy as np
import pytest
from envs.deploy import (
    I_CARROTS,
    I_CMD,
    I_MEAS,
    I_REMAINING,
    I_SCAN,
    LOOKAHEADS,
    MAX_RANGE,
    OBS_DIM,
    SCAN_BEAMS,
    SCAN_FOV,
    YAW_FEEDBACK_CHANNELS,
    ObservationContract,
    VelocityLimits,
    assemble,
    blind,
    clean,
    downsample,
    scale_action,
    sector,
)
from envs.route import Pose2D
from envs.rover_env import RoverEnv
from envs.scenarios import Scenario


@pytest.fixture
def contract() -> ObservationContract:
    return ObservationContract(diagonal=14.14)


def straight_route(length: float = 10.0) -> list[Pose2D]:
    """A route along +x from the origin, so carrots are easy to predict."""
    return [Pose2D(x, 0.0, 0.0) for x in np.linspace(0.0, length, 21)]


# ------------------------------------------------------------- the layout ---


def test_layout_is_the_one_tracker_v1_was_trained_on() -> None:
    assert (I_SCAN, I_CARROTS, I_REMAINING, I_CMD, I_MEAS) == (0, 60, 66, 67, 70)
    assert OBS_DIM == 73
    assert SCAN_BEAMS == 60
    assert LOOKAHEADS == (0.5, 1.5, 3.0)


def test_yaw_feedback_channels_are_the_two_wz_inputs() -> None:
    """Blinding these is the measured weave mitigation, so they must be exact."""
    assert YAW_FEEDBACK_CHANNELS == (69, 72)


def test_contract_dim_agrees_with_the_module_constants(
    contract: ObservationContract,
) -> None:
    assert contract.dim == OBS_DIM


def test_env_observation_space_matches_the_contract() -> None:
    env = RoverEnv(scenario=Scenario(), seed=0)
    assert env.observation_space.shape == (OBS_DIM,)
    assert env.contract.dim == OBS_DIM


def test_bad_contract_rejected() -> None:
    with pytest.raises(ValueError, match="diagonal must be positive"):
        ObservationContract(diagonal=0.0)


# --------------------------------------------------------- scan reduction ---


def test_downsample_takes_the_minimum_not_the_mean() -> None:
    """A narrow obstacle must survive reduction, which a mean would erase."""
    ranges = np.full(100, 10.0)
    ranges[42] = 0.5
    reduced = downsample(ranges, 10)
    assert reduced.min() == pytest.approx(0.5)
    assert np.count_nonzero(reduced < 10.0) == 1


def test_downsample_preserves_bearing_order() -> None:
    reduced = downsample(np.linspace(1.0, 10.0, 90), 9)
    assert np.all(np.diff(reduced) > 0.0)


def test_downsample_length() -> None:
    assert len(downsample(np.ones(1081), SCAN_BEAMS)) == SCAN_BEAMS


def test_downsample_identity_when_sizes_match() -> None:
    ranges = np.linspace(1.0, 2.0, 16)
    assert np.allclose(downsample(ranges, 16), ranges)


def test_downsample_cannot_upsample() -> None:
    with pytest.raises(ValueError, match="cannot upsample"):
        downsample(np.ones(10), 20)


def test_downsample_rejects_zero_beams() -> None:
    with pytest.raises(ValueError, match="beams must be at least 1"):
        downsample(np.ones(10), 0)


@pytest.mark.parametrize("raw", [65.533, np.inf, np.nan, 0.0, -1.0])
def test_clean_reads_every_non_distance_as_open_space(raw: float) -> None:
    """A no-return at min_range would be a phantom obstacle at 6 cm."""
    assert clean(np.array([raw]))[0] == pytest.approx(MAX_RANGE)


def test_clean_keeps_real_distances() -> None:
    assert np.allclose(clean(np.array([0.5, 2.0, 9.0])), [0.5, 2.0, 9.0])


def test_sector_keeps_a_matching_sweep_whole() -> None:
    n = 1081
    increment = SCAN_FOV / (n - 1)
    kept = sector(np.arange(n, dtype=float), -SCAN_FOV / 2.0, increment, SCAN_FOV)
    assert len(kept) == n


def test_sector_trims_a_wider_sweep_to_the_trained_one() -> None:
    """A 360 degree driver must not have its beams rebinned into wrong bearings."""
    n = 720
    increment = 2.0 * np.pi / n
    kept = sector(np.arange(n, dtype=float), -np.pi, increment, SCAN_FOV)
    assert len(kept) == pytest.approx(n * 0.75, abs=2)


def test_sector_rejects_a_scan_that_misses_the_sweep() -> None:
    with pytest.raises(ValueError, match="covers none of the policy"):
        sector(np.ones(10), np.pi, 0.01, SCAN_FOV)


# ------------------------------------------------------------- the action ---


def test_scale_action_uses_the_asymmetric_envelope() -> None:
    limits = VelocityLimits()
    assert np.allclose(scale_action([1.0, 1.0, 1.0], limits), [0.40, 0.02, 0.80])
    # vx has no reverse, so the whole negative half maps onto standstill.
    assert np.allclose(scale_action([-1.0, -1.0, -1.0], limits), [0.0, -0.02, -0.80])


def test_scale_action_clips_out_of_range_requests() -> None:
    limits = VelocityLimits()
    assert np.allclose(scale_action([5.0, 0.0, 0.0], limits), [0.40, 0.0, 0.0])


# -------------------------------------------------------- the observation ---


def test_assemble_fills_each_slice_with_its_own_quantity(
    contract: ObservationContract,
) -> None:
    ranges = np.full(SCAN_BEAMS, 5.0)
    route = straight_route(10.0)
    result = assemble(
        contract,
        ranges,
        (0.0, 0.0, 0.0),
        route,
        10.0,
        np.array([0.40, 0.0, 0.0]),
        np.array([0.20, 0.0, 0.0]),
    )
    vector = result.vector

    assert vector.shape == (OBS_DIM,)
    assert vector.dtype == np.float32
    assert np.allclose(vector[I_SCAN:I_CARROTS], 0.5)
    # Straight route ahead: every carrot sits at its own lookahead, which
    # normalises to 1.0 forward and 0.0 lateral.
    assert np.allclose(vector[I_CARROTS:I_REMAINING], [1.0, 0.0, 1.0, 0.0, 1.0, 0.0])
    assert vector[I_REMAINING] == pytest.approx(10.0 / contract.diagonal)
    assert np.allclose(vector[I_CMD : I_CMD + 3], [1.0, 0.0, 0.0])
    assert np.allclose(vector[I_MEAS : I_MEAS + 3], [0.5, 0.0, 0.0])
    assert result.carrot_bearing == pytest.approx(0.0)


def test_assemble_normalises_the_scan_by_max_range(
    contract: ObservationContract,
) -> None:
    """An unclamped no-return would read 6.55 instead of 1.0."""
    vector = assemble(
        contract,
        np.full(SCAN_BEAMS, MAX_RANGE),
        (0.0, 0.0, 0.0),
        straight_route(),
        10.0,
        np.zeros(3),
        np.zeros(3),
    ).vector
    assert np.allclose(vector[I_SCAN:I_CARROTS], 1.0)


def test_assemble_carrot_bearing_comes_from_the_middle_lookahead(
    contract: ObservationContract,
) -> None:
    """The reward's alignment law and subgoal_walker steer by the same point."""
    # Route along +y, rover facing +x: every carrot is 90 degrees to port.
    route = [Pose2D(0.0, y, 0.0) for y in np.linspace(0.0, 10.0, 21)]
    result = assemble(
        contract,
        np.full(SCAN_BEAMS, 10.0),
        (0.0, 0.0, 0.0),
        route,
        10.0,
        np.zeros(3),
        np.zeros(3),
    )
    assert result.carrot_bearing == pytest.approx(np.pi / 2)
    middle = I_CARROTS + 2
    assert result.vector[middle] == pytest.approx(0.0, abs=1e-6)
    assert result.vector[middle + 1] == pytest.approx(1.0)


def test_assemble_clips_a_carrot_into_range(contract: ObservationContract) -> None:
    """A route that doubles back puts the far carrot beyond its own lookahead."""
    route = straight_route(1.0)
    vector = assemble(
        contract,
        np.full(SCAN_BEAMS, 10.0),
        (-20.0, 0.0, 0.0),
        route,
        1.0,
        np.zeros(3),
        np.zeros(3),
    ).vector
    carrots = vector[I_CARROTS:I_REMAINING]
    assert np.all(np.abs(carrots) <= 1.0)


def test_assemble_caps_remaining_route_at_one(
    contract: ObservationContract,
) -> None:
    vector = assemble(
        contract,
        np.full(SCAN_BEAMS, 10.0),
        (0.0, 0.0, 0.0),
        straight_route(100.0),
        100.0,
        np.zeros(3),
        np.zeros(3),
    ).vector
    assert vector[I_REMAINING] == pytest.approx(1.0)


def test_env_fills_the_twist_slices_in_contract_order() -> None:
    """Commanded before measured, and the env must not transpose them.

    After a reset both are zero, so one forward step is what separates them:
    the command jumps to the accel limit while the measured twist lags it
    through the transport delay.
    """
    env = RoverEnv(scenario=Scenario(), seed=0)
    observation, info = env.reset(seed=0)
    assert np.allclose(observation[I_CMD:OBS_DIM], 0.0)
    assert observation[I_REMAINING] == pytest.approx(
        min(info["route_length"] / env.contract.diagonal, 1.0), abs=1e-6
    )

    observation, *_ = env.step(np.array([1.0, 0.0, 0.0]))
    assert observation[I_CMD] > 0.0
    assert observation[I_CMD + 1] == pytest.approx(0.0)
    assert observation[I_CMD + 2] == pytest.approx(0.0)


def test_blind_zeroes_only_the_named_channels() -> None:
    vector = np.arange(OBS_DIM, dtype=np.float32)
    blinded = blind(vector, YAW_FEEDBACK_CHANNELS)
    assert blinded[69] == 0.0
    assert blinded[72] == 0.0
    assert blinded[68] == 68.0
    # The caller's vector is left alone: the env reuses its observation.
    assert vector[69] == 69.0


def test_blind_with_no_channels_is_a_no_op() -> None:
    vector = np.arange(OBS_DIM, dtype=np.float32)
    assert blind(vector, ()) is vector
