"""The observation and action contract a trained policy was fitted to.

ONE definition, shared by the simulator that trains the policy and the ROS
node that deploys it. A policy is a function of exactly this vector, and a
mismatch between the two sides is silent: the network still returns a
plausible twist, it is just answering a different question. So there is no
second copy to drift, and the simulator reaches across to this module rather
than the other way round, because the package that ships on the rover has to
stand on its own.

The layout, 73 values for the shipped 60-beam scan:

    [ 0 .. 59]  fused scan / max_range     1.0 is open space
    [60 .. 65]  route carrots              2 per lookahead, body frame
    [66]        remaining route / diagonal
    [67 .. 69]  commanded twist / scale    what the smoother last emitted
    [70 .. 72]  measured twist / scale     from the EKF

Imports nothing from ROS, MuJoCo or torch.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from itertools import pairwise
from typing import NamedTuple

import numpy as np
from vla_bridge.goal_resolution import Pose2D
from vla_bridge.subgoal import point_at_arc, project

from rl_bridge.limits import VelocityLimits

#: Beams the policy sees. The real `/scan` carries 1081 at 40 Hz, far more
#: than a driving policy can use.
SCAN_BEAMS: int = 60

#: Angular sweep of the scan, radians. The UST-10LX sweeps 270 degrees, not a
#: full circle: the rear quadrant is the cable and the housing's dead zone.
SCAN_FOV: float = 1.5 * np.pi

#: Range reported where nothing is hit, metres. The driver advertises
#: range_max 30.0 and reports no-return as 65.533, but the furthest real
#: return measured across the bags was 9.02 m, so this is the honest ceiling.
#: A scan must be clamped to it before normalising, or a no-return reads 6.55
#: instead of 1.0.
MAX_RANGE: float = 10.0
MIN_RANGE: float = 0.06

#: Arc lengths ahead of the rover's projection at which the route is sampled,
#: metres. Three rather than one so the policy can see the route's CURVATURE
#: and start turning before a corner, not just its immediate direction. The
#: 1.5 m middle value matches subgoal_walker's DEFAULT_LOOKAHEAD, so the
#: policy and the VLA walker read the same route the same way.
LOOKAHEADS: tuple[float, ...] = (0.5, 1.5, 3.0)

#: Channel offsets for the shipped layout. Named rather than written as
#: literals because probes, blinding and any logging all index the same
#: vector, and 69 vs 72 is not a distinction worth making by hand.
I_SCAN: int = 0
I_CARROTS: int = SCAN_BEAMS
I_REMAINING: int = I_CARROTS + 2 * len(LOOKAHEADS)
I_CMD: int = I_REMAINING + 1
I_MEAS: int = I_CMD + 3
OBS_DIM: int = I_MEAS + 3

#: Commanded and measured yaw rate. The policy's response to these two is a
#: learned D-term through the loop's own lag: measured d(wz_out)/d(wz_in) of
#: -1.56 and -0.65, which closes a feedback loop and sustains a limit cycle
#: at about 1.7 Hz. Zeroing them at inference cut the oscillation by a third
#: (3.83 to 2.43 sign reversals/s) at no cost in success.
YAW_FEEDBACK_CHANNELS: tuple[int, int] = (I_CMD + 2, I_MEAS + 2)


class Observation(NamedTuple):
    """One assembled observation.

    Attributes:
        vector: The policy's input, float32.
        carrot_bearing: Bearing to the middle carrot, radians. Not part of
            the vector; the reward's alignment law and the deployment status
            both read it, and it falls out of the same geometry.
    """

    vector: np.ndarray
    carrot_bearing: float


@dataclass(frozen=True)
class ObservationContract:
    """What a given policy's observation vector means.

    Attributes:
        diagonal: Longest journey possible in this world, metres. Normalises
            the remaining-route channel, so it must come from the same map
            the policy will drive in: the live `/map` extent on the robot,
            the compiled world's extent in the simulator.
        beams: Scan values in the vector.
        fov: Angular sweep those beams cover, radians.
        max_range: Range a beam reports where nothing is hit, metres.
        lookaheads: Arc lengths the route is sampled at, metres.
        limits: Velocity envelope the twist channels are normalised by.
    """

    diagonal: float
    beams: int = SCAN_BEAMS
    fov: float = SCAN_FOV
    max_range: float = MAX_RANGE
    lookaheads: tuple[float, ...] = LOOKAHEADS
    limits: VelocityLimits = field(default_factory=VelocityLimits)

    def __post_init__(self) -> None:
        if self.diagonal <= 0.0:
            raise ValueError("diagonal must be positive")
        if self.beams < 2:
            raise ValueError("beams must be at least 2")
        if not self.lookaheads:
            raise ValueError("at least one lookahead is required")

    @property
    def dim(self) -> int:
        """Length of the observation vector."""
        return self.beams + 2 * len(self.lookaheads) + 7

    @property
    def angles(self) -> np.ndarray:
        """Beam bearings in the sensor frame, radians, ascending."""
        return np.linspace(-self.fov / 2.0, self.fov / 2.0, self.beams)


def scale_action(action: np.ndarray, limits: VelocityLimits) -> np.ndarray:
    """Map a -1..1 action onto the rover's asymmetric velocity envelope.

    Args:
        action: Policy output as [vx, vy, wz], clipped here rather than
            assumed in range.
        limits: The envelope to scale onto.

    Returns:
        A twist in m/s and rad/s.
    """
    a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
    # Forward and reverse limits differ, so each side scales separately
    # rather than by a single symmetric gain.
    hi = np.asarray(limits.max_velocity, dtype=np.float64)
    lo = np.asarray(limits.min_velocity, dtype=np.float64)
    return np.where(a >= 0.0, a * hi, -a * lo)


def assemble(
    contract: ObservationContract,
    ranges: np.ndarray,
    pose: Sequence[float],
    route: Sequence[Pose2D],
    route_length: float,
    commanded: np.ndarray,
    measured: np.ndarray,
) -> Observation:
    """Build the observation vector from live quantities.

    Each carrot is the body-frame offset to the route point that far ahead,
    divided by its own lookahead, so every component stays in -1..1 and a
    distant carrot never dominates the vector by being distant.

    Args:
        contract: What this policy's vector means.
        ranges: Scan in metres, one value per beam, ascending in bearing.
            Already reduced to `contract.beams` and clamped to range.
        pose: The pose the robot BELIEVES it has, [x, y, yaw] in the route's
            frame. Localisation error is part of what the policy was trained
            through, so this is the estimate, never ground truth.
        route: The planned route being followed.
        route_length: Its total arc length, metres.
        commanded: Last twist actually emitted, after rate limiting.
        measured: Twist the EKF reports, [vx, vy, wz] in the body frame.

    Returns:
        The vector and the middle carrot's bearing.
    """
    scan = (np.asarray(ranges, dtype=np.float64) / contract.max_range).astype(
        np.float32
    )
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    arc = project(route, x, y).arc
    # World to body is a rotation by -yaw.
    cos_yaw, sin_yaw = np.cos(-yaw), np.sin(-yaw)

    carrots: list[float] = []
    bearing = 0.0
    # The middle lookahead is the one subgoal_walker steers by, so the
    # reward's alignment law reads the same point.
    middle = contract.lookaheads[len(contract.lookaheads) // 2]
    for lookahead in contract.lookaheads:
        point = point_at_arc(route, arc + lookahead)
        dx, dy = point.x - x, point.y - y
        body_x = cos_yaw * dx - sin_yaw * dy
        body_y = sin_yaw * dx + cos_yaw * dy
        carrots.extend((body_x / lookahead, body_y / lookahead))
        if lookahead == middle:
            bearing = float(np.arctan2(body_y, body_x))

    left = max(0.0, route_length - arc)
    scale = contract.limits.scale
    vector = np.concatenate(
        [
            scan,
            np.clip(carrots, -1.0, 1.0),
            [min(left / contract.diagonal, 1.0)],
            np.asarray(commanded, dtype=np.float64) / scale,
            np.clip(np.asarray(measured, dtype=np.float64) / scale, -1.0, 1.0),
        ]
    ).astype(np.float32)
    return Observation(vector, bearing)


def blind(vector: np.ndarray, channels: Sequence[int]) -> np.ndarray:
    """Zero named channels before inference.

    Removes an input the policy leans on without retraining it. Used for
    YAW_FEEDBACK_CHANNELS, where the learned response is a self-feedback loop
    rather than useful information.

    Args:
        vector: An observation. Not modified.
        channels: Indices to zero. Empty leaves the vector alone.

    Returns:
        The vector, or a blinded copy of it.
    """
    if not len(channels):
        return vector
    out = np.asarray(vector).copy()
    out[..., list(channels)] = 0.0
    return out


def sector(
    ranges: np.ndarray, angle_min: float, angle_increment: float, fov: float
) -> np.ndarray:
    """Cut a full scan down to the sweep the policy was trained on.

    A driver whose sweep is wider than `fov`, or not centred on the sensor's
    x axis, would otherwise have its beams rebinned into the wrong bearings:
    the reduction below assumes the input already covers exactly `fov`.

    Args:
        ranges: Full-resolution ranges, ascending in bearing.
        angle_min: Bearing of the first beam, radians.
        angle_increment: Bearing step between beams, radians.
        fov: Sweep to keep, centred on straight ahead, radians.

    Returns:
        The beams inside the sweep.

    Raises:
        ValueError: If the scan does not cover the sweep at all.
    """
    bearings = angle_min + angle_increment * np.arange(len(ranges))
    inside = np.nonzero(np.abs(bearings) <= fov / 2.0 + 1e-9)[0]
    if inside.size == 0:
        raise ValueError(
            f"scan over [{bearings[0]:.2f}, {bearings[-1]:.2f}] rad "
            f"covers none of the policy's {fov:.2f} rad sweep"
        )
    return np.asarray(ranges)[inside[0] : inside[-1] + 1]


def downsample(ranges: np.ndarray, beams: int) -> np.ndarray:
    """Reduce a full-resolution scan to `beams` values.

    Each output beam takes the MINIMUM of its input bin, never the mean. A
    mean lets a narrow obstacle average away against the open space beside
    it, which is exactly the obstacle that matters.

    Args:
        ranges: Full-resolution ranges, ascending in bearing, covering the
            policy's sweep and no more.
        beams: Number of output beams. Must not exceed the input length.

    Returns:
        The reduced scan.

    Raises:
        ValueError: If `beams` exceeds the input length or is below 1.
    """
    if beams < 1:
        raise ValueError("beams must be at least 1")
    if beams > len(ranges):
        raise ValueError(f"cannot upsample {len(ranges)} ranges to {beams} beams")
    edges = np.linspace(0, len(ranges), beams + 1).astype(int)
    return np.array([ranges[a:b].min() for a, b in pairwise(edges)])


def clean(
    ranges: np.ndarray, max_range: float = MAX_RANGE, min_range: float = MIN_RANGE
) -> np.ndarray:
    """Turn a driver's raw ranges into the values the simulator produced.

    Three kinds of value are not a distance and all mean "nothing here": the
    no-return sentinel (65.533 on this driver), a non-finite value, and
    anything below the sensor's own minimum. All become `max_range`, which
    reads as open space. Mapping them to `min_range` instead would put a
    phantom obstacle at 6 cm and stop the rover dead; 1.84% of beams come
    back empty on every scan, so that choice is not a corner case.

    Args:
        ranges: Raw ranges in metres.
        max_range: Value an empty beam reports.
        min_range: Shortest real distance the sensor can report.

    Returns:
        Ranges clamped into [min_range, max_range].
    """
    out = np.asarray(ranges, dtype=np.float64).copy()
    out[~np.isfinite(out) | (out < min_range)] = max_range
    return np.clip(out, min_range, max_range)


def fuse(scan: np.ndarray, other: np.ndarray) -> np.ndarray:
    """Merge a second sensor's ranges into the scan, nearer wins.

    What a costmap does when two sources mark the same cell: either sensor
    seeing an obstacle is enough. The policy was trained on a scan fused
    this way from the laser and the ZED's depth, because Nav2's local
    costmap already fuses both (`observation_sources: scan zed_cloud`).

    Args:
        scan: Reduced ranges, one per beam.
        other: The second source, same length and bearings. Bins it does not
            cover must already hold `max_range`.

    Returns:
        The fused scan. Neither input is modified.
    """
    return np.minimum(np.asarray(scan), np.asarray(other))
