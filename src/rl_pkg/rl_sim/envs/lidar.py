"""Simulated 2D laser scan, matching the Hokuyo UST-10LX on the rover.

Rays are cast with `mj_multiRay`, which is fast enough to be irrelevant to
training cost: 60 beams measured 77,338 scans/s on one core, against the
4,800 scans/s that 240 environments at 20 Hz actually need.

The policy sees a DOWNSAMPLED scan, not the hardware's full resolution. The
real `/scan` carries 1081 beams at 40 Hz, far more than a driving policy can
use, and the ROS node must reduce it the same way this module does or the
observation will not match at deployment.

The scan is FUSED with ZED depth, because Nav2's local costmap already fuses
both (`observation_sources: scan zed_cloud`). The laser plane sits at 0.176 m
and misses anything shorter; the ZED catches obstacles from 36 mm up to
0.50 m out to 2.5 m. A policy given only the laser would be blind to a class
of obstacle its own baseline handles, which would make any comparison against
MPPI meaningless.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise

import mujoco
import numpy as np

#: The UST-10LX sweeps 270 degrees, not a full circle: the rear quadrant is
#: the cable and the housing's dead zone.
UST10LX_FOV: float = 1.5 * np.pi

#: Effective range. The driver advertises range_max 30.0 and reports
#: no-return as 65.533, but the furthest real return measured across the bags
#: was 9.02 m, so 10.0 is the honest ceiling. The ROS node must clamp to this
#: before normalising, or a no-return reads 6.55 instead of 1.0.
UST10LX_MAX_RANGE: float = 10.0
UST10LX_MIN_RANGE: float = 0.06

#: Half-angle of the sector counted as "ahead", radians. The rover is
#: 0.38 m wide and drives nose first, so this is the arc it can actually
#: drive into. Wider than the body alone to leave room to turn out.
FORWARD_SECTOR: float = np.deg2rad(40.0)

#: Fraction of beams returning nothing, measured over the bags. All of them
#: came back as 65.533 m, never zero, so min-pooling a real scan is safe.
UST10LX_DROPOUT: float = 0.0184

#: ZED 2i depth, from navigation_pkg's costmap config. The height band is
#: relative to the floor: below 0.036 m is the floor itself, above 0.50 m
#: passes over the rover.
ZED_FOV: float = np.deg2rad(110.0)
ZED_MIN_RANGE: float = 0.25
ZED_MAX_RANGE: float = 2.5
ZED_MIN_HEIGHT: float = 0.036
ZED_MAX_HEIGHT: float = 0.50

#: Elevations sampled per azimuth. Five is enough to catch a 4 cm object at
#: 2.5 m without casting a full depth image.
ZED_ELEVATIONS: tuple[float, ...] = (
    np.deg2rad(-28.0),
    np.deg2rad(-18.0),
    np.deg2rad(-10.0),
    np.deg2rad(-4.0),
    np.deg2rad(2.0),
)


@dataclass(frozen=True)
class LidarSpec:
    """Geometry of the simulated scan.

    Attributes:
        beams: Number of rays, evenly spaced across `fov`.
        fov: Angular sweep in radians, centred on the sensor's x axis.
        max_range: Range reported where nothing is hit, in metres.
        min_range: Shortest reportable range, in metres.
    """

    beams: int = 60
    fov: float = UST10LX_FOV
    max_range: float = UST10LX_MAX_RANGE
    min_range: float = UST10LX_MIN_RANGE

    def __post_init__(self) -> None:
        if self.beams < 2:
            raise ValueError("beams must be at least 2")
        if not 0.0 < self.fov <= 2.0 * np.pi:
            raise ValueError("fov must be in (0, 2*pi]")
        if not 0.0 <= self.min_range < self.max_range:
            raise ValueError("min_range must be below max_range")

    @property
    def angles(self) -> np.ndarray:
        """Beam bearings in the sensor frame, radians, ascending."""
        return np.linspace(-self.fov / 2.0, self.fov / 2.0, self.beams)


class Lidar:
    """Casts a planar scan from a named site on the robot."""

    def __init__(
        self, model: mujoco.MjModel, site: str, body: str, spec: LidarSpec
    ) -> None:
        """Resolve the site and body the scan is anchored to.

        Args:
            model: Compiled model the scan will be cast in.
            site: Site name the rays originate from, e.g. "laser".
            body: Body whose geoms the rays must ignore, e.g. "base_link".
                Without this every beam returns the robot's own hull.
            spec: Scan geometry.

        Raises:
            ValueError: If the site or body is not in the model.
        """
        self._site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site)
        if self._site < 0:
            raise ValueError(f"no site named {site!r} in the model")
        self._body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
        if self._body < 0:
            raise ValueError(f"no body named {body!r} in the model")

        self._spec = spec
        angles = spec.angles
        # Unit directions in the sensor frame, z zero because the scan is
        # planar. Held as (beams, 3) so the site's rotation applies as one
        # matrix product per scan.
        self._local = np.stack(
            [np.cos(angles), np.sin(angles), np.zeros(spec.beams)], axis=1
        )
        self._geomid = np.zeros(spec.beams, dtype=np.int32)
        self._dist = np.zeros(spec.beams, dtype=np.float64)

    @property
    def spec(self) -> LidarSpec:
        """The scan geometry this instance was built with."""
        return self._spec

    def scan(self, model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
        """Cast the scan at the current pose.

        Requires `mj_forward` or a completed `mj_step`, since site poses come
        from the kinematics.

        Args:
            model: The model being simulated.
            data: Current state.

        Returns:
            Ranges in metres, one per beam, ascending in bearing. Beams that
            hit nothing report `max_range`.
        """
        origin = data.site_xpos[self._site]
        # site_xmat rotates sensor frame to world; rows of (local @ R.T) are
        # the world-frame directions.
        rotation = data.site_xmat[self._site].reshape(3, 3)
        world = self._local @ rotation.T

        mujoco.mj_multiRay(
            model,
            data,
            origin,
            world.ravel(),
            None,
            1,
            self._body,
            self._geomid,
            self._dist,
            None,
            self._spec.beams,
            self._spec.max_range,
        )

        # mj_multiRay reports -1 where a ray hit nothing, which has to become
        # max_range rather than a negative distance.
        ranges = np.where(self._dist < 0.0, self._spec.max_range, self._dist)
        return np.clip(ranges, self._spec.min_range, self._spec.max_range)


class DepthFan:
    """The ZED's contribution to the fused scan.

    Casts a small fan of rays per azimuth bin and keeps the nearest hit whose
    impact point lies in the obstacle height band. That is the same test
    Nav2's voxel handling applies to the point cloud, reduced to 2D, rather
    than a full depth image the policy would then have to encode.

    Only the bins inside the camera's 110 degree field of view are touched;
    the rest of the scan is the laser's alone.
    """

    def __init__(
        self, model: mujoco.MjModel, site: str, body: str, spec: LidarSpec
    ) -> None:
        """Resolve the camera site and precompute the ray directions.

        Args:
            model: Compiled model the rays are cast in.
            site: Site the camera sits at, e.g. "zed_mount".
            body: Body whose geoms the rays ignore, e.g. "base_link".
            spec: The scan geometry being contributed to.

        Raises:
            ValueError: If the site or body is not in the model.
        """
        self._site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site)
        if self._site < 0:
            raise ValueError(f"no site named {site!r} in the model")
        self._body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
        if self._body < 0:
            raise ValueError(f"no body named {body!r} in the model")

        self._spec = spec
        azimuths = spec.angles
        self._bins = np.nonzero(np.abs(azimuths) <= ZED_FOV / 2.0)[0]
        # One ray per (bin, elevation). Held flat because mj_multiRay takes a
        # single array and one call per scan is much cheaper than many.
        directions = []
        for index in self._bins:
            bearing = azimuths[index]
            for elevation in ZED_ELEVATIONS:
                directions.append(
                    (
                        np.cos(elevation) * np.cos(bearing),
                        np.cos(elevation) * np.sin(bearing),
                        np.sin(elevation),
                    )
                )
        self._local = np.array(directions)
        self._n = len(directions)
        self._geomid = np.zeros(self._n, dtype=np.int32)
        self._dist = np.zeros(self._n, dtype=np.float64)

    @property
    def bins(self) -> np.ndarray:
        """Indices of the scan bins this fan can contribute to."""
        return self._bins

    def ranges(self, model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
        """Nearest in-band obstacle range per covered bin.

        Args:
            model: The model being simulated.
            data: Current state.

        Returns:
            One range per entry of `bins`, in metres. Bins with no in-band
            obstacle report the scan's `max_range`.
        """
        origin = data.site_xpos[self._site]
        rotation = data.site_xmat[self._site].reshape(3, 3)
        world = self._local @ rotation.T

        mujoco.mj_multiRay(
            model,
            data,
            origin,
            world.ravel(),
            None,
            1,
            self._body,
            self._geomid,
            self._dist,
            None,
            self._n,
            ZED_MAX_RANGE,
        )

        hit = self._dist >= 0.0
        # Impact height above the floor decides whether the costmap would
        # have marked this point at all.
        heights = origin[2] + self._dist * world[:, 2]
        in_band = (heights >= ZED_MIN_HEIGHT) & (heights <= ZED_MAX_HEIGHT)
        in_range = (self._dist >= ZED_MIN_RANGE) & (self._dist <= ZED_MAX_RANGE)
        usable = hit & in_band & in_range

        per_bin = np.where(usable, self._dist, self._spec.max_range)
        return per_bin.reshape(len(self._bins), len(ZED_ELEVATIONS)).min(axis=1)


def fuse(scan: np.ndarray, depth: np.ndarray, bins: np.ndarray) -> np.ndarray:
    """Merge the camera's ranges into the laser scan.

    Takes the nearer of the two per bin, which is what a costmap does when
    two sources mark the same cell: either sensor seeing an obstacle is
    enough.

    Args:
        scan: Laser ranges, one per beam.
        depth: Camera ranges, one per entry of `bins`.
        bins: Scan indices the camera covers.

    Returns:
        The fused scan. The input is not modified.
    """
    out = scan.copy()
    out[bins] = np.minimum(out[bins], depth)
    return out


def apply_dropout(
    ranges: np.ndarray,
    rate: float,
    max_range: float,
    rng: np.random.Generator,
) -> np.ndarray:
    """Blank a fraction of beams, as the real sensor does.

    Measured at 1.84% of beams across the bags, every one returning 65.533 m
    rather than zero. A dropped beam therefore reads as open space, not as an
    obstacle underfoot, so this sets them to `max_range`.

    Args:
        ranges: Ranges in metres.
        rate: Fraction of beams to blank, in 0..1.
        max_range: Value a blanked beam reports.
        rng: Source of randomness.

    Returns:
        A copy with the dropped beams blanked.
    """
    if rate <= 0.0:
        return ranges
    out = ranges.copy()
    out[rng.random(len(out)) < rate] = max_range
    return out


def downsample(ranges: np.ndarray, beams: int) -> np.ndarray:
    """Reduce a full-resolution scan to `beams` values.

    Each output beam takes the MINIMUM of its input bin, never the mean. A
    mean lets a narrow obstacle average away against the open space beside
    it, which is exactly the obstacle that matters.

    Intended for the ROS node, so a real `/scan` is reduced the same way the
    training scan was built.

    Args:
        ranges: Full-resolution ranges, ascending in bearing.
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


def forward_clearance(ranges: np.ndarray, spec: LidarSpec) -> float:
    """Shortest range inside the sector the rover can drive into.

    `min(ranges)` throws direction away, and a reward built on it can only
    ever teach "slow down when something is close". Measured: feeding the
    trained policy pure NOISE instead of a real scan produced an identical
    slowdown, because random values contain small numbers and the alarm is
    scalar. Restricting to the forward arc makes turning away from an
    obstacle visible in the reward, which is what a steering gradient needs.

    Args:
        ranges: Fused scan in metres, ascending in bearing.
        spec: Supplies the beam bearings.

    Returns:
        The nearest range within FORWARD_SECTOR of straight ahead.
    """
    ahead = np.abs(spec.angles) <= FORWARD_SECTOR
    return float(np.min(ranges[ahead]))
