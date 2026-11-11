"""Simulated 2D laser scan, matching the Hokuyo UST-10LX on the rover.

Rays are cast with `mj_multiRay`, which is fast enough to be irrelevant to
training cost: 60 beams measured 77,338 scans/s on one core, against the
4,800 scans/s that 240 environments at 20 Hz actually need.

The policy sees a DOWNSAMPLED scan, not the hardware's full resolution. The
real `/scan` carries ~1081 beams, far more than a driving policy can use, and
the ROS node must reduce it the same way this module does or the observation
will not match at deployment.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise

import mujoco
import numpy as np

#: The UST-10LX sweeps 270 degrees, not a full circle: the rear quadrant is
#: the cable and the housing's dead zone.
UST10LX_FOV: float = 1.5 * np.pi

#: Datasheet range. Below min_range the hardware reports garbage, so the
#: simulated scan floors at the same place rather than reporting a true zero.
UST10LX_MAX_RANGE: float = 10.0
UST10LX_MIN_RANGE: float = 0.06


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
