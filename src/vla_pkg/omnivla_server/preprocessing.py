"""Tensor assembly for OmniVLA-edge.

Per-image transforms are the vendor's; only the batching, concatenation and
goal-encoding rules live here. Shapes below were confirmed against the running
checkpoint, not inferred from the paper.
"""

from __future__ import annotations

import base64
import io
import math

import numpy as np
import torch
from PIL import Image
from vendor import transform_images_map, transform_images_PIL_mask

OBS_SIZE = (96, 96)
CLIP_SIZE = (224, 224)
SATELLITE_SIZE = (352, 352)

#: Model output is in units of this many metres per waypoint step.
WAYPOINT_SPACING = 0.1

#: Goals beyond this range are clipped along the same bearing, matching the
#: vendor's thres_dist. The model never saw farther goals during training.
GOAL_CLAMP_M = 30.0

# The vendor masks fisheye borders out of the observation. The ZED 2i is a
# rectilinear stereo camera, so nothing is masked.
_MASK_OBS = np.ones((*OBS_SIZE, 3), dtype=np.float32)
_MASK_CLIP = np.ones((*CLIP_SIZE, 3), dtype=np.float32)

_BLACK_SATELLITE = Image.new("RGB", SATELLITE_SIZE, color=(0, 0, 0))

# Helios feeds constant black for both satellite slots and for an unused goal
# image, so their transforms are computed once instead of every inference.
_SATELLITE_TENSOR: torch.Tensor | None = None
_BLANK_GOAL_TENSOR: torch.Tensor | None = None


def decode_jpeg(payload: str) -> Image.Image:
    """Decode a base64 JPEG into an RGB image."""
    return Image.open(io.BytesIO(base64.b64decode(payload))).convert("RGB")


def build_observation(frames: list[Image.Image]) -> tuple[torch.Tensor, torch.Tensor]:
    """Stack the observation history into the model's channel-wise input.

    Args:
        frames: Oldest first, current frame last.

    Returns:
        ``(obs_images, obs_current)`` shaped ``(1, 3n, 96, 96)`` and
        ``(1, 3, 96, 96)``.
    """
    resized = [f.resize(OBS_SIZE) for f in frames]
    obs = transform_images_PIL_mask(resized, _MASK_OBS)
    return obs, obs[:, -3:]


def build_clip_view(frame: Image.Image) -> torch.Tensor:
    """Render the current frame at the CLIP encoder's resolution."""
    return transform_images_PIL_mask(frame.resize(CLIP_SIZE), _MASK_CLIP)


def build_goal_image(goal: Image.Image | None) -> torch.Tensor:
    """Encode the egocentric goal image, black when the modality is unused."""
    global _BLANK_GOAL_TENSOR
    if goal is not None:
        return transform_images_PIL_mask(goal.resize(OBS_SIZE), _MASK_OBS)
    if _BLANK_GOAL_TENSOR is None:
        blank = Image.new("RGB", OBS_SIZE, (0, 0, 0))
        _BLANK_GOAL_TENSOR = transform_images_PIL_mask(blank, _MASK_OBS)
    return _BLANK_GOAL_TENSOR


def build_map_images(obs_current: torch.Tensor) -> torch.Tensor:
    """Concatenate the satellite pair with the current observation.

    Helios carries no overhead imagery, so both satellite slots are black; the
    model masks them out via the modality id.
    """
    global _SATELLITE_TENSOR
    if _SATELLITE_TENSOR is None:
        _SATELLITE_TENSOR = transform_images_map(_BLACK_SATELLITE)
    pair = _SATELLITE_TENSOR.to(obs_current)
    return torch.cat((pair, pair, obs_current), dim=1)


def encode_goal_pose(x: float, y: float, yaw: float) -> torch.Tensor:
    """Encode a body-frame goal into the model's 4-vector.

    Args:
        x: Forward offset in metres (REP-103 body frame).
        y: Leftward offset in metres.
        yaw: Goal heading relative to the robot, in radians.

    Returns:
        ``(1, 4)`` of ``[x / spacing, y / spacing, cos yaw, sin yaw]``.
    """
    radius = math.hypot(x, y)
    if radius > GOAL_CLAMP_M:
        scale = GOAL_CLAMP_M / radius
        x, y = x * scale, y * scale
    return torch.tensor(
        [[x / WAYPOINT_SPACING, y / WAYPOINT_SPACING, math.cos(yaw), math.sin(yaw)]],
        dtype=torch.float32,
    )
