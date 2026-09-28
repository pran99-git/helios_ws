"""Observation history for the policy's context window.

OmniVLA-edge expects six frames, oldest first. The camera runs far faster than
the policy, so frames are admitted on a fixed stride rather than on arrival.

`stride` is a TUNING PARAMETER, not a constant from the checkpoint. The vendor
demo feeds six copies of one frame ("the robot stopped"), so the spacing used
during training is not documented anywhere. 1/3 s matches the policy's 3 Hz
design rate and is the sensible starting point.
"""

from __future__ import annotations

from collections import deque
from typing import Generic, TypeVar

Frame = TypeVar("Frame")


class ContextBuffer(Generic[Frame]):
    """Fixed-length frame history admitted on a minimum time stride."""

    def __init__(self, size: int = 6, stride: float = 1.0 / 3.0) -> None:
        """
        Args:
            size: Frames the model expects, including the current one.
            stride: Minimum seconds between admitted frames.
        """
        if size < 1:
            raise ValueError("size must be at least 1")
        self._frames: deque[Frame] = deque(maxlen=size)
        self._size = size
        self._stride = stride
        self._last_stamp: float | None = None

    def push(self, frame: Frame, stamp: float) -> bool:
        """Offer a frame, admitting it only if the stride has elapsed.

        Returns:
            True if the frame was admitted.
        """
        if self._last_stamp is not None and stamp - self._last_stamp < self._stride:
            return False
        self._frames.append(frame)
        self._last_stamp = stamp
        return True

    def frames(self) -> list[Frame]:
        """Return exactly `size` frames, oldest first.

        Before the window fills, the oldest real frame is repeated at the
        front. That encodes "stationary until now", which is true at startup
        and is what the vendor demo assumes.

        Raises:
            RuntimeError: If no frame has been admitted yet.
        """
        if not self._frames:
            raise RuntimeError("context buffer is empty")
        padding = [self._frames[0]] * (self._size - len(self._frames))
        return padding + list(self._frames)

    @property
    def ready(self) -> bool:
        """Whether `frames()` can be called at all."""
        return bool(self._frames)

    @property
    def warm(self) -> bool:
        """Whether the window holds `size` real frames, with no padding."""
        return len(self._frames) == self._size

    def clear(self) -> None:
        """Drop all history, e.g. after the policy is re-goaled or paused."""
        self._frames.clear()
        self._last_stamp = None
