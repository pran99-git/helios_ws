"""Backend interface shared by every OmniVLA policy implementation."""

from __future__ import annotations

from typing import Protocol

import numpy as np
from PIL import Image


class PolicyBackend(Protocol):
    """A goal-conditioned navigation policy returning a waypoint chunk.

    Implementations wrap one model (OmniVLA-edge, the 7.5B checkpoint, or a
    TensorRT engine). They are swapped behind this interface so the ROS side
    never changes.
    """

    #: Waypoints per chunk, as baked into the checkpoint.
    chunk_size: int

    def act(
        self,
        frames: list[Image.Image],
        goal_text: str | None = None,
        goal_image: Image.Image | None = None,
        goal_pose: tuple[float, float, float] | None = None,
    ) -> tuple[np.ndarray, int]:
        """Predict a waypoint chunk.

        Args:
            frames: Observation history, oldest first, current frame last.
            goal_text: Natural-language instruction, or None.
            goal_image: Egocentric goal image, or None.
            goal_pose: Goal as ``(x, y, yaw)`` in metres and radians, in the
                robot body frame (REP-103: x forward, y left).

        Returns:
            A ``(chunk_size, 4)`` array of ``[dx, dy, cos, sin]`` in metres,
            where dx/dy are cumulative displacements from the robot, and the
            modality id the model was conditioned on.
        """
        ...

    def score(self, views: list[Image.Image], phrases: list[str]) -> np.ndarray:
        """Rate how well each view matches each phrase.

        Separate from `act` on purpose: this answers "which way looks like
        where I was told to go", which is a retrieval question, and goes
        through the vision-language encoder directly rather than through the
        action decoder.

        Args:
            views: Images to score.
            phrases: Descriptions to score them against.

        Returns:
            A ``(len(views), len(phrases))`` array of similarities, comparable
            across views for a single phrase.
        """
        ...
