"""Wire protocol to the OmniVLA policy server.

The server is a separate process, on loopback today and possibly on a
workstation later. Only the endpoint changes, so every failure here is treated
as "no plan this cycle" rather than an exception: the caller publishes an empty
path and the rover coasts to a stop on its existing watchdogs.
"""

from __future__ import annotations

import base64
import io
from collections.abc import Sequence
from dataclasses import dataclass

import requests
from PIL import Image

from vla_bridge.chunk_decoding import Waypoint, decode_chunk
from vla_bridge.goal_resolution import Pose2D

#: Largest resolution the model consumes (its CLIP branch). Frames are scaled
#: to this before the wire, which costs nothing in fidelity and keeps the
#: payload small enough to survive a move off-board.
WIRE_SIZE = 224


def encode_jpeg(image: Image.Image, size: int = WIRE_SIZE, quality: int = 90) -> str:
    """Downscale an RGB frame and encode it as a base64 JPEG."""
    buffer = io.BytesIO()
    image.resize((size, size)).save(buffer, format="JPEG", quality=quality)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


@dataclass(frozen=True)
class PolicyResult:
    """One decoded prediction."""

    waypoints: list[Waypoint]
    modality_id: int
    latency_ms: float


class PolicyClient:
    """Blocking HTTP client for the policy server."""

    def __init__(self, endpoint: str, timeout: float = 0.3) -> None:
        """
        Args:
            endpoint: Full URL of the server's /act route.
            timeout: Seconds before a request is abandoned. Keep this below
                the caller's staleness window so a slow server degrades into
                an empty path rather than a late one.
        """
        self._endpoint = endpoint
        self._timeout = timeout
        self._session = requests.Session()
        self.last_error: str | None = None

    def act(
        self,
        frames: Sequence[str],
        goal_text: str | None = None,
        goal_image: str | None = None,
        goal_pose: Pose2D | None = None,
    ) -> PolicyResult | None:
        """Request one waypoint chunk.

        Args:
            frames: Base64 JPEGs, oldest first, current last.
            goal_text: Language instruction, or None.
            goal_image: Base64 JPEG of the goal view, or None.
            goal_pose: Goal in the robot body frame, or None.

        Returns:
            The decoded prediction, or None on any transport or server error.
            `last_error` carries the reason.
        """
        payload: dict[str, object] = {"frames_b64": list(frames)}
        if goal_text:
            payload["goal_text"] = goal_text
        if goal_image:
            payload["goal_image_b64"] = goal_image
        if goal_pose is not None:
            payload["goal_pose"] = [goal_pose.x, goal_pose.y, goal_pose.yaw]

        try:
            response = self._session.post(
                self._endpoint, json=payload, timeout=self._timeout
            )
            response.raise_for_status()
            body = response.json()
        except requests.Timeout:
            self.last_error = f"timeout after {self._timeout:.2f}s"
            return None
        except requests.RequestException as exc:
            self.last_error = str(exc)
            return None

        try:
            result = PolicyResult(
                waypoints=decode_chunk(body["waypoints"]),
                modality_id=int(body["modality_id"]),
                latency_ms=float(body["latency_ms"]),
            )
        except (KeyError, TypeError, ValueError) as exc:
            self.last_error = f"malformed response: {exc}"
            return None

        self.last_error = None
        return result
