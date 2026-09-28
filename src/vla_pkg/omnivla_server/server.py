#!/usr/bin/env python3
"""HTTP front end for the OmniVLA policy.

Runs in its own virtualenv, deliberately outside ROS: OmniVLA pins torch and a
transformers fork that cannot coexist with ROS 2 Jazzy on Python 3.12. The
process boundary also makes the backend swappable (edge, 7.5B, TensorRT) and
lets the policy move to a workstation by changing one URL.

Stateless by design. The caller owns the observation history, so this process
can be restarted mid-run without resynchronising anything.
"""

from __future__ import annotations

import argparse
import time

import preprocessing
import uvicorn
from backends.base import PolicyBackend
from backends.edge import CONTEXT_FRAMES, DEFAULT_CHECKPOINT, OmniVLAEdgeBackend
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field

app = FastAPI(title="OmniVLA policy server")

_backend: PolicyBackend | None = None


class ActRequest(BaseModel):
    """One inference request."""

    frames_b64: list[str] = Field(
        description="Base64 JPEGs, oldest first, current last"
    )
    goal_text: str | None = None
    goal_image_b64: str | None = None
    goal_pose: list[float] | None = Field(
        default=None, description="[x, y, yaw] in metres/radians, robot body frame"
    )


class ActResponse(BaseModel):
    """A predicted waypoint chunk."""

    waypoints: list[list[float]] = Field(
        description="(N, 4) [dx, dy, cos, sin] in metres"
    )
    modality_id: int
    latency_ms: float


@app.get("/health")
def health() -> dict[str, object]:
    """Readiness probe, so callers can wait out the model load."""
    return {
        "ready": _backend is not None,
        "context_frames": CONTEXT_FRAMES,
        "chunk_size": _backend.chunk_size if _backend else None,
    }


@app.post("/act", response_model=ActResponse)
def act(request: ActRequest) -> ActResponse:
    """Run one inference and return the waypoint chunk."""
    if _backend is None:
        raise HTTPException(status_code=503, detail="backend still loading")

    started = time.perf_counter()
    frames = [preprocessing.decode_jpeg(f) for f in request.frames_b64]
    goal_image = (
        preprocessing.decode_jpeg(request.goal_image_b64)
        if request.goal_image_b64
        else None
    )
    pose = tuple(request.goal_pose) if request.goal_pose else None
    if pose is not None and len(pose) != 3:
        raise HTTPException(status_code=422, detail="goal_pose must be [x, y, yaw]")

    try:
        waypoints, modality = _backend.act(
            frames, goal_text=request.goal_text, goal_image=goal_image, goal_pose=pose
        )
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc

    return ActResponse(
        waypoints=waypoints.tolist(),
        modality_id=modality,
        latency_ms=(time.perf_counter() - started) * 1000.0,
    )


def main() -> None:
    """Load the backend, then serve until killed."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8777)
    parser.add_argument("--checkpoint", default=str(DEFAULT_CHECKPOINT))
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()

    global _backend
    _backend = OmniVLAEdgeBackend(checkpoint=args.checkpoint, device=args.device)
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
