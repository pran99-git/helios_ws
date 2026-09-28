"""OmniVLA-edge backend: 108 M parameters, ~98 ms per inference on an AGX Orin.

Inference-only. The vendor ships no training script for this variant, so the
checkpoint is frozen and carries the nonholonomic prior of its training mix.
"""

from __future__ import annotations

import pathlib
import time

import clip
import numpy as np
import preprocessing
import torch
from PIL import Image
from vendor import load_model

DEFAULT_CHECKPOINT = (
    pathlib.Path(__file__).resolve().parents[4]
    / "models"
    / "omnivla-edge"
    / "omnivla-edge.pth"
)

#: context_size is the history depth; the model is fed that many past frames
#: plus the current one.
CONTEXT_FRAMES = 6

MODEL_PARAMS: dict[str, object] = {
    "model_type": "omnivla-edge",
    "len_traj_pred": 8,
    "learn_angle": True,
    "context_size": CONTEXT_FRAMES - 1,
    "obs_encoder": "efficientnet-b0",
    "encoding_size": 256,
    "obs_encoding_size": 1024,
    "goal_encoding_size": 1024,
    "late_fusion": False,
    "mha_num_attention_heads": 4,
    "mha_num_attention_layers": 4,
    "mha_ff_dim_factor": 4,
    "clip_type": "ViT-B/32",
}

# Which goal inputs the model should attend to, keyed by
# (has_pose, has_image, has_text). Values are the vendor's modality ids; the
# satellite combinations (0-3) are unreachable because Helios has no overhead
# imagery. The vendor defines no id for "all three" or "none".
_MODALITY_IDS: dict[tuple[bool, bool, bool], int] = {
    (True, False, False): 4,
    (False, True, False): 6,
    (False, False, True): 7,
    (True, True, False): 5,
    (True, False, True): 8,
    (False, True, True): 9,
}

#: The vendor's placeholder prompt when language conditioning is unused.
_NULL_PROMPT = "xxxx"


class OmniVLAEdgeBackend:
    """Wraps the vendor OmniVLA-edge checkpoint behind `PolicyBackend`."""

    chunk_size = 8

    def __init__(
        self,
        checkpoint: pathlib.Path | str = DEFAULT_CHECKPOINT,
        device: str = "cuda:0",
    ) -> None:
        path = pathlib.Path(checkpoint)
        if not path.is_file():
            raise FileNotFoundError(f"OmniVLA-edge checkpoint not found at {path}")

        self.device = torch.device(device if torch.cuda.is_available() else "cpu")
        model, text_encoder, _ = load_model(str(path), MODEL_PARAMS, self.device)
        self.model = model.to(self.device).eval()
        self.text_encoder = text_encoder.to(self.device).eval()

        # The text embedding depends only on the prompt, and costs ~14 ms of a
        # ~98 ms inference. Goals change far less often than frames do.
        self._text_cache: dict[str, torch.Tensor] = {}

    def _encode_text(self, prompt: str) -> torch.Tensor:
        cached = self._text_cache.get(prompt)
        if cached is None:
            tokens = clip.tokenize(prompt, truncate=True).to(self.device)
            with torch.no_grad():
                cached = self.text_encoder.encode_text(tokens)
            self._text_cache[prompt] = cached
        return cached

    def act(
        self,
        frames: list[Image.Image],
        goal_text: str | None = None,
        goal_image: Image.Image | None = None,
        goal_pose: tuple[float, float, float] | None = None,
    ) -> tuple[np.ndarray, int]:
        """Predict a waypoint chunk. See `PolicyBackend.act`."""
        if len(frames) != CONTEXT_FRAMES:
            raise ValueError(f"expected {CONTEXT_FRAMES} frames, got {len(frames)}")

        key = (goal_pose is not None, goal_image is not None, bool(goal_text))
        modality = _MODALITY_IDS.get(key)
        if modality is None:
            raise ValueError(
                "unsupported goal combination "
                f"(pose={key[0]}, image={key[1]}, text={key[2]}); "
                "the model defines no modality id for it"
            )

        obs, obs_current = preprocessing.build_observation(frames)
        obs = obs.to(self.device)
        obs_current = obs_current.to(self.device)

        pose = preprocessing.encode_goal_pose(*(goal_pose or (0.0, 0.0, 0.0)))
        batch = {
            "obs": obs,
            "pose": pose.to(self.device),
            "maps": preprocessing.build_map_images(obs_current),
            "goal_image": preprocessing.build_goal_image(goal_image).to(self.device),
            "modality": torch.tensor([modality], device=self.device),
            "text": self._encode_text(goal_text or _NULL_PROMPT),
            "clip_view": preprocessing.build_clip_view(frames[-1]).to(self.device),
        }

        with torch.no_grad():
            actions, _, _ = self.model(
                batch["obs"],
                batch["pose"],
                batch["maps"],
                batch["goal_image"],
                batch["modality"],
                batch["text"],
                batch["clip_view"],
            )

        waypoints = actions[0].float().cpu().numpy().copy()
        # dx/dy leave the model cumulative and unscaled; cos/sin are already
        # unit-normalised and must not be touched.
        waypoints[:, :2] *= preprocessing.WAYPOINT_SPACING
        return waypoints, modality


def _benchmark(iterations: int = 30) -> None:
    """Time steady-state inference, mirroring the figures in the package README."""
    backend = OmniVLAEdgeBackend()
    frames = [Image.new("RGB", (96, 96)) for _ in range(CONTEXT_FRAMES)]

    for _ in range(5):
        backend.act(frames, goal_text="blue trash bin")
    torch.cuda.synchronize()

    start = time.perf_counter()
    for _ in range(iterations):
        backend.act(frames, goal_text="blue trash bin")
    torch.cuda.synchronize()
    elapsed = (time.perf_counter() - start) / iterations
    print(f"inference {elapsed * 1000:.1f} ms -> {1 / elapsed:.1f} Hz")


if __name__ == "__main__":
    _benchmark()
