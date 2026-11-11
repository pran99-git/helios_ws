"""OmniVLA-edge backend: 108 M parameters, ~98 ms per inference on an AGX Orin.

The vendor ships inference code only for this variant; its LoRA script covers
the 7.5B checkpoints. The model itself is a standard nn.Module, so fine-tuning
it means writing a loss and a training loop, not unlocking anything.

As shipped the checkpoint is un-adapted: it carries the nonholonomic prior of
its training mix and has never seen this camera height or field of view.
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
        model, text_encoder, clip_preprocess = load_model(
            str(path), MODEL_PARAMS, self.device
        )
        self.model = model.to(self.device).eval()
        # The full CLIP model, not just its text tower: `score` uses its image
        # side, and this is the same instance the policy conditions on, so no
        # second copy of the weights is loaded.
        self.text_encoder = text_encoder.to(self.device).eval()
        # CLIP's own transform, as returned by clip.load. Using the vendor's
        # observation transform here instead would feed the image tower a
        # different normalisation than it was trained with.
        self.clip_preprocess = clip_preprocess

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

    def score(self, views: list[Image.Image], phrases: list[str]) -> np.ndarray:
        """Cosine similarity between each view and each phrase.

        Used to ask "which of these directions looks like what I was told to
        find". This goes through CLIP DIRECTLY rather than through the policy,
        because the policy mean-pools its single text token in with six image
        tokens and so does not ground language; CLIP's own towers do.

        Args:
            views: Images to score, typically one per candidate direction.
            phrases: Natural-language descriptions to score them against.

        Returns:
            A ``(len(views), len(phrases))`` array of cosine similarities,
            typically 0.15 to 0.35. Comparable across views for one phrase,
            NOT calibrated across phrases.

        Raises:
            ValueError: If either list is empty.
        """
        if not views or not phrases:
            raise ValueError("score needs at least one view and one phrase")

        batch = torch.stack([self.clip_preprocess(v) for v in views]).to(self.device)
        tokens = clip.tokenize(phrases, truncate=True).to(self.device)
        with torch.no_grad():
            image_features = self.text_encoder.encode_image(batch)
            text_features = self.text_encoder.encode_text(tokens)

        # Cosine similarity is the dot product of the L2-normalised features.
        image_features = image_features / image_features.norm(dim=-1, keepdim=True)
        text_features = text_features / text_features.norm(dim=-1, keepdim=True)
        return (image_features @ text_features.T).float().cpu().numpy()


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
