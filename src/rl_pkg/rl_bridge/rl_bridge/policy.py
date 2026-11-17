"""Run an exported policy, without torch or stable-baselines3.

Training needs a 2 GB torch wheel; driving needs a 400 kB matrix multiply.
The actor is two 256-unit layers and a linear head, so onnxruntime on one CPU
core answers in well under a millisecond and the Orin's GPU stays free for
the ZED.

Only the ACTOR is exported. The critic predicts returns during training and
has no job on the robot.

Imports nothing from ROS.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import numpy as np

from rl_bridge.contract import blind


class OnnxPolicy:
    """A deterministic policy loaded from an .onnx file.

    The exported graph is the distribution's MEAN with a clip to -1..1, which
    is what `model.predict(deterministic=True)` returns. Sampling is an
    exploration device and belongs to training; on the robot the same
    observation must always produce the same command.
    """

    def __init__(
        self,
        path: Path | str,
        dim: int,
        blind_channels: Sequence[int] = (),
    ) -> None:
        """Load the graph and check it against the observation contract.

        Args:
            path: The exported .onnx file.
            dim: Observation length the caller will supply, from the
                contract. Checked against the graph, because a policy fed a
                vector of the wrong length is the one failure mode that
                produces plausible-looking commands rather than an error.
            blind_channels: Observation channels to zero before inference.

        Raises:
            ValueError: If the graph does not take `dim` inputs.
        """
        import onnxruntime as ort

        self._session = ort.InferenceSession(
            str(path), providers=["CPUExecutionProvider"]
        )
        spec = self._session.get_inputs()[0]
        self._input = spec.name
        expected = spec.shape[-1]
        if isinstance(expected, int) and expected != dim:
            raise ValueError(
                f"{Path(path).name} expects {expected} observation values, "
                f"the contract supplies {dim}"
            )
        self._dim = dim
        self._blind = tuple(int(c) for c in blind_channels)

    @property
    def blinded(self) -> tuple[int, ...]:
        """Channels zeroed before every inference."""
        return self._blind

    def act(self, observation: np.ndarray) -> np.ndarray:
        """Map one observation to an action in -1..1.

        Args:
            observation: The assembled vector.

        Returns:
            The action as [vx, vy, wz], still normalised.

        Raises:
            ValueError: If the observation is the wrong length.
        """
        obs = np.asarray(observation, dtype=np.float32)
        if obs.shape[-1] != self._dim:
            raise ValueError(
                f"expected {self._dim} observation values, got {obs.shape[-1]}"
            )
        obs = blind(obs, self._blind).reshape(1, -1).astype(np.float32)
        return np.asarray(self._session.run(None, {self._input: obs})[0][0])
