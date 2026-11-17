"""Export a trained policy to ONNX and prove the export did not change it.

    env -u PYTHONPATH ../../../.venvs/rl_train/bin/python tools/export_onnx.py \
        policies/tracker_v1.zip

Writes `<policy>.onnx` beside the zip, then drives the simulator with the
ORIGINAL policy and replays every observation through the exported one. The
two must agree to a tolerance far tighter than the action resolution the
rover can act on; anything looser is not an export, it is a different policy.

Only the ACTOR crosses over. The critic predicts returns during training and
has no job on the robot, and leaving it out of the graph keeps the file to
the matrix multiplies the rover actually runs.

Exports the DETERMINISTIC action: the distribution's mean, clipped to the
action space, which is what `model.predict(deterministic=True)` returns.
Sampling is an exploration device and belongs to training.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from envs import scenarios
from envs.deploy import OBS_DIM
from envs.lidar import LidarSpec
from envs.rover_env import RoverEnv
from rl_bridge.policy import OnnxPolicy

#: Largest action difference tolerated between the zip and the .onnx. The
#: rover's yaw command spans 1.6 rad/s over the -1..1 action range, so this
#: is roughly 16 nanoradians per second of disagreement.
TOLERANCE: float = 1e-5

#: ONNX opset. 18 is the lowest the installed exporter implements directly;
#: asking for less makes it export at 18 and convert back down, which no
#: Gemm-and-Tanh graph needs.
OPSET: int = 18


def build_env(scenario_name: str, seed: int) -> RoverEnv:
    """Construct the environment the parity rollout runs in."""
    if scenario_name.endswith(".yaml"):
        scenario = scenarios.map_scenario(Path(scenario_name).expanduser().resolve())
    else:
        scenario = (
            next(
                (s for s in scenarios.SUITE if s.name == scenario_name),
                None,
            )
            or scenarios.Scenario()
        )
    return RoverEnv(scenario=scenario, lidar_spec=LidarSpec(), seed=seed)


def actor_graph(policy):
    """Wrap an SB3 policy as a module that returns the deterministic action.

    Args:
        policy: The loaded `ActorCriticPolicy`.

    Returns:
        A `torch.nn.Module` taking (batch, OBS_DIM) and returning (batch, 3).
    """
    import torch
    from torch import nn

    class Actor(nn.Module):
        """Observation to action, with no critic and no sampling."""

        def __init__(self) -> None:
            super().__init__()
            self.policy = policy

        def forward(self, observation: torch.Tensor) -> torch.Tensor:
            features = self.policy.extract_features(observation)
            # Without a shared extractor SB3 hands back (pi, vf) features;
            # only the actor's half is wanted either way.
            if not self.policy.share_features_extractor:
                features = features[0]
            latent = self.policy.mlp_extractor.forward_actor(features)
            # predict() clips to the action space after taking the mean, so
            # the graph has to clip too or the two disagree at saturation,
            # which is most of the time at full throttle.
            return torch.clamp(self.policy.action_net(latent), -1.0, 1.0)

    actor = Actor()
    actor.eval()
    return actor


def export(policy_path: Path, out: Path) -> None:
    """Write the actor of `policy_path` to `out` as ONNX."""
    import torch
    from stable_baselines3 import PPO

    model = PPO.load(str(policy_path), device="cpu")
    dim = int(model.observation_space.shape[0])
    if dim != OBS_DIM:
        raise SystemExit(
            f"{policy_path.name} takes {dim} observation values but the "
            f"contract supplies {OBS_DIM}: it was trained against a "
            "different layout and cannot be deployed by this bridge"
        )

    actor = actor_graph(model.policy)
    with torch.no_grad():
        torch.onnx.export(
            actor,
            torch.zeros(1, dim, dtype=torch.float32),
            str(out),
            input_names=["observation"],
            output_names=["action"],
            # Batch stays dynamic so the same file serves one live
            # observation on the robot and a whole bank in a probe.
            dynamic_axes={"observation": {0: "batch"}, "action": {0: "batch"}},
            opset_version=OPSET,
            # One file. The default spills 340 kB of weights into a sidecar
            # .onnx.data, and copying the graph alone to the robot then
            # yields a policy with no weights.
            external_data=False,
        )
    print(f"wrote {out} ({out.stat().st_size / 1024:.0f} kB)")


def collect(policy_path: Path, scenario: str, seed: int, steps: int) -> np.ndarray:
    """Gather observations the policy actually visits.

    Random vectors would exercise corners of the input space the rover never
    reaches and miss the saturated commands it lives in. Driving the real
    policy through the simulator samples the distribution that matters.

    Args:
        policy_path: The .zip to drive with.
        scenario: Suite name or map yaml.
        seed: Episode seed.
        steps: Observations wanted.

    Returns:
        An array of shape (steps, OBS_DIM).
    """
    from stable_baselines3 import PPO

    model = PPO.load(str(policy_path), device="cpu")
    env = build_env(scenario, seed)
    bank: list[np.ndarray] = []
    episode = 0
    obs, _info = env.reset(seed=seed)
    while len(bank) < steps:
        bank.append(np.asarray(obs, dtype=np.float32))
        action, _state = model.predict(obs, deterministic=True)
        obs, _reward, terminated, truncated, _info = env.step(action)
        if terminated or truncated:
            episode += 1
            obs, _info = env.reset(seed=seed + episode)
    return np.asarray(bank[:steps])


def verify(policy_path: Path, onnx_path: Path, bank: np.ndarray) -> float:
    """Compare the zip's actions with the exported graph's, on the same inputs.

    Args:
        policy_path: The original .zip.
        onnx_path: The exported graph.
        bank: Observations to replay.

    Returns:
        The largest absolute action difference seen.
    """
    from stable_baselines3 import PPO

    model = PPO.load(str(policy_path), device="cpu")
    runner = OnnxPolicy(onnx_path, dim=bank.shape[1])

    reference = np.asarray(
        [model.predict(obs, deterministic=True)[0] for obs in bank], dtype=np.float64
    )
    exported = np.asarray([runner.act(obs) for obs in bank], dtype=np.float64)
    return float(np.abs(reference - exported).max())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("policy", type=Path, help="a trained .zip")
    ap.add_argument("--out", type=Path, default=None, help="defaults to <policy>.onnx")
    ap.add_argument(
        "--scenario",
        default="cluttered",
        help="suite name or map .yaml the parity rollout runs in",
    )
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument(
        "--samples",
        type=int,
        default=1000,
        help="observations replayed through both policies",
    )
    args = ap.parse_args()

    out = args.out or args.policy.with_suffix(".onnx")
    export(args.policy, out)

    print(f"collecting {args.samples} observations in '{args.scenario}'")
    bank = collect(args.policy, args.scenario, args.seed, args.samples)
    worst = verify(args.policy, out, bank)

    print(f"max action difference over {len(bank)} observations: {worst:.3e}")
    if worst > TOLERANCE:
        print(f"FAILED: above the {TOLERANCE:.0e} tolerance, do not deploy this file")
        return 1
    print(f"parity OK, within {TOLERANCE:.0e}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
