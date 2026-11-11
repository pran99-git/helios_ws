"""Train a point-goal navigation policy with PPO.

    env -u PYTHONPATH ../../../.venvs/rl_train/bin/python train/ppo.py
    env -u PYTHONPATH ../../../.venvs/rl_train/bin/python train/ppo.py \
        --config train/config/rover_ppo.yaml --timesteps 200000

Everything lands in runs/<name>_<timestamp>/, which is gitignored:

    config.yaml          the exact config this run used
    best/best_model.zip  best mean eval reward so far
    checkpoints/         periodic snapshots
    tb/                  tensorboard event files
    final.zip            the policy at the end of training
"""

from __future__ import annotations

import argparse
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

# The package root is two levels up. Same reason as test/conftest.py: this
# script is run directly, so `envs` is not otherwise importable.
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from envs import scenarios
from envs.lidar import LidarSpec
from envs.rover_env import RoverEnv

DEFAULT_CONFIG = ROOT / "train" / "config" / "rover_ppo.yaml"
RUNS = ROOT / "runs"


def find_scenario(name: str) -> scenarios.Scenario:
    """Resolve a scenario by suite name or by path to a saved map.

    A name ending in .yaml is treated as a slam_toolbox map, so training can
    run in the real room without the path being baked into the suite.

    Args:
        name: Suite name such as "sparse", or a path to a map yaml.

    Returns:
        The matching scenario.

    Raises:
        ValueError: If no scenario has that name.
        FileNotFoundError: If a map path does not exist.
    """
    if name.endswith(".yaml"):
        return scenarios.map_scenario(Path(name).expanduser().resolve())
    for scenario in scenarios.SUITE:
        if scenario.name == name:
            return scenario
    known = ", ".join(s.name for s in scenarios.SUITE)
    raise ValueError(f"unknown scenario {name!r}; known: {known}, or a map .yaml")


def make_env(cfg: dict[str, Any], scenario_name: str, seed: int):
    """Build a thunk that constructs one environment.

    SubprocVecEnv needs a callable per worker rather than an instance, since
    each environment is constructed inside its own process.

    Args:
        cfg: The parsed config.
        scenario_name: Scenario to instantiate.
        seed: Seed for this worker, offset per worker by the caller.

    Returns:
        A zero-argument callable returning a fresh environment.
    """
    env_cfg = cfg["env"]

    def _init() -> RoverEnv:
        return RoverEnv(
            scenario=find_scenario(scenario_name),
            lidar_spec=LidarSpec(beams=int(env_cfg["lidar_beams"])),
            decimation=int(env_cfg["decimation"]),
            mass_randomisation=float(env_cfg["mass_randomisation"]),
            seed=seed,
        )

    return _init


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument(
        "--timesteps", type=int, default=None, help="override run.total_timesteps"
    )
    ap.add_argument("--name", type=str, default=None, help="override run.name")
    args = ap.parse_args()

    cfg: dict[str, Any] = yaml.safe_load(args.config.read_text())
    run_cfg, ppo_cfg = cfg["run"], cfg["ppo"]
    timesteps = args.timesteps or int(run_cfg["total_timesteps"])
    name = args.name or str(run_cfg["name"])
    seed = int(run_cfg["seed"])
    n_envs = int(run_cfg["n_envs"])

    # UTC then converted to local, so the folder name reads as wall clock
    # while the call itself stays timezone aware.
    stamp = datetime.now(timezone.utc).astimezone()
    out = RUNS / f"{name}_{stamp:%Y%m%d_%H%M%S}"
    (out / "checkpoints").mkdir(parents=True, exist_ok=True)
    shutil.copy(args.config, out / "config.yaml")

    # Imported here, not at module scope: SB3 pulls in torch, which takes
    # seconds, and --help should not pay for it.
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor

    train_env = VecMonitor(
        SubprocVecEnv(
            [make_env(cfg, str(run_cfg["scenario"]), seed + i) for i in range(n_envs)]
        )
    )
    # One process for eval: it runs rarely and sequentially, so the worker
    # overhead buys nothing.
    eval_env = Monitor(make_env(cfg, str(cfg["eval"]["scenario"]), seed + 10_000)())

    model = PPO(
        "MlpPolicy",
        train_env,
        n_steps=int(ppo_cfg["n_steps"]),
        batch_size=int(ppo_cfg["batch_size"]),
        n_epochs=int(ppo_cfg["n_epochs"]),
        learning_rate=float(ppo_cfg["learning_rate"]),
        gamma=float(ppo_cfg["gamma"]),
        gae_lambda=float(ppo_cfg["gae_lambda"]),
        clip_range=float(ppo_cfg["clip_range"]),
        ent_coef=float(ppo_cfg["ent_coef"]),
        vf_coef=float(ppo_cfg["vf_coef"]),
        max_grad_norm=float(ppo_cfg["max_grad_norm"]),
        policy_kwargs={"net_arch": list(ppo_cfg["net_arch"])},
        tensorboard_log=str(out / "tb"),
        device=str(run_cfg["device"]),
        seed=seed,
        verbose=1,
    )

    callbacks = [
        EvalCallback(
            eval_env,
            best_model_save_path=str(out / "best"),
            log_path=str(out / "best"),
            # Divided by n_envs because SB3 counts callback frequency in
            # per-worker steps, not total steps.
            eval_freq=max(1, int(cfg["eval"]["every"]) // n_envs),
            n_eval_episodes=int(cfg["eval"]["episodes"]),
            deterministic=True,
        ),
        CheckpointCallback(
            save_freq=max(1, int(cfg["checkpoint"]["every"]) // n_envs),
            save_path=str(out / "checkpoints"),
            name_prefix="rover",
        ),
    ]

    print(f"run dir: {out}")
    print(f"tensorboard --logdir {out / 'tb'}")
    try:
        model.learn(total_timesteps=timesteps, callback=callbacks, progress_bar=False)
    except KeyboardInterrupt:
        print("\ninterrupted, saving what we have")
    finally:
        model.save(str(out / "final"))
        train_env.close()
        eval_env.close()

    print(f"saved {out / 'final.zip'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
