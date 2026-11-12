"""Watch a trained policy drive, in MuJoCo's viewer.

    env -u PYTHONPATH ../../../.venvs/rl_train/bin/python eval/play.py \
        runs/rover_ppo_20261111_220000/best/best_model.zip

Training itself cannot be watched: it runs 12 subprocess workers, each with
its own model and its own episode, so there is no single thing to look at.
Watch a checkpoint instead. Checkpoints land every 250 k steps, so a run can
be inspected while it is still going.

Needs the Jetson's own display. Over SSH the viewer has no GL context; use
`--frames` to write PNGs instead.

Keys in the viewer: space pauses, and the usual MuJoCo bindings apply.
Press Tab for the UI panels.
"""

from __future__ import annotations

import argparse
import sys
import time
from itertools import pairwise
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from envs import scenarios
from envs.lidar import LidarSpec
from envs.rover_env import RoverEnv

#: Route drawn slightly above the floor so it is not hidden by it.
ROUTE_Z: float = 0.02
ROUTE_RGBA: tuple[float, float, float, float] = (0.1, 0.8, 0.3, 1.0)
GOAL_RGBA: tuple[float, float, float, float] = (0.9, 0.2, 0.8, 1.0)


def draw_route(scene: mujoco.MjvScene, route: list, goal: np.ndarray) -> None:
    """Add the planned route and goal to the viewer's user scene.

    Without this the rover appears to wander: the route it is being rewarded
    for following is otherwise invisible, which makes the behaviour impossible
    to judge.

    Args:
        scene: The viewer's `user_scn`, cleared and refilled each frame.
        route: Route poses to connect.
        goal: Goal position as (x, y).
    """
    for a, b in pairwise(route):
        if scene.ngeom >= scene.maxgeom:
            break
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3),
            np.zeros(3),
            np.zeros(9),
            np.array(ROUTE_RGBA, dtype=np.float32),
        )
        mujoco.mjv_connector(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            0.015,
            np.array([a.x, a.y, ROUTE_Z]),
            np.array([b.x, b.y, ROUTE_Z]),
        )
        scene.ngeom += 1

    if scene.ngeom < scene.maxgeom:
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.10, 0.10, 0.10]),
            np.array([goal[0], goal[1], 0.10]),
            np.eye(3).ravel(),
            np.array(GOAL_RGBA, dtype=np.float32),
        )
        scene.ngeom += 1


def build_env(args: argparse.Namespace) -> RoverEnv:
    """Construct the environment named on the command line."""
    if args.scenario.endswith(".yaml"):
        scenario = scenarios.map_scenario(Path(args.scenario).expanduser().resolve())
    else:
        scenario = (
            next((s for s in scenarios.SUITE if s.name == args.scenario), None)
            or scenarios.Scenario()
        )
    return RoverEnv(
        scenario=scenario,
        lidar_spec=LidarSpec(beams=args.beams),
        seed=args.seed,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "policy", type=Path, nargs="?", help="a saved .zip; random if omitted"
    )
    ap.add_argument("--scenario", default="cluttered", help="suite name or map .yaml")
    ap.add_argument("--beams", type=int, default=60)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--episodes", type=int, default=5)
    ap.add_argument("--speed", type=float, default=1.0, help="x real time")
    ap.add_argument(
        "--camera", default="chase", help="model camera name, or -1 for free"
    )
    ap.add_argument(
        "--stride",
        type=int,
        default=2,
        help="write every Nth frame; 2 gives ~10 fps from the 20.8 Hz loop",
    )
    ap.add_argument(
        "--frames",
        type=Path,
        default=None,
        help="write PNGs here instead of opening a window, for headless use",
    )
    args = ap.parse_args()

    env = build_env(args)
    predict = None
    if args.policy:
        from stable_baselines3 import PPO

        model = PPO.load(str(args.policy), device="cpu")
        predict = lambda obs: model.predict(obs, deterministic=True)[0]
        print(f"policy: {args.policy}")
    else:
        print("no policy given, driving randomly")

    if args.frames:
        return _record(env, predict, args)
    return _watch(env, predict, args)


def _outcome(info: dict, truncated: bool) -> str:
    """One line describing how an episode ended."""
    if info["is_success"]:
        verdict = "SUCCESS"
    elif info["collided"]:
        verdict = "COLLISION"
    else:
        verdict = "timeout" if truncated else "ended"
    return (
        f"{verdict}, route left {info['route_remaining']:.2f} m, "
        f"cross-track {info['cross_track']:+.2f} m"
    )


def _step(env: RoverEnv, predict, obs: np.ndarray) -> tuple:
    action = predict(obs) if predict else env.action_space.sample()
    return env.step(action)


def _watch(env: RoverEnv, predict, args: argparse.Namespace) -> int:
    """Drive in the interactive viewer, one viewer per episode.

    The viewer binds to the MjModel and MjData it is handed. A scenario with
    random obstacles recompiles both on every reset, so a viewer opened once
    would render the FIRST episode's world forever while the env stepped a
    different one, and segfault when the old structs were collected out from
    under its render thread. Opening it per episode keeps it bound to the
    model actually being stepped.
    """
    import mujoco.viewer

    for episode in range(args.episodes):
        obs, _info = env.reset(seed=args.seed + episode)
        # Held for the whole episode so the render thread cannot outlive them.
        model, data = env.model, env.data
        steps = 0
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.user_scn.ngeom = 0
            draw_route(viewer.user_scn, env.route, env.goal)
            while viewer.is_running():
                started = time.perf_counter()
                obs, _reward, terminated, truncated, info = _step(env, predict, obs)
                steps += 1
                viewer.sync()
                if terminated or truncated:
                    print(
                        f"  episode {episode}: {steps} steps, "
                        f"{_outcome(info, truncated)}"
                    )
                    break
                # Pace to wall clock so the motion is readable.
                lag = env.dt / args.speed - (time.perf_counter() - started)
                if lag > 0:
                    time.sleep(lag)
            else:
                return 0  # the window was closed
    return 0


def _record(env: RoverEnv, predict, args: argparse.Namespace) -> int:
    """Write PNG frames offscreen, for when there is no display."""
    args.frames.mkdir(parents=True, exist_ok=True)
    obs, _info = env.reset(seed=args.seed)
    written = 0
    with mujoco.Renderer(env.model, 720, 1280) as renderer:
        for episode in range(args.episodes):
            obs, info = env.reset(seed=args.seed + episode)
            step = 0
            while True:
                obs, _reward, terminated, truncated, info = _step(env, predict, obs)
                step += 1
                if step % args.stride == 0:
                    renderer.update_scene(env.data, camera=args.camera)
                    # The renderer's scene takes the same user geoms the
                    # viewer's does, so the route shows in recordings too.
                    draw_route(renderer.scene, env.route, env.goal)
                    _save(renderer.render(), args.frames / f"f{written:05d}.png")
                    written += 1
                if terminated or truncated:
                    print(f"  episode {episode}: {_outcome(info, truncated)}")
                    break
    print(f"wrote {written} frames to {args.frames}")
    return 0


def _save(pixels: np.ndarray, path: Path) -> None:
    """Write an RGB array as a PNG, without pulling in a video stack."""
    from PIL import Image

    Image.fromarray(pixels).save(path)


if __name__ == "__main__":
    raise SystemExit(main())
