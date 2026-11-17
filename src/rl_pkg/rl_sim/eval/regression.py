"""Check a policy against the results that are already banked.

Four reward changes once cost 68 points of lab-map success before anyone
noticed, because every run was judged on the scenario it trained on. This
runs the checks that matter and says plainly whether a policy is worse than
what `policies/` already holds.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from envs import scenarios
from envs.rover_env import RoverEnv

#: Floors a new policy has to clear, pinned to FROZEN scenario definitions
#: below. The SUITE entries are tuning knobs and drift: `cluttered` grew
#: staleness 0.5/0.3 after tracker_v1 was measured at 100% there, which
#: silently changed what the name meant. Each floor's provenance:
#:
#:   lab              tracker_v1 scored 94%; the achievable ceiling is 94%
#:   cluttered-clean  tracker_v1 scored 100% with staleness off
#:   cluttered-stale  tracker_v1 scores ~63%; a blind carrot-follower 70%.
#:                    The floor only says "no worse than the tracker" - the
#:                    POINT of a new run is to clear the blind 70% too.
#:   corridor         tracker_v1 scored 96%
BASELINE: dict[str, float] = {
    "lab": 0.90,
    "cluttered-clean": 0.85,
    "cluttered-stale": 0.60,
    "corridor": 0.85,
}

LAB_MAP = (
    Path(__file__).resolve().parents[3]
    / "mapping_localization_pkg/slam_toolbox/maps"
    / "slam_toolbox_lab_indoor_run_with_VIO_feed.yaml"
)


def score(policy, scenario: scenarios.Scenario, episodes: int) -> tuple[float, float]:
    """Run a policy and report its success rate and mean cross-track."""
    env = RoverEnv(scenario=scenario, seed=0)
    wins, offsets = 0, []
    for episode in range(episodes):
        obs, _info = env.reset(seed=9000 + episode)
        while True:
            action = policy.predict(obs, deterministic=True)[0]
            obs, _reward, terminated, truncated, info = env.step(action)
            offsets.append(abs(info["cross_track"]))
            if terminated or truncated:
                break
        wins += info["is_success"]
    return wins / episodes, float(np.mean(offsets))


#: Largest acceptable wz sign-reversal rate on an empty room, 1/s. The
#: dither that per-step pose jitter taught reached 3.8; a clean controller
#: under held jitter sits near 1.0. Above this the rover visibly weaves.
WEAVE_LIMIT: float = 1.5


def weave(policy, episodes: int = 12) -> float:
    """Yaw-reversal rate on open_room, start transient excluded."""
    env = RoverEnv(
        scenario=next(s for s in scenarios.SUITE if s.name == "open_room"), seed=0
    )
    rates = []
    for episode in range(episodes):
        obs, _info = env.reset(seed=9000 + episode)
        step = 0
        while True:
            action = policy.predict(obs, deterministic=True)[0]
            obs, _reward, terminated, truncated, _i = env.step(action)
            step += 1
            if step > 60:
                rates.append(env._true_twist()[2])
            if terminated or truncated:
                break
    wz = np.array(rates)
    sign = np.sign(wz[np.abs(wz) > 0.02])
    return float(np.sum(np.diff(sign) != 0) / (len(wz) * env.dt))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("policy", type=Path)
    ap.add_argument("--episodes", type=int, default=50)
    args = ap.parse_args()

    from stable_baselines3 import PPO

    model = PPO.load(args.policy, device="cpu")
    suite = {s.name: s for s in scenarios.SUITE}
    base = suite["cluttered"]
    # Frozen here, deliberately NOT read from the SUITE: these definitions
    # are what the floors were measured against and must never drift.
    cases = {
        "lab": scenarios.map_scenario(LAB_MAP),
        "cluttered-clean": scenarios.Scenario(
            name="cluttered-clean",
            bounds=base.bounds,
            n_random=8,
            n_low=3,
            time_limit=60.0,
        ),
        "cluttered-stale": scenarios.Scenario(
            name="cluttered-stale",
            bounds=base.bounds,
            n_random=8,
            n_low=3,
            time_limit=60.0,
            stale_moved=0.5,
            stale_unmapped=0.3,
        ),
        "corridor": suite["corridor"],
    }

    print(f"{args.policy}   {args.episodes} episodes each\n")
    print(f"  {'scenario':16s} {'success':>9s} {'floor':>7s} {'xtrack':>8s}   verdict")
    failed = []
    for name, scenario in cases.items():
        rate, offset = score(model, scenario, args.episodes)
        ok = rate >= BASELINE[name]
        failed += [] if ok else [name]
        print(
            f"  {name:16s} {100 * rate:8.0f}% {100 * BASELINE[name]:6.0f}% "
            f"{offset:8.3f}   {'ok' if ok else 'REGRESSED'}"
        )
    reversals = weave(model)
    ok = reversals <= WEAVE_LIMIT
    failed += [] if ok else ["weave"]
    print(
        f"  {'weave (open)':16s} {reversals:7.2f}/s {WEAVE_LIMIT:5.1f}/s"
        f" {'':>9s}  {'ok' if ok else 'WEAVING'}"
    )

    if failed:
        print(f"\n  below the banked result on: {', '.join(failed)}")
        print("  policies/tracker_v1.zip is still the better policy.")
        return 1
    print("\n  clears every banked result.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
