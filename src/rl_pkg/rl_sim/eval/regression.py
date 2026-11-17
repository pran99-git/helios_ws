"""Check a policy against the results that are already banked.

Four reward changes once cost 68 points of lab-map success before anyone
noticed, because every run was judged on the scenario it trained on. This
runs the checks that matter and says plainly whether a policy is worse than
what `policies/` already holds.

A scenario fails only when its floor sits above the WHOLE 95% interval, not
merely above the measured rate. At the episode counts that run in minutes
the point estimate wobbles by several points, and a gate that false-alarms
on that is a gate nobody reads.
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
#:   lab              tracker_v1 scores 93.6% over 500 episodes, 95% interval
#:                    91.1 to 95.4; the achievable ceiling is 94%
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

#: The policy every floor above was measured from, and the one a new run has
#: to beat before it may be deployed.
CHAMPION: str = "tracker_v1.zip"

LAB_MAP = (
    Path(__file__).resolve().parents[3]
    / "mapping_localization_pkg/slam_toolbox/maps"
    / "slam_toolbox_lab_indoor_run_with_VIO_feed.yaml"
)


def score(policy, scenario: scenarios.Scenario, episodes: int) -> tuple[int, float]:
    """Run a policy and report its wins and mean cross-track."""
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
    return wins, float(np.mean(offsets))


def wilson(wins: int, trials: int, z: float = 1.96) -> tuple[float, float]:
    """A 95% interval for a success rate, by Wilson's score method.

    Used instead of comparing the point estimate to the floor, because the
    point estimate is noisy enough to cry wolf on its own. A policy truly at
    94% fails a 90% floor about one run in ten at forty episodes, purely by
    sampling, and a gate that false-alarms that often stops being read.

    Wilson rather than the normal approximation: near a rate of 1.0 the
    normal interval runs past 100% and goes to zero width, which is exactly
    the region every one of these floors sits in.

    Args:
        wins: Successes observed.
        trials: Episodes run.
        z: Standard deviations for the interval, 1.96 being 95%.

    Returns:
        The lower and upper bounds, each in 0..1.
    """
    if trials <= 0:
        return (0.0, 1.0)
    rate = wins / trials
    denominator = 1.0 + z**2 / trials
    centre = (rate + z**2 / (2 * trials)) / denominator
    spread = (
        z * np.sqrt(rate * (1.0 - rate) / trials + z**2 / (4 * trials**2)) / denominator
    )
    return (max(0.0, centre - spread), min(1.0, centre + spread))


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
    print(
        f"  {'scenario':16s} {'success':>9s} {'95% interval':>16s} "
        f"{'floor':>7s} {'xtrack':>8s}   verdict"
    )
    failed = []
    for name, scenario in cases.items():
        wins, offset = score(model, scenario, args.episodes)
        rate = wins / args.episodes
        low, high = wilson(wins, args.episodes)
        # Fails only on a shortfall the episode count can actually support:
        # the floor has to sit above the whole interval, not just above the
        # point estimate.
        ok = high >= BASELINE[name]
        failed += [] if ok else [name]
        print(
            f"  {name:16s} {100 * rate:8.0f}% "
            f"{100 * low:7.0f}% to {100 * high:3.0f}% "
            f"{100 * BASELINE[name]:6.0f}% "
            f"{offset:8.3f}   {'ok' if ok else 'REGRESSED'}"
        )
    reversals = weave(model)
    ok = reversals <= WEAVE_LIMIT
    failed += [] if ok else ["weave"]
    # No interval on this one: it is a mean over thousands of control steps,
    # and every policy measured so far misses the limit by more than twice.
    print(
        f"  {'weave (open)':16s} {reversals:7.2f}/s {'':>16s} "
        f"{WEAVE_LIMIT:6.1f}/s {'':>8s}   {'ok' if ok else 'WEAVING'}"
    )

    if failed:
        print(f"\n  below the banked result on: {', '.join(failed)}")
        # The champion cannot be told to defer to itself, and it misses the
        # weave limit too, so running it through here must not print that.
        if args.policy.name != CHAMPION:
            print(f"  policies/{CHAMPION} is still the better policy.")
        return 1
    print("\n  clears every banked result.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
