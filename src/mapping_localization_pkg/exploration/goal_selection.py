"""Choose which frontier to drive to next.

Ranks candidates on four terms, each normalised to roughly 0..1 so the weights
are directly comparable:

    + information   a wider opening reveals more map
    - travel        a nearer frontier is cheaper to reach
    - turning       a frontier behind the rover costs a turn before any
                    progress, measured at up to 4.6 s on this base
    - rejection     somewhere already tried and failed, decaying with time

A fifth term is reserved for a semantic score, so a phrase-driven run can
prefer frontiers that look like where it was told to go without this module
knowing anything about how that score was produced.

Imports no ROS and reads no transforms. The caller supplies the robot pose.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from typing import Final, NamedTuple

#: Opening width at which the information term saturates, metres. Beyond this
#: a frontier is "wide open" and extra width stops being a reason to prefer it.
DEFAULT_EDGE_CAP: Final[float] = 2.0

#: Range at which the travel term saturates, metres. Past this everything is
#: simply "far", and the other terms decide.
DEFAULT_RANGE_CAP: Final[float] = 12.0

#: Weight of each term. Information leads; travel is strong enough that a near
#: frontier beats a slightly wider distant one; turning is a tiebreaker.
DEFAULT_WEIGHTS: Final[dict[str, float]] = {
    "information": 1.00,
    "travel": 0.60,
    "turning": 0.25,
    "rejection": 1.50,
    "semantic": 0.00,
}

#: Time for a rejection's weight to halve, seconds. Long enough that the rover
#: does not immediately retry what just failed, short enough that a frontier
#: blocked by a passing obstacle becomes available again within a run.
DEFAULT_REJECTION_HALF_LIFE: Final[float] = 90.0

#: Radius over which a rejection taints its neighbourhood, metres. Retrying a
#: point 10 cm from a failure is retrying the same failure.
DEFAULT_REJECTION_RADIUS: Final[float] = 1.00

#: Below this weight a rejection is forgotten entirely.
_FORGET: Final[float] = 0.01


class Candidate(NamedTuple):
    """A frontier offered for selection, in the map frame."""

    x: float
    y: float
    yaw: float
    edge: float
    semantic: float = 0.0


class Scored(NamedTuple):
    """A candidate with its score and the geometry behind it."""

    candidate: Candidate
    score: float
    distance: float
    bearing: float


class RejectionMemory:
    """Remembers where selection failed, with influence that decays.

    Never a hard blacklist: a frontier can fail because a person was standing
    in the doorway, and that stops being true. Each rejection's weight halves
    every `half_life` seconds and falls off with distance, so a repeatedly
    failing spot stays suppressed while a one-off clears on its own.
    """

    def __init__(
        self,
        half_life: float = DEFAULT_REJECTION_HALF_LIFE,
        radius: float = DEFAULT_REJECTION_RADIUS,
    ) -> None:
        if half_life <= 0.0 or radius <= 0.0:
            raise ValueError("half_life and radius must be positive")
        self._half_life = half_life
        self._radius = radius
        self._entries: list[tuple[float, float, float]] = []

    def reject(self, x: float, y: float, now: float) -> None:
        """Record a failure at a point, replacing any older one nearby.

        Replacing rather than accumulating keeps one stubborn frontier from
        building an unbounded penalty that no decay can clear.
        """
        self._entries = [
            entry
            for entry in self._entries
            if math.hypot(entry[0] - x, entry[1] - y) > self._radius
        ]
        self._entries.append((x, y, now))

    def penalty(self, x: float, y: float, now: float) -> float:
        """Penalty at a point, in 0..1.

        Temporal decay is `0.5 ** (elapsed / half_life)`. Spatial falloff is
        Gaussian, `exp(-(d / radius)^2)`, so the penalty is near-full at the
        failure point and negligible beyond about twice the radius. The worst
        single rejection wins rather than the sum, keeping the term bounded.
        """
        worst = 0.0
        for ex, ey, at in self._entries:
            elapsed = max(0.0, now - at)
            age = 0.5 ** (elapsed / self._half_life)
            distance = math.hypot(ex - x, ey - y)
            worst = max(worst, age * math.exp(-((distance / self._radius) ** 2)))
        return worst

    def prune(self, now: float) -> None:
        """Forget rejections whose weight has decayed past noticing."""
        self._entries = [
            entry
            for entry in self._entries
            if 0.5 ** (max(0.0, now - entry[2]) / self._half_life) >= _FORGET
        ]

    def __len__(self) -> int:
        return len(self._entries)


class AttemptMonitor:
    """Times out an attempt on one frontier that is not closing on it.

    Deliberately separate from the walker's own stall detection, and much
    slower: the walker watches a single route over seconds, while this watches
    a whole attempt on one frontier, which legitimately includes replanning,
    turning in place and a detour around an obstacle. It is the backstop for
    the case where the walker has given up and nothing else would notice.
    """

    def __init__(self, min_progress: float, window: float) -> None:
        if min_progress <= 0.0 or window <= 0.0:
            raise ValueError("min_progress and window must be positive")
        self._min = min_progress
        self._window = window
        self._best: float | None = None
        self._since: float | None = None

    def update(self, distance: float, now: float) -> None:
        """Record the distance still to run to the committed frontier."""
        if self._best is None or distance < self._best - self._min:
            self._best = distance
            self._since = now

    def expired(self, now: float) -> bool:
        """True once `window` seconds have passed with no real closing."""
        if self._since is None:
            return False
        return now - self._since >= self._window

    def reset(self) -> None:
        """Clear state, on committing to a new frontier."""
        self._best = None
        self._since = None


def wrap_angle(angle: float) -> float:
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def score_candidates(
    candidates: Iterable[Candidate],
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    *,
    rejections: RejectionMemory | None = None,
    now: float = 0.0,
    weights: dict[str, float] | None = None,
    edge_cap: float = DEFAULT_EDGE_CAP,
    range_cap: float = DEFAULT_RANGE_CAP,
) -> list[Scored]:
    """Score and rank frontier candidates, best first.

    Args:
        candidates: Frontiers to rank, in the map frame.
        robot_x: Robot position in the map frame, metres.
        robot_y: Robot position in the map frame, metres.
        robot_yaw: Robot heading in the map frame, radians.
        rejections: Failure memory, or None to ignore history.
        now: Current time, seconds, on the same clock as `rejections`.
        weights: Per-term weights, defaulting to `DEFAULT_WEIGHTS`. Missing
            keys fall back to the default for that term.
        edge_cap: Opening width at which information saturates, metres.
        range_cap: Range at which travel cost saturates, metres.

    Returns:
        One `Scored` per candidate, sorted by score descending.

    Raises:
        ValueError: If `edge_cap` or `range_cap` is not positive.
    """
    if edge_cap <= 0.0 or range_cap <= 0.0:
        raise ValueError("edge_cap and range_cap must be positive")
    w = {**DEFAULT_WEIGHTS, **(weights or {})}

    scored: list[Scored] = []
    for candidate in candidates:
        dx, dy = candidate.x - robot_x, candidate.y - robot_y
        distance = math.hypot(dx, dy)
        bearing = wrap_angle(math.atan2(dy, dx) - robot_yaw) if distance > 1e-9 else 0.0

        information = min(candidate.edge, edge_cap) / edge_cap
        travel = min(distance, range_cap) / range_cap
        turning = abs(bearing) / math.pi
        penalty = (
            rejections.penalty(candidate.x, candidate.y, now) if rejections else 0.0
        )

        score = (
            w["information"] * information
            + w["semantic"] * candidate.semantic
            - w["travel"] * travel
            - w["turning"] * turning
            - w["rejection"] * penalty
        )
        scored.append(Scored(candidate, score, distance, bearing))

    scored.sort(key=lambda s: s.score, reverse=True)
    return scored


def nearest(
    candidates: Sequence[Candidate], x: float, y: float
) -> tuple[int, float] | None:
    """Index of the candidate closest to a point, with its distance.

    Used to decide whether a committed target still exists in a freshly
    detected list, since detection re-derives frontiers from scratch and the
    representative cell of the same opening moves between updates.

    Returns:
        The index and distance, or None when there are no candidates.
    """
    if not candidates:
        return None
    distances = [math.hypot(c.x - x, c.y - y) for c in candidates]
    index = min(range(len(distances)), key=distances.__getitem__)
    return index, distances[index]
