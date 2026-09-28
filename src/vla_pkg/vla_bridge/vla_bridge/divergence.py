"""Decide when a freshly predicted path is worth preempting the active one.

The policy republishes at several Hz, but preempting a Nav2 action that often
is wasteful: the controller re-plans against its costmap at 20 Hz regardless,
and every preemption throws away its warm optimiser state. A new path is only
sent when it actually disagrees with the one being followed.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

Point = tuple[float, float]

#: How far the comparison waypoint must move before a preemption is justified.
DEFAULT_THRESHOLD_M = 0.15

#: Which waypoint to compare. Mid-chunk: the first few barely move even when
#: the plan changes, and the last is the noisiest.
DEFAULT_INDEX = 4


def should_reissue(
    active: Sequence[Point],
    candidate: Sequence[Point],
    threshold: float = DEFAULT_THRESHOLD_M,
    index: int = DEFAULT_INDEX,
) -> bool:
    """Whether `candidate` differs enough from `active` to replace it.

    Args:
        active: Points of the path currently being followed, may be empty.
        candidate: Points of the newly predicted path, must be non-empty.
        threshold: Metres of movement at the comparison waypoint.
        index: Waypoint to compare, clamped to the shorter path.

    Returns:
        True if nothing is active, or if the comparison waypoint moved further
        than `threshold`.

    Raises:
        ValueError: If `candidate` is empty; retraction is the caller's job.
    """
    if not candidate:
        raise ValueError("candidate path is empty; cancel instead of reissuing")
    if not active:
        return True
    at = min(index, len(active) - 1, len(candidate) - 1)
    return math.dist(active[at], candidate[at]) > threshold
