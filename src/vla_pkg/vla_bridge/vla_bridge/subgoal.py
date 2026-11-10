"""Walk a global route, handing the policy a carrot it can actually reach.

The policy predicts about one metre per chunk and holds no map, so a distant
goal makes it drive at the bearing until an obstacle stops it. Giving it a
point a short way along a planned route turns every request into the short-goal
case it handles well, and leaves route choice to the planner.

No ROS imports: plain geometry over a polyline, testable without a ROS context.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from itertools import pairwise
from typing import Final, NamedTuple

from vla_bridge.goal_resolution import Pose2D, wrap_angle

#: Carrot distance along the route, metres. Deliberately beyond the policy's
#: ~1 m reach so it is always heading somewhere rather than arriving.
DEFAULT_LOOKAHEAD: Final[float] = 1.5

#: Segments shorter than this are treated as duplicate points.
_EPS: Final[float] = 1e-9


class Projection(NamedTuple):
    """Where the robot sits relative to the route."""

    #: Index of the segment the robot projects onto. Named `segment` rather
    #: than `index` because NamedTuple would shadow tuple.index().
    segment: int
    #: Arc length from the route start to the projected point, metres.
    arc: float
    #: Perpendicular distance from the robot to the route, metres.
    offset: float


def route_length(route: Sequence[Pose2D]) -> float:
    """Total arc length of a polyline, metres."""
    return sum(math.dist((a.x, a.y), (b.x, b.y)) for a, b in pairwise(route))


def project(route: Sequence[Pose2D], x: float, y: float) -> Projection:
    """Find the closest point on the route to `(x, y)`.

    Projects onto each segment and keeps the nearest, so a robot beside the
    route is placed at its true closest point rather than the nearest vertex.

    Raises:
        ValueError: If the route has fewer than two points.
    """
    if len(route) < 2:
        raise ValueError("route needs at least two points")

    best = Projection(0, 0.0, math.inf)
    arc = 0.0
    for i, (a, b) in enumerate(pairwise(route)):
        dx, dy = b.x - a.x, b.y - a.y
        seg = math.hypot(dx, dy)
        if seg < _EPS:
            continue
        # Scalar projection of the robot onto this segment, clamped to it.
        t = ((x - a.x) * dx + (y - a.y) * dy) / (seg * seg)
        t = max(0.0, min(1.0, t))
        px, py = a.x + t * dx, a.y + t * dy
        offset = math.dist((x, y), (px, py))
        if offset < best.offset:
            best = Projection(i, arc + t * seg, offset)
        arc += seg
    return best


def point_at_arc(route: Sequence[Pose2D], arc: float) -> Pose2D:
    """The route point at a given arc length, oriented along the route tangent.

    Past the end of the route this returns the final point, so a carrot that
    runs off the end degrades to the real goal rather than failing.
    """
    if len(route) < 2:
        raise ValueError("route needs at least two points")
    if arc <= 0.0:
        heading = math.atan2(route[1].y - route[0].y, route[1].x - route[0].x)
        return Pose2D(route[0].x, route[0].y, heading)

    travelled = 0.0
    for a, b in pairwise(route):
        seg = math.dist((a.x, a.y), (b.x, b.y))
        if seg < _EPS:
            continue
        if travelled + seg >= arc:
            f = (arc - travelled) / seg
            return Pose2D(
                a.x + f * (b.x - a.x),
                a.y + f * (b.y - a.y),
                math.atan2(b.y - a.y, b.x - a.x),
            )
        travelled += seg

    last, prev = route[-1], route[-2]
    return Pose2D(last.x, last.y, math.atan2(last.y - prev.y, last.x - prev.x))


def carrot(
    route: Sequence[Pose2D],
    x: float,
    y: float,
    lookahead: float = DEFAULT_LOOKAHEAD,
) -> Pose2D:
    """The subgoal to hand the policy, given where the robot is now.

    Recomputed from the live pose every cycle rather than advanced through
    discrete waypoints, so there is no "have I arrived" state to get wrong: the
    carrot simply slides forward as the robot moves.
    """
    return point_at_arc(route, project(route, x, y).arc + lookahead)


def remaining(route: Sequence[Pose2D], x: float, y: float) -> float:
    """Arc length from the robot's projection to the end of the route, metres."""
    return max(0.0, route_length(route) - project(route, x, y).arc)


def heading_error(carrot_pose: Pose2D, robot_yaw: float) -> float:
    """Signed angle the robot must turn to face the carrot, radians."""
    return wrap_angle(carrot_pose.yaw - robot_yaw)


class ProgressMonitor:
    """Detects a rover that has stopped advancing ALONG the route.

    Distance travelled is the wrong test: a rover circling or hunting covers
    ground without getting closer to the goal. Arc length along the route only
    grows when real progress is made.
    """

    def __init__(self, min_progress: float, window: float) -> None:
        if min_progress <= 0.0 or window <= 0.0:
            raise ValueError("min_progress and window must be positive")
        self._min = min_progress
        self._window = window
        self._best_arc: float | None = None
        self._since: float | None = None

    def update(self, arc: float, now: float) -> None:
        """Record progress. Call once per cycle with the projected arc length."""
        if self._best_arc is None or arc > self._best_arc + self._min:
            self._best_arc = arc
            self._since = now

    def stuck(self, now: float) -> bool:
        """True once `window` seconds have passed with no real advance."""
        if self._since is None:
            return False
        return now - self._since >= self._window

    def reset(self) -> None:
        """Clear state, on a new route or a new goal."""
        self._best_arc = None
        self._since = None
