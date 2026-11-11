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

#: Carrot bearing beyond which the policy cannot be used, radians.
#:
#: The policy only ever proposes forward motion, so a carrot behind the rover
#: produces a chunk pointing the wrong way. RotationShimController cannot
#: rescue that: it sits DOWNSTREAM of the policy and aligns to the chunk, which
#: already reads as straight ahead, so the real heading error is never seen.
#: Above this the walker must turn the rover itself.
DEFAULT_TURN_FIRST: Final[float] = 0.90

#: Forward speed paired with a turn-first rotation, m/s.
#:
#: A pure spin is invisible to slam_toolbox, whose `shouldProcessScan` gates on
#: translation alone, so the yaw error accumulated during one arrives as a lurch
#: when driving resumes. A slow arc keeps scans processing.
DEFAULT_TURN_VX: Final[float] = 0.10

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
    """Signed angle to align with the carrot's own orientation, radians."""
    return wrap_angle(carrot_pose.yaw - robot_yaw)


def bearing_to(robot: Pose2D, target: Pose2D) -> float:
    """Signed angle from the robot's heading to the direction OF the target.

    Distinct from `heading_error`, which compares orientations. This is "which
    way must I turn to point at it", which is what decides whether the policy
    can be used at all.
    """
    return wrap_angle(math.atan2(target.y - robot.y, target.x - robot.x) - robot.yaw)


def turn_in_place(
    bearing: float, max_rate: float, max_accel: float, current_rate: float, dt: float
) -> float:
    """Yaw rate to turn toward `bearing`, decelerating so it does not overshoot.

    Ramps down inside the stopping distance `w^2 / 2a`, because a constant-rate
    turn overshoots by exactly that much and then has to come back.
    """
    if max_rate <= 0.0 or max_accel <= 0.0 or dt <= 0.0:
        raise ValueError("max_rate, max_accel and dt must be positive")
    # Fastest rate from which the remaining angle can still be stopped in.
    feasible = math.sqrt(2.0 * max_accel * abs(bearing))
    target = math.copysign(min(max_rate, feasible), bearing)
    # Respect the acceleration limit from whatever we are doing now.
    step = max_accel * dt
    return max(current_rate - step, min(current_rate + step, target))


def turn_arc_speed(bearing: float, max_speed: float) -> float:
    """Forward speed to pair with a turn, so the mapper still sees translation.

    Scaled by `max(0, cos(bearing))`: full speed once aligned, tapering to zero
    at 90 degrees and staying there beyond it. The rover therefore translates
    only once it substantially faces where it is going, never into the space
    behind it, and hands over to the policy already rolling rather than stopped.

    Args:
        bearing: Signed angle to the target, radians.
        max_speed: Speed commanded when perfectly aligned, m/s.

    Returns:
        Forward speed in m/s, never negative.

    Raises:
        ValueError: If `max_speed` is negative.
    """
    if max_speed < 0.0:
        raise ValueError("max_speed must not be negative")
    return max_speed * max(0.0, math.cos(bearing))


class ProgressMonitor:
    """Detects a rover that has stopped advancing TOWARD the goal.

    Distance travelled is the wrong test: a rover circling or hunting covers
    ground without getting closer. Arc length travelled is also wrong, because
    a route replanned from the rover's live pose restarts its arc at zero, so a
    rover making perfect progress would look stalled. Distance still to run is
    anchored at the goal, so it survives replanning and only falls on real
    progress.
    """

    def __init__(self, min_progress: float, window: float) -> None:
        if min_progress <= 0.0 or window <= 0.0:
            raise ValueError("min_progress and window must be positive")
        self._min = min_progress
        self._window = window
        self._best: float | None = None
        self._since: float | None = None

    def update(self, remaining: float, now: float) -> None:
        """Record progress. Call once per cycle with the distance still to run."""
        if self._best is None or remaining < self._best - self._min:
            self._best = remaining
            self._since = now

    def stuck(self, now: float) -> bool:
        """True once `window` seconds have passed with no real advance."""
        if self._since is None:
            return False
        return now - self._since >= self._window

    def reset(self) -> None:
        """Clear state, on a new route or a new goal."""
        self._best = None
        self._since = None
