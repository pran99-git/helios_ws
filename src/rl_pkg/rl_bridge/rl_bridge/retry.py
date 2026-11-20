"""Back off between attempts at an operation that keeps failing.

Lives beside the node rather than inside it because it is a timing decision
with state, which is where this repo keeps such logic, and because the
arithmetic is worth testing without a ROS context.

Imports no ROS and reads no clock: the caller supplies the time, as
`goal_selection.AttemptMonitor` does.
"""

from __future__ import annotations


class Backoff:
    """A doubling wait between attempts, floored and capped.

    Starts ready, so a first attempt is never delayed. Each failure doubles
    the wait from `first` up to `ceiling`; a success clears it, so a caller
    that is working is never gated.
    """

    def __init__(self, first: float, ceiling: float) -> None:
        """Set the wait schedule.

        Args:
            first: Wait after the first failure, seconds. Zero disables the
                backoff entirely, leaving every attempt ungated.
            ceiling: Longest wait between attempts, seconds.

        Raises:
            ValueError: If either is negative.
        """
        if first < 0.0 or ceiling < 0.0:
            raise ValueError(
                f"first and ceiling must be non-negative, got {first} and {ceiling}"
            )
        self._first = first
        self._ceiling = ceiling
        self._wait = 0.0
        self._failed_at = 0.0

    @property
    def wait(self) -> float:
        """Current wait between attempts, seconds. Zero while healthy."""
        return self._wait

    def failed(self, now: float) -> None:
        """Record a failure, doubling the wait.

        The floor and the cap are applied in one expression, so a `first`
        above `ceiling` still cannot produce a wait past the ceiling:
        `min(max(2w, first), ceiling)`.
        """
        self._failed_at = now
        self._wait = min(max(self._wait * 2.0, self._first), self._ceiling)

    def succeeded(self) -> None:
        """Clear the wait, so the next attempt is not gated."""
        self._wait = 0.0

    def remaining(self, now: float) -> float:
        """Seconds still to wait before attempting again, zero when ready."""
        return max(0.0, self._wait - (now - self._failed_at))

    def ready(self, now: float) -> bool:
        """Whether enough time has passed to attempt again."""
        return self.remaining(now) <= 0.0
