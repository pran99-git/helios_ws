"""Latching emergency-stop state.

Latching is the point. A momentary stop that releases when you let go of the
button is not an emergency stop: it hands the rover back the instant your
attention moves. Once engaged this stays engaged until someone deliberately
clears it, and clearing is made harder than engaging on purpose.

No ROS here. The node feeds it button states and a clock.
"""

from __future__ import annotations

#: Seconds the clear input must be held continuously. Long enough that a
#: bounce, a brush, or a double-press cannot silently re-arm the rover.
DEFAULT_CLEAR_HOLD = 1.0


class EstopState:
    """Tracks whether the emergency stop is engaged."""

    def __init__(self, clear_hold: float = DEFAULT_CLEAR_HOLD) -> None:
        """
        Args:
            clear_hold: Seconds the clear input must be held to release.

        Raises:
            ValueError: If `clear_hold` is not positive. An instant clear
                would make the latch meaningless.
        """
        if clear_hold <= 0.0:
            raise ValueError(
                "clear_hold must be positive; a latch that clears instantly is not one"
            )
        self._clear_hold = clear_hold
        self._engaged = False
        self._clearing_since: float | None = None

    @property
    def engaged(self) -> bool:
        """Whether the stop is currently engaged."""
        return self._engaged

    def update(self, engage: bool, clear: bool, now: float) -> bool:
        """Advance the state machine by one input sample.

        Args:
            engage: Engage input is active. Any single sample latches.
            clear: Clear input is active. Must stay active for `clear_hold`.
            now: Monotonic seconds, used only to time the clear hold.

        Returns:
            Whether the stop is engaged after this sample.
        """
        if engage:
            # Engaging always wins, and abandons any clear in progress.
            self._engaged = True
            self._clearing_since = None
            return True

        if not self._engaged or not clear:
            self._clearing_since = None
            return self._engaged

        if self._clearing_since is None:
            self._clearing_since = now
        elif now - self._clearing_since >= self._clear_hold:
            self._engaged = False
            self._clearing_since = None
        return self._engaged

    def clear_progress(self, now: float) -> float:
        """Fraction of the clear hold completed, 0.0 when not clearing.

        Only for operator feedback; the state machine does not use it.
        """
        if self._clearing_since is None:
            return 0.0
        return min(1.0, (now - self._clearing_since) / self._clear_hold)
