"""ROS adapter: a latching emergency stop on top of twist_mux.

Engaging does two things at once, because neither alone is sufficient:

    /cmd_vel_estop   zeroed Twist at priority 250, ABOVE the lock, so it is
                     not blocked by it. This is what actually stops the rover,
                     and it stops it now: a commanded zero starts the driver's
                     ramp immediately (~0.09 m from 0.40 m/s), where waiting
                     for the driver's own cmd_timeout would cost ~0.28 m.

    /cmd_vel_lock    latched true at priority 200, which blocks teleop (100)
                     and Nav2 (10). It outlives this node: twist_mux holds the
                     lock with timeout 0, so a crash here leaves the rover
                     blocked rather than handing control straight back.

Escape hatch, and a hazard worth knowing: the latch lives in memory, so
RESTARTING THIS NODE CLEARS IT. That is how you recover if the gamepad is
dead, and it also means the rover becomes drivable the moment it comes back.

All state lives in `estop_state`; this file handles parameters, pub/sub and
message conversion only.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from estop.estop_state import EstopState


class EstopNode(Node):
    """Latching e-stop driven by a gamepad button or an external request."""

    def __init__(self) -> None:
        super().__init__("estop")

        self.declare_parameter("joy_topic", "/joy")
        # Verify against `ros2 topic echo /joy`: 8BitDo indices shift between
        # X-input, D-input and Switch modes.
        self.declare_parameter("engage_button", 4)  # L shoulder
        self.declare_parameter("clear_button", 7)  # Start
        self.declare_parameter("clear_hold", 1.0)  # s held to release the latch
        self.declare_parameter("request_topic", "/estop/request")
        self.declare_parameter("estop_cmd_topic", "/cmd_vel_estop")
        self.declare_parameter("lock_topic", "/cmd_vel_lock")
        self.declare_parameter("status_topic", "/estop/engaged")
        self.declare_parameter("publish_rate", 20.0)  # Hz

        def text(name: str) -> str:
            return str(self.get_parameter(name).value)

        self._engage_button = int(self.get_parameter("engage_button").value)
        self._clear_button = int(self.get_parameter("clear_button").value)
        self._state = EstopState(
            clear_hold=float(self.get_parameter("clear_hold").value)
        )
        # Starts False, not None, so coming up disengaged is not logged as a
        # transition. The initial state is still published below.

        self._cmd_pub = self.create_publisher(Twist, text("estop_cmd_topic"), 1)
        self._lock_pub = self.create_publisher(Bool, text("lock_topic"), 1)
        self._status_pub = self.create_publisher(Bool, text("status_topic"), 1)
        self._last_reported = False
        self._status_pub.publish(Bool(data=False))

        self.create_subscription(Joy, text("joy_topic"), self._on_joy, 10)
        self.create_subscription(Bool, text("request_topic"), self._on_request, 1)

        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self._step)

        self.get_logger().info(
            f"estop ready: button {self._engage_button} engages, "
            f"hold button {self._clear_button} for "
            f"{self.get_parameter('clear_hold').value}s to clear"
        )

    @staticmethod
    def _pressed(msg: Joy, index: int) -> bool:
        """Read one button, tolerating a controller with fewer than expected."""
        return 0 <= index < len(msg.buttons) and msg.buttons[index] == 1

    def _on_joy(self, msg: Joy) -> None:
        self._advance(
            engage=self._pressed(msg, self._engage_button),
            clear=self._pressed(msg, self._clear_button),
        )

    def _on_request(self, msg: Bool) -> None:
        """External engage, for an app or a GPIO button later.

        Engage-only by design. Clearing a latched stop should take a
        deliberate physical action, not a stray message on a topic.
        """
        if msg.data:
            self._advance(engage=True, clear=False)

    def _advance(self, engage: bool, clear: bool) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self._report(self._state.update(engage, clear, now))

    def _report(self, engaged: bool) -> None:
        if engaged == self._last_reported:
            return
        self._last_reported = engaged
        self._status_pub.publish(Bool(data=engaged))
        if engaged:
            self.get_logger().warn("E-STOP ENGAGED: teleop and autonomy are blocked")
        else:
            self.get_logger().warn("e-stop cleared: the rover can move again")

    def _step(self) -> None:
        """Republish the lock always, and zeros only while engaged."""
        engaged = self._state.engaged
        self._lock_pub.publish(Bool(data=engaged))
        if engaged:
            self._cmd_pub.publish(Twist())


def main(args: list[str] | None = None) -> None:
    """Spins the e-stop node until interrupted."""
    rclpy.init(args=args)
    node = EstopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
