# Copyright 2026
#
# Joystick teleop for the Helios A4WD3 mecanum rover, targeting the 8BitDo
# SN30 Pro over Bluetooth.
#
# Axis/button indices are read from CONFIG, not hardcoded, because they
# depend on which mode the SN30 Pro is paired in (D-input/X-input/Switch)
# and which joystick backend `joy_node` picks up on this machine. Confirm
# yours before trusting the defaults below:
#   ros2 run joy joy_node
#   ros2 topic echo /joy
# then note which index in `axes`/`buttons` changes as you move each stick
# and press each button, and set config/teleop.yaml accordingly.
#
# Left stick  -> vx (forward/back), vy (strafe)
# Right stick -> omega (rotate)
#
# Stick deflection is scaled into TRUE SI units before publishing: /cmd_vel
# carries m/s and rad/s, not a normalized [-1, 1] range. Full deflection maps
# to max_vx/max_vy/max_omega from the config.
#
# A deadman button must be HELD for any nonzero output. This is what makes
# it safe to verify the axis mapping via `ros2 topic echo /joy` first: an
# unverified/wrong mapping just shows up as numbers on that topic, not
# motion, until the deadman button is actually pressed.
#
# Also watches for a stale/missing /joy stream (controller powered off or
# Bluetooth dropped) and forces zero output in that case -- this is
# independent of, and in addition to, roboclaw_driver_node's own /cmd_vel
# watchdog, since that one only protects against *this* node itself dying.

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy


class TeleopJoyNode(Node):
    def __init__(self):
        super().__init__("teleop_joy")

        # --- Axis/button mapping (UNVERIFIED defaults -- see module docstring) --
        self.declare_parameter("axis_vx", 1)  # left stick Y
        self.declare_parameter("axis_vy", 0)  # left stick X
        self.declare_parameter("axis_omega", 3)  # right stick X
        self.declare_parameter("invert_vx", False)
        self.declare_parameter("invert_vy", False)
        self.declare_parameter("invert_omega", False)
        self.declare_parameter("deadman_button", 4)  # e.g. left shoulder button

        # --- Shaping ------------------------------------------------------
        # Velocity at full stick deflection. /cmd_vel carries true SI units, so
        # these convert a unitless axis into something the driver can act on;
        # the driver clamps to its own ceilings independently. Keep them at or
        # below the driver's max_vx/max_vy/max_omega or the stick goes dead
        # over part of its travel.
        self.declare_parameter("max_vx", 0.25)  # m/s
        self.declare_parameter("max_vy", 0.25)  # m/s
        self.declare_parameter("max_omega", 0.5)  # rad/s

        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("cmd_topic", "cmd_vel")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("joy_timeout", 0.5)  # s; stale/missing joy -> zero

        g = self.get_parameter
        self.axis_vx = g("axis_vx").value
        self.axis_vy = g("axis_vy").value
        self.axis_omega = g("axis_omega").value
        self.invert_vx = g("invert_vx").value
        self.invert_vy = g("invert_vy").value
        self.invert_omega = g("invert_omega").value
        self.deadman_button = g("deadman_button").value
        self.deadzone = g("deadzone").value
        self.joy_timeout = g("joy_timeout").value
        # Magnitudes, matching the driver's handling of the same parameter
        # names: direction belongs to the invert_* flags alone, so a negative
        # limit here must not silently flip an axis.
        self.max_vx = abs(g("max_vx").value)
        self.max_vy = abs(g("max_vy").value)
        self.max_omega = abs(g("max_omega").value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, g("cmd_topic").value, qos)
        self.create_subscription(Joy, "joy", self._on_joy, qos)

        self._latest = Twist()
        self._last_joy_time = None

        rate = g("publish_rate").value
        self.timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"teleop_joy started: axes(vx={self.axis_vx}, vy={self.axis_vy}, "
            f"omega={self.axis_omega}), deadman_button={self.deadman_button}, "
            f"deadzone={self.deadzone}. Full stick = {self.max_vx} m/s, "
            f"{self.max_vy} m/s strafe, {self.max_omega} rad/s. Axis/button "
            f"indices are UNVERIFIED defaults -- confirm with "
            f"`ros2 topic echo /joy` before trusting them."
        )

    def _apply_deadzone(self, value):
        return 0.0 if abs(value) < self.deadzone else value

    def _axis(self, msg, index, invert):
        if not (0 <= index < len(msg.axes)):
            return 0.0
        v = self._apply_deadzone(msg.axes[index])
        return -v if invert else v

    def _on_joy(self, msg):
        self._last_joy_time = self.get_clock().now()

        twist = Twist()
        held = (
            0 <= self.deadman_button < len(msg.buttons)
            and msg.buttons[self.deadman_button] == 1
        )

        if held:
            # Axis deflection is unitless; scale it into m/s and rad/s.
            twist.linear.x = self.max_vx * self._axis(msg, self.axis_vx, self.invert_vx)
            twist.linear.y = self.max_vy * self._axis(msg, self.axis_vy, self.invert_vy)
            twist.angular.z = self.max_omega * self._axis(
                msg, self.axis_omega, self.invert_omega
            )

        self._latest = twist

    def _publish(self):
        # Republish at a fixed rate rather than only on /joy callback, so
        # roboclaw_driver_node's own watchdog sees a steady stream and doesn't
        # trip between individual joystick updates.
        stale = (
            self._last_joy_time is None
            or (self.get_clock().now() - self._last_joy_time).nanoseconds * 1e-9
            > self.joy_timeout
        )
        self.cmd_pub.publish(Twist() if stale else self._latest)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoyNode()
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
