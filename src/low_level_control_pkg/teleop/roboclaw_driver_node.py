# Copyright 2026
#
# Sole owner of both RoboClaw serial connections for the Helios A4WD3
# mecanum rover. This is the hardware-I/O boundary of the low-level-control
# subsystem: everything that actually talks to the RoboClaws over serial
# lives here, in exactly one process.
#
# RoboClaw's packet-serial protocol is half-duplex on a single UART -- two
# independent OS processes issuing transactions on the same port would
# interleave/corrupt each other's packets. So this node does BOTH jobs that
# used to be split across two processes:
#   - READ: polls all four quadrature encoders and publishes them as a
#     sensor_msgs/JointState on `encoder_topic` (position = raw encoder
#     counts, NOT radians -- perception_pkg's wheel_odometry_node does the
#     counts->meters conversion and mecanum forward kinematics).
#   - WRITE: subscribes to /cmd_vel (geometry_msgs/Twist, body frame) and
#     issues per-wheel duty-cycle commands.
#
# OPEN-LOOP control: uses RoboClaw's duty-cycle command, not closed-loop
# Speed/QPPS. The RoboClaw's internal velocity PID and QPPS limits are not
# configured/verified on this hardware, so vx/vy/omega here are treated as
# *normalized* commands in [-1, 1], not true m/s or rad/s.
#
# Mecanum inverse kinematics (X-roller config -- the exact inverse of the
# forward kinematics wheel_odometry_node uses, so the sign conventions
# match): L = lx + ly = (wheelbase + track_width) / 2
#   v_fl = vx - vy - L*omega      v_fr = vx + vy + L*omega
#   v_rl = vx + vy - L*omega      v_rr = vx - vy + L*omega
#
# Comms watchdog: if no /cmd_vel arrives within `cmd_timeout` seconds (e.g.
# the Bluetooth controller drops), all four wheels are zeroed.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from teleop.roboclaw_driver import RoboClaw, SerialBus

CORNERS = ('front_left', 'front_right', 'rear_left', 'rear_right')


class RoboclawDriverNode(Node):

    def __init__(self):
        super().__init__('roboclaw_driver')

        # --- Connection parameters ---------------------------------------
        # Stable udev symlinks (keyed to physical USB port), NOT /dev/ttyACM*
        # which renumber by insertion order. See udev/99-roboclaw.rules.
        self.declare_parameter('left_port', '/dev/roboclaw_left')
        self.declare_parameter('right_port', '/dev/roboclaw_right')
        self.declare_parameter('left_address', 0x80)
        self.declare_parameter('right_address', 0x80)
        self.declare_parameter('baud', 115200)
        self.declare_parameter('serial_timeout', 0.1)

        # Per-corner sign inversion for the WRITE path. Independent of
        # wheel_odometry's own invert_* flags (separate concern, separate
        # convention) -- verify with a bench test (wheels up).
        self.declare_parameter('invert_front_left', False)
        self.declare_parameter('invert_front_right', False)
        self.declare_parameter('invert_rear_left', False)
        self.declare_parameter('invert_rear_right', False)

        # --- Geometry (only the L = lx+ly term is needed for the IK) ------
        self.declare_parameter('wheelbase', 0.220)
        self.declare_parameter('track_width', 0.330)

        # --- Command shaping ------------------------------------------------
        self.declare_parameter('max_linear_duty', 0.5)   # duty fraction at |vx| or |vy| = 1
        self.declare_parameter('max_angular_duty', 0.5)  # duty fraction at |omega| = 1
        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('cmd_timeout', 0.5)   # s; watchdog e-stop
        self.declare_parameter('control_rate', 20.0)  # Hz, write-path rate

        # --- Encoder publishing ---------------------------------------------
        self.declare_parameter('encoder_topic', 'roboclaw/wheel_encoders')
        self.declare_parameter('encoder_publish_rate', 30.0)  # Hz, read-path rate

        g = self.get_parameter
        self.wheelbase = g('wheelbase').value
        self.track_width = g('track_width').value
        self.l_sum = 0.5 * (self.wheelbase + self.track_width)
        self.max_linear_duty = g('max_linear_duty').value
        self.max_angular_duty = g('max_angular_duty').value
        self.cmd_timeout = g('cmd_timeout').value
        self.inversions = {c: g(f'invert_{c}').value for c in CORNERS}

        # --- Open serial bus(es) and controllers -----------------------------
        baud = g('baud').value
        timeout = g('serial_timeout').value
        left_port = g('left_port').value
        right_port = g('right_port').value

        self._buses = {}
        try:
            left_bus = self._get_bus(left_port, baud, timeout)
            right_bus = self._get_bus(right_port, baud, timeout)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().fatal(f'Failed to open RoboClaw serial port: {exc}')
            raise

        self.left = RoboClaw(left_bus, g('left_address').value)
        self.right = RoboClaw(right_bus, g('right_address').value)

        # --- Write-path (cmd_vel -> duty) state ------------------------------
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.last_cmd_time = self.get_clock().now()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Twist, g('cmd_topic').value, self._on_cmd_vel, qos)

        control_rate = g('control_rate').value
        self.control_timer = self.create_timer(1.0 / control_rate, self._control_step)

        # --- Read-path (encoders -> JointState) ------------------------------
        self.encoder_pub = self.create_publisher(
            JointState, g('encoder_topic').value, qos)

        encoder_rate = g('encoder_publish_rate').value
        self.encoder_timer = self.create_timer(1.0 / encoder_rate, self._read_encoders)

        self.get_logger().info(
            f'roboclaw_driver started: left={left_port}, right={right_port}, '
            f'baud={baud}, L=lx+ly={self.l_sum:.4f} m, '
            f'max_linear_duty={self.max_linear_duty}, '
            f'max_angular_duty={self.max_angular_duty}, '
            f'cmd_timeout={self.cmd_timeout}s, '
            f'encoder_topic="{g("encoder_topic").value}" @ {encoder_rate} Hz')

    def _get_bus(self, port, baud, timeout):
        """Reuse one SerialBus per physical port (handles a shared bus too)."""
        if port not in self._buses:
            self._buses[port] = SerialBus(port, baud, timeout)
        return self._buses[port]

    def _signed(self, corner, value):
        return -value if self.inversions[corner] else value

    # --- Write path -----------------------------------------------------------

    def _on_cmd_vel(self, msg):
        # Treated as normalized [-1, 1] commands, not m/s (see module docstring).
        self.vx = max(-1.0, min(1.0, msg.linear.x))
        self.vy = max(-1.0, min(1.0, msg.linear.y))
        self.omega = max(-1.0, min(1.0, msg.angular.z))
        self.last_cmd_time = self.get_clock().now()

    def _control_step(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            vx = vy = omega = 0.0
        else:
            vx, vy, omega = self.vx, self.vy, self.omega

        lin_x = self.max_linear_duty * vx
        lin_y = self.max_linear_duty * vy
        ang = self.max_angular_duty * omega

        wheel = {
            'front_left':  lin_x - lin_y - ang,
            'front_right': lin_x + lin_y + ang,
            'rear_left':   lin_x + lin_y - ang,
            'rear_right':  lin_x - lin_y + ang,
        }
        for c in CORNERS:
            wheel[c] = self._signed(c, wheel[c])

        # Fixed wiring:
        #   LEFT  controller (ACM0): front_left = M2, rear_left = M1
        #   RIGHT controller (ACM1): front_right = M1, rear_right = M2
        left_ok = self.left.duty_m1m2(wheel['rear_left'], wheel['front_left'])
        right_ok = self.right.duty_m1m2(wheel['front_right'], wheel['rear_right'])

        if not (left_ok and right_ok):
            self.get_logger().warn('RoboClaw duty command not acknowledged '
                                   '(CRC/timeout) -- check serial connection',
                                   throttle_duration_sec=2.0)

    # --- Read path ------------------------------------------------------------

    def _read_encoders(self):
        left_reading = self.left.read_encoders()    # (M1, M2) or None
        right_reading = self.right.read_encoders()

        if left_reading is None or right_reading is None:
            self.get_logger().warn('RoboClaw encoder read failed (CRC/timeout); '
                                   'skipping this cycle', throttle_duration_sec=2.0)
            return

        # Fixed wiring. read_encoders() returns (M1, M2).
        #   LEFT  controller (ACM0): front = M2, rear = M1
        #   RIGHT controller (ACM1): front = M1, rear = M2
        counts = {
            'front_left': left_reading[1],
            'rear_left': left_reading[0],
            'front_right': right_reading[0],
            'rear_right': right_reading[1],
        }

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(CORNERS)
        # Raw quadrature counts, not radians -- consumers convert using their
        # own counts_per_rev/wheel_radius.
        msg.position = [float(counts[c]) for c in CORNERS]
        self.encoder_pub.publish(msg)

    def destroy_node(self):
        # Best-effort stop on shutdown
        try:
            self.left.stop()
            self.right.stop()
        except Exception:  # noqa: BLE001
            pass
        for bus in self._buses.values():
            bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoboclawDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
