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
#     issues per-wheel speed commands in encoder counts/sec.
#
# CLOSED-LOOP control: uses RoboClaw's Speed/QPPS command with an acceleration
# ramp, not the raw duty cycle. The controller's own velocity PID holds the
# commanded counts/sec, so vx/vy/omega here are true m/s and rad/s, clamped to
# max_vx/max_vy/max_omega -- not a normalized [-1, 1] range.
#
# Mecanum inverse kinematics (X-roller config -- the exact inverse of the
# forward kinematics wheel_odometry_node uses, so the sign conventions
# match): L = lx + ly = (wheelbase + track_width) / 2
#   v_fl = vx - vy - L*omega      v_fr = vx + vy + L*omega
#   v_rl = vx + vy - L*omega      v_rr = vx - vy + L*omega
#
# Comms watchdog: if no /cmd_vel arrives within `cmd_timeout` seconds (e.g.
# the Bluetooth controller drops), all four wheels are zeroed.

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from teleop.mecanum_kinematics import (
    CORNERS,
    apply_inversions,
    body_twist_to_wheel_counts,
    scale_to_ceiling,
    validate_geometry,
)
from teleop.roboclaw_driver import RoboClaw, SerialBus
from teleop.speed_limits import QppsLimit, resolve_max_qpps


class RoboclawDriverNode(Node):
    def __init__(self):
        super().__init__("roboclaw_driver")

        # --- Connection parameters ---------------------------------------
        # Stable udev symlinks (keyed to physical USB port), NOT /dev/ttyACM*
        # which renumber by insertion order. See udev/99-roboclaw.rules.
        self.declare_parameter("left_port", "/dev/roboclaw_left")
        self.declare_parameter("right_port", "/dev/roboclaw_right")
        self.declare_parameter("left_address", 0x80)
        self.declare_parameter("right_address", 0x80)
        self.declare_parameter("baud", 115200)
        self.declare_parameter("serial_timeout", 0.1)

        # Per-corner sign inversion for the WRITE path. Independent of
        # wheel_odometry's own invert_* flags (separate concern, separate
        # convention) -- verify with a bench test (wheels up).
        self.declare_parameter("invert_front_left", False)
        self.declare_parameter("invert_front_right", False)
        self.declare_parameter("invert_rear_left", False)
        self.declare_parameter("invert_rear_right", False)

        # --- Geometry --------------------------------------------------------
        # wheel_radius and counts_per_rev convert m/s to counts/sec and must
        # match wheel_odometry.yaml, or the command path and the odometry path
        # disagree on scale.
        self.declare_parameter("wheelbase", 0.220)
        self.declare_parameter("track_width", 0.330)
        self.declare_parameter("wheel_radius", 0.076)
        self.declare_parameter("counts_per_rev", 2448.0)

        # --- Command limits --------------------------------------------------
        # Ceilings on an incoming /cmd_vel, in SI units. Deliberately well
        # under what the hardware can do: at 7590 qpps the wheels top out near
        # 1.48 m/s, so 0.25 leaves plenty of traction margin.
        self.declare_parameter("max_vx", 0.25)  # m/s
        self.declare_parameter("max_vy", 0.25)  # m/s
        self.declare_parameter("max_omega", 0.5)  # rad/s

        # Acceleration ramp applied by the controller, counts/sec^2. 6000 is
        # about 1.2 m/s^2 -- roughly 0.12 g, chosen to stay clear of the
        # traction limit that an unramped duty step blows straight through.
        # It is also the deceleration for every stop, including the watchdog
        # and a released deadman button, so lowering it lengthens the stop.
        self.declare_parameter("drive_accel", 6000)

        self.declare_parameter("cmd_topic", "cmd_vel")
        self.declare_parameter("cmd_timeout", 0.5)  # s; watchdog e-stop
        self.declare_parameter("control_rate", 20.0)  # Hz, write-path rate

        # --- Encoder publishing ---------------------------------------------
        self.declare_parameter("encoder_topic", "roboclaw/wheel_encoders")
        self.declare_parameter("encoder_publish_rate", 30.0)  # Hz, read-path rate

        g = self.get_parameter
        self.wheelbase = g("wheelbase").value
        self.track_width = g("track_width").value
        self.l_sum = 0.5 * (self.wheelbase + self.track_width)
        self.wheel_radius = g("wheel_radius").value
        self.counts_per_rev = g("counts_per_rev").value
        self.max_vx = abs(g("max_vx").value)
        self.max_vy = abs(g("max_vy").value)
        self.max_omega = abs(g("max_omega").value)
        self.drive_accel = g("drive_accel").value
        self.cmd_timeout = g("cmd_timeout").value
        self.inversions = {c: g(f"invert_{c}").value for c in CORNERS}

        # Fail at startup rather than per control cycle: the conversion would
        # otherwise raise 20 times a second inside a timer callback.
        validate_geometry(self.wheel_radius, self.counts_per_rev)

        # --- Open serial bus(es) and controllers -----------------------------
        baud = g("baud").value
        timeout = g("serial_timeout").value
        left_port = g("left_port").value
        right_port = g("right_port").value

        self._buses = {}
        try:
            left_bus = self._get_bus(left_port, baud, timeout)
            right_bus = self._get_bus(right_port, baud, timeout)
        except Exception as exc:
            # Opening the second port can fail with the first already open.
            # Release it here: __init__ is about to raise, so the half-built
            # node never reaches destroy_node and nothing else will.
            self._close_buses()
            self.get_logger().fatal(f"Failed to open RoboClaw serial port: {exc}")
            raise

        self.left = RoboClaw(left_bus, g("left_address").value)
        self.right = RoboClaw(right_bus, g("right_address").value)

        # --- Speed ceiling ---------------------------------------------------
        # Read before any command goes out: this is the per-wheel ceiling that
        # scale_to_ceiling() enforces in _control_step, so a failed read means
        # commands are bounded by an assumed value rather than a measured one.
        limit = self._resolve_speed_ceiling()
        self.max_qpps = limit.ceiling
        if limit.ok:
            self.get_logger().info(limit.status)
        else:
            self.get_logger().warning(limit.status)

        # --- Write-path (cmd_vel -> counts/sec) state -------------------------
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.last_cmd_time = self.get_clock().now()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Twist, g("cmd_topic").value, self._on_cmd_vel, qos)

        control_rate = g("control_rate").value
        self.control_timer = self.create_timer(1.0 / control_rate, self._control_step)

        # --- Read-path (encoders -> JointState) ------------------------------
        self.encoder_pub = self.create_publisher(
            JointState, g("encoder_topic").value, qos
        )

        encoder_rate = g("encoder_publish_rate").value
        self.encoder_timer = self.create_timer(1.0 / encoder_rate, self._read_encoders)

        self.get_logger().info(
            f"roboclaw_driver started: left={left_port}, right={right_port}, "
            f"baud={baud}, L=lx+ly={self.l_sum:.4f} m, "
            f"closed-loop limits vx={self.max_vx} m/s vy={self.max_vy} m/s "
            f"omega={self.max_omega} rad/s, drive_accel={self.drive_accel} "
            f"counts/s^2, cmd_timeout={self.cmd_timeout}s, "
            f'encoder_topic="{g("encoder_topic").value}" @ {encoder_rate} Hz'
        )

    def _read_pid(self, name: str, controller: RoboClaw) -> int | None:
        """Reads one controller's velocity PID and returns its QPPS ceiling.

        Logs the gains on the way through: they are worth having in a bag file,
        and a mismatch between the two sides explains transient differences
        that are otherwise easy to blame on the wheels.

        A failed read is deliberately non-fatal -- the caller falls back to an
        assumed ceiling. That includes a raised SerialException, which is why
        this catches broadly: this runs during __init__, and letting it escape
        would skip destroy_node() and leave both ports open with whatever the
        previous process left applied to the motors.

        Args:
            name: Side label, for log messages.
            controller: The controller to query.

        Returns:
            The QPPS ceiling, or None if it could not be read.
        """
        try:
            pid = controller.read_velocity_pid()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"{name} controller: velocity PID read raised ({exc})"
            )
            return None

        if pid is None:
            self.get_logger().warning(
                f"{name} controller: velocity PID read failed (CRC/timeout)"
            )
            return None

        p, i, d, qpps = pid
        self.get_logger().info(
            f"{name} controller: P={p:.3f} I={i:.3f} D={d:.3f} qpps={qpps}"
        )
        return qpps

    def _resolve_speed_ceiling(self) -> QppsLimit:
        """Resolves the QPPS ceiling both controllers can honour.

        Returns:
            The resolved ceiling, status message and nominal flag.
        """
        return resolve_max_qpps(
            self._read_pid("left", self.left),
            self._read_pid("right", self.right),
        )

    def _get_bus(self, port, baud, timeout):
        """Reuse one SerialBus per physical port (handles a shared bus too)."""
        if port not in self._buses:
            self._buses[port] = SerialBus(port, baud, timeout)
        return self._buses[port]

    # --- Write path -----------------------------------------------------------

    def _on_cmd_vel(self, msg):
        # A non-finite component would survive the clamp below and read as a
        # full-speed command: min(0.25, nan) is 0.25, because nan < 0.25 is
        # False. Drop the message without refreshing the timestamp, so the
        # watchdog ramps to a stop instead of the rover driving away.
        if not all(
            math.isfinite(v) for v in (msg.linear.x, msg.linear.y, msg.angular.z)
        ):
            self.get_logger().warning(
                "Discarding /cmd_vel with a non-finite component",
                throttle_duration_sec=2.0,
            )
            return

        # True SI units: m/s and rad/s, clamped to this node's own ceilings.
        self.vx = max(-self.max_vx, min(self.max_vx, msg.linear.x))
        self.vy = max(-self.max_vy, min(self.max_vy, msg.linear.y))
        self.omega = max(-self.max_omega, min(self.max_omega, msg.angular.z))
        self.last_cmd_time = self.get_clock().now()

    def _control_step(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            vx = vy = omega = 0.0
        else:
            vx, vy, omega = self.vx, self.vy, self.omega

        # Zero speed goes out through the same ramp, so a watchdog trip or a
        # released deadman button decelerates instead of stopping dead.
        wheel = apply_inversions(
            scale_to_ceiling(
                body_twist_to_wheel_counts(
                    vx,
                    vy,
                    omega,
                    wheel_radius=self.wheel_radius,
                    counts_per_rev=self.counts_per_rev,
                    l_sum=self.l_sum,
                ),
                self.max_qpps,
            ),
            self.inversions,
        )

        # Fixed wiring:
        #   LEFT  controller (ACM0): front_left = M2, rear_left = M1
        #   RIGHT controller (ACM1): front_right = M1, rear_right = M2
        left_ok = self.left.speed_accel_m1m2(
            self.drive_accel, wheel["rear_left"], wheel["front_left"]
        )
        right_ok = self.right.speed_accel_m1m2(
            self.drive_accel, wheel["front_right"], wheel["rear_right"]
        )

        if not (left_ok and right_ok):
            self.get_logger().warning(
                "RoboClaw speed command not acknowledged "
                "(CRC/timeout) -- check serial connection",
                throttle_duration_sec=2.0,
            )

    # --- Read path ------------------------------------------------------------

    def _read_encoders(self):
        left_reading = self.left.read_encoders()  # (M1, M2) or None
        right_reading = self.right.read_encoders()

        if left_reading is None or right_reading is None:
            self.get_logger().warn(
                "RoboClaw encoder read failed (CRC/timeout); skipping this cycle",
                throttle_duration_sec=2.0,
            )
            return

        # Fixed wiring. read_encoders() returns (M1, M2).
        #   LEFT  controller (ACM0): front = M2, rear = M1
        #   RIGHT controller (ACM1): front = M1, rear = M2
        counts = {
            "front_left": left_reading[1],
            "rear_left": left_reading[0],
            "front_right": right_reading[0],
            "rear_right": right_reading[1],
        }

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(CORNERS)
        # Raw quadrature counts, not radians -- consumers convert using their
        # own counts_per_rev/wheel_radius.
        msg.position = [float(counts[c]) for c in CORNERS]
        self.encoder_pub.publish(msg)

    def _close_buses(self) -> None:
        """Closes every serial port this node opened. Safe to call twice."""
        for bus in self._buses.values():
            bus.close()
        self._buses.clear()

    def destroy_node(self):
        # Best-effort stop on shutdown -- don't leave motors spinning.
        try:
            self.left.stop()
            self.right.stop()
        except Exception:  # noqa: BLE001
            pass
        self._close_buses()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    # Bound to None first so the finally block can tell a failed construction
    # from a failed spin: a node that never finished __init__ has no
    # destroy_node to call, but rclpy still has to be shut down.
    node = None
    try:
        node = RoboclawDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
