# Copyright 2026
#
# Mecanum-drive wheel odometry from two RoboClaw controllers.
#
# Rover: Lynxmotion A4WD3 Mecanum (4x 152 mm mecanum wheels, holonomic).
#
# Wiring (final): ACM0 = LEFT controller, ACM1 = RIGHT controller. The front
# wheel is on a different channel per side:
#   front_left  = left  M2     rear_left  = left  M1   (left side: front on M2)
#   front_right = right M1     rear_right = right M2   (right side: front on M1)
#
# Because mecanum wheels can roll laterally, all FOUR wheels are read to recover
# the body twist (vx, vy, wz). The node applies the standard mecanum forward
# kinematics, integrates a full planar pose, and publishes nav_msgs/Odometry.
#
# Mecanum forward kinematics (wheel linear displacements d_*, wheel radius r,
# L = lx + ly with lx = wheelbase/2, ly = track_width/2):
#   dx_body =  (d_fl + d_fr + d_rl + d_rr) / 4
#   dy_body =  (-d_fl + d_fr + d_rl - d_rr) / 4
#   dtheta  =  (-d_fl + d_fr - d_rl + d_rr) / (4 * L)
# (X-roller configuration; flip per-wheel invert flags if signs are off.)

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from wheel_odometry.roboclaw_driver import RoboClaw, SerialBus

CORNERS = ('front_left', 'front_right', 'rear_left', 'rear_right')


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        # --- Connection parameters -------------------------------------------
        # Stable udev symlinks (keyed to physical USB port), NOT /dev/ttyACM*
        # which renumber by insertion order. See udev/99-roboclaw.rules.
        self.declare_parameter('left_port', '/dev/roboclaw_left')
        self.declare_parameter('right_port', '/dev/roboclaw_right')
        self.declare_parameter('left_address', 0x80)
        self.declare_parameter('right_address', 0x80)
        self.declare_parameter('baud', 115200)
        self.declare_parameter('serial_timeout', 0.1)

        # Per-corner sign inversion so forward motion reads positive.
        self.declare_parameter('invert_front_left', False)
        self.declare_parameter('invert_front_right', False)
        self.declare_parameter('invert_rear_left', False)
        self.declare_parameter('invert_rear_right', False)

        # --- Robot geometry / encoder parameters -----------------------------
        # A4WD3 Mecanum: 152 mm wheel diameter -> 0.076 m radius.
        self.declare_parameter('wheel_radius', 0.076)
        # Wheel-CENTER distances measured on the rover (not published by vendor).
        self.declare_parameter('wheelbase', 0.220)    # front<->rear center dist (m)
        self.declare_parameter('track_width', 0.330)  # left<->right center dist (m)
        # Counts per WHEEL revolution = encoder_PPR(12) * 4(quadrature) * gear_ratio.
        # This rover: 51:1 -> 12*4*51 = 2448 (confirmed).
        self.declare_parameter('counts_per_rev', 2448.0)

        # --- ROS interface parameters ----------------------------------------
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_topic', 'wheel/odometry')
        self.declare_parameter('publish_tf', True)
        # Covariance diagonals: [x, y, z, roll, pitch, yaw].
        self.declare_parameter('pose_covariance_diagonal',
                               [0.01, 0.01, 1e6, 1e6, 1e6, 0.05])
        self.declare_parameter('twist_covariance_diagonal',
                               [0.01, 0.01, 1e6, 1e6, 1e6, 0.05])

        g = self.get_parameter
        self.wheel_radius = g('wheel_radius').value
        self.wheelbase = g('wheelbase').value
        self.track_width = g('track_width').value
        self.counts_per_rev = g('counts_per_rev').value
        self.odom_frame_id = g('odom_frame_id').value
        self.base_frame_id = g('base_frame_id').value
        self.publish_tf = g('publish_tf').value
        self.pose_cov = list(g('pose_covariance_diagonal').value)
        self.twist_cov = list(g('twist_covariance_diagonal').value)

        # Per-corner sign inversion.
        self.inversions = {c: g(f'invert_{c}').value for c in CORNERS}

        # L = lx + ly = (wheelbase + track_width) / 2, used for the rotation term.
        self.l_sum = 0.5 * (self.wheelbase + self.track_width)

        if self.wheelbase <= 0.0 or self.track_width <= 0.0 or self.counts_per_rev <= 0.0:
            self.get_logger().warn(
                'wheelbase, track_width and counts_per_rev must be positive for '
                'meaningful odometry. Current: '
                f'wheelbase={self.wheelbase}, track_width={self.track_width}, '
                f'counts_per_rev={self.counts_per_rev}')

        # Meters travelled per encoder count.
        self.meters_per_count = (
            (2.0 * math.pi * self.wheel_radius) / self.counts_per_rev
            if self.counts_per_rev > 0.0 else 0.0)

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

        # --- Pose / odometry state -------------------------------------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev = None       # {corner: absolute count}
        self.prev_time = None

        # --- Publishers / timer ----------------------------------------------
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.create_publisher(Odometry, g('odom_topic').value, qos)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        rate = g('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.update)
        self.get_logger().info(
            f'wheel_odometry (mecanum) started: left={left_port}, right={right_port}, '
            f'baud={baud}, rate={rate} Hz, r={self.wheel_radius} m, '
            f'L=lx+ly={self.l_sum:.4f} m, counts/rev={self.counts_per_rev}, '
            f'topic="{g("odom_topic").value}"')

    def _get_bus(self, port, baud, timeout):
        """Reuse one SerialBus per physical port (handles a shared bus too)."""
        if port not in self._buses:
            self._buses[port] = SerialBus(port, baud, timeout)
        return self._buses[port]

    def _signed(self, corner, value):
        return -value if self.inversions[corner] else value

    def update(self):
        left_reading = self.left.read_encoders()    # (M1, M2) or None
        right_reading = self.right.read_encoders()

        now = self.get_clock().now()

        if left_reading is None or right_reading is None:
            self.get_logger().warn('RoboClaw encoder read failed (CRC/timeout); '
                                   'skipping this cycle', throttle_duration_sec=2.0)
            return

        # Fixed wiring. read_encoders() returns (M1, M2).
        #   LEFT  controller (ACM0): front = M2, rear = M1
        #   RIGHT controller (ACM1): front = M1, rear = M2
        curr = {
            'front_left': self._signed('front_left', left_reading[1]),    # left M2
            'rear_left': self._signed('rear_left', left_reading[0]),      # left M1
            'front_right': self._signed('front_right', right_reading[0]),  # right M1
            'rear_right': self._signed('rear_right', right_reading[1]),    # right M2
        }

        # First good reading just seeds the previous state.
        if self.prev is None:
            self.prev = curr
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        # Per-wheel distance travelled since last cycle (meters).
        d = {c: (curr[c] - self.prev[c]) * self.meters_per_count for c in CORNERS}

        # Mecanum forward kinematics -> body-frame displacement.
        dx_body = 0.25 * (d['front_left'] + d['front_right']
                          + d['rear_left'] + d['rear_right'])
        dy_body = 0.25 * (-d['front_left'] + d['front_right']
                          + d['rear_left'] - d['rear_right'])
        dtheta = ((-d['front_left'] + d['front_right']
                   - d['rear_left'] + d['rear_right']) / (4.0 * self.l_sum)
                  if self.l_sum > 0.0 else 0.0)

        # Integrate in the world frame using the midpoint heading.
        mid = self.theta + 0.5 * dtheta
        self.x += dx_body * math.cos(mid) - dy_body * math.sin(mid)
        self.y += dx_body * math.sin(mid) + dy_body * math.cos(mid)
        self.theta = math.atan2(math.sin(self.theta + dtheta),
                                math.cos(self.theta + dtheta))

        # Body-frame velocities for the twist field (expressed in base_link).
        vx = dx_body / dt
        vy = dy_body / dt
        omega = dtheta / dt

        self.prev = curr
        self.prev_time = now

        self.publish(now, vx, vy, omega)

    def publish(self, stamp, vx, vy, omega):
        q = yaw_to_quaternion(self.theta)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q

        # Twist is expressed in child_frame_id (base_link); mecanum has vy.
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        for i in range(6):
            odom.pose.covariance[i * 7] = self.pose_cov[i]
            odom.twist.covariance[i * 7] = self.twist_cov[i]

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        for bus in self._buses.values():
            bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
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
