# Copyright 2026
#
# Mecanum-drive wheel odometry from raw encoder counts..
#
# This node does NOT talk to the RoboClaws directly -- that hardware I/O is
# owned exclusively by low_level_control_pkg's roboclaw_driver_node. 
# Instead, this node subscribes to `encoder_topic` (sensor_msgs/JointState, position = raw
# quadrature counts per corner) and does everything downstream of that: the
# counts->meters conversion, mecanum forward kinematics, pose integration,
# and nav_msgs/Odometry (+ TF) publishing.
#
# Because mecanum wheels can roll laterally, all FOUR wheels are read to
# recover the body twist (vx, vy, wz). The node applies the standard mecanum
# forward kinematics, integrates a full planar pose, and publishes
# nav_msgs/Odometry.
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
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

CORNERS = ('front_left', 'front_right', 'rear_left', 'rear_right')


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        # --- Encoder source ----------------------------------------------
        self.declare_parameter('encoder_topic', 'roboclaw/wheel_encoders')

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

        # --- Pose / odometry state -------------------------------------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev = None       # {corner: absolute count}
        self.prev_time = None

        # --- Publishers / subscription ----------------------------------------
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.create_publisher(Odometry, g('odom_topic').value, qos)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        encoder_topic = g('encoder_topic').value
        self.create_subscription(JointState, encoder_topic, self._on_joint_state, qos)

        self.get_logger().info(
            f'wheel_odometry (mecanum) started: encoder_topic="{encoder_topic}", '
            f'r={self.wheel_radius} m, L=lx+ly={self.l_sum:.4f} m, '
            f'counts/rev={self.counts_per_rev}, topic="{g("odom_topic").value}"')

    def _signed(self, corner, value):
        return -value if self.inversions[corner] else value

    def _on_joint_state(self, msg):
        try:
            counts_by_name = dict(zip(msg.name, msg.position))
            curr = {c: self._signed(c, counts_by_name[c]) for c in CORNERS}
        except KeyError:
            self.get_logger().warn(
                'encoder JointState missing expected corner names; got '
                f'{list(msg.name)}', throttle_duration_sec=5.0)
            return

        now = self.get_clock().now()

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

