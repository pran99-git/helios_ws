# Copyright 2026
#
# Mecanum-drive wheel odometry from raw encoder counts.
#
# This node does NOT talk to the RoboClaws -- that hardware I/O is owned
# exclusively by low_level_control_pkg's roboclaw_driver_node. It subscribes to
# `encoder_topic` (sensor_msgs/JointState, position = raw quadrature counts per
# corner) and publishes nav_msgs/Odometry, optionally with the odom->base_link
# TF.
#
# Thin adapter: parameters, pub/sub and message conversion only. The counts to
# meters conversion, the mecanum forward kinematics and the pose integration
# live in mecanum_odometry.py, which has no ROS dependency.

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from wheel_odometry.mecanum_odometry import (
    CORNERS,
    PlanarPose,
    body_displacement,
    lever_arm,
    meters_per_count,
    validate_geometry,
    yaw_to_quaternion_zw,
)


class WheelOdometryNode(Node):
    """Publishes mecanum wheel odometry from a raw encoder JointState topic."""

    def __init__(self) -> None:
        """Declares parameters, prepares the pub/sub pair, and logs the setup."""
        super().__init__("wheel_odometry")

        # --- Encoder source ---------------------------------------------------
        self.declare_parameter("encoder_topic", "roboclaw/wheel_encoders")

        # Per-corner sign inversion so forward motion reads positive.
        self.declare_parameter("invert_front_left", False)
        self.declare_parameter("invert_front_right", False)
        self.declare_parameter("invert_rear_left", False)
        self.declare_parameter("invert_rear_right", False)

        # --- Robot geometry / encoder parameters ------------------------------
        # A4WD3 Mecanum: 152 mm wheel diameter -> 0.076 m radius.
        self.declare_parameter("wheel_radius", 0.076)
        # Wheel-CENTER distances measured on the rover (not published by vendor).
        self.declare_parameter("wheelbase", 0.220)
        self.declare_parameter("track_width", 0.330)
        # Counts per WHEEL revolution = encoder_PPR(12) * 4(quadrature) * 51:1.
        self.declare_parameter("counts_per_rev", 2448.0)
        # Empirical correction on the yaw term only -- see the note in
        # config/wheel_odometry.yaml. 1.0 is the uncorrected geometric model.
        self.declare_parameter("yaw_scale", 1.0)

        # --- ROS interface parameters -----------------------------------------
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("odom_topic", "wheel/odometry")
        self.declare_parameter("publish_tf", True)
        # Covariance diagonals: [x, y, z, roll, pitch, yaw].
        self.declare_parameter(
            "pose_covariance_diagonal", [0.01, 0.01, 1e6, 1e6, 1e6, 0.05]
        )
        self.declare_parameter(
            "twist_covariance_diagonal", [0.01, 0.01, 1e6, 1e6, 1e6, 0.05]
        )

        g = self.get_parameter
        wheel_radius = g("wheel_radius").value
        wheelbase = g("wheelbase").value
        track_width = g("track_width").value
        counts_per_rev = g("counts_per_rev").value

        validate_geometry(wheelbase, track_width, counts_per_rev)

        self.yaw_scale = g("yaw_scale").value
        self.odom_frame_id = g("odom_frame_id").value
        self.base_frame_id = g("base_frame_id").value
        self.pose_cov = list(g("pose_covariance_diagonal").value)
        self.twist_cov = list(g("twist_covariance_diagonal").value)
        self.inversions = {c: g(f"invert_{c}").value for c in CORNERS}

        self.meters_per_count = meters_per_count(wheel_radius, counts_per_rev)
        self.lever_arm = lever_arm(wheelbase, track_width)

        self.pose = PlanarPose()
        self.prev: dict[str, float] | None = None
        self.prev_time: Time | None = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.create_publisher(Odometry, g("odom_topic").value, qos)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if g("publish_tf").value else None
        )

        encoder_topic = g("encoder_topic").value
        self.create_subscription(JointState, encoder_topic, self._on_joint_state, qos)

        self.get_logger().info(
            f'wheel_odometry (mecanum) started: encoder_topic="{encoder_topic}", '
            f"r={wheel_radius} m, L=lx+ly={self.lever_arm:.4f} m, "
            f"yaw_scale={self.yaw_scale}, counts/rev={counts_per_rev}, "
            f'topic="{g("odom_topic").value}"'
        )

    def _on_joint_state(self, msg: JointState) -> None:
        """Converts one encoder message into an odometry update.

        Args:
            msg: Raw quadrature counts per corner in `position`, named by
                the entries of `CORNERS`.
        """
        try:
            counts_by_name = dict(zip(msg.name, msg.position))
            curr = {
                c: -counts_by_name[c] if self.inversions[c] else counts_by_name[c]
                for c in CORNERS
            }
        except KeyError:
            self.get_logger().warn(
                "encoder JointState missing expected corner names; got "
                f"{list(msg.name)}",
                throttle_duration_sec=5.0,
            )
            return

        now = self.get_clock().now()

        # First good reading only seeds the previous state; there is no
        # displacement to integrate yet.
        if self.prev is None or self.prev_time is None:
            self.prev = curr
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        wheel_distances = {
            c: (curr[c] - self.prev[c]) * self.meters_per_count for c in CORNERS
        }
        dx_body, dy_body, dtheta = body_displacement(
            wheel_distances, self.lever_arm, self.yaw_scale
        )
        self.pose.integrate(dx_body, dy_body, dtheta)

        self.prev = curr
        self.prev_time = now

        self._publish(now, dx_body / dt, dy_body / dt, dtheta / dt)

    def _publish(self, stamp: Time, vx: float, vy: float, omega: float) -> None:
        """Emits the Odometry message and, when enabled, the odom->base TF.

        Args:
            stamp: Time to stamp both messages with.
            vx: Body-frame forward velocity, m/s.
            vy: Body-frame left velocity, m/s.
            omega: Yaw rate, rad/s.
        """
        q = Quaternion()
        q.z, q.w = yaw_to_quaternion_zw(self.pose.theta)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = self.pose.x
        odom.pose.pose.position.y = self.pose.y
        odom.pose.pose.orientation = q

        # Twist is expressed in child_frame_id (base_link); mecanum has vy.
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        # Index i*7 walks the diagonal of the row-major 6x6 covariance.
        for i in range(6):
            odom.pose.covariance[i * 7] = self.pose_cov[i]
            odom.twist.covariance[i * 7] = self.twist_cov[i]

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = self.pose.x
            t.transform.translation.y = self.pose.y
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)


def main(args: list[str] | None = None) -> None:
    """Spins the wheel odometry node until interrupted.

    Args:
        args: Command line arguments forwarded to rclpy.
    """
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


if __name__ == "__main__":
    main()
