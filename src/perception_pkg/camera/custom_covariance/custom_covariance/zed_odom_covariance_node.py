"""Republish the ZED's odometry with a real twist covariance attached.

WHY THIS EXISTS
---------------
The ZED wrapper publishes /zed/zed_node/odom with `twist.covariance` left as
all zeros. It is not a configuration mistake and there is no parameter that
fixes it: in zed_camera_component_main.cpp's ZedCamera::publishOdom() the
message is default-constructed (so every covariance entry starts at 0.0), the
POSE covariance is then filled from the SDK's slPose.pose_covariance, the twist
VALUES are filled -- and `twist.covariance` is never assigned before the message
is published. The root cause is upstream of the wrapper: the SDK's sl::Pose
exposes a pose covariance but has no velocity covariance to copy from.

That matters because ekf.yaml fuses ONLY the twist half of this message
(vx, vy, vyaw). Measured on this rover with robot_localization's debug output:

  * robot_localization sees a zero variance, substitutes 1e-9, and logs
    "measurement had very small error covariance for index 6/7/11. Adding some
    noise to maintain filter stability." -- 34044 times in ~30 seconds. Those
    warnings only go to the debug stream, never the console.
  * The resulting Kalman gain for vx/vy/vyaw is EXACTLY 1.0. The corrected
    state equals the measurement digit for digit: the ZED overwrites the wheel
    odometry rather than being blended with it, and the covariance tuning in
    wheel_odometry.yaml is bypassed.
  * The same 1e-9 feeds odom1_twist_rejection_threshold's Mahalanobis test, so
    ordinary innovations look enormous: 5867 of 29003 twist measurements (~20%)
    were rejected outright.

Over-trusted when accepted, discarded a fifth of the time. Neither is fusion.

robot_localization has no per-input covariance override -- verified by dumping
every parameter the node accepts: the only covariance parameters are
process_noise_covariance, initial_estimate_covariance and
dynamic_process_noise_covariance, all filter-level, none per-sensor.
Measurement covariance can only arrive ON THE MESSAGE. Hence this node.

It is the same thing wheel_odometry_node already does for its own output
(see its pose_cov/twist_cov application) -- the wheels get their covariance
from a YAML we own, and this gives the camera the same treatment, because
its vendor does not.

WHAT IT DOES
------------
Subscribes to the wrapper's odometry, replaces `twist.covariance` with the
diagonal from config/zed_odom_covariance.yaml, republishes. Everything else --
header, child_frame_id, pose, pose covariance, twist values -- is passed
through untouched.

The POSE covariance is deliberately NOT overwritten: the SDK populates it with
real, live-varying data, and ekf.yaml does not fuse the pose anyway.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Order of the diagonal, matching nav_msgs/Odometry's twist covariance layout
# and wheel_odometry.yaml's convention.
AXES = ("vx", "vy", "vz", "vroll", "vpitch", "vyaw")


class ZedOdomCovariance(Node):
    """Republishes ZED odometry with a nonzero twist covariance attached."""

    def __init__(self) -> None:
        """Declares parameters, validates the diagonal, and wires the pub/sub.

        Raises:
            RuntimeError: If `twist_covariance_diagonal` is not six positive
                entries, which would recreate the bug this node exists to fix.
        """
        super().__init__("zed_odom_covariance")

        self.declare_parameter("input_topic", "/zed/zed_node/odom")
        self.declare_parameter("output_topic", "/zed/odom_with_cov")
        self.declare_parameter(
            "twist_covariance_diagonal", [0.02, 0.02, 1.0e6, 1.0e6, 1.0e6, 0.01]
        )

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self.twist_cov = list(self.get_parameter("twist_covariance_diagonal").value)

        if len(self.twist_cov) != 6:
            raise RuntimeError(
                "twist_covariance_diagonal must have exactly 6 entries "
                f"({', '.join(AXES)}); got {len(self.twist_cov)}."
            )
        if any(v <= 0.0 for v in self.twist_cov):
            raise RuntimeError(
                "twist_covariance_diagonal entries must all be > 0 -- a zero "
                "here would recreate exactly the bug this node exists to fix. "
                f"Got {self.twist_cov}."
            )

        # Match the wrapper's RELIABLE QoS; a mismatch here silently delivers
        # nothing, which looks identical to the camera not running.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.pub = self.create_publisher(Odometry, out_topic, qos)
        self.sub = self.create_subscription(Odometry, in_topic, self._on_odom, qos)

        self.count = 0
        self._warned_upstream_fixed = False
        # One-shot liveness check: a wrong input_topic is otherwise invisible,
        # because "no output" and "camera not running" look the same.
        self._startup_timer = self.create_timer(10.0, self._check_alive)

        self.get_logger().info(
            f"Republishing {in_topic} -> {out_topic} with twist covariance "
            f"[{', '.join(f'{a}={v:g}' for a, v in zip(AXES, self.twist_cov))}]"
        )

    def _check_alive(self) -> None:
        """Warns once if nothing arrived on the input topic within 10 s."""
        self._startup_timer.cancel()
        if self.count == 0:
            self.get_logger().warn(
                f"No messages on {self.get_parameter('input_topic').value} "
                "after 10 s. Is the ZED wrapper running, and is the topic name "
                "right? The EKF gets nothing until this node forwards."
            )

    def _on_odom(self, msg: Odometry) -> None:
        """Overwrites the twist covariance and republishes the message.

        Args:
            msg: Odometry from the ZED wrapper. Header, child_frame_id, pose,
                pose covariance and twist values pass through untouched.
        """
        # If Stereolabs ever starts populating this, our hardcoded numbers would
        # silently override real SDK data. Say so, once, rather than hiding it.
        if not self._warned_upstream_fixed and any(
            v != 0.0 for v in msg.twist.covariance
        ):
            self._warned_upstream_fixed = True
            self.get_logger().warn(
                "Incoming twist.covariance is NON-ZERO -- the ZED wrapper now "
                "provides its own. This node is overriding it with config "
                "values. Re-evaluate whether it is still needed."
            )

        # Replace outright rather than merging: setting only the diagonal would
        # leave any upstream off-diagonal terms mixed in with our numbers.
        for i in range(36):
            msg.twist.covariance[i] = 0.0
        for i in range(6):
            msg.twist.covariance[i * 7] = self.twist_cov[i]

        self.pub.publish(msg)
        self.count += 1


def main() -> None:
    """Spins the covariance republisher until interrupted."""
    rclpy.init()
    node = ZedOdomCovariance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
