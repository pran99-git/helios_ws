"""ROS adapter: turn camera frames and a goal into a path Nav2 can follow.

Holds the observation history, asks the policy server for a waypoint chunk,
and republishes it as a nav_msgs/Path. The policy itself lives in another
process, so nothing here imports torch.

The path is stamped in `odom`, never `map`: map -> odom jumps whenever AMCL
relocalises, and a 2.4 s path that teleports mid-execution makes the
controller lurch. Odometry drift over that horizon is negligible.

Publishes no transforms. The EKF owns odom -> base_link and the active mapper
owns map -> odom, exactly as with Nav2.

All geometry lives in the modules beside this file; this handles parameters,
pub/sub, TF and message conversion only.
"""

from __future__ import annotations

import math

import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from PIL import Image as PILImage
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String

from vla_bridge.chunk_decoding import Waypoint
from vla_bridge.context_buffer import ContextBuffer
from vla_bridge.goal_resolution import (
    Pose2D,
    compose_pose,
    relative_pose,
    yaw_from_quaternion,
)
from vla_bridge.policy_client import PolicyClient, encode_jpeg

#: The policy defines no modality for "every goal at once" or "no goal", so
#: those are refused locally instead of being bounced by the server each cycle.
_MIN_GOALS = 1
_MAX_GOALS = 2


class VlaBridgeNode(Node):
    """Bridges the OmniVLA policy server to Nav2."""

    def __init__(self) -> None:
        super().__init__("vla_bridge")

        self.declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("goal_text_topic", "/vla/goal_text")
        self.declare_parameter("goal_image_topic", "/vla/goal_image")
        self.declare_parameter("plan_topic", "/vla/plan")
        self.declare_parameter("status_topic", "/vla/status")
        self.declare_parameter("endpoint", "http://127.0.0.1:8777/act")
        self.declare_parameter("request_timeout", 0.30)  # s
        self.declare_parameter("staleness_timeout", 1.00)  # s
        self.declare_parameter("inference_rate", 5.0)  # Hz
        self.declare_parameter("context_frames", 6)
        self.declare_parameter("context_stride", 0.333)  # s
        self.declare_parameter("wire_size", 224)  # px
        self.declare_parameter("path_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tf_timeout", 0.10)  # s

        def flt(name: str) -> float:
            return float(self.get_parameter(name).value)

        def text(name: str) -> str:
            return str(self.get_parameter(name).value)

        self._path_frame = text("path_frame")
        self._base_frame = text("base_frame")
        self._wire_size = int(self.get_parameter("wire_size").value)
        self._staleness = flt("staleness_timeout")
        self._tf_timeout = Duration(seconds=flt("tf_timeout"))

        self._history: ContextBuffer[str] = ContextBuffer(
            size=int(self.get_parameter("context_frames").value),
            stride=flt("context_stride"),
        )
        self._client = PolicyClient(text("endpoint"), timeout=flt("request_timeout"))
        self._cv = CvBridge()

        self._goal_pose: PoseStamped | None = None
        self._goal_text: str | None = None
        self._goal_image: str | None = None
        self._last_plan_time: float | None = None
        self._last_status = "waiting for a goal"

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._plan_pub = self.create_publisher(Path, text("plan_topic"), 1)
        self._status_pub = self.create_publisher(String, text("status_topic"), 1)

        # Frames are perishable: a queued one is a stale one, so keep depth 1
        # and match the sensor's best-effort reliability.
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, text("image_topic"), self._on_image, sensor_qos)
        self.create_subscription(
            PoseStamped, text("goal_pose_topic"), self._on_goal_pose, 1
        )
        self.create_subscription(String, text("goal_text_topic"), self._on_goal_text, 1)
        self.create_subscription(
            Image, text("goal_image_topic"), self._on_goal_image, 1
        )

        rate = flt("inference_rate")
        self.create_timer(1.0 / rate, self._step)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"vla_bridge up: {text('endpoint')} at {rate} Hz, "
            f"path in '{self._path_frame}', "
            f"{int(self.get_parameter('context_frames').value)} frames "
            f"every {flt('context_stride')} s"
        )

    # ---------------------------------------------------------------- inputs ---

    def _to_pil(self, msg: Image) -> PILImage.Image:
        """Convert a ROS image to RGB PIL, whatever encoding the camera used."""
        return PILImage.fromarray(self._cv.imgmsg_to_cv2(msg, desired_encoding="rgb8"))

    def _on_image(self, msg: Image) -> None:
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if not self._history.would_accept(stamp):
            return
        self._history.push(encode_jpeg(self._to_pil(msg), self._wire_size), stamp)

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        self._goal_pose = msg
        self.get_logger().info(
            f"pose goal ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) "
            f"in '{msg.header.frame_id}'"
        )

    def _on_goal_text(self, msg: String) -> None:
        self._goal_text = msg.data.strip() or None
        self.get_logger().info(f"language goal: {self._goal_text or '(cleared)'}")

    def _on_goal_image(self, msg: Image) -> None:
        self._goal_image = encode_jpeg(self._to_pil(msg), self._wire_size)
        self.get_logger().info("image goal received")

    # ------------------------------------------------------------------- TF ---

    def _base_pose_in(self, frame: str) -> Pose2D | None:
        """Pose of base_link within `frame`, or None if TF cannot answer."""
        try:
            tf = self._tf_buffer.lookup_transform(
                frame, self._base_frame, Time(), timeout=self._tf_timeout
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f"TF {frame} -> {self._base_frame} unavailable: {exc}", once=True
            )
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return Pose2D(t.x, t.y, yaw_from_quaternion(r.x, r.y, r.z, r.w))

    # ------------------------------------------------------------------ loop ---

    def _step(self) -> None:
        """One inference cycle."""
        goals = sum(
            g is not None for g in (self._goal_pose, self._goal_text, self._goal_image)
        )
        if not _MIN_GOALS <= goals <= _MAX_GOALS:
            self._fail(
                "waiting for a goal" if goals == 0 else "too many goal modalities set"
            )
            return
        if not self._history.ready:
            self._fail("no camera frames yet")
            return

        relative_goal = None
        if self._goal_pose is not None:
            frame = self._goal_pose.header.frame_id
            robot = self._base_pose_in(frame)
            if robot is None:
                self._fail(f"no TF for goal frame '{frame}'")
                return
            p = self._goal_pose.pose
            r = p.orientation
            goal = Pose2D(
                p.position.x, p.position.y, yaw_from_quaternion(r.x, r.y, r.z, r.w)
            )
            relative_goal = relative_pose(robot, goal)

        result = self._client.act(
            self._history.frames(),
            goal_text=self._goal_text,
            goal_image=self._goal_image,
            goal_pose=relative_goal,
        )
        if result is None:
            self._fail(f"policy unavailable: {self._client.last_error}")
            return

        base = self._base_pose_in(self._path_frame)
        if base is None:
            self._fail(f"no TF {self._path_frame} -> {self._base_frame}")
            return

        self._plan_pub.publish(self._build_path(base, result.waypoints))
        self._last_plan_time = self._now()
        self._last_status = (
            f"ok modality={result.modality_id} "
            f"policy={result.latency_ms:.0f}ms warm={self._history.warm}"
        )

    def _build_path(self, base: Pose2D, waypoints: list[Waypoint]) -> Path:
        """Lift body-frame waypoints into the path frame."""
        path = Path()
        path.header.frame_id = self._path_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for wp in waypoints:
            world = compose_pose(base, Pose2D(wp.x, wp.y, wp.yaw))
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = world.x
            pose.pose.position.y = world.y
            pose.pose.orientation.z = math.sin(world.yaw / 2.0)
            pose.pose.orientation.w = math.cos(world.yaw / 2.0)
            path.poses.append(pose)
        return path

    # ---------------------------------------------------------------- output ---

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _fail(self, reason: str) -> None:
        """Record why this cycle produced no plan, retracting a stale one.

        A plan still inside the staleness window is left alone: the controller
        is mid-chunk and one dropped cycle should not stop the rover. Once the
        window lapses the path is retracted exactly once, and the roboclaw
        watchdog takes over from there.
        """
        self._last_status = reason
        if self._last_plan_time is None:
            return
        if self._now() - self._last_plan_time <= self._staleness:
            return
        empty = Path()
        empty.header.frame_id = self._path_frame
        empty.header.stamp = self.get_clock().now().to_msg()
        self._plan_pub.publish(empty)
        self._last_plan_time = None
        self.get_logger().warn(f"plan retracted after {self._staleness:.2f}s: {reason}")

    def _publish_status(self) -> None:
        self._status_pub.publish(String(data=self._last_status))


def main() -> None:
    rclpy.init()
    node = VlaBridgeNode()
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
