"""ROS adapter: hand the policy's path to Nav2's controller, but not too often.

Sits between vla_bridge and controller_server. The policy republishes several
times a second; this node forwards a path only when it actually disagrees with
the one being followed, or when the active goal has gone stale.

Nav2's MPPI is what keeps the rover off obstacles. Routing through FollowPath
rather than straight to /cmd_vel is deliberate: the controller re-optimises
against the live costmap at 20 Hz and rejects trajectories its obstacle critics
dislike, which a policy's raw waypoints know nothing about.

An empty path means "retract", and cancels the active goal.
"""

from __future__ import annotations

import rclpy
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from vla_bridge.divergence import should_reissue


class PathArbiterNode(Node):
    """Forwards policy paths to controller_server, throttled by divergence."""

    def __init__(self) -> None:
        super().__init__("path_arbiter")

        self.declare_parameter("plan_topic", "/vla/plan")
        self.declare_parameter("status_topic", "/vla/arbiter_status")
        self.declare_parameter("action_name", "follow_path")
        # These three must match navigation_pkg/config/nav2_params.yaml. A
        # mismatch is accepted by the server and then fails with
        # INVALID_CONTROLLER rather than anything more obvious.
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "goal_checker")
        self.declare_parameter("progress_checker_id", "progress_checker")
        self.declare_parameter("divergence_threshold", 0.15)  # m
        self.declare_parameter("divergence_index", 4)
        self.declare_parameter("reissue_timeout", 1.0)  # s
        self.declare_parameter("server_timeout", 5.0)  # s

        def text(name: str) -> str:
            return str(self.get_parameter(name).value)

        self._controller_id = text("controller_id")
        self._goal_checker_id = text("goal_checker_id")
        self._progress_checker_id = text("progress_checker_id")
        self._threshold = float(self.get_parameter("divergence_threshold").value)
        self._index = int(self.get_parameter("divergence_index").value)
        self._reissue_timeout = float(self.get_parameter("reissue_timeout").value)
        self._server_timeout = float(self.get_parameter("server_timeout").value)

        self._active: list[tuple[float, float]] = []
        self._goal_handle = None
        self._sent_at: float | None = None
        self._status = "idle"

        self._client = ActionClient(self, FollowPath, text("action_name"))
        self._status_pub = self.create_publisher(String, text("status_topic"), 1)
        self.create_subscription(Path, text("plan_topic"), self._on_plan, 1)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"path_arbiter up: '{text('action_name')}' via controller "
            f"'{self._controller_id}', reissue past {self._threshold} m "
            f"at waypoint {self._index}"
        )

    # ------------------------------------------------------------------ plan ---

    def _on_plan(self, msg: Path) -> None:
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not points:
            self._retract()
            return

        stale = (
            self._sent_at is None or self._now() - self._sent_at > self._reissue_timeout
        )
        if not stale and not should_reissue(
            self._active, points, self._threshold, self._index
        ):
            return

        self._send(msg, points, "stale" if stale else "diverged")

    def _send(self, path: Path, points: list[tuple[float, float]], why: str) -> None:
        """Forward a path, letting the action server preempt any active goal."""
        ready = self._client.server_is_ready() or self._client.wait_for_server(
            timeout_sec=self._server_timeout
        )
        if not ready:
            self._status = "controller_server unavailable"
            self.get_logger().warn(self._status, throttle_duration_sec=5.0)
            return

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self._controller_id
        goal.goal_checker_id = self._goal_checker_id
        goal.progress_checker_id = self._progress_checker_id

        self._client.send_goal_async(goal).add_done_callback(self._on_goal_response)
        self._active = points
        self._sent_at = self._now()
        self._status = f"following ({why}, {len(points)} poses)"

    def _retract(self) -> None:
        """Cancel the active goal so the controller stops driving."""
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._active = []
        self._sent_at = None
        self._status = "retracted"

    # ------------------------------------------------------------- callbacks ---

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self._status = "goal rejected by controller_server"
            self.get_logger().warn(self._status)
            self._sent_at = None
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result().result
        if result.error_code != FollowPath.Result.NONE:
            self._status = f"controller error {result.error_code}: {result.error_msg}"
            self.get_logger().warn(self._status)
        self._goal_handle = None

    # ---------------------------------------------------------------- output ---

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_status(self) -> None:
        self._status_pub.publish(String(data=self._status))


def main() -> None:
    rclpy.init()
    node = PathArbiterNode()
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
