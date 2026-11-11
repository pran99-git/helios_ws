#!/usr/bin/env python3
"""ROS adapter: commit to one frontier at a time and drive the rover to it.

Consumes:  /exploration/frontiers        geometry_msgs/PoseArray, map frame
           /exploration/frontier_edges   std_msgs/Float32MultiArray, metres
           /exploration/enable           std_msgs/Bool, latched
           TF map -> base_link           read only
Produces:  /vla/route_goal               geometry_msgs/PoseStamped
           /exploration/selector_status  std_msgs/String

The goal topic is a parameter. Pointed at `/vla/route_goal` the policy drives,
through the subgoal walker; pointed at `/goal_pose` the classical Nav2 stack
does, which is how the two are compared on the same exploration run.

Commits to a frontier and stays with it: re-choosing every cycle would make the
rover oscillate between two equally good openings and never reach either. The
commitment ends on arrival, on the frontier being mapped away, or on a timeout.

Publishes no transforms. All scoring lives in `goal_selection.py`, which
imports no ROS.
"""

from __future__ import annotations

import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseArray, PoseStamped
from goal_selection import (
    DEFAULT_RANGE_CAP,
    DEFAULT_REJECTION_HALF_LIFE,
    DEFAULT_REJECTION_RADIUS,
    DEFAULT_WEIGHTS,
    AttemptMonitor,
    Candidate,
    RejectionMemory,
    nearest,
    score_candidates,
)
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, String


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw of a quaternion, radians, assuming negligible roll and pitch."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class GoalSelectorNode(Node):
    """Picks a frontier, drives to it, and moves on when it is done."""

    def __init__(self) -> None:
        super().__init__("goal_selector")

        self.declare_parameter("frontiers_topic", "/exploration/frontiers")
        self.declare_parameter("edges_topic", "/exploration/frontier_edges")
        self.declare_parameter("enable_topic", "/exploration/enable")
        self.declare_parameter("status_topic", "/exploration/selector_status")
        # /vla/route_goal sends the policy through the walker; /goal_pose sends
        # the classical stack instead, which is how the two are compared.
        self.declare_parameter("goal_topic", "/vla/route_goal")
        self.declare_parameter("start_enabled", True)
        self.declare_parameter("select_period", 1.0)  # s
        # Looser than the walker's 0.25 m tolerance: arriving anywhere near a
        # frontier maps it, after which it disappears from the list anyway.
        self.declare_parameter("arrival_radius", 0.60)  # m
        # A frontier already within reach is somewhere the rover is standing,
        # not somewhere to drive. Selecting one makes it arrive instantly and
        # re-commit on the next cycle, republishing the goal every cycle and
        # resetting the walker's route before a plan can land.
        self.declare_parameter("min_select_range", 0.80)  # m
        # How far the same opening may move between detections and still count
        # as the one committed to. Detection re-derives its representative cell
        # from scratch, so it shifts as the map fills in.
        self.declare_parameter("match_radius", 1.50)  # m
        self.declare_parameter("attempt_progress", 0.30)  # m of closing required
        self.declare_parameter("attempt_window", 25.0)  # s allowed without it
        self.declare_parameter("rejection_half_life", DEFAULT_REJECTION_HALF_LIFE)  # s
        self.declare_parameter("rejection_radius", DEFAULT_REJECTION_RADIUS)  # m
        # Hold off while even the best frontier is this freshly rejected.
        # Without it, a room with one unreachable exit retries the same spot
        # every attempt_window forever instead of waiting for the decay.
        self.declare_parameter("retry_penalty", 0.50)
        self.declare_parameter("range_cap", DEFAULT_RANGE_CAP)  # m
        self.declare_parameter("weight_information", DEFAULT_WEIGHTS["information"])
        self.declare_parameter("weight_travel", DEFAULT_WEIGHTS["travel"])
        self.declare_parameter("weight_turning", DEFAULT_WEIGHTS["turning"])
        self.declare_parameter("weight_rejection", DEFAULT_WEIGHTS["rejection"])
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        def flt(name: str) -> float:
            return float(self.get_parameter(name).value)

        def text(name: str) -> str:
            return str(self.get_parameter(name).value)

        self._arrival = flt("arrival_radius")
        self._min_range = flt("min_select_range")
        self._match = flt("match_radius")
        self._range_cap = flt("range_cap")
        self._retry_penalty = flt("retry_penalty")
        self._map_frame = text("map_frame")
        self._base_frame = text("base_frame")
        self._weights = {
            "information": flt("weight_information"),
            "travel": flt("weight_travel"),
            "turning": flt("weight_turning"),
            "rejection": flt("weight_rejection"),
        }

        self._enabled = bool(self.get_parameter("start_enabled").value)
        self._candidates: list[Candidate] = []
        # Edge lengths ride a separate topic and may land either side of the
        # poses, so the two are reconciled by length rather than assumed.
        self._edges: list[float] = []
        self._target: Candidate | None = None
        self._status = "starting"
        self._explored = 0
        self._rejections = RejectionMemory(
            flt("rejection_half_life"), flt("rejection_radius")
        )
        self._attempt = AttemptMonitor(flt("attempt_progress"), flt("attempt_window"))

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._goal_pub = self.create_publisher(PoseStamped, text("goal_topic"), 1)
        self._status_pub = self.create_publisher(String, text("status_topic"), 1)

        self.create_subscription(
            PoseArray, text("frontiers_topic"), self._on_frontiers, 1
        )
        self.create_subscription(
            Float32MultiArray, text("edges_topic"), self._on_edges, 1
        )
        # Latched, so an enable published before this node started still
        # arrives and a run is not silently idle.
        self.create_subscription(
            Bool,
            text("enable_topic"),
            self._on_enable,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.create_timer(flt("select_period"), self._select)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"goal_selector up: goals to '{text('goal_topic')}', "
            f"{'enabled' if self._enabled else 'disabled, waiting for enable'}"
        )

    # --------------------------------------------------------------- inputs ---

    def _on_frontiers(self, msg: PoseArray) -> None:
        """Rebuild the candidate list, keeping any edge lengths already held.

        Edges arrive on their own topic and may land either side of this one,
        so a candidate keeps a zero edge until its length turns up rather than
        being dropped.
        """
        edges = self._edges if len(self._edges) == len(msg.poses) else []
        self._candidates = [
            Candidate(
                x=pose.position.x,
                y=pose.position.y,
                yaw=yaw_from_quaternion(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
                edge=edges[index] if edges else 0.0,
            )
            for index, pose in enumerate(msg.poses)
        ]

    def _on_edges(self, msg: Float32MultiArray) -> None:
        self._edges = list(msg.data)
        # Fill in lengths for a candidate list that arrived first.
        if len(self._edges) == len(self._candidates):
            self._candidates = [
                candidate._replace(edge=self._edges[index])
                for index, candidate in enumerate(self._candidates)
            ]

    def _on_enable(self, msg: Bool) -> None:
        if msg.data == self._enabled:
            return
        self._enabled = msg.data
        self.get_logger().info(f"exploration {'enabled' if msg.data else 'disabled'}")
        if not self._enabled:
            self._release("disabled", cancel=True)

    # ------------------------------------------------------------ machinery ---

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _robot_pose(self) -> tuple[float, float, float] | None:
        """Robot pose in the map frame, or None while TF is not ready."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quaternion(r.x, r.y, r.z, r.w)

    def _release(self, reason: str, *, cancel: bool) -> None:
        """Drop the current commitment, optionally standing the rover down.

        `cancel` publishes the non-finite sentinel the walker and vla_bridge
        both read as "no goal". Without it the rover would keep driving to a
        frontier this node has stopped tracking.
        """
        self._target = None
        self._attempt.reset()
        self._status = reason
        if cancel:
            self._publish_goal(None)

    def _publish_goal(self, target: Candidate | None) -> None:
        """Send a frontier as the route goal, or the cancel sentinel.

        Only ever called on a CHANGE of target. Republishing the same goal
        every cycle would reset the walker's own stall detection on every tick
        and it could never fire.
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        if target is None:
            msg.pose.position.x = math.nan
            msg.pose.position.y = math.nan
        else:
            msg.pose.position.x = target.x
            msg.pose.position.y = target.y
            msg.pose.orientation.z = math.sin(target.yaw / 2.0)
            msg.pose.orientation.w = math.cos(target.yaw / 2.0)
        self._goal_pub.publish(msg)

    # ------------------------------------------------------------ selection ---

    def _select(self) -> None:
        """One cycle: honour the commitment, or make a new one."""
        if not self._enabled:
            self._status = "disabled"
            return

        pose = self._robot_pose()
        if pose is None:
            self._status = f"no TF {self._map_frame} -> {self._base_frame}"
            return
        robot_x, robot_y, robot_yaw = pose
        now = self._now()
        self._rejections.prune(now)

        if self._target is not None and not self._review(robot_x, robot_y, now):
            return

        if not self._candidates:
            self._status = (
                f"exploration complete: {self._explored} frontiers visited"
                if self._explored
                else "no frontiers yet"
            )
            return

        # Standing on a frontier is not a reason to drive to it. Filtering at
        # selection only, never at review, so an approach still completes.
        reachable = [
            candidate
            for candidate in self._candidates
            if math.hypot(candidate.x - robot_x, candidate.y - robot_y)
            > self._min_range
        ]
        if not reachable:
            self._status = (
                f"{len(self._candidates)} frontiers, all within "
                f"{self._min_range:.2f} m: waiting for the map to fill in"
            )
            return

        ranked = score_candidates(
            reachable,
            robot_x,
            robot_y,
            robot_yaw,
            rejections=self._rejections,
            now=now,
            weights=self._weights,
            range_cap=self._range_cap,
        )
        best = ranked[0]
        penalty = self._rejections.penalty(best.candidate.x, best.candidate.y, now)
        if penalty > self._retry_penalty:
            # Ranking already prefers the least-rejected, so the best being
            # this tainted means they all are. Wait for the decay.
            self._status = (
                f"holding: all {len(ranked)} frontiers failed recently "
                f"(penalty {penalty:.2f})"
            )
            return

        self._target = best.candidate
        self._attempt.reset()
        self._publish_goal(self._target)
        self._status = (
            f"driving to ({self._target.x:.2f}, {self._target.y:.2f}), "
            f"{best.distance:.2f} m, {self._target.edge:.2f} m wide, "
            f"score {best.score:+.2f} of {len(ranked)}"
        )
        self.get_logger().info(self._status)

    def _review(self, robot_x: float, robot_y: float, now: float) -> bool:
        """Decide whether the current commitment still stands.

        Returns:
            True when the commitment has ended and a new one should be chosen
            this cycle, False to keep driving to the frontier already set.
        """
        target = self._target
        assert target is not None  # guarded by the caller

        distance = math.hypot(target.x - robot_x, target.y - robot_y)
        if distance <= self._arrival:
            self._explored += 1
            self.get_logger().info(f"reached frontier, {distance:.2f} m")
            self._release("arrived", cancel=False)
            return True

        # Mapped away rather than failed: driving toward it filled in the
        # unknown, which is exactly the point, so this is a success.
        match = nearest(self._candidates, target.x, target.y)
        if match is None or match[1] > self._match:
            self._explored += 1
            self.get_logger().info("frontier mapped away, choosing another")
            self._release("frontier mapped", cancel=False)
            return True

        # Keep tracking the same opening as its representative cell shifts.
        self._target = self._candidates[match[0]]

        self._attempt.update(distance, now)
        if self._attempt.expired(now):
            self._rejections.reject(target.x, target.y, now)
            self.get_logger().warn(
                f"giving up on ({target.x:.2f}, {target.y:.2f}), "
                f"no progress in {self._attempt_window():.0f} s"
            )
            self._release("attempt timed out", cancel=True)
            return True

        self._status = f"driving to ({target.x:.2f}, {target.y:.2f}), {distance:.2f} m"
        return False

    def _attempt_window(self) -> float:
        return float(self.get_parameter("attempt_window").value)

    def _publish_status(self) -> None:
        self._status_pub.publish(String(data=self._status))


def main() -> None:
    """Spin the goal selector until interrupted."""
    rclpy.init()
    node = GoalSelectorNode()
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
