"""Turn a distant goal into a stream of short ones the policy can handle.

Asks planner_server for a route to the final goal, then publishes a carrot a
fixed distance along it as the policy's pose goal. Route choice stays with the
planner, which has the map; the policy only ever sees the short-goal case it
handles well.

Publishes NO transforms. All geometry lives in `subgoal.py`, which imports no
ROS, so it is tested without a ROS context.
"""

from __future__ import annotations

import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from vla_bridge.goal_resolution import Pose2D, yaw_from_quaternion
from vla_bridge.subgoal import (
    DEFAULT_LOOKAHEAD,
    DEFAULT_TURN_FIRST,
    ProgressMonitor,
    bearing_to,
    carrot,
    project,
    remaining,
    turn_in_place,
)


class SubgoalWalkerNode(Node):
    """Walks a planned route, emitting carrots for the policy."""

    def __init__(self) -> None:
        super().__init__("subgoal_walker")

        self.declare_parameter("route_goal_topic", "/vla/route_goal")
        self.declare_parameter("subgoal_topic", "/vla/goal_pose")
        self.declare_parameter("route_topic", "/vla/route")
        self.declare_parameter("status_topic", "/vla/walker_status")
        self.declare_parameter("planner_action", "compute_path_to_pose")
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("lookahead_distance", DEFAULT_LOOKAHEAD)  # m
        # The policy only proposes forward motion, so a carrot behind the rover
        # yields a chunk pointing the wrong way and nothing downstream corrects
        # it. Above this bearing the walker turns the rover itself.
        self.declare_parameter("turn_first_angle", DEFAULT_TURN_FIRST)  # rad
        self.declare_parameter("turn_rate", 0.60)  # rad/s
        self.declare_parameter("turn_accel", 2.00)  # rad/s^2
        self.declare_parameter("turn_cmd_topic", "/cmd_vel_walker")
        # Below this carrot range the bearing is mostly pose noise, and the
        # translation left is shorter than the turn would cost.
        self.declare_parameter("turn_min_range", 0.50)  # m
        self.declare_parameter("tick_rate", 5.0)  # Hz
        self.declare_parameter("replan_period", 1.0)  # s
        self.declare_parameter("route_deviation", 1.0)  # m before replanning early
        self.declare_parameter("goal_tolerance", 0.25)  # m, looser than Nav2's
        self.declare_parameter("stuck_progress", 0.15)  # m along the route
        self.declare_parameter("stuck_window", 8.0)  # s without that progress
        self.declare_parameter("planner_timeout", 5.0)  # s to wait for the server
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        def flt(name: str) -> float:
            return float(self.get_parameter(name).value)

        def text(name: str) -> str:
            return str(self.get_parameter(name).value)

        self._lookahead = flt("lookahead_distance")
        self._turn_first = flt("turn_first_angle")
        self._turn_rate = flt("turn_rate")
        self._turn_accel = flt("turn_accel")
        self._turn_min_range = flt("turn_min_range")
        self._turning = False
        self._turn_wz = 0.0
        self._tick_dt = 1.0 / flt("tick_rate")
        self._replan_period = flt("replan_period")
        self._deviation = flt("route_deviation")
        self._goal_tolerance = flt("goal_tolerance")
        self._map_frame = text("map_frame")
        self._base_frame = text("base_frame")
        self._planner_id = text("planner_id")

        self._goal: PoseStamped | None = None
        self._route: list[Pose2D] = []
        self._route_at: float | None = None
        self._planning = False
        self._status = "idle"
        # True while the bridge holds a carrot from us. Every tick either
        # publishes one or clears it: a stale carrot is pursued forever.
        self._emitting = False
        self._monitor = ProgressMonitor(flt("stuck_progress"), flt("stuck_window"))

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._subgoal_pub = self.create_publisher(PoseStamped, text("subgoal_topic"), 1)
        self._route_pub = self.create_publisher(Path, text("route_topic"), 1)
        self._status_pub = self.create_publisher(String, text("status_topic"), 1)
        # Same twist_mux input the controller uses, so the joystick still
        # outranks this and the e-stop still overrides it.
        self._turn_pub = self.create_publisher(Twist, text("turn_cmd_topic"), 1)
        self.create_subscription(
            PoseStamped, text("route_goal_topic"), self._on_route_goal, 1
        )

        self._planner = ActionClient(self, ComputePathToPose, text("planner_action"))
        if not self._planner.wait_for_server(timeout_sec=flt("planner_timeout")):
            self.get_logger().warn(
                f"planner action '{text('planner_action')}' not available; "
                "routes cannot be computed until it is"
            )

        self.create_timer(1.0 / flt("tick_rate"), self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"subgoal_walker up: carrot {self._lookahead:.2f} m along "
            f"'{self._planner_id}' routes, replan every {self._replan_period:.1f}s"
        )

    # --------------------------------------------------------------- goals ---

    def _on_route_goal(self, msg: PoseStamped) -> None:
        self._goal = msg
        self._route = []
        self._route_at = None
        self._monitor.reset()
        self.get_logger().info(
            f"route goal ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) "
            f"in '{msg.header.frame_id}'"
        )
        self._request_route()

    def _abandon(self, reason: str) -> None:
        """Drop the goal, stop emitting carrots, and clear the one already sent.

        The retraction itself is handled by _tick, which clears whenever a
        cycle produces no carrot.
        """
        self._goal = None
        self._route = []
        self._route_at = None
        self._monitor.reset()
        self._status = reason
        if self._turning:
            self._turning = False
            self._turn_wz = 0.0
            self._publish_turn(0.0)
        self.get_logger().warn(f"route abandoned: {reason}")

    # ---------------------------------------------------------------- route ---

    def _request_route(self) -> None:
        if self._goal is None or self._planning:
            return
        if not self._planner.server_is_ready():
            self._status = "planner unavailable"
            return
        self._planning = True
        request = ComputePathToPose.Goal()
        request.goal = self._goal
        request.planner_id = self._planner_id
        request.use_start = False  # plan from the robot's live pose
        self._planner.send_goal_async(request).add_done_callback(
            self._on_route_response
        )

    def _on_route_response(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self._planning = False
            self._status = "route rejected by planner"
            return
        handle.get_result_async().add_done_callback(self._on_route_result)

    def _on_route_result(self, future) -> None:
        self._planning = False
        result = future.result().result
        if result.error_code != ComputePathToPose.Result.NONE:
            self._status = f"planner error {result.error_code}: {result.error_msg}"
            self.get_logger().warn(self._status)
            return
        poses = result.path.poses
        if len(poses) < 2:
            self._status = "route too short to follow"
            return
        self._route = [
            Pose2D(
                p.pose.position.x,
                p.pose.position.y,
                yaw_from_quaternion(
                    p.pose.orientation.x,
                    p.pose.orientation.y,
                    p.pose.orientation.z,
                    p.pose.orientation.w,
                ),
            )
            for p in poses
        ]
        self._route_at = self._now()
        self._route_pub.publish(result.path)

    # ----------------------------------------------------------------- tick ---

    def _robot_pose(self) -> Pose2D | None:
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
        return Pose2D(t.x, t.y, yaw_from_quaternion(r.x, r.y, r.z, r.w))

    def _walk(self) -> bool:
        if self._goal is None:
            self._status = "idle"
            return False

        robot = self._robot_pose()
        if robot is None:
            self._status = f"no TF {self._map_frame} -> {self._base_frame}"
            return False

        if not self._route:
            self._request_route()
            self._status = "waiting for a route"
            return False

        now = self._now()
        projection = project(self._route, robot.x, robot.y)
        left = remaining(self._route, robot.x, robot.y)

        if left <= self._goal_tolerance:
            self.get_logger().info(f"route complete, {left:.2f} m remaining")
            self._abandon("arrived")
            return False

        self._monitor.update(projection.arc, now)
        if self._monitor.stuck(now):
            self._abandon("stuck: no progress along the route")
            return False

        # Replan on a timer, or early if the rover has strayed off the route.
        stale = self._route_at is None or now - self._route_at > self._replan_period
        if stale or projection.offset > self._deviation:
            self._request_route()

        target = carrot(self._route, robot.x, robot.y, self._lookahead)

        # Turn to face the carrot before handing it over. RotationShimController
        # cannot do this: it sits downstream of the policy and aligns to the
        # chunk, which always reads as straight ahead.
        bearing = bearing_to(robot, target)
        carrot_range = math.hypot(target.x - robot.x, target.y - robot.y)
        far_enough = carrot_range > self._turn_min_range
        if far_enough and (
            abs(bearing) > self._turn_first
            or (self._turning and abs(bearing) > self._turn_first * 0.4)
        ):
            self._turning = True
            self._turn_wz = turn_in_place(
                bearing,
                self._turn_rate,
                self._turn_accel,
                self._turn_wz,
                self._tick_dt,
            )
            self._publish_turn(self._turn_wz)
            self._status = (
                f"turning to face the carrot ({math.degrees(bearing):+.0f} deg, "
                f"{left:.2f} m left)"
            )
            # Still an active goal, but the policy must not drive meanwhile.
            return False

        if self._turning:
            self._turning = False
            self._turn_wz = 0.0
            self._publish_turn(0.0)

        self._publish_subgoal(target)
        self._status = (
            f"following ({left:.2f} m left, {projection.offset:.2f} m off route)"
        )
        return True

    def _tick(self) -> None:
        """One cycle. Publishes a carrot, or clears the one already out there.

        The walker has many ways to stop producing carrots: no TF, no route, a
        degenerate route, a planner error, arrival, or a stuck detection. The
        bridge keeps its last pose goal indefinitely, so EVERY one of those
        must retract it. Enforcing that here rather than at each exit means a
        path added later cannot forget.
        """
        emitting = self._walk()
        if not emitting and self._emitting:
            self._clear_subgoal()
        self._emitting = emitting

    def _publish_turn(self, wz: float) -> None:
        """Command a pure rotation. Hysteresis in _walk stops it chattering."""
        msg = Twist()
        msg.angular.z = wz
        self._turn_pub.publish(msg)

    def _clear_subgoal(self) -> None:
        """Tell the bridge to drop its pose goal.

        A non-finite position is the bridge's agreed "no goal" sentinel; see
        vla_bridge_node._on_goal_pose.
        """
        msg = PoseStamped()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = math.nan
        msg.pose.position.y = math.nan
        msg.pose.orientation.w = 1.0
        self._subgoal_pub.publish(msg)

    def _publish_subgoal(self, target: Pose2D) -> None:
        msg = PoseStamped()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = target.x
        msg.pose.position.y = target.y
        msg.pose.orientation.z = math.sin(target.yaw / 2.0)
        msg.pose.orientation.w = math.cos(target.yaw / 2.0)
        self._subgoal_pub.publish(msg)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_status(self) -> None:
        self._status_pub.publish(String(data=self._status))

    def shutdown_clear(self) -> None:
        """Clear the bridge's goal on the way out.

        Guarded rather than wrapped in try/except: publishing after the
        context is gone is the only failure mode, and rclpy.ok() detects it.
        """
        if rclpy.ok():
            self._clear_subgoal()


def main() -> None:
    # Match amcl_recovery_node: rclpy's own handler tears the context down
    # before an exception surfaces, so shutdown is handled explicitly.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SubgoalWalkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl+C here must not leave the policy chasing the last carrot.
        node.shutdown_clear()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
