"""Drive the rover with the policy trained in rl_sim.

Rebuilds the observation the policy was fitted to from live sensors, runs the
exported actor, and publishes the resulting body twist. Everything that
defines what the policy MEANS lives in `contract.py`, which the simulator
imports too, so the two cannot assemble a vector differently.

Publishes NO transforms, and no path: the policy steers directly rather than
handing a path to Nav2's controller. Nav2 stays below this node in twist_mux,
so going silent hands the rover back to it.

What this node does NOT do, on purpose: decide whether the policy is coping.
It publishes what the policy says, holds the rover still when its inputs are
unusable, and reports what it sees. Demoting to Nav2 belongs to a supervisor
above it, and overriding belongs to the joystick and the e-stop, both of
which outrank this input already.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.msg import Path as RoutePath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from vla_bridge.goal_resolution import Pose2D, yaw_from_quaternion
from vla_bridge.subgoal import project, route_length

from rl_bridge.contract import (
    LOOKAHEADS,
    MAX_RANGE,
    OBS_DIM,
    SCAN_BEAMS,
    SCAN_FOV,
    YAW_FEEDBACK_CHANNELS,
    ObservationContract,
    assemble,
    clean,
    downsample,
    fuse,
    scale_action,
    sector,
)
from rl_bridge.limits import RateLimiter, VelocityLimits
from rl_bridge.policy import OnnxPolicy


class RlBridgeNode(Node):
    """Runs the RL policy as the rover's controller."""

    def __init__(self) -> None:
        super().__init__("rl_bridge")

        self.declare_parameter("policy", "")  # path to an exported .onnx
        self.declare_parameter("scan_topic", "/scan")
        # A second scan to fuse, nearer wins. The policy was TRAINED on a
        # scan fused from the laser and the ZED's depth, because Nav2's
        # local costmap fuses both. Left empty it runs laser-only, which is
        # blind below the laser plane at 0.176 m.
        self.declare_parameter("depth_scan_topic", "")
        self.declare_parameter("odom_topic", "/odometry/filtered")  # the EKF
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic", "/rl/goal_pose")
        # Above navigation in twist_mux and below the joystick, so Nav2 is
        # the fallback and a human is still the override.
        self.declare_parameter("cmd_topic", "/cmd_vel_rl")
        self.declare_parameter("status_topic", "/rl/status")
        self.declare_parameter("planner_action", "compute_path_to_pose")
        self.declare_parameter("planner_id", "GridBased")
        # 20.83 Hz is the simulator's control period (48 ms) to the step. The
        # policy's commanded-twist channel is its own last output, so running
        # it at a different rate changes what that channel means.
        self.declare_parameter("control_rate", 20.83)  # Hz
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("goal_tolerance", 0.25)  # m, matches the walker
        self.declare_parameter("replan_period", 1.0)  # s
        self.declare_parameter("route_deviation", 1.0)  # m before replanning early
        self.declare_parameter("planner_timeout", 5.0)  # s to wait for the server
        # Stale inputs hold the rover rather than driving on them: a scan
        # this old is further out of date than the braking distance.
        self.declare_parameter("sensor_timeout", 0.30)  # s
        # A leash for the first live runs. Scales what is PUBLISHED, not what
        # the policy believes it asked for: the gap then appears in the
        # measured twist, which is the slip channel the observation already
        # carries and was trained with.
        self.declare_parameter("speed_scale", 1.0)
        # Zeroes the commanded and measured yaw-rate channels. The policy's
        # response to them is a learned D-term through the loop's own lag,
        # which sustains a limit cycle; blinding them cut the oscillation
        # from 3.83 to 2.43 sign reversals/s at no cost in success.
        self.declare_parameter("blind_yaw_feedback", True)

        def flt(name: str) -> float:
            return float(self.get_parameter(name).value)

        def text(name: str) -> str:
            return str(self.get_parameter(name).value)

        self._map_frame = text("map_frame")
        self._base_frame = text("base_frame")
        self._planner_id = text("planner_id")
        self._goal_tolerance = flt("goal_tolerance")
        self._replan_period = flt("replan_period")
        self._deviation = flt("route_deviation")
        self._sensor_timeout = flt("sensor_timeout")
        self._speed_scale = flt("speed_scale")

        self._limits = VelocityLimits()
        self._dt = 1.0 / flt("control_rate")
        self._rate = RateLimiter(self._limits, self._dt)

        policy_path = text("policy")
        if not policy_path:
            raise RuntimeError("the 'policy' parameter must name an exported .onnx")
        self._policy = OnnxPolicy(
            policy_path,
            dim=OBS_DIM,
            blind_channels=(
                YAW_FEEDBACK_CHANNELS
                if bool(self.get_parameter("blind_yaw_feedback").value)
                else ()
            ),
        )

        self._scan: np.ndarray | None = None
        self._scan_at = 0.0
        self._depth: np.ndarray | None = None
        self._depth_at = 0.0
        self._measured = np.zeros(3)
        self._odom_at = 0.0
        # Needs the map's extent, which normalises the remaining-route
        # channel, so there is no contract until /map arrives.
        self._contract: ObservationContract | None = None
        self._goal: PoseStamped | None = None
        self._route: list[Pose2D] = []
        self._route_length = 0.0
        self._route_at: float | None = None
        self._planning = False
        self._driving = False
        self._status = "idle"
        self._note = ""
        self._bearing = 0.0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._cmd_pub = self.create_publisher(Twist, text("cmd_topic"), 1)
        self._status_pub = self.create_publisher(String, text("status_topic"), 1)
        self._route_pub = self.create_publisher(RoutePath, "/rl/route", 1)

        # Depth 1 and the default reliability: urg_node2 publishes the scan
        # RELIABLE, so this matches it, and a queue of one keeps the freshest
        # scan rather than a backlog the control loop would drive on late.
        self.create_subscription(LaserScan, text("scan_topic"), self._on_scan, 1)
        depth_topic = text("depth_scan_topic")
        if depth_topic:
            self.create_subscription(LaserScan, depth_topic, self._on_depth, 1)
        else:
            self.get_logger().warn(
                "no depth_scan_topic: the policy was trained on a laser+ZED "
                "fused scan and will run blind below the laser plane"
            )
        self.create_subscription(Odometry, text("odom_topic"), self._on_odom, 1)
        # The map server latches one message, so a late subscriber needs
        # transient-local durability to ever see it.
        self.create_subscription(
            OccupancyGrid,
            text("map_topic"),
            self._on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(PoseStamped, text("goal_topic"), self._on_goal, 1)

        self._planner = ActionClient(self, ComputePathToPose, text("planner_action"))
        if not self._planner.wait_for_server(timeout_sec=flt("planner_timeout")):
            self.get_logger().warn(
                f"planner action '{text('planner_action')}' not available; "
                "routes cannot be computed until it is"
            )

        self.create_timer(self._dt, self._tick)
        self.create_timer(1.0, self._publish_status)
        blinded = self._policy.blinded
        self.get_logger().info(
            f"rl_bridge up: {policy_path.split('/')[-1]} at "
            f"{1.0 / self._dt:.1f} Hz, speed x{self._speed_scale:.2f}, "
            f"{'blinding channels ' + str(blinded) if blinded else 'all channels live'}"
        )

    # ----------------------------------------------------------- the clock ---

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------------------------------------------------- the senses ---

    def _reduce(self, msg: LaserScan) -> np.ndarray | None:
        """Turn a full-resolution scan into the policy's beams.

        Cleaned before reduction, never after: the minimum of a bin holding a
        no-return sentinel or a NaN is not a distance.

        Args:
            msg: One scan.

        Returns:
            `SCAN_BEAMS` ranges in metres, or None if this scan cannot cover
            the policy's sweep.
        """
        try:
            swept = sector(
                clean(np.asarray(msg.ranges, dtype=np.float64), MAX_RANGE),
                float(msg.angle_min),
                float(msg.angle_increment),
                SCAN_FOV,
            )
            return downsample(swept, SCAN_BEAMS)
        except ValueError as error:
            self._note = f"unusable scan: {error}"
            return None

    def _on_scan(self, msg: LaserScan) -> None:
        reduced = self._reduce(msg)
        if reduced is not None:
            self._scan = reduced
            self._scan_at = self._now()

    def _on_depth(self, msg: LaserScan) -> None:
        reduced = self._reduce(msg)
        if reduced is not None:
            self._depth = reduced
            self._depth_at = self._now()

    def _on_odom(self, msg: Odometry) -> None:
        # Odometry carries its twist in child_frame_id, which is base_link:
        # already the body frame the observation and /cmd_vel use.
        self._measured = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.angular.z,
            ]
        )
        self._odom_at = self._now()

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Take the world's extent, which normalises the remaining route.

        The simulator divides the route left to run by the diagonal of the
        world it compiled, so the robot must divide by the diagonal of the
        same map or that channel means something else.
        """
        width = msg.info.width * msg.info.resolution
        height = msg.info.height * msg.info.resolution
        diagonal = float(math.hypot(width, height))
        if diagonal <= 0.0:
            self.get_logger().warn("ignoring a map with no extent")
            return
        self._contract = ObservationContract(
            diagonal=diagonal,
            beams=SCAN_BEAMS,
            fov=SCAN_FOV,
            max_range=MAX_RANGE,
            lookaheads=LOOKAHEADS,
            limits=self._limits,
        )
        self.get_logger().info(
            f"map {width:.1f} x {height:.1f} m, diagonal {diagonal:.2f} m"
        )

    def _robot_pose(self) -> Pose2D | None:
        """The pose the robot BELIEVES it has, in the route's frame."""
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
        ) as error:
            self._note = f"no {self._map_frame} -> {self._base_frame}: {error}"
            return None
        t = tf.transform
        return Pose2D(
            t.translation.x,
            t.translation.y,
            yaw_from_quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
        )

    # ----------------------------------------------------------- the goals ---

    def _on_goal(self, msg: PoseStamped) -> None:
        # A non-finite position means "stand down", the same sentinel
        # subgoal_walker_node accepts on its own goal topic.
        if not (
            math.isfinite(msg.pose.position.x) and math.isfinite(msg.pose.position.y)
        ):
            self._abandon("cancelled")
            return
        self._goal = msg
        self._route = []
        self._route_at = None
        self._note = ""
        self.get_logger().info(
            f"goal ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) "
            f"in '{msg.header.frame_id}'"
        )
        self._request_route()

    def _abandon(self, reason: str) -> None:
        """Drop the goal and stop the rover."""
        self._goal = None
        self._route = []
        self._route_at = None
        self._status = reason
        self._note = reason
        self._halt()
        self.get_logger().info(f"standing down: {reason}")

    # ----------------------------------------------------------- the route ---

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
            self._note = "route rejected by planner"
            return
        handle.get_result_async().add_done_callback(self._on_route_result)

    def _on_route_result(self, future) -> None:
        self._planning = False
        result = future.result().result
        if result.error_code != ComputePathToPose.Result.NONE:
            self._note = f"planner error {result.error_code}: {result.error_msg}"
            self.get_logger().warn(self._note)
            return
        poses = result.path.poses
        if len(poses) < 2:
            self._note = "route too short to follow"
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
        self._route_length = route_length(self._route)
        self._route_at = self._now()
        self._note = ""
        self._route_pub.publish(result.path)

    # ------------------------------------------------------------ the tick ---

    def _halt(self) -> None:
        """Hold the rover still, and keep holding it.

        Publishing zeros rather than going silent is deliberate while a goal
        is live: this input outranks Nav2, so falling silent would hand a
        moving rover to whatever Nav2 last wanted. The rate limiter is reset
        with it, because the policy's commanded-twist channel has to agree
        with what the wheels were actually told.
        """
        self._rate.reset()
        self._cmd_pub.publish(Twist())
        self._driving = False

    def _tick(self) -> None:
        if self._goal is None:
            # Nothing of ours to say: stay off the topic so the rest of the
            # stack is untouched, after one zero to cancel our last command.
            if self._driving:
                self._halt()
            return

        if self._contract is None:
            self._status = "waiting for the map"
            self._halt()
            return

        now = self._now()
        if self._scan is None or now - self._scan_at > self._sensor_timeout:
            self._status = "scan stale"
            self._halt()
            return
        if now - self._odom_at > self._sensor_timeout:
            self._status = "odometry stale"
            self._halt()
            return

        pose = self._robot_pose()
        if pose is None:
            self._status = "pose unavailable"
            self._halt()
            return

        if self._reached(pose):
            self._abandon("goal reached")
            return

        if not self._route:
            self._status = "no route"
            self._request_route()
            self._halt()
            return

        projection = project(self._route, pose.x, pose.y)
        age = now - (self._route_at or now)
        if age > self._replan_period or abs(projection.offset) > self._deviation:
            self._request_route()

        scan = self._scan
        # Either sensor seeing an obstacle is enough, as in a costmap cell.
        if self._depth is not None and now - self._depth_at <= self._sensor_timeout:
            scan = fuse(scan, self._depth)

        observation = assemble(
            self._contract,
            scan,
            (pose.x, pose.y, pose.yaw),
            self._route,
            self._route_length,
            # Last command actually emitted, which is what the policy saw as
            # its own output while this scan and this odometry were measured.
            self._rate.current,
            self._measured,
        )
        self._bearing = observation.carrot_bearing

        action = self._policy.act(observation.vector)
        twist = self._rate.step(scale_action(action, self._limits))

        command = Twist()
        command.linear.x = float(twist[0] * self._speed_scale)
        command.linear.y = float(twist[1] * self._speed_scale)
        command.angular.z = float(twist[2] * self._speed_scale)
        self._cmd_pub.publish(command)
        self._driving = True
        self._status = "driving"

    def _reached(self, pose: Pose2D) -> bool:
        """Whether the rover is within tolerance of the goal position."""
        if self._goal is None:
            return False
        return (
            math.hypot(
                self._goal.pose.position.x - pose.x,
                self._goal.pose.position.y - pose.y,
            )
            <= self._goal_tolerance
        )

    # ---------------------------------------------------------- the report ---

    def _publish_status(self) -> None:
        """Say what the policy is seeing, once a second.

        The first live runs are judged on this line, so it carries the
        quantities that explain a command rather than just the verdict.
        """
        clearance = float(self._scan.min()) if self._scan is not None else float("nan")
        parts = [
            self._status,
            f"clearance {clearance:.2f} m",
            f"carrot {math.degrees(self._bearing):+.0f} deg",
        ]
        if self._route:
            parts.append(f"route {self._route_length:.1f} m")
        if self._note:
            parts.append(self._note)
        self._status_pub.publish(String(data=" | ".join(parts)))

    def shutdown_halt(self) -> None:
        """Stop the rover on the way out.

        Guarded rather than wrapped in try/except: publishing after the
        context is gone is the only failure mode, and rclpy.ok() detects it.
        """
        if rclpy.ok():
            self._cmd_pub.publish(Twist())


def main() -> None:
    # Match amcl_recovery_node: rclpy's own handler tears the context down
    # before an exception surfaces, so shutdown is handled explicitly.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RlBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl+C here must not leave the rover holding the last command.
        node.shutdown_halt()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
