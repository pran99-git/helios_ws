#!/usr/bin/env python3
"""ROS adapter: turn the live occupancy grid into a list of frontier poses.

Consumes:  /map                            nav_msgs/OccupancyGrid, latched
Produces:  /exploration/frontiers          geometry_msgs/PoseArray, map frame
           /exploration/frontier_edges     std_msgs/Float32MultiArray, metres
           /exploration/frontier_markers   visualization_msgs/MarkerArray
           /exploration/frontier_status    std_msgs/String

The two data topics are published together and share an order, so index `i` of
`frontier_edges` is the edge length of pose `i`. A consumer that only needs the
biggest frontier can take pose 0 and ignore the edges entirely.

Publishes no transforms and reads none: frontiers are already in the map frame,
which is the grid's own frame. All geometry lives in `frontier_detection.py`,
which imports no ROS.
"""

from __future__ import annotations

import math

import rclpy
from frontier_detection import (
    DEFAULT_MIN_CLEARANCE,
    DEFAULT_MIN_EDGE,
    Frontier,
    GridInfo,
    detect_frontiers,
)
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray


def pose_from(frontier: Frontier) -> Pose:
    """Convert a frontier to a Pose, with its yaw as a quaternion.

    A yaw-only rotation about z is `(0, 0, sin(yaw/2), cos(yaw/2))`.
    """
    pose = Pose()
    pose.position.x = frontier.x
    pose.position.y = frontier.y
    pose.orientation.z = math.sin(frontier.yaw / 2.0)
    pose.orientation.w = math.cos(frontier.yaw / 2.0)
    return pose


class FrontierNode(Node):
    """Detects frontiers on a timer and publishes them, largest edge first."""

    def __init__(self) -> None:
        super().__init__("frontier_detector")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("frontiers_topic", "/exploration/frontiers")
        self.declare_parameter("edges_topic", "/exploration/frontier_edges")
        self.declare_parameter("markers_topic", "/exploration/frontier_markers")
        self.declare_parameter("status_topic", "/exploration/frontier_status")
        # Matches slam_toolbox's map_update_interval: detecting faster than the
        # map changes just re-reads the same grid.
        self.declare_parameter("detect_period", 2.0)  # s
        self.declare_parameter("min_edge", DEFAULT_MIN_EDGE)  # m
        self.declare_parameter("min_clearance", DEFAULT_MIN_CLEARANCE)  # m
        # Drop a frontier the rover could not stand on. These are mostly the
        # ragged end of the laser sweep against a wall, and chasing one costs
        # a whole attempt timeout.
        self.declare_parameter("require_clearance", True)
        # Guard against a pathological map: detection is O(cells) and runs on
        # the same CPU as the policy and the ZED.
        self.declare_parameter("max_frontiers", 40)

        def flt(name: str) -> float:
            return float(self.get_parameter(name).value)

        def text(name: str) -> str:
            return str(self.get_parameter(name).value)

        self._min_edge = flt("min_edge")
        self._min_clearance = flt("min_clearance")
        self._require_clearance = bool(self.get_parameter("require_clearance").value)
        self._max_frontiers = int(self.get_parameter("max_frontiers").value)

        self._grid: OccupancyGrid | None = None
        self._status = "waiting for a map"

        self._frontiers_pub = self.create_publisher(
            PoseArray, text("frontiers_topic"), 1
        )
        self._edges_pub = self.create_publisher(
            Float32MultiArray, text("edges_topic"), 1
        )
        self._markers_pub = self.create_publisher(MarkerArray, text("markers_topic"), 1)
        self._status_pub = self.create_publisher(String, text("status_topic"), 1)

        # The mapper latches /map, so a late subscriber needs transient_local
        # to receive the grid already published rather than waiting for the
        # next update.
        self.create_subscription(
            OccupancyGrid,
            text("map_topic"),
            self._on_map,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.create_timer(flt("detect_period"), self._detect)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"frontier_detector up: keeping frontiers over {self._min_edge:.2f} m "
            f"of edge with {self._min_clearance:.2f} m clearance"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._grid = msg

    def _detect(self) -> None:
        """Detect on the newest grid and publish, even when empty.

        An empty PoseArray is meaningful: it says the reachable space is fully
        mapped. Skipping the publish would leave a consumer acting on a stale
        list of frontiers that no longer exist.
        """
        grid = self._grid
        if grid is None:
            return

        info = GridInfo(
            resolution=grid.info.resolution,
            origin_x=grid.info.origin.position.x,
            origin_y=grid.info.origin.position.y,
            width=grid.info.width,
            height=grid.info.height,
        )
        try:
            frontiers = detect_frontiers(
                grid.data,
                info,
                min_edge=self._min_edge,
                min_clearance=self._min_clearance,
                require_clearance=self._require_clearance,
            )
        except ValueError as exc:
            self._status = f"bad grid: {exc}"
            self.get_logger().warn(self._status)
            return

        frontiers = frontiers[: self._max_frontiers]
        self._publish(frontiers, grid)
        self._status = (
            f"{len(frontiers)} frontiers, largest {frontiers[0].edge:.2f} m"
            if frontiers
            else "no frontiers: reachable space is mapped"
        )

    def _publish(self, frontiers: list[Frontier], grid: OccupancyGrid) -> None:
        """Publish the poses, their edge lengths and the RViz markers."""
        stamp = self.get_clock().now().to_msg()
        frame = grid.header.frame_id or "map"

        poses = PoseArray()
        poses.header.stamp = stamp
        poses.header.frame_id = frame
        poses.poses = [pose_from(f) for f in frontiers]
        self._frontiers_pub.publish(poses)

        edges = Float32MultiArray()
        edges.data = [f.edge for f in frontiers]
        self._edges_pub.publish(edges)

        self._markers_pub.publish(self._markers(frontiers, stamp, frame))

    def _markers(self, frontiers: list[Frontier], stamp, frame: str) -> MarkerArray:
        """Build arrows pointing the way each frontier faces.

        DELETEALL leads the array because the previous publish may have held
        more markers than this one, and RViz keeps any it is not told to drop.
        """
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        for index, frontier in enumerate(frontiers):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = frame
            marker.ns = "frontiers"
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose_from(frontier)
            # Longer arrow for a wider opening, so the ranking is visible.
            marker.scale.x = 0.25 + min(frontier.edge, 2.0) * 0.25
            marker.scale.y = 0.06
            marker.scale.z = 0.06
            # Green for the largest, fading to yellow down the list.
            share = index / max(1, len(frontiers) - 1)
            marker.color.r = float(share)
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.9
            array.markers.append(marker)
        return array

    def _publish_status(self) -> None:
        self._status_pub.publish(String(data=self._status))


def main() -> None:
    """Spin the frontier detector until interrupted."""
    rclpy.init()
    node = FrontierNode()
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
