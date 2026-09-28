"""ROS telemetry and opt-in catalog process management; no arbitrary commands."""
import threading
from pathlib import Path

from .operations import OperationError, OperationManager
from .processes import ProcessBackend

from .server import TelemetryServer, arguments, token_from_environment
from .state import TelemetryState


def main():
    # Keep ROS imports out of state/server so the fixture and tests run anywhere.
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    args, ros_args = arguments("Helios read-only ROS telemetry gateway")
    token = token_from_environment()
    workspace = Path(args.workspace).expanduser().resolve()
    if args.enable_commands:
        if not args.exclusive_stack_control:
            raise ValueError("Commands require --exclusive-stack-control and an otherwise stopped ROS stack")
        if not (workspace / "install/setup.bash").is_file() or not (workspace / "src/mapping_localization_pkg").is_dir():
            raise ValueError("--workspace must identify a built trusted Helios workspace")
    state = TelemetryState()
    rclpy.init(args=ros_args)

    class GatewayNode(Node):
        def __init__(self):
            super().__init__("mobile_gateway")
            self.declare_parameter("odometry_topic", "/odometry/filtered")
            self.declare_parameter("scan_topic", "/scan")
            self.create_subscription(Odometry, self.get_parameter("odometry_topic").value,
                                     self.on_odometry, qos_profile_sensor_data)
            self.create_subscription(LaserScan, self.get_parameter("scan_topic").value,
                                     self.on_scan, qos_profile_sensor_data)

        def on_odometry(self, message):
            pose, twist = message.pose.pose, message.twist.twist
            q = pose.orientation
            state.odometry(frame_id=message.header.frame_id,
                           child_frame_id=message.child_frame_id,
                           x=pose.position.x, y=pose.position.y,
                           qx=q.x, qy=q.y, qz=q.z, qw=q.w,
                           linear_x=twist.linear.x, linear_y=twist.linear.y,
                           angular_z=twist.angular.z)

        def on_scan(self, message):
            state.scan(frame_id=message.header.frame_id, ranges=message.ranges,
                       range_min=message.range_min, range_max=message.range_max)

    node = None
    server = None
    worker = None
    operations = None
    try:
        node = GatewayNode()
        def external_guard(owned, operation_id):
            # Discovery is an additional check, not proof of exclusivity. The
            # operator must stop all externally launched stack processes first.
            groups = {
                'motors': {'roboclaw_driver'},
                'sensors': {'ekf_filter_node', 'wheel_odometry', 'zed_odom_covariance', 'urg_node2', 'zed_node', 'robot_state_publisher'},
                'joystick': {'joy_node', 'teleop_joy'},
                'slam_mapping': {'slam_toolbox'}, 'slam_localization': {'slam_toolbox'},
                'rtab_mapping': {'rtabmap'}, 'rtab_localization': {'rtabmap'},
                'amcl': {'amcl', 'map_server', 'amcl_recovery'},
                'navigation': {'controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'velocity_smoother'},
            }
            allowed = set().union(*(groups.get(key, set()) for key in owned)) if owned else set()
            known = set().union(*groups.values())
            names = node.get_node_names()
            foreign = (set(names) & known) - allowed
            foreign.update(name for name in known if names.count(name) > 1)
            foreign.update(info.node_name for info in node.get_publishers_info_by_topic('/cmd_vel') if info.node_name not in allowed)
            if foreign:
                raise OperationError(409, 'external_stack', 'Externally managed or duplicate ROS nodes detected; stop them outside this gateway: ' + ', '.join(sorted(foreign)))
        operations = OperationManager(ProcessBackend(cwd=str(workspace)), workspace,
                                      enabled=args.enable_commands, external_guard=external_guard)
        server = TelemetryServer((args.host, args.port), state, token, operations=operations)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        node.get_logger().info("Mobile gateway started; managed commands " + ("enabled" if args.enable_commands else "disabled"))
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
        if worker is not None:
            worker.join(timeout=5)
        if operations is not None:
            operations.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
