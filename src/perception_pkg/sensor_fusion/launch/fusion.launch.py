"""Core fusion: robot_localization EKF (odom->base_link) + slam_toolbox (map->odom).

Assumes the sensor TOPICS are already being published (/wheel/odometry,
/zed/zed_node/odom, /scan) and that nothing else publishes odom->base_link.
Use bringup.launch.py to start the whole stack including drivers.

slam_toolbox's async node is a LIFECYCLE node that does NOT auto-activate on this
build, so it is declared as a LifecycleNode and driven through
configure -> activate automatically via event handlers (same pattern as
urg_node2.launch.py). This makes it come up 'active' (subscribed to /scan,
publishing /map + map->odom) with no manual `ros2 lifecycle set` needed.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = get_package_share_directory('sensor_fusion')
    ekf_yaml = os.path.join(pkg, 'config', 'ekf.yaml')
    slam_yaml = os.path.join(pkg, 'config', 'slam_toolbox.yaml')

    use_slam = LaunchConfiguration('slam')

    # Local EKF: sole owner of odom -> base_link.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml],
    )

    # LiDAR SLAM (lifecycle): map -> odom + map.
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_yaml],
        condition=IfCondition(use_slam),
    )

    # On start -> configure (unconfigured -> inactive).
    slam_configure = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )),
            ],
        ),
        condition=IfCondition(use_slam),
    )

    # On reaching 'inactive' after configuring -> activate (inactive -> active).
    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(use_slam),
    )

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='true',
                              description='Run slam_toolbox (LiDAR map -> odom)'),
        ekf_node,
        slam_node,
        slam_configure,
        slam_activate,
    ])
