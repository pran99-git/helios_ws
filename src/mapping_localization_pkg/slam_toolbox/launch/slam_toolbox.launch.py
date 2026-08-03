"""slam_toolbox: 2D LiDAR SLAM -- owns map -> odom and publishes /map.

Consumes /scan (Hokuyo UST-10LX, frame 'laser') and the EKF's odom -> base_link,
and produces the 2D occupancy grid plus the map -> odom correction (REP-105:
SLAM owns map -> odom; the EKF owns odom -> base_link).

Runs SEPARATELY from the sensor stack. Start the sensors + EKF first, then this:

    ros2 launch sensor_fusion bringup.launch.py
    ros2 launch mapping_localization_pkg slam_toolbox.launch.py

slam_toolbox's async node is a LIFECYCLE node that does NOT auto-activate on
this build, so it is declared as a LifecycleNode and driven through
configure -> activate automatically via event handlers (same pattern as
urg_node2.launch.py). This makes it come up 'active' (subscribed to /scan,
publishing /map + map -> odom) with no manual `ros2 lifecycle set` needed.

DO NOT run this at the same time as rtabmap.launch.py with
publish_tf_map:=true -- both would publish map -> odom and fight over it.
rtabmap defaults to publish_tf_map:=false precisely so the two can coexist,
with slam_toolbox staying the TF authority.

Save the current map with: slam_toolbox/scripts/save_map.sh <name>

Toggle with: rviz:=true
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
    pkg = get_package_share_directory('mapping_localization_pkg')
    slam_yaml = os.path.join(pkg, 'slam_toolbox', 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(pkg, 'slam_toolbox', 'rviz', 'slam.rviz')

    use_rviz = LaunchConfiguration('rviz')

    # LiDAR SLAM (lifecycle): map -> odom + /map.
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_yaml],
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
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description="Launch RViz with the slam_toolbox view."),
        slam_node,
        slam_configure,
        slam_activate,
        rviz_node,
    ])
