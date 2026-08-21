"""Hokuyo UST-10LX (urg_node2) driver — workspace-owned launch.

Replaces the submodule's own urg_node2.launch.py. That file hardcodes reading
config/params_ether.yaml out of urg_node2's share/ directory and exposes no
argument to override it, so the only way to change a LiDAR parameter was to
edit tracked content inside the pinned submodule -- where `git submodule
update` silently reverts it. This file declares the same LifecycleNode against
custom_config's own config/urg_node2.yaml instead, leaving the submodule
pristine and used purely as a driver binary.

This package sits beside the submodule in LiDAR/ so that everything
laser-related is in one place, mirroring Camera/custom_covariance -- but it is
OUR content, outside the submodule boundary, which is the whole point.

Behaviour is otherwise identical to upstream's: the node is a LIFECYCLE node
that does not auto-activate, so it is driven configure -> activate via event
handlers (the same pattern slam_toolbox.launch.py uses).

  ros2 launch custom_config lidar.launch.py
  ros2 launch custom_config lidar.launch.py scan_topic:=/scan_raw
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = get_package_share_directory('custom_config')
    default_config = os.path.join(pkg, 'config', 'urg_node2.yaml')

    auto_start = LaunchConfiguration('auto_start')

    # node_name is declared here AND passed explicitly by any caller, because
    # launch configurations leak between sibling includes: bringup.launch.py
    # includes the ZED first, which declares its own 'node_name', and a
    # DeclareLaunchArgument only fills in a value that is not already set.
    # That is how this driver once came up as '/zed_node'.
    lidar_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name'),
        namespace='',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[('scan', LaunchConfiguration('scan_topic'))],
    )

    # unconfigured -> inactive
    configure = RegisterEventHandler(
        OnProcessStart(
            target_action=lidar_node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lidar_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )),
            ],
        ),
        condition=IfCondition(auto_start),
    )

    # inactive -> active
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lidar_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(auto_start),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='urg_node2 parameter file. Defaults to '
                        "sensor_fusion's own, NOT the submodule's."),
        DeclareLaunchArgument(
            'node_name', default_value='urg_node2',
            description='ROS node name for the driver.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='scan',
            description='Topic the LaserScan is published on.'),
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description='Drive the lifecycle node to active automatically.'),
        lidar_node,
        configure,
        activate,
    ])
