"""Autonomous exploration: frontier detection plus goal selection.

Runs ON TOP of a stack that is already up, and starts no mapper, no planner
and no controller of its own. It needs, in this order:

    1. low_level_control_pkg roboclaw_driver.launch.py   (consumes /cmd_vel)
    2. sensor_fusion bringup.launch.py                   (odom -> base_link, /scan)
    3. mapping_localization_pkg slam_toolbox.launch.py   (map -> odom, /map)
    4. navigation_pkg navigation.launch.py               (planner + costmaps)
    5. vla_bridge vla_navigation.launch.py walker:=true  (the driver)

Step 3 must be slam_toolbox in MAPPING mode, not AMCL: exploration needs a map
that grows. Running amcl_localization.launch.py instead gives a fixed map with
no unknown space left to find, and the selector will report that the reachable
space is already mapped.

Two nodes:

    frontier_detector   /map -> a ranked list of frontier poses
    goal_selector       that list -> one /vla/route_goal at a time

The selector commits to a frontier and stays with it until it arrives, the
frontier is mapped away, or an attempt times out. Re-choosing every cycle would
make the rover oscillate between two equally good openings forever.

Point it at the classical stack instead with goal_topic:=/goal_pose, which runs
the same exploration with Nav2 driving rather than the policy. Do NOT do that
while vla_navigation.launch.py is running: both would command the controller.

Pause and resume a run without restarting anything:
    ros2 topic pub --once /exploration/enable std_msgs/Bool "{data: false}"

Toggle with: start_enabled:=false goal_topic:=/goal_pose
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Builds the exploration launch description."""
    share = get_package_share_directory("mapping_localization_pkg")
    params = os.path.join(share, "exploration", "config", "exploration.yaml")

    params_file = LaunchConfiguration("params_file")
    goal_topic = LaunchConfiguration("goal_topic")
    start_enabled = LaunchConfiguration("start_enabled")

    arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value=params,
            description="Parameters for frontier_detector and goal_selector",
        ),
        DeclareLaunchArgument(
            "goal_topic",
            default_value="/vla/route_goal",
            description=(
                "Where selected goals go. /vla/route_goal drives with the "
                "policy via the subgoal walker; /goal_pose drives with "
                "Nav2's bt_navigator instead"
            ),
        ),
        DeclareLaunchArgument(
            "start_enabled",
            default_value="true",
            description=(
                "Start exploring immediately. false waits for a true on "
                "/exploration/enable"
            ),
        ),
    ]

    nodes = [
        Node(
            package="mapping_localization_pkg",
            executable="frontier_node",
            name="frontier_detector",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="mapping_localization_pkg",
            executable="goal_selector_node",
            name="goal_selector",
            parameters=[
                params_file,
                {"goal_topic": goal_topic, "start_enabled": start_enabled},
            ],
            output="screen",
        ),
    ]

    return LaunchDescription([*arguments, *nodes])
