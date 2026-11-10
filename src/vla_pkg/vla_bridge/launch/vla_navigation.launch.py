"""OmniVLA policy navigation for Helios: a goal in, a Nav2 path out.

Runs ON TOP of a stack that is already up. It needs, in this order:

    1. low_level_control_pkg roboclaw_driver.launch.py   (consumes /cmd_vel)
    2. sensor_fusion bringup.launch.py                   (odom -> base_link, /scan)
    3. mapping_localization_pkg amcl_localization.launch.py   (map -> odom)
    4. navigation_pkg navigation.launch.py               (controller_server + costmaps)

Step 4 matters: this bypasses the global planner and hands paths straight to
controller_server's follow_path action, so MPPI and the costmaps are what keep
the rover off obstacles. Without them the policy drives blind.

Starts three processes, plus an optional fourth:

    omnivla_server       the policy, OUTSIDE ROS in its own virtualenv
    vla_bridge_node      frames + goal -> nav_msgs/Path in odom
    path_arbiter_node    Path -> FollowPath, throttled by divergence
    subgoal_walker_node  optional: a distant goal -> a stream of near ones

The walker exists because the policy predicts about a metre per chunk and holds
no map, so a distant goal makes it drive at the bearing until an obstacle stops
it. With the walker, planner_server owns route choice and the policy only ever
sees the short goal it handles well.

It is OFF by default: it changes where goals go, and it has not yet driven the
rover. Two interfaces, pick one:

    walker:=false   publish the goal to /vla/goal_pose   (policy drives at it)
    walker:=true    publish the goal to /vla/route_goal  (planner routes, the
                    walker feeds /vla/goal_pose a carrot 1.5 m along it)

The server runs under a different Python than ROS because OmniVLA pins torch
and a transformers fork that cannot coexist with Jazzy on Python 3.12. Its
PYTHONPATH is cleared deliberately: sourcing ROS prepends /opt/ros/jazzy to it,
a virtualenv does NOT override that, and the server would then import ROS's
packages instead of its own.

If the policy server exits, everything here shuts down with it rather than
leaving the bridge publishing empty paths at a rover that cannot move.

DO NOT send goals to Nav2's bt_navigator at the same time. Both would command
the same controller. The pose goal here is /vla/goal_pose, NOT /goal_pose, for
exactly that reason; point RViz's "2D Goal Pose" tool at it.

Keep ZED depth ON despite the policy needing RGB only: the local costmap takes
the ZED point cloud as its second obstacle source, which is what sees obstacles
below the 0.176 m laser plane. Measured with the cloud, the policy and MPPI all
running: 100.2 ms inference and no new EKF rate misses.

Toggle with: walker:=true start_server:=false
             endpoint:=http://<workstation>:8777/act
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

#: The policy lives in the source tree, not the install space: it is not a ROS
#: package and colcon is told to ignore it. Override `workspace` if the
#: checkout moves.
DEFAULT_WORKSPACE = os.path.expanduser("~/helios_ws")


def generate_launch_description() -> LaunchDescription:
    """Builds the VLA navigation launch description."""
    share = get_package_share_directory("vla_bridge")

    workspace = LaunchConfiguration("workspace")
    params_file = LaunchConfiguration("params_file")
    endpoint = LaunchConfiguration("endpoint")
    start_server = LaunchConfiguration("start_server")
    walker = LaunchConfiguration("walker")

    arguments = [
        DeclareLaunchArgument(
            "workspace",
            default_value=DEFAULT_WORKSPACE,
            description="Workspace root holding .venvs/ and src/vla_pkg/",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "config", "vla_bridge.yaml"),
            description="Parameters for vla_bridge and path_arbiter",
        ),
        DeclareLaunchArgument(
            "endpoint",
            default_value="http://127.0.0.1:8777/act",
            description="Policy server /act URL; a workstation IP also works",
        ),
        DeclareLaunchArgument(
            "start_server",
            default_value="true",
            description="Start the policy server here, or point at a running one",
        ),
        DeclareLaunchArgument(
            "walker",
            default_value="false",
            description=(
                "Route distant goals through planner_server and feed the policy "
                "a carrot. Goals then go to /vla/route_goal, not /vla/goal_pose"
            ),
        ),
    ]

    server = ExecuteProcess(
        name="omnivla_server",
        condition=IfCondition(start_server),
        cmd=[
            PathJoinSubstitution(
                [workspace, ".venvs", "omnivla_edge", "bin", "python"]
            ),
            PathJoinSubstitution(
                [workspace, "src", "vla_pkg", "omnivla_server", "server.py"]
            ),
        ],
        # Empty PYTHONPATH, or ROS's site-packages shadow the virtualenv's.
        # XDG_CACHE_HOME keeps CLIP's weights off the 11 GB eMMC root.
        additional_env={"PYTHONPATH": "", "XDG_CACHE_HOME": "/mnt/ssd/cache"},
        output="screen",
    )

    nodes = [
        Node(
            package="vla_bridge",
            executable="vla_bridge_node",
            name="vla_bridge",
            parameters=[params_file, {"endpoint": endpoint}],
            output="screen",
        ),
        Node(
            package="vla_bridge",
            executable="path_arbiter_node",
            name="path_arbiter",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="vla_bridge",
            executable="subgoal_walker_node",
            name="subgoal_walker",
            condition=IfCondition(walker),
            parameters=[params_file],
            output="screen",
        ),
    ]

    # A dead policy is not a degraded mode worth staying up for.
    on_server_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=server, on_exit=[Shutdown(reason="policy server exited")]
        )
    )

    return LaunchDescription([*arguments, server, on_server_exit, *nodes])
