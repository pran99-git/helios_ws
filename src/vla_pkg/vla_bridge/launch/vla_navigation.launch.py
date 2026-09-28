"""OmniVLA policy navigation for Helios: a goal in, a Nav2 path out.

Runs ON TOP of a stack that is already up. It needs, in this order:

    1. low_level_control_pkg roboclaw_driver.launch.py   (consumes /cmd_vel)
    2. sensor_fusion bringup.launch.py                   (odom -> base_link, /scan)
    3. mapping_localization_pkg amcl_localization.launch.py   (map -> odom)
    4. navigation_pkg navigation.launch.py               (controller_server + costmaps)

Step 4 matters: this bypasses the global planner and hands paths straight to
controller_server's follow_path action, so MPPI and the costmaps are what keep
the rover off obstacles. Without them the policy drives blind.

Starts three processes:

    omnivla_server      the policy, OUTSIDE ROS in its own virtualenv
    vla_bridge_node     frames + goal -> nav_msgs/Path in odom
    path_arbiter_node   Path -> FollowPath, throttled by divergence

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

Recommended companion setting: run the ZED with depth off in this mode. The
policy needs RGB only, and NEURAL_LIGHT depth is the documented cause of the
EKF starvation in the rtabmap README.

Toggle with: start_server:=false endpoint:=http://<workstation>:8777/act
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
    ]

    # A dead policy is not a degraded mode worth staying up for.
    on_server_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=server, on_exit=[Shutdown(reason="policy server exited")]
        )
    )

    return LaunchDescription([*arguments, server, on_server_exit, *nodes])
