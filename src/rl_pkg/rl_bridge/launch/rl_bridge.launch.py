"""RL policy navigation for Helios: a goal in, a body twist out.

Runs ON TOP of a stack that is already up. It needs, in this order:

    1. low_level_control_pkg roboclaw_driver.launch.py   (consumes /cmd_vel)
    2. sensor_fusion bringup.launch.py                   (odom -> base_link, /scan)
    3. mapping_localization_pkg amcl_localization.launch.py   (map -> odom, /map)
    4. navigation_pkg navigation.launch.py               (planner_server)

Step 4 is needed for planner_server ALONE. Unlike the VLA stack, this does
not hand paths to controller_server: the policy steers the rover itself and
publishes to /cmd_vel_rl, which sits above navigation in twist_mux. Nav2's
controller is the fallback underneath, not a layer in the loop.

DO NOT send goals to Nav2's bt_navigator at the same time. Both would drive.
The goal topic here is /rl/goal_pose, NOT /goal_pose, for that reason; point
RViz's "2D Goal Pose" tool at it.

The policy is an exported .onnx, written and parity-checked by
rl_sim/tools/export_onnx.py. It carries no scan-interpretation of its own:
the observation is rebuilt here by the same code the simulator trained
against.

First live runs: keep the joystick in hand as the deadman, and start with
speed_scale below 1.0.

    ros2 launch rl_bridge rl_bridge.launch.py \
        policy:=/path/to/tracker_v1.onnx speed_scale:=0.6
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("rl_bridge")
    default_config = os.path.join(share, "config", "rl_bridge.yaml")

    policy = LaunchConfiguration("policy")
    config = LaunchConfiguration("config")
    speed_scale = LaunchConfiguration("speed_scale")
    blind_yaw_feedback = LaunchConfiguration("blind_yaw_feedback")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "policy",
                description="exported .onnx to drive with (required)",
            ),
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="parameter file",
            ),
            DeclareLaunchArgument(
                "speed_scale",
                default_value="1.0",
                description="leash on the published twist; 1.0 is 0.40 m/s",
            ),
            DeclareLaunchArgument(
                "blind_yaw_feedback",
                default_value="true",
                description="zero the two wz observation channels, which cuts "
                "the policy's yaw oscillation by about a third",
            ),
            Node(
                package="rl_bridge",
                executable="rl_bridge_node",
                name="rl_bridge",
                output="screen",
                parameters=[
                    config,
                    # After the file, so the command line wins.
                    {
                        "policy": policy,
                        "speed_scale": speed_scale,
                        "blind_yaw_feedback": blind_yaw_feedback,
                    },
                ],
            ),
        ]
    )
