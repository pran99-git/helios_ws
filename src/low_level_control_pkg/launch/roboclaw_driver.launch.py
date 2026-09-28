"""RoboClaw hardware driver -- owner of both RoboClaw serial ports.

Publishes raw per-wheel encoder counts (roboclaw/wheel_encoders,
sensor_msgs/JointState) for perception_pkg's wheel_odometry_node to consume,
and subscribes to /cmd_vel (true m/s and rad/s) for closed-loop speed control.

Also starts twist_mux, which is the only publisher of /cmd_vel. Everything
that wants to drive goes through it by priority:

    /cmd_vel_estop      estop_node, zeroed        priority 250
    /cmd_vel_joy        teleop_joy_node           priority 100
    /cmd_vel_smoothed   Nav2's velocity_smoother  priority  10
    /cmd_vel_lock       estop_node, latched lock  lock     200

The joystick outranks autonomy, so grabbing it overrides Nav2 without having
to shut the nav stack down. It lives here rather than in a launch file of its
own because the driver must never be up without it: with the mux missing,
nothing publishes /cmd_vel and the rover simply will not move.

Requires: sudo apt install ros-jazzy-twist-mux

Run this exactly once, whenever the rover is powered and connected -- it's
the low-level-control subsystem's hardware-I/O boundary. Everything else
that needs the RoboClaws (wheel odometry, teleop, path-planner
control node) talks to it over ROS topics, not the serial port directly.

  ros2 launch low_level_control_pkg roboclaw_driver.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Builds the RoboClaw driver launch description.

    Returns:
        The driver node, parameterised from config/roboclaw.yaml.
    """
    pkg = get_package_share_directory("low_level_control_pkg")
    config = os.path.join(pkg, "config", "roboclaw.yaml")
    mux_config = os.path.join(pkg, "config", "twist_mux.yaml")
    estop_config = os.path.join(pkg, "config", "estop.yaml")

    return LaunchDescription(
        [
            Node(
                package="low_level_control_pkg",
                executable="roboclaw_driver_node",
                name="roboclaw_driver",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="twist_mux",
                executable="twist_mux",
                name="twist_mux",
                output="screen",
                parameters=[mux_config],
                remappings=[("cmd_vel_out", "cmd_vel")],
            ),
            Node(
                package="low_level_control_pkg",
                executable="estop_node",
                name="estop",
                output="screen",
                parameters=[estop_config],
            ),
        ]
    )
