"""Bluetooth-joystick input for Helios (8BitDo SN30 Pro)

Chain: joy_node (raw controller -> sensor_msgs/Joy)
       -> teleop_joy (Joy -> /cmd_vel, deadman-gated)

Publishes /cmd_vel only. Does NOT touch the RoboClaws -- safe to run
alongside anything, including the full sensor_fusion/RTAB-Map stack, as
long as something is separately running roboclaw_driver.launch.py to
actually consume /cmd_vel and drive the motors.

Pair the SN30 Pro over Bluetooth first (see README), then:
  ros2 launch low_level_control_pkg joy_teleop.launch.py
  ros2 launch low_level_control_pkg joy_teleop.launch.py joy_dev:=/dev/input/js1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Builds the joystick teleop launch description.

    Returns:
        The joy driver and teleop node, parameterised from
        config/teleop.yaml.
    """
    pkg = get_package_share_directory("low_level_control_pkg")
    config = os.path.join(pkg, "config", "teleop.yaml")

    joy_dev = LaunchConfiguration("joy_dev")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joy_dev",
                default_value="/dev/input/js0",
                description="Joystick device node (check with `ls /dev/input/js*` "
                "or `jstest` after pairing)",
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "device_name": "",
                        "dev": joy_dev,
                        "deadzone": 0.05,
                        "autorepeat_rate": 20.0,
                    }
                ],
            ),
            Node(
                package="low_level_control_pkg",
                executable="teleop_joy_node",
                name="teleop_joy",
                output="screen",
                parameters=[config],
            ),
        ]
    )
