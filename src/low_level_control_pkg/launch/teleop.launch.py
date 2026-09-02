"""Standalone bench-test bundle: joystick teleop + RoboClaw driver together.

Chain: joy_node (raw controller -> sensor_msgs/Joy)
       -> teleop_joy (Joy -> /cmd_vel, deadman-gated)
       -> roboclaw_driver (/cmd_vel -> RoboClaw speed commands,
          + publishes roboclaw/wheel_encoders)

Bundles joy_teleop.launch.py + roboclaw_driver.launch.py for convenience when
bench-testing low_level_control_pkg in isolation (e.g. wheels off the ground,
no perception/SLAM stack running).

SAFETY: roboclaw_driver_node is the  owner of the RoboClaw serial ports.
Do NOT run this launch file if roboclaw_driver.launch.py is already running
elsewhere (e.g. as part of a full ground test) -- use joy_teleop.launch.py
by itself in that case, since the driver is already up.

Pair the SN30 Pro over Bluetooth first (see README), then:
  ros2 launch low_level_control_pkg teleop.launch.py
  ros2 launch low_level_control_pkg teleop.launch.py joy_dev:=/dev/input/js1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Builds the combined bench-test launch description.

    Returns:
        The joy_teleop and roboclaw_driver launch files, included together.
    """
    pkg = get_package_share_directory("low_level_control_pkg")

    joy_dev = LaunchConfiguration("joy_dev")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joy_dev",
                default_value="/dev/input/js0",
                description="Joystick device node (check with `ls /dev/input/js*` "
                "or `jstest` after pairing)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, "launch", "roboclaw_driver.launch.py")
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, "launch", "joy_teleop.launch.py")
                ),
                launch_arguments={"joy_dev": joy_dev}.items(),
            ),
        ]
    )
