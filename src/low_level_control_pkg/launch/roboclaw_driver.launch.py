"""RoboClaw hardware driver -- owner of both RoboClaw serial ports.

Publishes raw per-wheel encoder counts (roboclaw/wheel_encoders,
sensor_msgs/JointState) for perception_pkg's wheel_odometry_node to consume,
and subscribes to /cmd_vel for open-loop duty-cycle motor control.

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


def generate_launch_description():
    pkg = get_package_share_directory('low_level_control_pkg')
    config = os.path.join(pkg, 'config', 'teleop.yaml')

    return LaunchDescription([
        Node(
            package='low_level_control_pkg',
            executable='roboclaw_driver_node',
            name='roboclaw_driver',
            output='screen',
            parameters=[config],
        ),
    ])
