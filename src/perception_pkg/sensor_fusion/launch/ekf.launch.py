"""Core fusion: robot_localization EKF, sole owner of odom -> base_link.

Fuses /wheel/odometry (mecanum wheel odometry) with /zed/zed_node/odom (ZED
visual-inertial odometry) per config/ekf.yaml, and publishes the fused result
on /odometry/filtered plus the odom -> base_link transform.

Assumes the sensor TOPICS are already being published and that nothing else
publishes odom -> base_link. Use bringup.launch.py to start the whole sensor
stack including drivers.

This launch file deliberately does NOT start any mapper. map -> odom belongs to
the mapping layer (mapping_localization_pkg), which runs separately:

    ros2 launch mapping_localization_pkg slam_toolbox.launch.py   # 2D LiDAR SLAM
    ros2 launch mapping_localization_pkg rtabmap.launch.py        # 3D RGB-D SLAM

Keeping them apart means perception_pkg has no dependency on the mapping layer,
and either mapper can be swapped in without touching sensing or fusion.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("sensor_fusion")
    ekf_yaml = os.path.join(pkg, "config", "ekf.yaml")

    # Local EKF: sole owner of odom -> base_link.
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_yaml],
    )

    return LaunchDescription(
        [
            ekf_node,
        ]
    )
