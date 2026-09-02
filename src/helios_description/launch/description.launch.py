"""Publish the Helios rover's URDF frame tree.

Expands ``urdf/helios.urdf.xacro`` at launch time and feeds the result to
``robot_state_publisher``, which broadcasts every ``base_link -> <part>``
transform. This package owns nothing above ``base_link``: ``odom -> base_link``
comes from the EKF in ``sensor_fusion``.

The four wheels are continuous joints, so something must report their angle.
``gui:=true`` gives slider control for inspecting the model; ``gui:=false``
publishes zeros, which is what the real rover uses because nothing downstream
reads wheel angle.

``sensor_fusion/launch/bringup.launch.py`` includes this file with
``gui:=false``, so on the rover it is never launched directly.

Toggle with: gui:=false rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build the launch description for the robot description stack.

    Returns:
        The robot_state_publisher, a joint state publisher, and optionally RViz.
    """
    pkg = get_package_share_directory("helios_description")
    xacro_file = os.path.join(pkg, "urdf", "helios.urdf.xacro")
    rviz_config = os.path.join(pkg, "rviz", "view.rviz")

    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("rviz")

    # Expanded at launch, not at build time, so xacro edits take effect on the
    # next launch without a rebuild (given --symlink-install).
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description=("Use joint_state_publisher_gui sliders for the wheels."),
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz with the rover view.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(gui),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=UnlessCondition(gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
