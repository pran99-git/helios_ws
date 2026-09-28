"""Nav2 autonomous navigation for Helios: goal in, /cmd_vel out.

Runs ON TOP of a stack that is already up. It needs, in this order:

    1. low_level_control_pkg roboclaw_driver.launch.py   (consumes /cmd_vel)
    2. sensor_fusion bringup.launch.py                   (odom -> base_link, /scan)
    3. a map source owning map -> odom, exactly one of:
         slam_toolbox.launch.py            build a map and navigate in it
         amcl_localization.launch.py       navigate in a saved map
         rtabmap.launch.py publish_tf_map:=true

Nav2 publishes NO transforms. The EKF keeps odom -> base_link and the mapper
keeps map -> odom, exactly as before.

Six servers are launched, not the nine nav2_bringup would start. smoother_server,
waypoint_follower and docking_server are omitted because nothing here uses them
and this Jetson is already CPU-bound by the ZED. Add them when a use appears.

joy_teleop can run alongside this. twist_mux, started with the driver, ranks
the joystick above Nav2, so holding the deadman overrides a goal in progress
and releasing it hands control back. Teleop has a deadman; Nav2 does not.
See the README.

Toggle with: rviz:=true autostart:=false params_file:=/custom/nav2.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Order matters: lifecycle_manager transitions these in sequence and STOPS at the
# first one that will not activate, leaving everything after it inactive.
#
# planner_server is therefore last of the movement-capable nodes. It carries the
# GLOBAL costmap, which blocks on map -> odom, so it cannot activate until
# something has localized the robot. Put it earlier and it takes the behaviors
# down with it: amcl_recovery then cannot drive the rover to localize it, and
# nothing ever localizes, which is a deadlock the robot cannot leave on its own.
#
# Verified live: behavior_server activates cleanly with no map -> odom at all.
# It declares global_frame: map but Spin and DriveOnHeading both work in
# local_frame: odom, so it never needs the map edge.
#
# Everything before planner_server needs only odom -> base_link, which the EKF
# publishes from the moment the sensors are up.
LIFECYCLE_NODES = [
    "controller_server",
    "behavior_server",
    "velocity_smoother",
    "bt_navigator",
    "planner_server",
]


def generate_launch_description() -> LaunchDescription:
    """Builds the Nav2 launch description.

    Returns:
        The five navigation servers, the lifecycle manager that activates them,
        and an optional RViz node.
    """
    pkg = get_package_share_directory("navigation_pkg")
    default_params = os.path.join(pkg, "config", "nav2_params.yaml")
    rviz_config = os.path.join(pkg, "rviz", "navigation.rviz")

    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_rviz = LaunchConfiguration("rviz")

    # controller_server publishes to cmd_vel by default. Routing it through the
    # smoother instead means the driver only ever sees ramped commands:
    #     controller -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed
    # and twist_mux, started with the driver, is what forwards it to /cmd_vel.
    # /cmd_vel is what roboclaw_driver_node subscribes to (roboclaw.yaml).
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file],
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[params_file],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )

    # Every server above is a lifecycle node and none self-activates. The manager
    # drives them configure -> activate in LIFECYCLE_NODES order, then holds a
    # bond with each one so a server that dies takes the stack down visibly
    # rather than leaving a half-active system that silently ignores goals.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{"autostart": autostart, "node_names": LIFECYCLE_NODES}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Nav2 parameter file. Defaults to this package's own.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Drive the lifecycle nodes to active automatically. "
                "false leaves them unconfigured for manual transitions.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Open RViz with the navigation layout. Costs CPU on "
                "the Jetson; prefer running it on a laptop.",
            ),
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            velocity_smoother,
            lifecycle_manager,
            rviz,
        ]
    )
