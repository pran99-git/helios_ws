"""AMCL localization in an already-built map: owns map -> odom.

The counterpart to slam_toolbox.launch.py: that one BUILDS a map, this one
REUSES a saved one. The map is read-only -- AMCL never writes to the .pgm.

Runs SEPARATELY from the sensor stack, on top of it:

    ros2 launch sensor_fusion bringup.launch.py
    ros2 launch mapping_localization_pkg amcl_localization.launch.py \
        map:=<path-to>/slam_toolbox_20260728_175429.yaml

DO NOT run this alongside slam_toolbox.launch.py, or rtabmap with
publish_tf_map:=true -- all three publish map -> odom and would fight. The EKF
still owns odom -> base_link in every case.

Starts three nodes, because nav2_bringup is NOT installed on this machine
(only the individual nav2_* packages are):
  map_server        - serves the saved .pgm/.yaml on /map
  amcl              - particle filter, publishes map -> odom + /particlecloud
  lifecycle_manager - drives both through configure -> activate, since they
                      are lifecycle nodes that do not self-activate

AFTER LAUNCHING, the rover does not know where it is yet. Give it a pose:
  - RViz "2D Pose Estimate": click the rover's position, drag its heading.
    Works from anywhere in the map; this is the normal path.
  - or `ros2 service call /reinitialize_global_localization
    std_srvs/srv/Empty` to scatter particles over the whole map instead.
Then DRIVE. AMCL only updates after update_min_d/update_min_a of motion, so a
stationary rover never converges. Teleop is fine; pushing it by hand works
too, since AMCL reads odometry rather than commands. Watch /particlecloud in
RViz tighten as it goes.

Toggle with: rviz:=true use_map_topic:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Builds the AMCL localization launch description.

    Returns:
        map_server, amcl and the lifecycle_manager that activates them,
        plus an optional RViz node.
    """
    pkg = get_package_share_directory("mapping_localization_pkg")
    amcl_yaml = os.path.join(pkg, "localization", "config", "amcl.yaml")
    rviz_config = os.path.join(pkg, "slam_toolbox", "rviz", "slam.rviz")

    map_yaml = LaunchConfiguration("map")
    use_rviz = LaunchConfiguration("rviz")
    autostart = LaunchConfiguration("autostart")

    # The map path arrives as a launch argument, so it has to override what is
    # in amcl.yaml rather than being baked into it -- a saved map is a run
    # input, not a fixed property of the package.
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[amcl_yaml, {"yaml_filename": map_yaml}],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_yaml],
    )

    # Both nodes above are lifecycle nodes and neither self-activates.
    # lifecycle_manager transitions them in the listed order (map_server
    # first, so a map is being served before AMCL tries to localize against
    # it) and afterwards watches them via its bond timer.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "autostart": autostart,
                "node_names": ["map_server", "amcl"],
            }
        ],
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
                "map",
                description="REQUIRED. Path to the saved map .yaml (the file next "
                "to the .pgm), e.g. "
                "src/mapping_localization_pkg/slam_toolbox/maps/"
                "slam_toolbox_20260728_175429.yaml",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Open RViz with the SLAM layout (Map + LaserScan + TF "
                "are what you want to watch here).",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Drive map_server and amcl to active automatically.",
            ),
            map_server,
            amcl,
            lifecycle_manager,
            rviz,
        ]
    )
