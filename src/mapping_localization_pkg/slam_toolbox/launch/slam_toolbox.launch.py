"""slam_toolbox: 2D LiDAR SLAM -- owns map -> odom and publishes /map.

Consumes /scan (Hokuyo UST-10LX, frame 'laser') and the EKF's odom -> base_link,
and produces the 2D occupancy grid plus the map -> odom correction (REP-105:
SLAM owns map -> odom; the EKF owns odom -> base_link).

Runs SEPARATELY from the sensor stack. Start the sensors + EKF first, then this:

    ros2 launch sensor_fusion bringup.launch.py
    ros2 launch mapping_localization_pkg slam_toolbox.launch.py

slam_toolbox's nodes are LIFECYCLE nodes that do NOT auto-activate on this
build, so they are declared as LifecycleNodes and driven through
configure -> activate automatically via event handlers (same pattern as
lidar.launch.py). This makes it come up 'active' (subscribed to /scan,
publishing /map + map -> odom) with no manual `ros2 lifecycle set` needed.

DO NOT run this at the same time as rtabmap.launch.py with
publish_tf_map:=true, or amcl_localization.launch.py -- they all publish
map -> odom and would fight over it. rtabmap defaults to
publish_tf_map:=false precisely so the two can coexist, with slam_toolbox
staying the TF authority.

TWO MODES, and they are DIFFERENT EXECUTABLES, not just a parameter:

  mapping (default)  async_slam_toolbox_node -- builds a new map.
      ros2 launch mapping_localization_pkg slam_toolbox.launch.py

  localization       localization_slam_toolbox_node -- loads a previously
                     SERIALIZED pose graph and localizes against it without
                     growing it. Requires BOTH a pose graph and a starting
                     pose; slam_toolbox refuses to configure without the
                     latter.
      ros2 launch mapping_localization_pkg slam_toolbox.launch.py \\
          localization:=true \\
          map_file_name:=<path/to/slam_toolbox_NAME> \\
          map_start_pose:="[0.0, 0.0, 0.0]"

Saving, at the end of a mapping run -- one script, all four files, one name:
  slam_toolbox/scripts/save_slam.sh <name>
    -> .pgm + .yaml        for AMCL and nav2 costmaps
    -> .posegraph + .data  the ONLY format localization:=true can load, and
                           the only one you can extend later
Neither pair can be regenerated from the other, so both are written by
default (--map-only / --graph-only narrow it if you really want just one).

Toggle with: rviz:=true localization:=true map_file_name:=<prefix>
             map_start_pose:="[x, y, yaw]" | map_start_at_dock:=true
"""
import ast
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _as_bool(value):
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def _parse_start_pose(raw):
    """Parses "[x, y, yaw]" (or "x, y, yaw") into a list of three floats.

    slam_toolbox wants map_start_pose as a real double array, and a launch
    argument is always a string -- hence the parse. Done here in an
    OpaqueFunction rather than with a PythonExpression substitution because
    the latter would hand ROS the *string* "[0.0, 0.0, 0.0]" and the
    parameter would come out typed as a string.
    """
    text = str(raw).strip()
    if not text:
        return None
    if not text.startswith('['):
        text = '[' + text + ']'
    try:
        pose = [float(v) for v in ast.literal_eval(text)]
    except (ValueError, SyntaxError) as exc:
        raise RuntimeError(
            f'map_start_pose must look like "[x, y, yaw]", got {raw!r}'
        ) from exc
    if len(pose) != 3:
        raise RuntimeError(
            f'map_start_pose needs exactly 3 values [x, y, yaw], got {pose}'
        )
    return pose


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('mapping_localization_pkg')
    slam_yaml = os.path.join(pkg, 'slam_toolbox', 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(pkg, 'slam_toolbox', 'rviz', 'slam.rviz')

    localization = _as_bool(LaunchConfiguration('localization').perform(context))
    map_file_name = LaunchConfiguration('map_file_name').perform(context).strip()
    start_at_dock = _as_bool(
        LaunchConfiguration('map_start_at_dock').perform(context))
    start_pose_raw = LaunchConfiguration('map_start_pose').perform(context)

    overrides = {'mode': 'localization' if localization else 'mapping'}

    if localization:
        # Fail here, with a message that says what to do, rather than letting
        # slam_toolbox come up and quietly localize against nothing.
        if not map_file_name:
            raise RuntimeError(
                'localization:=true requires map_file_name:=<prefix> -- the '
                'path printed by save_slam.sh, WITHOUT the .posegraph/'
                '.data extension.')
        if not os.path.exists(map_file_name + '.posegraph'):
            raise RuntimeError(
                f'No pose graph at {map_file_name}.posegraph. Note this is a '
                'SERIALIZED pose graph, not the .pgm/.yaml that save_slam.sh '
                'writes alongside it -- those are for AMCL and cannot be '
                'loaded here.')
        overrides['map_file_name'] = map_file_name

        # slam_toolbox refuses to configure in localization mode without one
        # of these two: "Map starting pose not specified. Set either
        # map_start_pose or map_start_at_dock."
        if start_at_dock:
            overrides['map_start_at_dock'] = True
        else:
            pose = _parse_start_pose(start_pose_raw)
            if pose is None:
                raise RuntimeError(
                    'localization:=true needs a starting pose: pass '
                    'map_start_pose:="[x, y, yaw]" or map_start_at_dock:=true.')
            overrides['map_start_pose'] = pose

    executable = ('localization_slam_toolbox_node' if localization
                  else 'async_slam_toolbox_node')

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable=executable,
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_yaml, overrides],
    )

    # On start -> configure (unconfigured -> inactive).
    slam_configure = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )),
            ],
        ),
    )

    # On reaching 'inactive' after configuring -> activate (inactive -> active).
    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [slam_node, slam_configure, slam_activate, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description="Launch RViz with the slam_toolbox view."),
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='true = load a serialized pose graph and localize '
                        'against it (needs map_file_name + a start pose) '
                        'instead of building a new map.'),
        DeclareLaunchArgument(
            'map_file_name', default_value='',
            description='Serialized pose-graph prefix, WITHOUT the .posegraph/'
                        '.data extension, as printed by save_slam.sh. '
                        'Required when localization:=true.'),
        DeclareLaunchArgument(
            'map_start_pose', default_value='[0.0, 0.0, 0.0]',
            description='Starting pose in the loaded map as "[x, y, yaw]" '
                        '(metres/radians). Used only with localization:=true, '
                        'and ignored if map_start_at_dock:=true.'),
        DeclareLaunchArgument(
            'map_start_at_dock', default_value='false',
            description='Start from the pose the map was originally begun at, '
                        'instead of map_start_pose.'),
        OpaqueFunction(function=_launch_setup),
    ])
