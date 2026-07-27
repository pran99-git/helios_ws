"""RTAB-Map (RGB-D + LiDAR registration, GTSAM pose-graph SLAM) for Helios.

Runs ALONGSIDE the existing slam_toolbox + EKF stack (sensor_fusion/bringup.
launch.py) rather than replacing it -- this does NOT own map->odom TF by
default (publish_tf_map:=false), so its 3D map/loop-closure quality can be
validated risk-free before anything is switched over to depend on it live.

Reuses the already-fused EKF odometry (/odometry/filtered) instead of
estimating its own visual odometry -- avoids running two competing pose
estimators. Fuses the Hokuyo scan alongside ZED RGB-D for registration
(Reg/Strategy=2 "VisIcp", passed via 'args' since it's one of RTAB-Map's own
internal parameters, not a ROS node parameter -- see `ros2 run rtabmap_slam
rtabmap --params` for the full list).

GTSAM is already the default pose-graph optimizer in this RTAB-Map build
(Optimizer/Strategy=2)

To later make this TF-authoritative (replacing slam_toolbox's map->odom):
publish_tf_map:=true, and stop launching slam_toolbox in fusion.launch.py.

Each run gets its own database under maps/ (named by 'run_name', default a
timestamp).
The 3D map lives entirely inside that .db (the whole SLAM session: keyframes,
pose graph, loop closures). To also save a standalone 2D map (.pgm/.yaml)
for the SAME run, use maps/save_2d_map.sh <run_name> while this is still
running (it saves the live /rtabmap/map topic, so the map node must be up).

Toggle with: publish_tf_map:=true rviz:=true rtabmap_viz:=true localization:=true
             run_name:=my_test database_path:=/custom/path.db
"""
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rtabmap_launch_pkg = get_package_share_directory('rtabmap_launch')

    maps_dir = os.path.expanduser(
        '~/helios_ws/src/mapping_localization_pkg/maps')
    default_run_name = datetime.now().strftime('%Y%m%d_%H%M%S')

    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_tf_map', default_value='false',
            description='Own map->odom TF. Keep false until validated -- '
                        'slam_toolbox stays the live TF authority until then.'),
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='true = localize against the saved database instead '
                        'of building a new map.'),
        DeclareLaunchArgument(
            'run_name', default_value=default_run_name,
            description='Names this run\'s database (and, if you also run '
                        'save_2d_map.sh, its 2D map) -- maps/rtabmap_<run_name>.db'),
        DeclareLaunchArgument(
            'database_path',
            default_value=[maps_dir + '/rtabmap_', LaunchConfiguration('run_name'), '.db']),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description="RTAB-Map's own visualization GUI."),
        DeclareLaunchArgument('rviz', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtabmap_launch_pkg, 'launch', 'rtabmap.launch.py')),
            launch_arguments={
                # Sensors: ZED 2i RGB-D + Hokuyo scan.
                'rgb_topic': '/zed/zed_node/rgb/color/rect/image',
                'depth_topic': '/zed/zed_node/depth/depth_registered',
                'camera_info_topic': '/zed/zed_node/rgb/color/rect/camera_info',
                'subscribe_scan': 'true',
                'scan_topic': '/scan',
                'approx_sync': 'true',

                # Odometry: reuse the EKF's fused wheel+VIO output.
                'visual_odometry': 'false',
                'odom_topic': '/odometry/filtered',

                # Frames / TF ownership.
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'publish_tf_map': LaunchConfiguration('publish_tf_map'),

                # RTAB-Map's own internal parameters (not ROS params).
                #   Optimizer/Strategy=2 (GTSAM): must be explicit.
                #   Reg/Strategy=2 (VisIcp): register using vision + LiDAR,
                #     not vision alone -- both sensors are available here.
                #   Optimizer/GravitySigma=0.3: keep optimized poses
                #     gravity-aligned, meaningful since VIO feeds this.
                'args': '--Optimizer/Strategy 2 --Reg/Strategy 2 '
                        '--Optimizer/GravitySigma 0.3',

                'localization': LaunchConfiguration('localization'),
                'database_path': LaunchConfiguration('database_path'),
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
    ])
