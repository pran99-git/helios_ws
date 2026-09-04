"""RTAB-Map (RGB-D + LiDAR registration, GTSAM pose-graph SLAM) for Helios.

Runs ALONGSIDE the sensor stack (sensor_fusion/bringup.launch.py) and, if you
want it, slam_toolbox.launch.py -- this does NOT own map->odom TF by default
(publish_tf_map:=false), so its 3D map/loop-closure quality can be validated
risk-free before anything is switched over to depend on it live.

Reuses the already-fused EKF odometry (/odometry/filtered) instead of
estimating its own visual odometry -- avoids running two competing pose
estimators. Fuses the Hokuyo scan alongside ZED RGB-D for registration
(Reg/Strategy=2 "VisIcp", passed via 'args' since it's one of RTAB-Map's own
internal parameters, not a ROS node parameter -- see `ros2 run rtabmap_slam
rtabmap --params` for the full list).

GTSAM is already the default pose-graph optimizer in this RTAB-Map build
(Optimizer/Strategy=2)

To later make this TF-authoritative (replacing slam_toolbox's map->odom):
publish_tf_map:=true, and stop running slam_toolbox.launch.py.

Each run gets its own database under rtabmap/maps/ (named by 'run_name',
default a timestamp). The 3D map lives entirely inside that .db (the whole
SLAM session: keyframes, pose graph, loop closures) -- inspect it afterward
with rtabmap-databaseViewer.

Toggle with: publish_tf_map:=true rviz:=true rtabmap_viz:=true localization:=true
             run_name:=my_test database_path:=/custom/path.db
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Builds the RTAB-Map launch description.

    Returns:
        The rtabmap_launch include, wrapped in a group that applies our
        topic remappings, plus an optional RViz node.
    """
    rtabmap_launch_pkg = get_package_share_directory("rtabmap_launch")

    # Deliberately points into the SOURCE tree, not the installed share/ dir:
    # databases are run outputs that must survive a rebuild, and are far too
    # large to copy into install/ (hence maps/ is not an install() target).
    maps_dir = os.path.expanduser(
        "~/helios_ws/src/mapping_localization_pkg/rtabmap/maps"
    )
    # astimezone() attaches the local zone so the timestamp is tz-aware; the
    # formatted name is local wall-clock time either way.
    default_run_name = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")

    rviz_config = os.path.join(
        get_package_share_directory("mapping_localization_pkg"),
        "rtabmap",
        "rviz",
        "rtabmap.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "publish_tf_map",
                default_value="false",
                description="Own map->odom TF. Keep false until validated -- "
                "slam_toolbox stays the live TF authority until then.",
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="false",
                description="true = localize against the saved database instead "
                "of building a new map.",
            ),
            DeclareLaunchArgument(
                "run_name",
                default_value=default_run_name,
                description="Names this run's database -- "
                "rtabmap/maps/rtabmap_<run_name>.db",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value=[
                    maps_dir + "/rtabmap_",
                    LaunchConfiguration("run_name"),
                    ".db",
                ],
                description="Full path to the run database. Overrides run_name.",
            ),
            DeclareLaunchArgument(
                "rtabmap_viz",
                default_value="false",
                description="RTAB-Map's own visualization GUI.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Live RViz on rtabmap/rviz/rtabmap.rviz. Costs CPU on "
                "the Jetson -- see the note above the rviz node below.",
            ),
            DeclareLaunchArgument(
                "wait_for_transform",
                default_value="0.5",
                description="Wait for TF before starting mapping.",
            ),
            DeclareLaunchArgument(
                "sync_queue_size",
                default_value="30",
                description="Queue size for approximate time sync of RGB-D + LiDAR + odometry.",
            ),
            # scoped=True is REQUIRED, not stylistic. IncludeLaunchDescription does
            # NOT scope its launch_arguments: each becomes a SetLaunchConfiguration
            # in the CURRENT scope, so passing 'rviz': 'false' below would overwrite
            # our own `rviz` configuration for every later action, including the
            # rviz2 node at the bottom of this file. Symptom: `rviz:=true` launches
            # rtabmap with no rviz2 process and no error.
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                rtabmap_launch_pkg, "launch", "rtabmap.launch.py"
                            )
                        ),
                        launch_arguments={
                            # Sensors: ZED 2i RGB-D + Hokuyo scan.
                            "rgb_topic": "/zed/zed_node/rgb/color/rect/image",
                            "depth_topic": "/zed/zed_node/depth/depth_registered",
                            "camera_info_topic": "/zed/zed_node/rgb/color/rect/camera_info",
                            "subscribe_scan": "true",
                            "scan_topic": "/scan",
                            "approx_sync": "true",
                            # rgbd_sync is what fixes "Not enough inliers 0/20". 11.9% of depth
                            # frames have no RGB partner with a matching stamp, and ApproximateTime
                            # pairs those with the nearest survivor instead of dropping the set. The
                            # error is yaw_rate x skew, so closures fail in bursts while turning.
                            # rgbd_sync pre-packs rgb+depth+camera_info into one RGBDImage and
                            # approx_sync_max_interval makes a mismatched set get DROPPED. It also
                            # cuts rtabmap's own sync from 5 topics to 3. See the README for the
                            # measurements. The node then logs "subscribe_depth is set to false",
                            # which is expected.
                            "rgbd_sync": "true",
                            # 0.02 s is under one frame period at any rate we run, so it rejects
                            # cross-frame pairs while accepting the 88% that are already exact.
                            "approx_sync_max_interval": "0.02",
                            # Interpolate the odometry pose to each sensor's stamp via TF
                            # instead of taking the synced odom message as-is. Removes the
                            # odom-vs-sensor half of the same skew problem.
                            "odom_sensor_sync": "true",
                            # Odometry: reuse the EKF's fused wheel+VIO output.
                            "visual_odometry": "false",
                            "odom_topic": "/odometry/filtered",
                            # Gravity constraints only -- visual_odometry is false, so this
                            # is never used for VIO. See the Optimizer/GravitySigma note
                            # below.
                            "imu_topic": "/zed/zed_node/imu/data",
                            # Frames / TF ownership.
                            "frame_id": "base_link",
                            "map_frame_id": "map",
                            "publish_tf_map": LaunchConfiguration("publish_tf_map"),
                            # RTAB-Map's own internal parameters, passed as 'args' because they are
                            # not ROS node parameters. Full reasoning and the measurements behind
                            # each value are in the README's "Why these parameter values" section.
                            #
                            # Two are load-bearing and easy to undo by accident:
                            #  * Optimizer/Robust false + RGBD/OptimizeMaxError 0 disable BOTH
                            #    graph-level rejection mechanisms on purpose, so the optimizer must
                            #    distribute loop error rather than discard the closures. Outlier
                            #    protection lives in the front end instead: a closure only arrives
                            #    after PnP RANSAC with Vis/MinInliers >= 20. Raise Vis/MinInliers
                            #    before re-enabling either one here.
                            #  * Optimizer/GravitySigma needs imu_topic below AND
                            #    sensors.publish_imu_tf on the ZED side, or it is inert and the
                            #    graph holds zero Gravity links.
                            "args": "--Optimizer/Strategy 2 --Reg/Strategy 2 "
                            "--Optimizer/GravitySigma 0.3 "
                            "--RGBD/NeighborLinkRefining true "
                            "--Optimizer/Robust false "
                            "--RGBD/OptimizeMaxError 0 "
                            "--Mem/SaveDepth16Format true",
                            "localization": LaunchConfiguration("localization"),
                            "database_path": LaunchConfiguration("database_path"),
                            "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
                            # HARDCODED false; our rviz2 node below replaces it. Upstream's
                            # rviz:=true also spawns a rtabmap_util/point_cloud_xyzrgb node that
                            # rebuilds a coloured cloud on EVERY frame to feed its bundled
                            # rgbd.rviz, which is a permanent extra CPU consumer on the Jetson.
                            "rviz": "false",
                            "wait_for_transform": LaunchConfiguration(
                                "wait_for_transform"
                            ),
                            "sync_queue_size": LaunchConfiguration("sync_queue_size"),
                        }.items(),
                    )
                ],
            ),
            # --- Live RViz -------------------------------------------------------
            # Our own config rather than rtabmap_launch's rgbd.rviz. Both enable a
            # MapCloud on /rtabmap/mapData, which carries the full RGB+depth payload
            # of every node in working memory and costs more as the map grows; ours
            # halves the point count with a coarser voxel:
            #
            #             decimation   voxel    max depth
            #   ours          4        0.02 m     4 m
            #   upstream      4        0.01 m     4 m
            #
            # MapCloud is the expensive display here. Untick it in the Displays
            # panel if RViz is costing more than the 3D view is worth. Note it has
            # no outlier filtering of any kind, so the live cloud always shows more
            # stereo flying pixels than a save_rtabmap.sh export, which applies
            # --noise_radius/--noise_k. "Filter ceiling (m)" is the one live knob
            # that helps, and it is currently 0 (off).
            #
            # MapGraph is the display to watch while mapping: neighbour links in
            # blue, global loop closures in RED. Red links appearing on revisit is
            # the fastest read on whether a run is healthy.
            #
            # Cheapest option is not to render here at all: leave rviz:=false and run
            # `rviz2 -d <this file>` on a laptop sharing the ROS_DOMAIN_ID. The
            # subscription still costs the Jetson something, since several of these
            # topics are published lazily.
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
