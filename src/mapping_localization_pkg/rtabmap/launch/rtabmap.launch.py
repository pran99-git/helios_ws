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
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rtabmap_launch_pkg = get_package_share_directory('rtabmap_launch')

    # Deliberately points into the SOURCE tree, not the installed share/ dir:
    # databases are run outputs that must survive a rebuild, and are far too
    # large to copy into install/ (hence maps/ is not an install() target).
    maps_dir = os.path.expanduser(
        '~/helios_ws/src/mapping_localization_pkg/rtabmap/maps')
    default_run_name = datetime.now().strftime('%Y%m%d_%H%M%S')

    rviz_config = os.path.join(
        get_package_share_directory('mapping_localization_pkg'),
        'rtabmap', 'rviz', 'rtabmap.rviz')

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
            description='Names this run\'s database -- '
                        'rtabmap/maps/rtabmap_<run_name>.db'),
        DeclareLaunchArgument(
            'database_path',
            default_value=[maps_dir + '/rtabmap_', LaunchConfiguration('run_name'), '.db']),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description="RTAB-Map's own visualization GUI."),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Live RViz on rtabmap/rviz/rtabmap.rviz. Costs CPU on '
                        'the Jetson -- see the note above the rviz node below.'),
        DeclareLaunchArgument(
            'wait_for_transform', default_value='0.5',
            description='Wait for TF before starting mapping.'),
        DeclareLaunchArgument(
            'sync_queue_size', default_value='30',
            description='Queue size for approximate time sync of RGB-D + LiDAR + odometry.'),

        # GroupAction(scoped=True) is REQUIRED, not stylistic. A bare
        # IncludeLaunchDescription does NOT scope its launch_arguments: each one
        # becomes a SetLaunchConfiguration in the CURRENT scope, so passing
        # 'rviz': 'false' below overwrote our own `rviz` configuration for every
        # action after the include -- including the rviz2 node at the bottom of
        # this file, whose IfCondition then read 'false' no matter what the user
        # typed. Symptom: `rviz:=true` silently launches rtabmap with no rviz2
        # process and no error message. Scoping contains the assignment to the
        # included description.
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
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

                # rgbd_sync -- THE fix for "Not enough inliers 0/20
                # (matches=94)". Those rejections are not a threshold being too
                # strict: zero inliers out of ~90 matches means RANSAC found no
                # camera pose consistent with ANY subset, so lowering
                # Vis/MinInliers would change nothing. Appearance matching
                # succeeds (the place IS recognised) and only the geometry
                # fails, which happens when the 3D points behind the keypoints
                # are wrong -- RTAB-Map samples them from the depth image.
                #
                # Measured 2026-08-25 by subscribing to both topics at once:
                # 88.1% of depth frames share a bit-identical stamp with an RGB
                # frame, so they do come from the same grab -- but 11.9% have no
                # RGB partner at all, and ApproximateTime silently pairs those
                # with the nearest survivor instead of dropping the set. The
                # resulting error is yaw_rate x skew, which is why closures fail
                # in bursts while turning: on the 140013 run (max 0.82 rad/s) a
                # ~100 ms mispair is 4.7 deg of camera rotation, ~33 cm at 4 m,
                # against a Vis/PnPReprojError budget of 2 PIXELS.
                #
                # rgbd_sync pre-packs rgb+depth+camera_info into one RGBDImage,
                # and approx_sync_max_interval makes a mismatched set get
                # DROPPED rather than silently paired. It also cuts rtabmap's
                # own sync from 5 topics to 3 (rgbd_image, scan, odom).
                # subscribe_rgbd defaults to rgbd_sync upstream, so it follows
                # automatically; the node then reports "subscribe_depth is set
                # to false", which is expected, not an error.
                #
                # 0.02 s is well under one frame period at any publish rate we
                # run, so it rejects cross-frame pairs while accepting the 88%
                # that are exact. approx_rgbd_sync is left at its default true:
                # exact sync would be stronger still, but camera_info was
                # measured publishing at a different rate than the images
                # (29.3 vs 10.0 Hz), so requiring all three to match exactly
                # risks starving the pipeline. Tighten only with that measured.
                'rgbd_sync': 'true',
                'approx_sync_max_interval': '0.02',

                # Interpolate the odometry pose to each sensor's stamp via TF
                # instead of taking the synced odom message as-is. Removes the
                # odom-vs-sensor half of the same skew problem.
                'odom_sensor_sync': 'true',

                # Odometry: reuse the EKF's fused wheel+VIO output.
                'visual_odometry': 'false',
                'odom_topic': '/odometry/filtered',

                # Gravity constraints only -- visual_odometry is false, so this
                # is never used for VIO. See the Optimizer/GravitySigma note
                # below.
                'imu_topic': '/zed/zed_node/imu/data',

                # Frames / TF ownership.
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'publish_tf_map': LaunchConfiguration('publish_tf_map'),

                # RTAB-Map's own internal parameters.
                # Optimizer/Strategy=2 (GTSAM): must be explicit.
                #  Reg/Strategy=2 (VisIcp): register using vision + LiDAR,
                #  not vision alone -- both sensors are available here.
                # RGBD/NeighborLinkRefining=true: neighbor (sequential) edges
                #  default to a raw copy of the external EKF odometry delta,
                #  which can carry multi-meter jumps if VIO glitches/resets.
                #
                # Optimizer/Robust=false + RGBD/OptimizeMaxError=0 -- BOTH
                #  graph-level rejection mechanisms disabled, deliberately.
                #
                #  History, because this looks reckless without it. First we
                #  had OptimizeMaxError=3.0 (hard rejection: drop a closure
                #  when |error|/link_stddev > 3). That rejected everything, so
                #  on 2026-08-24 we swapped to Optimizer/Robust=true (Vertigo
                #  switchable constraints, soft down-weighting). The diagnosis
                #  was right, the fix was not: Vertigo rejects the SAME links
                #  for the SAME reason, just silently.
                #
                #  Measured on rtabmap_20260825_150118.db, a run where the
                #  rover drove a 16.8 m loop and returned to its start:
                #    - odometry claimed it ended 37.99 deg rotated from node 1
                #    - 44 visual loop closures said ~1.1 deg
                #    - the optimizer settled at 33.57 deg, applying 4 of the
                #      36 deg correction the closures demanded
                #  The LiDAR settles it independently: brute-force alignment of
                #  node 1's scan against node 296's finds +2.0 deg and 5 cm,
                #  residual 0.0097 m. Forcing 37.99 deg gives 0.2233 -- 23x
                #  worse. The vision was right and the graph threw it away.
                #
                #  Why the optimizer prefers to discard it: RGBD/
                #  NeighborLinkRefining below gives each odometry edge an ICP
                #  covariance from two nearly-identical consecutive scans,
                #  median sigma_yaw 0.10 deg (info_rr ~331800). Bending 181 of
                #  those to absorb 36 deg costs 181*(0.199/0.104)^2 ~= 660,
                #  while switching off 44 loop links costs Vertigo far less. It
                #  is making a rational choice from dishonest inputs. Note
                #  RTAB-Map already applies RGBD/ProximityMergedScanCovFactor
                #  =100 to inflate proximity-link covariance for exactly this
                #  reason -- there is no equivalent knob for neighbor links.
                #
                #  With both mechanisms off the optimizer runs plain weighted
                #  least squares and MUST distribute the loop error; a 36 deg
                #  correction spread over 181 edges is 0.2 deg each, which is
                #  physically nothing. Outlier protection is not lost, it just
                #  moves entirely to the front end: a closure only reaches the
                #  optimizer after passing PnP RANSAC with Vis/MinInliers>=20,
                #  which is what all the "Not enough inliers 0/20" log lines
                #  are -- that filter is working and rejects most candidates.
                #  If a false closure ever does corrupt a map, raise
                #  Vis/MinInliers before re-enabling either mechanism here.
                #
                # Icp/PointToPlane: TRIED false 2026-08-24, REVERTED to the
                #  vendor default true. The theory was that a single-plane 2D
                #  Hokuyo sweep cannot give well-conditioned normals, so
                #  point-to-plane was producing the "Transform is found ... but
                #  no correspondences has been found!? Variance is unknown!"
                #  warnings. Wrong: with Icp/Strategy=1 (libpointmatcher) the
                #  2D case is handled, and Icp/PointToPlaneLowComplexityStrategy
                #  is the safeguard for degenerate geometry -- which only
                #  applies when point-to-plane is ON. Turning it off made that
                #  warning go from occasional to near-constant and cost ~18% of
                #  the lidar proximity closures (0.164 -> 0.135 per node). Do
                #  not set this false again without re-measuring.
                #
                # Optimizer/GravitySigma=0.3 + imu_topic below -- roll/pitch
                #  observability. Until 2026-08-25 GravitySigma was set but
                #  INERT: no imu_topic meant no subscription, and all three
                #  databases to that date held 0 links of type Gravity.
                #
                #  An earlier note here argued the gain was small because
                #  ekf.yaml sets two_d_mode:=true, which already pins roll and
                #  pitch. That had it backwards. two_d_mode pins them to ZERO,
                #  which is an assumption, not a measurement -- and nothing
                #  else in the graph can contradict it: neighbor links come
                #  from that flat EKF odometry, and RGBD/NeighborLinkRefining
                #  refines them with ICP on a single-plane 2D Hokuyo sweep,
                #  which observes x, y and yaw only. So the pose chain is
                #  rigidly flat while the visual loop closures (full 6-DoF PnP)
                #  keep asserting real roll/pitch. The optimizer splits the
                #  difference and the clouds fan out. Gravity links give each
                #  node a unary roll/pitch constraint from measured gravity, so
                #  the flat chain can actually be corrected.
                #
                #  Needs sensors.publish_imu_tf on the ZED side -- set as a
                #  launch argument in sensor_fusion/bringup.launch.py. The imu
                #  subscription is standalone in CoreWrapper, NOT part of the
                #  5-topic approx_sync, so it cannot starve the sync.
                #
                # Mem/SaveDepth16Format=true -- the depth topic is 32-bit
                #  float, and .rvl cannot carry that, so every frame silently
                #  fell back to PNG (the wrapper warns about this on startup).
                #  PNG of a 32-bit depth image is why Compressing_data measured
                #  37% of total frame time, and why the 2026-08-25 run wrote
                #  1074 MB for 448 nodes (2.4 MB/node). true converts depth to
                #  16 mm-precision uint16 and uses RVL. Cost: depth values over
                #  65 m are dropped -- irrelevant for a stereo pair with a
                #  0.120 m baseline indoors, where depth is already meaningless
                #  past ~15 m.
                # === REVERTED 2026-08-28 -- read before re-adding any of this ===
                #
                #  On 2026-08-27 eight parameters were added here in two batches:
                #  a "drive faster" batch (Rtabmap/DetectionRate 2,
                #  Icp/MaxCorrespondenceDistance 0.15, Vis/PnPReprojError 3) and
                #  a "see low obstacles" batch (Grid/Sensor 2 + 7 dependants).
                #  All eight are removed. The args below are exactly the set that
                #  produced rtabmap_20260827_134811.db -- confirmed by reading
                #  that database's own stored parameters, not from memory.
                #
                #  WHY. Measured on rtabmap_20260828_120512.db, the first run with
                #  all eight active:
                #                                    0827 good    0828 changed
                #    Optimization error, median          5.72           21.48
                #    Optimizer max angular error         0.23 deg        0.55 deg
                #    GlobalClosure links per keyframe    0.09            1.88
                #    empty_cells (grid free space)       4.91 MB         0.00 MB
                #    database size per metre driven      7.7 MB/m       21.8 MB/m
                #
                #  Read the 0828 column with care: 76% of that run was STATIONARY
                #  (416 of 544 frames under 0.02 m/s, ~235 s of 311 s; one node
                #  carries weight 233 from rehearsed duplicate frames), so it is
                #  not a clean comparison. The 246 GlobalClosure links have a
                #  median endpoint separation of 1 mm -- they connect a parked
                #  robot to itself, which is why the count exploded. The optimizer
                #  residual and the grid loss are real regardless.
                #
                #  Grid/Sensor 2 was independently disproven BEFORE that run, by
                #  reprocessing the good database with rtabmap-reprocess: same 439
                #  nodes, same poses, only the grid source changed.
                #                        Sensor=0    Sensor=2   Sensor=2+RayTracing
                #    ground_cells         0.00 MB     2.81 MB        0.00 MB
                #    obstacle_cells       3.43 MB     1.65 MB        1.65 MB
                #    empty_cells          4.91 MB     0.00 MB        2.99 MB
                #  Obstacles more than HALVED and free space fell ~40%, because a
                #  ~110 deg camera cone replaces a 270 deg laser sweep.
                #  Grid/RayTracing converts ground back into empty rather than
                #  adding anything. Grid/Sensor 2 is a net loss on this robot
                #  until it must detect obstacles below the ~0.126 m laser plane
                #  for navigation -- and even then Grid/RangeMax must be passed
                #  explicitly, because its auto-set to unlimited is conditional on
                #  Grid/Sensor being 0 and grid range silently drops 10 m -> 5 m.
                #
                #  If you retry any of this: change ONE parameter, and compare
                #  against a drive of similar length and speed. The 0827 baseline
                #  is 47.7 m at 0.131 m/s median, 1.27 LocalSpaceClosure links per
                #  node, 0.0159 m / 0.23 deg optimizer error.
                'args': '--Optimizer/Strategy 2 --Reg/Strategy 2 '
                        '--Optimizer/GravitySigma 0.3 '
                        '--RGBD/NeighborLinkRefining true '
                        '--Optimizer/Robust false '
                        '--RGBD/OptimizeMaxError 0 '
                        '--Mem/SaveDepth16Format true',

                'localization': LaunchConfiguration('localization'),
                'database_path': LaunchConfiguration('database_path'),
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),

                # HARDCODED false -- our rviz2 node below replaces it.
                # Upstream's rviz:=true does two things, and the second one is
                # not obvious: besides rviz2 it also spawns a
                # rtabmap_util/point_cloud_xyzrgb node
                # (rtabmap_launch/rtabmap.launch.py:381) that rebuilds a
                # coloured cloud from RGB+depth on EVERY frame, purely to feed
                # the "Camera Cloud" display in its bundled rgbd.rviz. That is
                # a permanent extra CPU consumer on the Jetson, on top of the
                # ZED's ~150%. Forwarding our flag here would drag it in.
                'rviz': 'false',
                'wait_for_transform': LaunchConfiguration('wait_for_transform'),
                'sync_queue_size': LaunchConfiguration('sync_queue_size'),
            }.items(),
        )]),

        # --- Live RViz -------------------------------------------------------
        # Our own config, not rtabmap_launch's rgbd.rviz. That one has a
        # MapCloud display on /rtabmap/mapData enabled by default, and mapData
        # carries the FULL RGB+depth payload of every node in the working
        # memory -- RViz decompresses and re-renders all of it, and the cost
        # grows with the map. On this Jetson that is the same class of load
        # that rtabmap_viz was root-caused to in 2026-08: with rtabmap_viz on,
        # load average hit 6.9-10.6, RTAB-Map's own frame time went
        # 0.1759 -> 0.3741 s, and "Could not convert rgb/depth msgs" fired
        # continuously. rviz2 alone was separately measured at 62% CPU (load
        # average 9.39 -> 5.72 when it was closed).
        #
        # So rtabmap.rviz shows only the cheap topics by default -- the 2D grid
        # map, the pose graph, /scan, TF, the robot model, and rtabmap's Info
        # overlay. MapCloud IS in the config but starts DISABLED; tick it on in
        # the Displays panel when you want the 3D cloud and accept the cost.
        # It is also pre-set to decimation 8 / 5 cm voxels / 4 m max depth
        # rather than upstream's 4 / 1 cm / 4 m.
        #
        # MapGraph is the display worth watching while mapping: it draws the
        # optimized pose graph with neighbour links in blue and GLOBAL LOOP
        # CLOSURES IN RED. A run that is working shows red links appearing when
        # you revisit somewhere; a run that is not shows a bare blue chain.
        # That is the single fastest read on whether a mapping run is healthy.
        #
        # BEST OPTION IF YOU HAVE A SECOND MACHINE: do not run this on the
        # Jetson at all. Leave rviz:=false here, and on a laptop on the same
        # network with the same ROS_DOMAIN_ID run
        #     rviz2 -d <this file>
        # Rendering then costs the Jetson nothing. Note the subscription itself
        # is still not free -- several of these topics are published lazily, so
        # a remote subscriber can still trigger work on the robot.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
