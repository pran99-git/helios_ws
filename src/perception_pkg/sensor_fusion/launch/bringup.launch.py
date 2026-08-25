"""Full Helios sensor bring-up: description + drivers + wheel odometry + EKF.

This is the PERCEPTION layer only: read every sensor, and fuse them into one
odometry estimate that the mapping/localization layer can consume. It stops at
/odometry/filtered -- it deliberately starts no mapper.

TF ownership (REP-105):
  base_link -> sensors/wheels : robot_state_publisher (URDF)
  odom -> base_link           : robot_localization EKF  (ONLY publisher)
  map -> odom                 : NOT owned here -- belongs to the mapping layer

  * wheel_odometry is launched with publish_tf:=false (EKF owns odom->base_link).
  * the ZED wrapper is launched with publish_tf:=false (no odom/map TF from it);
    it still publishes its /odom and /imu TOPICS, which the EKF consumes.
  * the raw IMU is NOT fused separately (it is inside the ZED VIO) -- see ekf.yaml.

Run a mapper separately, on top of this:
  ros2 launch mapping_localization_pkg slam_toolbox.launch.py   # 2D LiDAR SLAM
  ros2 launch mapping_localization_pkg rtabmap.launch.py        # 3D RGB-D SLAM

Toggle parts with: camera:=false lidar:=false rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg = get_package_share_directory('helios_description')
    wheel_pkg = get_package_share_directory('wheel_odometry')
    zed_pkg = get_package_share_directory('zed_wrapper')
    fusion_pkg = get_package_share_directory('sensor_fusion')
    zed_cov_pkg = get_package_share_directory('custom_covariance')
    zed_tune_pkg = get_package_share_directory('zed_custom_tuning')
    lidar_pkg = get_package_share_directory('custom_config')

    use_camera = LaunchConfiguration('camera')
    use_lidar = LaunchConfiguration('lidar')
    use_rviz = LaunchConfiguration('rviz')

    wheel_yaml = os.path.join(wheel_pkg, 'config', 'wheel_odometry.yaml')
    zed_overrides = os.path.join(zed_tune_pkg, 'config', 'zed_overrides.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true',
                              description='Launch the ZED 2i wrapper'),
        DeclareLaunchArgument('lidar', default_value='true',
                              description='Launch the Hokuyo urg_node2 driver'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz'),

        # 1) Robot description -> base_link TF tree (no RViz/GUI here).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'description.launch.py')),
            launch_arguments={'rviz': use_rviz, 'gui': 'false'}.items(),
        ),

        # 2) Wheel odometry -- publishes /wheel/odometry, TF disabled (EKF owns it).
        Node(
            package='wheel_odometry',
            executable='wheel_odometry_node',
            name='wheel_odometry',
            output='screen',
            parameters=[wheel_yaml, {'publish_tf': False}],
        ),

        # 3) ZED 2i -- VIO + IMU topics; TF disabled so it doesn't fight the EKF.
        #
        # enable_ipc:=false is REQUIRED here, not an optimization. With IPC on
        # (the wrapper's default), zed_camera_component_main.cpp's
        # publishCameraTFs() cannot use a StaticTransformBroadcaster (latched
        # QoS is incompatible with intra-process comms), so it republishes the
        # camera's *geometrically static* internal frames (zed_camera_link ->
        # zed_camera_center -> zed_left_camera_frame -> ..._optical) as DYNAMIC
        # transforms on /tf, at grab rate, from the same thread doing depth.
        # Under load that thread slips, the pseudo-static TF goes stale, and
        # every consumer doing a timestamped lookup into that chain fails with
        # "extrapolation into the future" -- which is what was making RTAB-Map
        # discard ~1/3 of all RGB-D frames. IPC buys nothing in this setup
        # anyway: the ZED component is alone in its container, so every
        # consumer is cross-process regardless.
        #
        # ros_params_override_path is how our ZED settings stay OUT of the
        # pinned submodule. The wrapper appends that file after its own
        # common_stereo.yaml/zed2i.yaml, so it wins over them -- but the
        # launch-argument dict below it wins over the file, which is why
        # publish_tf is set here as an argument and NOT in the YAML, where it
        # would be silently ignored. See zed_custom_tuning/README.md.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zed_pkg, 'launch', 'zed_camera.launch.py')),
            launch_arguments={'camera_model': 'zed2i',
                              'publish_tf': 'false',
                              # Static zed_left_camera_frame -> zed_imu_link TF.
                              # RTAB-Map needs it to transform /zed/zed_node/imu/data
                              # into base_link before it can build gravity links;
                              # without it the IMU is silently dropped.
                              #
                              # The launch argument's own description claims this is
                              # "Ignored if publish_tf is False" -- it is not.
                              # publishImuFrameAndTopic()
                              # (zed_camera_component_main.cpp:4749) gates only on
                              # mPublishImuTF and broadcasts a STATIC transform whose
                              # parent is zed_left_camera_frame, which
                              # robot_state_publisher already ties to base_link. No
                              # odom/map TF is involved, so publish_tf:=false above
                              # still holds and the EKF stays the sole owner of
                              # odom->base_link.
                              'publish_imu_tf': 'true',
                              'enable_ipc': 'false',
                              'ros_params_override_path': zed_overrides}.items(),
            condition=IfCondition(use_camera),
        ),

        # 3b) ZED twist covariance -- REQUIRED for the EKF to fuse the camera.
        #
        # The wrapper publishes /zed/zed_node/odom with twist.covariance all
        # zeros and offers no parameter to set it (its SDK has no velocity
        # covariance to copy from). ekf.yaml fuses only the twist, so
        # robot_localization substituted 1e-9 and gave the ZED a Kalman gain of
        # exactly 1.0 -- the camera overwrote the wheel odometry rather than
        # being blended with it, and the same 1e-9 made the rejection gate
        # discard ~20% of measurements. This node republishes the identical data
        # on /zed/odom_with_cov with a real covariance from its own YAML, which
        # is what ekf.yaml's odom1 subscribes to.
        #
        # Tied to the camera condition: without the ZED there is nothing to
        # republish, and ekf.yaml simply runs on wheel odometry alone.
        Node(
            package='custom_covariance',
            executable='zed_odom_covariance_node',
            name='zed_odom_covariance',
            output='screen',
            parameters=[os.path.join(zed_cov_pkg, 'config',
                                     'zed_odom_covariance.yaml')],
            condition=IfCondition(use_camera),
        ),

        # 4) Hokuyo LiDAR -- publishes /scan in frame 'laser'.
        #
        # Uses custom_config's lidar.launch.py, not the one inside the
        # urg_node2 submodule. Upstream's launch file hardcodes its parameter
        # file path with no override, so owning the launch is what makes the
        # LiDAR config live in this workspace instead of in tracked submodule
        # content that `git submodule update` reverts. custom_config sits in
        # LiDAR/ beside the submodule (mirroring Camera/custom_covariance) but
        # is our content, outside the submodule boundary. See its
        # config/urg_node2.yaml.
        #
        # node_name is still passed EXPLICITLY. Launch configurations leak
        # between sibling includes in the same LaunchDescription, and the ZED
        # include above declares its own 'node_name' (default 'zed_node'). By
        # the time this include runs, 'node_name' is already set in the shared
        # context, so a DeclareLaunchArgument default ('urg_node2') is ignored
        # -- it only fills in a value that isn't already present. The LiDAR
        # then came up as /zed_node, putting its lifecycle topic at
        # /zed_node/transition_event and silently breaking anything that
        # addresses it by node name (ros2 param calls, lifecycle transitions).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_pkg, 'launch', 'lidar.launch.py')),
            launch_arguments={'node_name': 'urg_node2'}.items(),
            condition=IfCondition(use_lidar),
        ),

        # 5) Fusion: EKF (odom->base_link). No mapper -- see module docstring.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fusion_pkg, 'launch', 'ekf.launch.py')),
        ),
    ])
