"""Full Helios bring-up: description + drivers + wheel odometry + fusion.

TF ownership (REP-105):
  base_link -> sensors/wheels : robot_state_publisher (URDF)
  odom -> base_link           : robot_localization EKF  (ONLY publisher)
  map -> odom                 : slam_toolbox

  * wheel_odometry is launched with publish_tf:=false (EKF owns odom->base_link).
  * the ZED wrapper is launched with publish_tf:=false (no odom/map TF from it);
    it still publishes its /odom and /imu TOPICS, which the EKF consumes.
  * the raw IMU is NOT fused separately (it is inside the ZED VIO) -- see ekf.yaml.

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
    urg_pkg = get_package_share_directory('urg_node2')
    fusion_pkg = get_package_share_directory('sensor_fusion')

    use_camera = LaunchConfiguration('camera')
    use_lidar = LaunchConfiguration('lidar')
    use_rviz = LaunchConfiguration('rviz')

    wheel_yaml = os.path.join(wheel_pkg, 'config', 'wheel_odometry.yaml')

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zed_pkg, 'launch', 'zed_camera.launch.py')),
            launch_arguments={'camera_model': 'zed2i',
                              'publish_tf': 'false',
                              'enable_ipc': 'false'}.items(),
            condition=IfCondition(use_camera),
        ),

        # 4) Hokuyo LiDAR -- publishes /scan in frame 'laser'.
        #
        # node_name is passed EXPLICITLY on purpose. Launch configurations leak
        # between sibling includes in the same LaunchDescription, and the ZED
        # include above declares its own 'node_name' (default 'zed_node'). By
        # the time urg_node2.launch.py runs, 'node_name' is already set in the
        # shared context, so its own DeclareLaunchArgument default ('urg_node2')
        # is ignored -- DeclareLaunchArgument only fills in a value that isn't
        # already present. The LiDAR then came up as /zed_node, putting its
        # lifecycle topic at /zed_node/transition_event and silently breaking
        # anything that addresses it by node name (params files keyed
        # 'urg_node2:', ros2 param calls, lifecycle transitions).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(urg_pkg, 'launch', 'urg_node2.launch.py')),
            launch_arguments={'node_name': 'urg_node2'}.items(),
            condition=IfCondition(use_lidar),
        ),

        # 5) Fusion: EKF (odom->base_link) + slam_toolbox (map->odom).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fusion_pkg, 'launch', 'fusion.launch.py')),
            launch_arguments={'slam': use_lidar}.items(),
        ),
    ])
