"""Full Helios bring-up: description + drivers + wheel odometry + fusion.

TF ownership (REP-105), with the gotchas handled:
  base_link -> sensors/wheels : robot_state_publisher (URDF)
  odom -> base_link           : robot_localization EKF  (ONLY publisher)
  map -> odom                 : slam_toolbox

Gotcha handling:
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zed_pkg, 'launch', 'zed_camera.launch.py')),
            launch_arguments={'camera_model': 'zed2i',
                              'publish_tf': 'false'}.items(),
            condition=IfCondition(use_camera),
        ),

        # 4) Hokuyo LiDAR -- publishes /scan in frame 'laser'.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(urg_pkg, 'launch', 'urg_node2.launch.py')),
            condition=IfCondition(use_lidar),
        ),

        # 5) Fusion: EKF (odom->base_link) + slam_toolbox (map->odom).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fusion_pkg, 'launch', 'fusion.launch.py')),
            launch_arguments={'slam': use_lidar}.items(),
        ),
    ])
