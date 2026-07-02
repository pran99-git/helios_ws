import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('wheel_odometry'),
        'config',
        'wheel_odometry.yaml',
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the wheel_odometry parameter file.',
    )

    node = Node(
        package='wheel_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([config_arg, node])
