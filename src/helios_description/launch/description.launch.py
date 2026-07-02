import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('helios_description')
    xacro_file = os.path.join(pkg, 'urdf', 'helios.urdf.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'view.rviz')

    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('rviz')

    # Process the xacro at launch time into the robot_description string.
    robot_description = ParameterValue(Command(['xacro ', xacro_file]),
                                       value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Use joint_state_publisher_gui sliders for the wheels'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz with the rover view'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Wheels are continuous joints -> need joint states. GUI sliders by
        # default; plain publisher (all zeros) when gui:=false.
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(gui),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
