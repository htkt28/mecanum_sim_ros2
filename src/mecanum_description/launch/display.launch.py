"""Launch RViz2 and robot_state_publisher to inspect the Mecanum URDF model."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description for URDF visualization."""
    pkg_path = get_package_share_directory('mecanum_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_core.xacro')
    default_rviz_config = os.path.join(pkg_path, 'rviz', 'display.rviz')

    use_gui = LaunchConfiguration('gui')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start joint_state_publisher_gui for interactive wheel testing',
    )

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RViz configuration file',
    )

    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}],
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui),
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        declare_gui_cmd,
        declare_rviz_config_cmd,
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz,
    ])
