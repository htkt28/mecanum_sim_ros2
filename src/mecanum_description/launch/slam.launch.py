"""Launch Online Async SLAM Toolbox and RViz2 for mapping with the Mecanum robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for SLAM Toolbox mapping."""
    pkg_path = get_package_share_directory('mecanum_description')
    default_slam_params = os.path.join(pkg_path, 'config', 'slam_params.yaml')
    default_rviz_config = os.path.join(pkg_path, 'rviz', 'mecanum.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_rviz = LaunchConfiguration('rviz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )
    declare_slam_params_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params,
        description='Full path to SLAM Toolbox parameters YAML file',
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 configured for SLAM and Navigation',
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_slam_params_cmd,
        declare_rviz_cmd,
        slam_toolbox_launch,
        node_rviz,
    ])
