"""Launch Nav2 stack (AMCL + MPPI Omni holonomic controller) and RViz2 for Mecanum robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Nav2 holonomic navigation."""
    pkg_path = get_package_share_directory('mecanum_description')
    default_map = os.path.join(pkg_path, 'maps', 'my_gazebo_map.yaml')
    default_params = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(pkg_path, 'rviz', 'mecanum.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('rviz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map YAML file to load',
    )
    declare_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to Nav2 parameters YAML file',
    )
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Run SLAM simultaneously with Nav2 instead of AMCL static map localization',
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically transition Nav2 lifecycle nodes to active',
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 configured for Navigation',
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            )
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'slam': slam,
            'autostart': autostart,
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
        declare_map_cmd,
        declare_params_cmd,
        declare_slam_cmd,
        declare_autostart_cmd,
        declare_rviz_cmd,
        nav2_bringup_launch,
        node_rviz,
    ])
