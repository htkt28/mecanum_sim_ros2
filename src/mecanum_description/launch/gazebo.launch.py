"""Launch Gazebo Harmonic simulation, ros2_control, EKF IMU fusion, and /cmd_vel bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description for Gazebo Harmonic simulation with LD14 & MPU6050."""
    pkg_path = get_package_share_directory('mecanum_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_core.xacro')
    default_world = os.path.join(pkg_path, 'worlds', 'obstacles.sdf')
    default_rviz_config = os.path.join(pkg_path, 'rviz', 'sim.rviz')
    bridge_params = os.path.join(pkg_path, 'config', 'gz_bridge.yaml')
    ekf_params = os.path.join(pkg_path, 'config', 'ekf.yaml')

    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('rviz')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')
    spawn_yaw = LaunchConfiguration('yaw')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Gazebo world file path (e.g. obstacles.sdf or empty.sdf)',
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 alongside Gazebo (set false when using slam/navigation launch)',
    )
    declare_x_cmd = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X pose')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y pose')
    declare_z_cmd = DeclareLaunchArgument('z', default_value='0.1', description='Spawn Z pose')
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw', default_value='0.0', description='Spawn Yaw angle (rad)'
    )

    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': [world, ' -r']}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mecanum_robot',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_params, 'use_sim_time': True}],
        output='screen',
    )

    # Chuyển đổi /cmd_vel (Twist) chuẩn từ Teleop / Nav2 sang TwistStamped cho bộ điều khiển
    node_twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
        parameters=[{'use_sim_time': True, 'frame_id': 'base_link'}],
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/mecanum_base_controller/reference'),
        ],
    )

    # Bộ lọc EKF hợp nhất vận tốc bánh xe (/wheel/odom) và con quay MPU6050 (/imu/data)
    # để xuất /odom chuẩn và phát biến đổi tọa độ TF odom -> base_link chống trượt
    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}],
        remappings=[('odometry/filtered', '/odom')],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--ros-args', '-p', 'use_sim_time:=true'],
    )

    # Xuất Odometry thô của encoder bánh xe vào /wheel/odom làm đầu vào cho bộ lọc EKF
    mecanum_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mecanum_base_controller',
            '--controller-ros-args',
            '-r /mecanum_base_controller/odometry:=/wheel/odom',
            '--ros-args',
            '-p',
            'use_sim_time:=true',
        ],
    )

    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, mecanum_controller_spawner],
        )
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_world_cmd,
        declare_rviz_cmd,
        declare_x_cmd,
        declare_y_cmd,
        declare_z_cmd,
        declare_yaw_cmd,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
        node_twist_stamper,
        node_ekf,
        delayed_controllers,
        node_rviz,
    ])
