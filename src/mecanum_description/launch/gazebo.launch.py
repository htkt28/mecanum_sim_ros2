import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_path = get_package_share_directory('mecanum_description')
    
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_core.xacro')
    
    # Bọc kết quả xacro vào ParameterValue(value_type=str) để chống lỗi parse YAML
    robot_description_config = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    
    params = {'robot_description': robot_description_config, 'use_sim_time': True}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # [MỚI 1] Phát TF tĩnh nối base_link với base_footprint cho SLAM Toolbox và Nav2
    node_base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        parameters=[{'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'mecanum_robot',
                   '-z', '0.1']
    )

    bridge_params = os.path.join(pkg_path, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_params, 'use_sim_time': True}],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--ros-args', '-p', 'use_sim_time:=true'],
    )

    # [MỚI 2] Remap topic tf_odometry của bộ điều khiển Mecanum về /tf chuẩn
    mecanum_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mecanum_base_controller',
            '--controller-ros-args',
            '-r /mecanum_base_controller/tf_odometry:=/tf',
            '--ros-args',
            '-p',
            'use_sim_time:=true'
        ],
    )

    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, mecanum_controller_spawner]
        )
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        node_base_footprint_tf,
        spawn_entity,
        ros_gz_bridge,
        delayed_controllers
    ])