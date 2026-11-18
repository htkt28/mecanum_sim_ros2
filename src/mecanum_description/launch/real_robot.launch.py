"""Launch real Mecanum hardware bringup on Raspberry Pi 5 with automatic USB port detection."""

import glob
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import serial


def detect_usb_ports(lidar_arg: str, esp32_arg: str):
    """Auto-detect which ttyUSB/ttyACM port belongs to LD14 LiDAR vs ESP32."""
    if lidar_arg != 'auto' and esp32_arg != 'auto':
        return lidar_arg, esp32_arg

    candidates = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    detected_lidar = None
    other_ports = []

    for port in candidates:
        try:
            s = serial.Serial()
            s.port = port
            s.baudrate = 115200
            s.timeout = 0.2
            s.dtr = False
            s.rts = False
            s.open()
            time.sleep(0.18)
            data = s.read(128)
            s.close()
            if b'\x54\x2c' in data:
                detected_lidar = port
            else:
                other_ports.append(port)
        except Exception:
            other_ports.append(port)

    final_lidar = lidar_arg if lidar_arg != 'auto' else (detected_lidar or '/dev/ttyUSB0')
    remaining = [p for p in candidates if p != final_lidar]
    final_esp32 = esp32_arg if esp32_arg != 'auto' else (
        other_ports[0] if other_ports else (remaining[0] if remaining else '/dev/ttyUSB1')
    )
    return final_lidar, final_esp32


def launch_setup(context, *args, **kwargs):
    """Resolve USB ports and construct hardware bringup nodes."""
    pkg_path = get_package_share_directory('mecanum_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_core.xacro')
    ekf_config_file = os.path.join(pkg_path, 'config', 'ekf.yaml')

    lidar_arg = LaunchConfiguration('lidar_port').perform(context)
    esp32_arg = LaunchConfiguration('esp32_port').perform(context)
    lidar_port, esp32_port = detect_usb_ports(lidar_arg, esp32_arg)

    use_lidar = LaunchConfiguration('use_lidar')
    use_esp32 = LaunchConfiguration('use_esp32')
    use_ekf = LaunchConfiguration('use_ekf')

    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    log_ports = LogInfo(
        msg=f'[Auto-USB] LiDAR LD14 -> {lidar_port} | ESP32 Controller -> {esp32_port}'
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': False,
        }],
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=UnlessCondition(use_esp32),
    )

    node_esp32_bridge = Node(
        package='mecanum_description',
        executable='esp32_bridge.py',
        name='esp32_mecanum_bridge',
        output='screen',
        parameters=[{
            'serial_port': esp32_port,
            'baudrate': 115200,
            'use_sim_time': False,
        }],
        condition=IfCondition(use_esp32),
    )

    node_ld14_lidar = Node(
        package='ldlidar_sl_ros2',
        executable='ldlidar_sl_ros2_node',
        name='ldlidar_publisher_ld14',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD14'},
            {'laser_scan_topic_name': 'scan'},
            {'point_cloud_2d_topic_name': 'pointcloud2d'},
            {'frame_id': 'laser_frame'},
            {'port_name': lidar_port},
            {'serial_baudrate': 115200},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0},
        ],
        condition=IfCondition(use_lidar),
    )

    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': False}],
        remappings=[('odometry/filtered', '/odom')],
        condition=IfCondition(use_ekf),
    )

    return [
        log_ports,
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_esp32_bridge,
        node_ld14_lidar,
        node_ekf,
    ]


def generate_launch_description():
    """Generate launch description for real Mecanum robot bringup on Raspberry Pi."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_port',
            default_value='auto',
            description='Serial port for LD14 LiDAR (or auto)',
        ),
        DeclareLaunchArgument(
            'esp32_port',
            default_value='auto',
            description='Serial port for ESP32 controller (or auto)',
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Start LD14 LiDAR driver node',
        ),
        DeclareLaunchArgument(
            'use_esp32',
            default_value='true',
            description='Start ESP32 USB-Serial bridge node',
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Start EKF node fusing /wheel/odom and /imu/data',
        ),
        OpaqueFunction(function=launch_setup),
    ])
