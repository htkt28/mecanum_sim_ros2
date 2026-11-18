#!/usr/bin/env python3
"""ROS 2 Serial Bridge Node connecting Raspberry Pi 5 to ESP32 Mecanum + MPU6050 controller."""

import math
import time

from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
import serial


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert planar yaw angle (radians) into a ROS geometry_msgs/Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class Esp32MecanumBridge(Node):
    """Bidirectional USB-Serial bridge between ROS 2 and ESP32 motor/IMU controller."""

    def __init__(self) -> None:
        """Initialize ROS 2 publishers, subscribers, parameters, and serial port."""
        super().__init__('esp32_mecanum_bridge')

        self.declare_parameter('serial_port', '/dev/esp32')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('cmd_timeout_sec', 1.5)

        self.port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.imu_frame = self.get_parameter('imu_frame_id').get_parameter_value().string_value
        self.cmd_timeout = self.get_parameter('cmd_timeout_sec').get_parameter_value().double_value

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 20)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 20)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 20)

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.ser = None
        self.last_reconnect_attempt = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw_odom = 0.0
        self.yaw_imu = 0.0
        self.gyro_bias_z = 0.0
        self.bias_samples = 0
        self.last_odom_time = None

        self.latest_cmd = (0.0, 0.0, 0.0)
        self.last_cmd_msg_time = 0.0
        self.cmd_active = False

        self.wheel_joints = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]

        self.connect_serial()
        # Vòng lặp đọc Telemetry 100 Hz và gửi giữ nhịp lệnh /cmd_vel 20 Hz
        self.read_timer = self.create_timer(0.01, self.read_serial_loop)
        self.cmd_timer = self.create_timer(0.05, self.send_cmd_keepalive)

    def connect_serial(self) -> None:
        """Attempt to open the USB serial connection to the ESP32."""
        now = time.monotonic()
        if now - self.last_reconnect_attempt < 2.0:
            return
        self.last_reconnect_attempt = now
        try:
            ser = serial.Serial()
            ser.port = self.port_name
            ser.baudrate = self.baudrate
            ser.timeout = 0.005
            ser.dtr = False
            ser.rts = False
            ser.open()
            ser.reset_input_buffer()
            self.ser = ser
            self.get_logger().info(
                f'Connected to ESP32 on {self.port_name} @ {self.baudrate} baud'
            )
        except serial.SerialException as exc:
            self.ser = None
            self.get_logger().warn(
                f'Waiting for ESP32 on {self.port_name}: {exc}',
                throttle_duration_sec=5.0,
            )

    def write_line(self, text: str) -> None:
        """Write an ASCII command line safely to the ESP32 serial port."""
        if self.ser is None or not self.ser.is_open:
            return
        try:
            self.ser.write(text.encode('ascii'))
        except serial.SerialException as exc:
            self.get_logger().error(f'ESP32 serial write error: {exc}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Forward incoming /cmd_vel Twist immediately to ESP32."""
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)
        self.latest_cmd = (vx, vy, wz)
        self.last_cmd_msg_time = time.monotonic()
        self.cmd_active = (abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(wz) > 1e-4)
        self.write_line(f'V,{vx:.3f},{vy:.3f},{wz:.3f}\n')

    def send_cmd_keepalive(self) -> None:
        """Stop motors cleanly if /cmd_vel times out."""
        if not self.cmd_active:
            return
        elapsed = time.monotonic() - self.last_cmd_msg_time
        if elapsed > self.cmd_timeout:
            self.latest_cmd = (0.0, 0.0, 0.0)
            self.cmd_active = False
            self.write_line('V,0.000,0.000,0.000\n')
        else:
            vx, vy, wz = self.latest_cmd
            self.write_line(f'V,{vx:.3f},{vy:.3f},{wz:.3f}\n')

    def read_serial_loop(self) -> None:
        """Read and parse all pending telemetry lines from the ESP32."""
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()
            return

        try:
            while self.ser.in_waiting > 0:
                raw = self.ser.readline()
                if not raw:
                    break
                line = raw.decode('ascii', errors='ignore').strip()
                if line:
                    self.handle_line(line)
        except serial.SerialException as exc:
            self.get_logger().error(f'ESP32 serial read error: {exc}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def handle_line(self, line: str) -> None:
        """Parse a telemetry frame D,vx,vy,wz,yaw,gz,ax,ay,az,fl,fr,rl,rr."""
        if line.startswith('#'):
            self.get_logger().info(f'ESP32: {line}')
            if 'MPU6050_READY' in line:
                self.x = 0.0
                self.y = 0.0
                self.yaw_odom = 0.0
                self.yaw_imu = 0.0
                self.gyro_bias_z = 0.0
                self.bias_samples = 0
                self.last_odom_time = None
                self.write_line('R\n')
            return
        if not line.startswith('D,'):
            return

        parts = line.split(',')
        if len(parts) != 13:
            return

        try:
            vals = [float(x) for x in parts[1:]]
        except ValueError:
            return

        vx, vy, wz = vals[0], vals[1], vals[2]
        _, gz_rad_s = vals[3], vals[4]
        ax, ay, az = vals[5], vals[6], vals[7]
        pos_fl, pos_fr, pos_rl, pos_rr = vals[8], vals[9], vals[10], vals[11]

        now_msg = self.get_clock().now().to_msg()
        now_mono = time.monotonic()

        dt = 0.0
        if self.last_odom_time is not None:
            dt = max(0.0, min(0.2, now_mono - self.last_odom_time))
            cos_y = math.cos(self.yaw_odom)
            sin_y = math.sin(self.yaw_odom)
            self.x += (vx * cos_y - vy * sin_y) * dt
            self.y += (vx * sin_y + vy * cos_y) * dt
            self.yaw_odom += wz * dt
        self.last_odom_time = now_mono

        # Thuật toán ZUPT (Zero-Velocity Update) khử trôi Gyro MPU6050 khi xe đứng yên
        wheels_stationary = (
            not self.cmd_active
            and abs(vx) < 0.002
            and abs(vy) < 0.002
            and abs(wz) < 0.005
        )
        if wheels_stationary and self.bias_samples < 40:
            self.gyro_bias_z = (
                (self.gyro_bias_z * self.bias_samples) + gz_rad_s
            ) / (self.bias_samples + 1)
            self.bias_samples += 1
            gz_clean = 0.0
        elif wheels_stationary and abs(gz_rad_s - self.gyro_bias_z) < 0.35:
            self.gyro_bias_z = 0.95 * self.gyro_bias_z + 0.05 * gz_rad_s
            gz_clean = 0.0
        else:
            gz_clean = gz_rad_s - self.gyro_bias_z
            if abs(gz_clean) < 0.008:
                gz_clean = 0.0
            self.yaw_imu += gz_clean * dt

        # 1. Xuất /wheel/odom (hiệp phương sai chuẩn cho bộ lọc EKF)
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw_odom)
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.03
        odom.pose.covariance[35] = 0.05
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance[0] = 0.005
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        # 2. Xuất /imu/data từ MPU6050 đã khử trôi ZUPT
        imu = Imu()
        imu.header.stamp = now_msg
        imu.header.frame_id = self.imu_frame
        imu.orientation = yaw_to_quaternion(self.yaw_imu)
        imu.orientation_covariance[0] = 0.01
        imu.orientation_covariance[4] = 0.01
        imu.orientation_covariance[8] = 0.005
        imu.angular_velocity.z = gz_clean
        imu.angular_velocity_covariance[8] = 0.001
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.linear_acceleration_covariance[0] = 0.05
        imu.linear_acceleration_covariance[4] = 0.05
        imu.linear_acceleration_covariance[8] = 0.05
        self.imu_pub.publish(imu)

        # 3. Xuất /joint_states góc quay thực của 4 bánh xe
        js = JointState()
        js.header.stamp = now_msg
        js.name = self.wheel_joints
        js.position = [pos_fl, pos_fr, pos_rl, pos_rr]
        self.joint_pub.publish(js)


def main(args=None) -> None:
    """Entry point for esp32_mecanum_bridge node."""
    rclpy.init(args=args)
    node = Esp32MecanumBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.write_line('V,0.000,0.000,0.000\n')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
