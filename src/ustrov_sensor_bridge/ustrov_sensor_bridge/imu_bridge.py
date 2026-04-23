#!/usr/bin/env python3
"""PX4 IMU bridge — 将 PX4 的 IMU 数据转换为 EKF 可融合的标准 ROS 消息。

合并 SensorCombined（角速度 + 线加速度）与 VehicleAttitude（姿态四元数），
发布为统一的 sensor_msgs/Imu 消息。

Input / 输入:
  /fmu/out/sensor_combined  (px4_msgs/SensorCombined)  — 角速度 + 线加速度
  /fmu/out/vehicle_attitude (px4_msgs/VehicleAttitude)  — 姿态四元数 (FRD→NED)

Output / 输出:
  /sensor/imu               (sensor_msgs/Imu)           — EKF 标准 IMU 输入
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from px4_msgs.msg import SensorCombined, VehicleAttitude


class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_imu = self.create_publisher(Imu, '/sensor/imu', 10)

        # 最新姿态缓存
        self._orientation_w = 1.0
        self._orientation_x = 0.0
        self._orientation_y = 0.0
        self._orientation_z = 0.0
        self._has_attitude = False

        self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined',
            self._sensor_combined_cb, qos)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self._attitude_cb, qos)

        self._pub_count = 0
        self.create_timer(5.0, self._stats_timer)
        self.get_logger().info(
            'ImuBridge: /fmu/out/sensor_combined + /fmu/out/vehicle_attitude -> /sensor/imu')

    _QUAT_NORM_TOL = 0.01

    def _attitude_cb(self, msg: VehicleAttitude):
        """缓存最新姿态四元数 (PX4 Hamilton: w,x,y,z)，校验范数。"""
        w, x, y, z = float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if abs(norm - 1.0) > self._QUAT_NORM_TOL:
            self.get_logger().warn(
                f'VehicleAttitude 四元数范数异常: norm={norm:.6f}, '
                f'q=[{w:.4f},{x:.4f},{y:.4f},{z:.4f}], 丢弃该帧')
            return
        self._orientation_w = w
        self._orientation_x = x
        self._orientation_y = y
        self._orientation_z = z
        self._has_attitude = True

    # PX4 SensorCombined 中加速度计时间戳的无效哨兵值
    _ACCEL_TIMESTAMP_INVALID = 2147483647

    def _sensor_combined_cb(self, msg: SensorCombined):
        """收到 gyro+accel 时，与缓存姿态合并发布 Imu。"""
        # 在收到第一个有效 attitude 之前不发布，避免 EKF 用不完整数据初始化
        if not self._has_attitude:
            return

        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'

        # 姿态（此处 _has_attitude 必为 True）
        if self._has_attitude:
            out.orientation.w = self._orientation_w
            out.orientation.x = self._orientation_x
            out.orientation.y = self._orientation_y
            out.orientation.z = self._orientation_z
            # 姿态协方差（小值表示可信）
            out.orientation_covariance[0] = 0.01
            out.orientation_covariance[4] = 0.01
            out.orientation_covariance[8] = 0.01
        else:
            # 姿态不可用时，设 -1 告知 EKF 忽略
            out.orientation_covariance[0] = -1.0

        # 角速度 (FRD body frame, rad/s) — 始终有效
        out.angular_velocity.x = float(msg.gyro_rad[0])
        out.angular_velocity.y = float(msg.gyro_rad[1])
        out.angular_velocity.z = float(msg.gyro_rad[2])
        out.angular_velocity_covariance[0] = 0.001
        out.angular_velocity_covariance[4] = 0.001
        out.angular_velocity_covariance[8] = 0.001

        # 线加速度 (FRD body frame, m/s^2)
        # accelerometer_timestamp_relative == INVALID 时数据无效，设协方差 -1 告知 EKF 忽略
        accel_valid = (msg.accelerometer_timestamp_relative
                       != self._ACCEL_TIMESTAMP_INVALID)
        if accel_valid:
            out.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
            out.linear_acceleration.y = float(msg.accelerometer_m_s2[1])
            out.linear_acceleration.z = float(msg.accelerometer_m_s2[2])
            out.linear_acceleration_covariance[0] = 0.1
            out.linear_acceleration_covariance[4] = 0.1
            out.linear_acceleration_covariance[8] = 0.1
        else:
            out.linear_acceleration_covariance[0] = -1.0

        self.pub_imu.publish(out)
        self._pub_count += 1

    def _stats_timer(self):
        self.get_logger().info(f'ImuBridge: published={self._pub_count}, attitude_ready={self._has_attitude}')


def main(args=None):
    rclpy.init(args=args)
    node = ImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
