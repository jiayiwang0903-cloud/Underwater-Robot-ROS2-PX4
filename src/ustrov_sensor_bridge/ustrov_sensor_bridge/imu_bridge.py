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

    def _attitude_cb(self, msg: VehicleAttitude):
        """缓存最新姿态四元数 (PX4 Hamilton: w,x,y,z)。"""
        self._orientation_w = float(msg.q[0])
        self._orientation_x = float(msg.q[1])
        self._orientation_y = float(msg.q[2])
        self._orientation_z = float(msg.q[3])
        self._has_attitude = True

    def _sensor_combined_cb(self, msg: SensorCombined):
        """收到 gyro+accel 时，与缓存姿态合并发布 Imu。"""
        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'

        # 姿态
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

        # 角速度 (FRD body frame, rad/s)
        out.angular_velocity.x = float(msg.gyro_rad[0])
        out.angular_velocity.y = float(msg.gyro_rad[1])
        out.angular_velocity.z = float(msg.gyro_rad[2])
        out.angular_velocity_covariance[0] = 0.001
        out.angular_velocity_covariance[4] = 0.001
        out.angular_velocity_covariance[8] = 0.001

        # 线加速度 (FRD body frame, m/s^2)
        out.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        out.linear_acceleration.y = float(msg.accelerometer_m_s2[1])
        out.linear_acceleration.z = float(msg.accelerometer_m_s2[2])
        out.linear_acceleration_covariance[0] = 0.1
        out.linear_acceleration_covariance[4] = 0.1
        out.linear_acceleration_covariance[8] = 0.1

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
