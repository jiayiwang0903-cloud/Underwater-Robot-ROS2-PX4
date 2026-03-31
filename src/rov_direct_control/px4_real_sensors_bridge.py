#!/usr/bin/env python3
"""Bridge PX4 topics to EKF real-sensor topics.

Publishes:
- /sensor/imu_real  (sensor_msgs/Imu)
- /sensor/dvl_px4_fallback (geometry_msgs/TwistWithCovarianceStamped)
- /sensor/depth     (geometry_msgs/PoseWithCovarianceStamped)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from px4_msgs.msg import SensorCombined, VehicleAttitude, VehicleLocalPosition


class Px4RealSensorsBridge(Node):
    def __init__(self):
        super().__init__('px4_real_sensors_bridge')

        self._last_att_q = [1.0, 0.0, 0.0, 0.0]

        self.pub_imu = self.create_publisher(Imu, '/sensor/imu_real', 10)
        self.pub_dvl = self.create_publisher(TwistWithCovarianceStamped, '/sensor/dvl_px4_fallback', 10)
        self.pub_depth = self.create_publisher(PoseWithCovarianceStamped, '/sensor/depth', 10)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self._att_cb, px4_qos)
        self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self._sensor_combined_cb, px4_qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._local_pos_cb, px4_qos)

        self.get_logger().info(
            'PX4 real-sensor bridge started: '
            '/fmu/out/sensor_combined -> /sensor/imu_real, '
            '/fmu/out/vehicle_local_position_v1 -> /sensor/dvl_px4_fallback,/sensor/depth'
        )

    def _att_cb(self, msg: VehicleAttitude):
        self._last_att_q = [float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])]

    def _sensor_combined_cb(self, msg: SensorCombined):
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'

        imu.orientation.w = self._last_att_q[0]
        imu.orientation.x = self._last_att_q[1]
        imu.orientation.y = self._last_att_q[2]
        imu.orientation.z = self._last_att_q[3]

        imu.angular_velocity.x = float(msg.gyro_rad[0])
        imu.angular_velocity.y = float(msg.gyro_rad[1])
        imu.angular_velocity.z = float(msg.gyro_rad[2])

        imu.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        imu.linear_acceleration.y = float(msg.accelerometer_m_s2[1])
        imu.linear_acceleration.z = float(msg.accelerometer_m_s2[2])

        # Conservative default covariances, tune with real logs.
        imu.orientation_covariance[0] = 0.01
        imu.orientation_covariance[4] = 0.01
        imu.orientation_covariance[8] = 0.01
        imu.angular_velocity_covariance[0] = 0.01
        imu.angular_velocity_covariance[4] = 0.01
        imu.angular_velocity_covariance[8] = 0.01
        imu.linear_acceleration_covariance[0] = 0.04
        imu.linear_acceleration_covariance[4] = 0.04
        imu.linear_acceleration_covariance[8] = 0.04

        self.pub_imu.publish(imu)

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        stamp = self.get_clock().now().to_msg()

        dvl = TwistWithCovarianceStamped()
        dvl.header.stamp = stamp
        dvl.header.frame_id = 'base_link'
        dvl.twist.twist.linear.x = float(msg.vx)
        dvl.twist.twist.linear.y = float(msg.vy)
        dvl.twist.twist.linear.z = float(msg.vz)
        dvl.twist.covariance[0] = 0.05
        dvl.twist.covariance[7] = 0.05
        dvl.twist.covariance[14] = 0.05
        self.pub_dvl.publish(dvl)

        depth = PoseWithCovarianceStamped()
        depth.header.stamp = stamp
        depth.header.frame_id = 'odom'
        depth.pose.pose.position.z = float(msg.z)
        depth.pose.covariance[14] = 0.05
        self.pub_depth.publish(depth)


def main(args=None):
    rclpy.init(args=args)
    node = Px4RealSensorsBridge()
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
