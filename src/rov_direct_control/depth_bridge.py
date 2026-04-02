#!/usr/bin/env python3
"""Pressure-to-depth bridge for EKF.

Subscribes PX4 barometer pressure and publishes EKF-friendly depth pose:
- Input:  /fmu/out/sensor_baro (px4_msgs/SensorBaro)
- Output: /sensor/depth (geometry_msgs/PoseWithCovarianceStamped)

Depth is computed from hydrostatic equation:
    depth = (P - P0) / (rho * g)
where P0 is surface pressure baseline.
"""

from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import SensorBaro


class DepthBridge(Node):
    def __init__(self):
        super().__init__('depth_bridge')

        self.declare_parameter('pressure_topic', '/fmu/out/sensor_baro') #默认订阅 PX4 的气压计数据
        self.declare_parameter('depth_topic', '/sensor/depth') #发布深度数据，供 EKF 使用
        self.declare_parameter('water_density', 1025.0)  # kg/m^3 密度
        self.declare_parameter('gravity', 9.80665)  # m/s^2
        self.declare_parameter('surface_pressure_pa', -1.0)  #如果提供了正值，则直接使用该值作为水面压力基准，否则自动从前 N 个样本平均计算 P0
        self.declare_parameter('calibration_samples', 80) #样本数量，前80个样本平均作为水面压力基准
        self.declare_parameter('depth_covariance_z', 0.05)  #深度测量的协方差，供 EKF 使用
        self.declare_parameter('min_depth_m', 0.0)  #最小深度限制，防止水面扰动导致的负深度值
        self.declare_parameter('frame_id', 'odom')  #深度坐标系

        self.pressure_topic = self.get_parameter('pressure_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.rho = float(self.get_parameter('water_density').value)
        self.g = float(self.get_parameter('gravity').value)
        self.surface_pressure_pa = float(self.get_parameter('surface_pressure_pa').value)
        self.calibration_samples = int(self.get_parameter('calibration_samples').value)
        self.depth_covariance_z = float(self.get_parameter('depth_covariance_z').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self._pressure_window = deque(maxlen=max(1, self.calibration_samples))
        self._surface_pressure_ready = self.surface_pressure_pa > 0.0
        self._msg_count = 0 #接收的气压样本数量
        self._pub_count = 0 #发布的深度消息数量
        self._warned_no_data = False
        self._last_pressure_pa = 0.0 #最后一个接收到的压力值
        self._last_depth_m = 0.0 #最后一个发布的深度值


        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # 发布深度消息，供 EKF 订阅
        self.pub_depth = self.create_publisher(
            PoseWithCovarianceStamped, self.depth_topic, 10)
        self.sub_pressure = self.create_subscription(
            SensorBaro,
            self.pressure_topic,
            self._pressure_cb,
            qos,
        )
        
        self.create_timer(1.0, self._stats_timer)
        
        if self._surface_pressure_ready:
            self.get_logger().info(
                f'DepthBridge: using fixed surface pressure P0={self.surface_pressure_pa:.1f} Pa'
            )
        else:
            self.get_logger().info(
                f'DepthBridge: auto-calibrating P0 from first {self.calibration_samples} samples'
            )

        self.get_logger().info(
            f'DepthBridge: {self.pressure_topic} -> {self.depth_topic}, rho={self.rho}, g={self.g}'
        )


    def _pressure_cb(self, msg: SensorBaro):
        self._msg_count += 1
        pressure_pa = float(msg.pressure)
        self._last_pressure_pa = pressure_pa

        if not self._surface_pressure_ready:
            self._pressure_window.append(pressure_pa)
            if len(self._pressure_window) >= self.calibration_samples:
                self.surface_pressure_pa = sum(self._pressure_window) / len(self._pressure_window)
                self._surface_pressure_ready = True
                self.get_logger().info(
                    f'DepthBridge: P0 calibrated at {self.surface_pressure_pa:.1f} Pa'
                )
            return

        depth_m = (pressure_pa - self.surface_pressure_pa) / (self.rho * self.g)
        depth_m = max(depth_m, self.min_depth_m)
        self._last_depth_m = depth_m

        depth_msg = PoseWithCovarianceStamped()
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = self.frame_id
        depth_msg.pose.pose.position.z = float(depth_m)
        depth_msg.pose.covariance[14] = self.depth_covariance_z

        self.pub_depth.publish(depth_msg)
        self._pub_count += 1

    def _stats_timer(self):
        if self._msg_count == 0:
            if not self._warned_no_data:
                topics = self.get_topic_names_and_types()
                fmu_out_topics = sorted(name for name, _types in topics if name.startswith('/fmu/out/'))
                baro_topics = sorted(name for name, _types in topics if 'baro' in name.lower())

                self.get_logger().warn(
                    f'No pressure samples received on {self.pressure_topic}. '
                    'DepthBridge is waiting for PX4 barometer topic.'
                )
                if baro_topics:
                    self.get_logger().warn(f'Baro-related topics discovered: {baro_topics}')
                else:
                    self.get_logger().warn('No baro-related ROS topics discovered under current graph.')

                if fmu_out_topics:
                    self.get_logger().warn(
                        f'Available /fmu/out topics snapshot ({len(fmu_out_topics)}): {fmu_out_topics}'
                    )

                self.get_logger().warn(
                    'Check PX4 shell: `listener sensor_baro 5` to confirm the sensor is detected by PX4. '
                    'If PX4 has sensor_baro but ROS topic is missing, add sensor_baro to uXRCE-DDS exported topics.'
                )
                self._warned_no_data = True

        if self._surface_pressure_ready:
            self.get_logger().info(
                f'DepthBridge: pressure_rx={self._msg_count}, depth_tx={self._pub_count}, '
                f'P0={self.surface_pressure_pa:.1f} Pa, P={self._last_pressure_pa:.1f} Pa, '
                f'depth={self._last_depth_m:.3f} m'
            )
        else:
            self.get_logger().info(
                f'DepthBridge: calibrating... {len(self._pressure_window)}/{self.calibration_samples} samples'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DepthBridge()
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
