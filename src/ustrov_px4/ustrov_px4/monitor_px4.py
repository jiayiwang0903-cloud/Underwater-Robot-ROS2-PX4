#!/usr/bin/env python3
"""实时监控 PX4/估计器关键传感器数据。"""

import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from px4_msgs.msg import VehicleImu, VehicleAttitude, SensorCombined, SensorBaro


@dataclass
class TimedValue:
    stamp: float = 0.0


class Px4SensorMonitor(Node):
    def __init__(self):
        super().__init__('px4_sensor_monitor')
        self._odom = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'yaw': 0.0,
            'age': TimedValue(),
        }
        self._dvl = {
            'vx': 0.0,
            'vy': 0.0,
            'vz': 0.0,
            'age': TimedValue(),
        }
        self._depth = {
            'z': 0.0,
            'age': TimedValue(),
        }

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/sensor/dvl', self._dvl_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/sensor/depth', self._depth_cb, 10)

        self.create_timer(0.2, self._print_dashboard)  # 5 Hz
        self.get_logger().info('Sensor fusion started: monitoring IMU/ATT/BARO/DEPTH/DVL/ODOM')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9
    
    def _euler_from_quaternion(self, w, x, y, z): # PX4 的四元数顺序是 (w, x, y, z)，而 ROS 通常是 (x, y, z, w)，注意区分
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    def _odom_cb(self, msg: Odometry): #EKF融合后的里程计数据，包含位置和姿态（以四元数形式），需要转换成欧拉角以便显示
        self._odom['x'] = msg.pose.pose.position.x
        self._odom['y'] = msg.pose.pose.position.y
        self._odom['z'] = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        _, _, yaw = self._euler_from_quaternion(q.w, q.x, q.y, q.z)
        self._odom['yaw'] = yaw
        self._odom['age'].stamp = self._now_sec()

    def _dvl_cb(self, msg: TwistWithCovarianceStamped): #DVL 测量的速度数据，直接使用线速度部分，忽略角速度
        self._dvl['vx'] = msg.twist.twist.linear.x
        self._dvl['vy'] = msg.twist.twist.linear.y
        self._dvl['vz'] = msg.twist.twist.linear.z
        self._dvl['age'].stamp = self._now_sec()

    def _depth_cb(self, msg: PoseWithCovarianceStamped): #DepthBridge 发布的深度数据，直接使用 z 坐标表示深度
        self._depth['z'] = float(msg.pose.pose.position.z)
        self._depth['age'].stamp = self._now_sec()

    def _age_ms(self, timed: TimedValue) -> float: #计算数据的年龄（当前时间 - 数据时间戳），单位为毫秒。如果时间戳无效（<=0），返回 -1.0 表示不可用
        if timed.stamp <= 0.0:
            return -1.0
        return (self._now_sec() - timed.stamp) * 1000.0

    def _print_dashboard(self):
        odom_age = self._age_ms(self._odom['age'])
        dvl_age = self._age_ms(self._dvl['age'])
        depth_age = self._age_ms(self._depth['age'])

        def fmt_age(ms: float) -> str:
            return 'N/A' if ms < 0.0 else f'{ms:6.1f} ms'

        print('\033[2J\033[H', end='')
        print('========== USTROV Sensor Monitor ==========', flush=False)
        print(
            f"DEPTH age : {fmt_age(depth_age)}"
        )
        print(
            f"DVL age : {fmt_age(dvl_age)} | ODOM age : {fmt_age(odom_age)}"
        )
        print('-' * 58)
        print(
            'ODOM pose (NED)     '
            f"x={self._odom['x']:7.3f} y={self._odom['y']:7.3f} z={self._odom['z']:7.3f} "
            f"yaw={math.degrees(self._odom['yaw']):7.2f}"
        )
        print(
            'DVL  vel [m/s]       '
            f"vx={self._dvl['vx']:7.3f} vy={self._dvl['vy']:7.3f} vz={self._dvl['vz']:7.3f}"
        )
        print(
            'DEPTH bridge          '
            f"z={self._depth['z']:7.3f} m"
        )
        print('=' * 58, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = Px4SensorMonitor()
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