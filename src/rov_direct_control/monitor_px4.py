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

        self._imu = {
            'p': 0.0,
            'q': 0.0,
            'r': 0.0,
            'ax': 0.0,
            'ay': 0.0,
            'az': 0.0,
            'source': 'none',
            'age': TimedValue(),
        }
        self._att = {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'age': TimedValue(),
        }
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
        self._baro = {
            'pressure_pa': 0.0,
            'temp_c': 0.0,
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

        self.create_subscription(VehicleImu, '/fmu/out/vehicle_imu', self._imu_cb, px4_qos)
        self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self._sensor_combined_cb, px4_qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self._att_cb, px4_qos)
        self.create_subscription(SensorBaro, '/fmu/out/sensor_baro', self._baro_cb, px4_qos)
        self.create_subscription(SensorBaro, '/fmu/out/sensor_baro_v1', self._baro_cb, px4_qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/sensor/dvl', self._dvl_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/sensor/depth', self._depth_cb, 10)

        self.create_timer(0.2, self._print_dashboard)  # 5 Hz
        self.get_logger().info('Sensor fusion started: monitoring IMU/ATT/BARO/DEPTH/DVL/ODOM')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _euler_from_quaternion(self, w, x, y, z):
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    def _imu_cb(self, msg: VehicleImu):
        dt_angle = max(msg.delta_angle_dt * 1e-6, 1e-6)
        dt_vel = max(msg.delta_velocity_dt * 1e-6, 1e-6)

        self._imu['p'] = float(msg.delta_angle[0]) / dt_angle
        self._imu['q'] = float(msg.delta_angle[1]) / dt_angle
        self._imu['r'] = float(msg.delta_angle[2]) / dt_angle
        self._imu['ax'] = float(msg.delta_velocity[0]) / dt_vel
        self._imu['ay'] = float(msg.delta_velocity[1]) / dt_vel
        self._imu['az'] = float(msg.delta_velocity[2]) / dt_vel
        self._imu['source'] = 'vehicle_imu'
        self._imu['age'].stamp = self._now_sec()

    def _sensor_combined_cb(self, msg: SensorCombined):
        # Fallback path for firmwares that only expose sensor_combined.
        self._imu['p'] = float(msg.gyro_rad[0])
        self._imu['q'] = float(msg.gyro_rad[1])
        self._imu['r'] = float(msg.gyro_rad[2])
        self._imu['ax'] = float(msg.accelerometer_m_s2[0])
        self._imu['ay'] = float(msg.accelerometer_m_s2[1])
        self._imu['az'] = float(msg.accelerometer_m_s2[2])
        self._imu['source'] = 'sensor_combined'
        self._imu['age'].stamp = self._now_sec()

    def _att_cb(self, msg: VehicleAttitude):
        roll, pitch, yaw = self._euler_from_quaternion(msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        self._att['roll'] = roll
        self._att['pitch'] = pitch
        self._att['yaw'] = yaw
        self._att['age'].stamp = self._now_sec()

    def _odom_cb(self, msg: Odometry):
        self._odom['x'] = msg.pose.pose.position.x
        self._odom['y'] = msg.pose.pose.position.y
        self._odom['z'] = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        _, _, yaw = self._euler_from_quaternion(q.w, q.x, q.y, q.z)
        self._odom['yaw'] = yaw
        self._odom['age'].stamp = self._now_sec()

    def _dvl_cb(self, msg: TwistWithCovarianceStamped):
        self._dvl['vx'] = msg.twist.twist.linear.x
        self._dvl['vy'] = msg.twist.twist.linear.y
        self._dvl['vz'] = msg.twist.twist.linear.z
        self._dvl['age'].stamp = self._now_sec()

    def _baro_cb(self, msg: SensorBaro):
        self._baro['pressure_pa'] = float(msg.pressure)
        self._baro['temp_c'] = float(msg.temperature)
        self._baro['age'].stamp = self._now_sec()

    def _depth_cb(self, msg: PoseWithCovarianceStamped):
        self._depth['z'] = float(msg.pose.pose.position.z)
        self._depth['age'].stamp = self._now_sec()

    def _age_ms(self, timed: TimedValue) -> float:
        if timed.stamp <= 0.0:
            return -1.0
        return (self._now_sec() - timed.stamp) * 1000.0

    def _print_dashboard(self):
        imu_age = self._age_ms(self._imu['age'])
        att_age = self._age_ms(self._att['age'])
        odom_age = self._age_ms(self._odom['age'])
        dvl_age = self._age_ms(self._dvl['age'])
        baro_age = self._age_ms(self._baro['age'])
        depth_age = self._age_ms(self._depth['age'])

        def fmt_age(ms: float) -> str:
            return 'N/A' if ms < 0.0 else f'{ms:6.1f} ms'

        print('\033[2J\033[H', end='')
        print('========== USTROV Sensor Monitor ==========', flush=False)
        print(
            f"IMU age : {fmt_age(imu_age)} ({self._imu['source']}) | "
            f"ATT age : {fmt_age(att_age)} | BARO age : {fmt_age(baro_age)} | "
            f"DEPTH age : {fmt_age(depth_age)}"
        )
        print(
            f"DVL age : {fmt_age(dvl_age)} | ODOM age : {fmt_age(odom_age)}"
        )
        print('-' * 58)
        print(
            'IMU  rates[rad/s]   '
            f"p={self._imu['p']:7.3f} q={self._imu['q']:7.3f} r={self._imu['r']:7.3f}"
        )
        print(
            'IMU  accel[m/s^2]   '
            f"ax={self._imu['ax']:7.3f} ay={self._imu['ay']:7.3f} az={self._imu['az']:7.3f}"
        )
        print(
            'ATT  euler[deg]     '
            f"roll={math.degrees(self._att['roll']):7.2f} "
            f"pitch={math.degrees(self._att['pitch']):7.2f} "
            f"yaw={math.degrees(self._att['yaw']):7.2f}"
        )
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
            'BARO raw             '
            f"P={self._baro['pressure_pa']:9.1f} Pa T={self._baro['temp_c']:6.2f} C"
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