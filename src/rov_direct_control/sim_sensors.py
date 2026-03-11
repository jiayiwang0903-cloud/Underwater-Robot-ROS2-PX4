#!/usr/bin/env python3
"""
仿真传感器发生器 (Sim Sensors Node)
订阅 Gazebo 的地面真值，添加高斯噪声后，发布标准的 ROS 2 传感器话题。
用于在没有物理设备的情况下测试 EKF 算法。
"""

import rclpy
from rclpy.node import Node
import math
import random
import time

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V


class SimSensorsNode(Node):
    def __init__(self):
        super().__init__('sim_sensors_node')
        
        # ROS 2 传感器话题发布
        self.pub_imu = self.create_publisher(Imu, '/sensor/imu', 10)
        self.pub_dvl = self.create_publisher(TwistWithCovarianceStamped, '/sensor/dvl', 10)
        self.pub_depth = self.create_publisher(PoseWithCovarianceStamped, '/sensor/depth', 10)
        
        self.get_logger().info("启动 Gazebo 完美状态监听并注入噪声...")
        
        # 订阅 Gazebo 
        self._gz_node = GzNode()
        self._gz_node.subscribe(Pose_V, '/world/default/dynamic_pose/info', self._pose_cb)
        
        self.last_time = time.time()
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_z = 0.0

    def euler_from_quaternion(self, w, x, y, z):
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    def _pose_cb(self, msg: Pose_V):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt < 0.05:  # 限制到 20Hz
            return
            
        for pose in msg.pose:
            if pose.name != 'ustrov_0':
                continue
                
            # 获取完美值 (FLU -> NED 转换)
            # 注意: DVL 和 IMU 需要机体坐标系速度，我们需要计算
            x_ned = pose.position.x
            y_ned = -pose.position.y
            z_ned = -pose.position.z
            
            w, x, y, z = pose.orientation.w, pose.orientation.x, -pose.orientation.y, -pose.orientation.z
            roll, pitch, yaw = self.euler_from_quaternion(w, x, y, z)
            
            # 数值微分算速度(世界坐标系)
            vx_world = (x_ned - self.last_x) / dt
            vy_world = (y_ned - self.last_y) / dt
            vz_world = (z_ned - self.last_z) / dt
            
            self.last_time = current_time
            self.last_x, self.last_y, self.last_z = x_ned, y_ned, z_ned
            
            # --- 发布模拟深度计 (只管 Z) ---
            # 加入 10cm 级别的高斯噪声
            noisy_depth = z_ned + random.gauss(0, 0.1)
            
            depth_msg = PoseWithCovarianceStamped()
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = 'odom'
            depth_msg.pose.pose.position.z = noisy_depth
            depth_msg.pose.covariance[14] = 0.01  # Z的方差
            self.pub_depth.publish(depth_msg)
            
            # --- 发布模拟 DVL (计算体轴系速度) ---
            # 简化版：仅根据yaw旋转到机体坐标
            vx_body = math.cos(-yaw) * vx_world - math.sin(-yaw) * vy_world
            vy_body = math.sin(-yaw) * vx_world + math.cos(-yaw) * vy_world
            vz_body = vz_world
            
            # DVL 加噪声 (如 0.05 m/s)
            noisy_vx = vx_body + random.gauss(0, 0.05)
            noisy_vy = vy_body + random.gauss(0, 0.05)
            noisy_vz = vz_body + random.gauss(0, 0.05)
            
            dvl_msg = TwistWithCovarianceStamped()
            dvl_msg.header.stamp = self.get_clock().now().to_msg()
            dvl_msg.header.frame_id = 'base_link'
            dvl_msg.twist.twist.linear.x = noisy_vx
            dvl_msg.twist.twist.linear.y = noisy_vy
            dvl_msg.twist.twist.linear.z = noisy_vz
            dvl_msg.twist.covariance[0] = 0.0025  # X速度方差
            dvl_msg.twist.covariance[7] = 0.0025  # Y速度方差
            dvl_msg.twist.covariance[14] = 0.0025 # Z速度方差
            self.pub_dvl.publish(dvl_msg)
            
            # --- 发布模拟 IMU ---
            # 引入角度噪声 0.02 rad (~1度)
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'base_link'
            imu_msg.orientation.w = w
            imu_msg.orientation.x = x + random.gauss(0, 0.01)
            imu_msg.orientation.y = y + random.gauss(0, 0.01)
            imu_msg.orientation.z = z + random.gauss(0, 0.01)
            
            # 假设一个粗略的角度方差
            imu_msg.orientation_covariance[0] = 0.001
            imu_msg.orientation_covariance[4] = 0.001
            imu_msg.orientation_covariance[8] = 0.001
            self.pub_imu.publish(imu_msg)
            
            break

def main(args=None):
    rclpy.init(args=args)
    node = SimSensorsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()