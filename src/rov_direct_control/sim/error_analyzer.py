#!/usr/bin/env python3
"""
EKF 误差监控节点 仅限仿真使用
同时监听带有噪声滤波后的 EKF 结果和 Gazebo 地面真值，实时打印误差均方根。
"""

import rclpy
from rclpy.node import Node
import math

from nav_msgs.msg import Odometry
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

class ErrorAnalyzer(Node):
    def __init__(self):
        super().__init__('error_analyzer')
        
        self.ekf_x = 0.0
        self.ekf_y = 0.0
        self.ekf_z = 0.0
        self.ekf_yaw = 0.0
        
        self.gt_x = 0.0
        self.gt_y = 0.0
        self.gt_z = 0.0
        self.gt_yaw = 0.0
        
        # 监听 EKF 的结果
        self.sub_ekf = self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 10)
        
        # 直接使用 Gazebo transport 监听真值
        self._gz_node = GzNode()
        self._gz_node.subscribe(Pose_V, '/world/default/dynamic_pose/info', self.gz_cb)
        
        # 每秒刷新一次屏幕
        self.timer = self.create_timer(1.0, self.timer_cb)
        self.get_logger().info("启动 EKF 误差监视器...")

    def euler_from_quaternion(self, w, x, y, z):
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    def ekf_cb(self, msg):
        self.ekf_x = msg.pose.pose.position.x
        self.ekf_y = msg.pose.pose.position.y
        self.ekf_z = msg.pose.pose.position.z
        
        q = msg.pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion(q.w, q.x, q.y, q.z)
        self.ekf_yaw = yaw

    def gz_cb(self, msg):
        for pose in msg.pose:
            if pose.name == 'ustrov_0':
                # 必须将 Gazebo FLU 转换为 NED，才能和 EKF 的输出对称比较
                self.gt_x = pose.position.x
                self.gt_y = -pose.position.y
                self.gt_z = -pose.position.z
                
                # 四元数转换 FLU -> NED
                w, x, y, z = pose.orientation.w, pose.orientation.x, -pose.orientation.y, -pose.orientation.z
                _, _, yaw = self.euler_from_quaternion(w, x, y, z)
                self.gt_yaw = yaw
                break

    def timer_cb(self):
        # 计算 X, Y, Z 三轴分别的物理偏差
        err_x = self.ekf_x - self.gt_x
        err_y = self.ekf_y - self.gt_y
        err_z = self.ekf_z - self.gt_z
        
        # 偏航角偏差，由于有 -pi 到 pi 的跳变环绕，使用 atan2(sin, cos) 求最小求异
        err_yaw = math.atan2(math.sin(self.ekf_yaw - self.gt_yaw), math.cos(self.ekf_yaw - self.gt_yaw))
        err_yaw_deg = math.degrees(err_yaw)
        
        # 计算总体距离偏差 (欧氏距离)
        err_dist = math.sqrt(err_x**2 + err_y**2 + err_z**2)
        
        print("\033[2J\033[H", end="") # 清屏效果，让输出一直保持在固定位置
        print("======== EKF (传感器滤波) VS Gazebo (完美真值) ========")
        print(f"当前真值 (Ground Truth): X={self.gt_x: 6.3f} | Y={self.gt_y: 6.3f} | Z={self.gt_z: 6.3f} | Yaw={math.degrees(self.gt_yaw): 6.1f}°")
        print(f"当前滤波 (EKF Estimate) : X={self.ekf_x: 6.3f} | Y={self.ekf_y: 6.3f} | Z={self.ekf_z: 6.3f} | Yaw={math.degrees(self.ekf_yaw): 6.1f}°")
        print("-" * 55)
        print(f"三轴平移误差 : [dX: {abs(err_x):.3f} m, dY: {abs(err_y):.3f} m, dZ: {abs(err_z):.3f} m]")
        print(f"姿态角度误差 : {abs(err_yaw_deg):.2f}°")
        
        # 用颜色直观显示距离状态
        color = "\033[92m" if err_dist < 0.1 else "\033[93m" if err_dist < 0.3 else "\033[91m"
        print(f"总体绝对漂移 : {color}{err_dist:.3f} 米\033[0m")
        print("=======================================================")

def main(args=None):
    rclpy.init(args=args)
    node = ErrorAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
