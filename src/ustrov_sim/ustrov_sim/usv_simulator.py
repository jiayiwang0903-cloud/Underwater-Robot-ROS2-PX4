#!/usr/bin/env python3
"""
水面无人船 (USV) 模拟器 - 为 ROV 提供动态视觉和绝对坐标基准
最强大脑策略：
1. 它通过 Gazebo Transport 自动将物理模型注入世界，并在水面执行预设轨迹(例如走圆形/8字形)，扮演真实的 USV。
2. 它发布 /usv/gps_odom (模拟极高精度的 RTK-GPS 位置)，这是整个 EKF 系统里绝对没有慢漂移的“真北之锚”。
"""

import rclpy
from rclpy.node import Node
import math
import os
import time
from pathlib import Path

from nav_msgs.msg import Odometry
from gz.transport13 import Node as GzNode
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.msgs10.quaternion_pb2 import Quaternion

class USVSimulator(Node):
    def __init__(self):
        super().__init__('usv_simulator')
        self.get_logger().info("🌊 启动水面 USV 模拟器与 GPS 发生器...")

        # 发布完美的 USV "GPS" 坐标信息给系统参考
        self.pub_odom = self.create_publisher(Odometry, '/usv/gps_odom', 10)
        
        # Gazebo Transport 节点，用于发送模型控制指令
        self._gz_node = GzNode()
        
        # 1. 初始化时，先确保场景里有那块代表船底的 ArUco 板子
        self._spawn_usv_model()
        
        # 2. 模拟 USV 在水面上运动 (半径 3 米的圆周运动)
        self.start_time = time.time()
        self.radius = 3.0   # 运动半径
        self.speed = 0.2    # 角速度 (rad/s)
        self.usv_z = 0.0    # 永远贴在水面
        
        # 3. 定时器不断控制 USV 移动，并发送 GPS 信号
        self.timer = self.create_timer(0.05, self._timer_cb)

    def _spawn_usv_model(self):
        # 通过 Gazebo 服务注入模型
        req = EntityFactory()
        req.name = "usv_aruco"

        # 使用环境变量或常见目录自动定位模型路径，避免硬编码路径失效
        model_path = self._resolve_usv_model_path()
        if model_path is None:
            self.get_logger().error("未找到 usv_target.sdf，请设置 PX4_AUTOPILOT_PATH 环境变量")
            return

        req.sdf_filename = str(model_path)
        req.allow_renaming = True
        
        # 发送请求 (需要 5 个参数: 服务名、请求实例、请求类型、响应类型、超时时间)
        from gz.msgs10.boolean_pb2 import Boolean
        success, res = self._gz_node.request("/world/default/create", req, EntityFactory, Boolean, 1000)
        if success:
            self.get_logger().info("✅ USV 目标已成功部署至水面！")
        else:
            self.get_logger().warn("⚠️ USV 目标未生成(可能已存在)。")

    def _resolve_usv_model_path(self):
        candidates = []

        px4_path = os.environ.get("PX4_AUTOPILOT_PATH")
        if px4_path:
            candidates.append(Path(px4_path) / "Tools/simulation/gz/models/ustrov/usv_target.sdf")

        home = Path.home()
        candidates.append(home / "PX4-Autopilot/Tools/simulation/gz/models/ustrov/usv_target.sdf")

        # 从当前文件位置推导工作区，再尝试 sibling 目录下 PX4-Autopilot
        ws_root = Path(__file__).resolve().parents[3]
        candidates.append(ws_root.parent / "PX4-Autopilot/Tools/simulation/gz/models/ustrov/usv_target.sdf")

        for path in candidates:
            if path.exists():
                self.get_logger().info(f"使用 USV 模型: {path}")
                return path

        return None

    def _timer_cb(self):
        t = time.time() - self.start_time
        
        # 固定 USV 当前位置为静止绝对坐标 (例如 0, 0)
        current_x = 0.0
        current_y = 0.0
        
        # 固定USV的朝向
        current_yaw = 0.0
        
        # ==========================================
        # 1. 强制在 Gazebo 里移动物理模型 (让 ROV 相机能看到它的位移)
        # ==========================================
        pose_msg = Pose()
        pose_msg.name = "usv_aruco"
        pose_msg.position.x = current_x
        pose_msg.position.y = current_y
        pose_msg.position.z = self.usv_z
        
        # RPY -> 四元数
        cy = math.cos(current_yaw * 0.5)
        sy = math.sin(current_yaw * 0.5)
        cp = math.cos(0.0 * 0.5)
        sp = math.sin(0.0 * 0.5)
        cr = math.cos(math.pi * 0.5)  # 翻转 180度 让 ArUco 面朝水下
        sr = math.sin(math.pi * 0.5)
        
        pose_msg.orientation.w = cr*cp*cy + sr*sp*sy
        pose_msg.orientation.x = sr*cp*cy - cr*sp*sy
        pose_msg.orientation.y = cr*sp*cy + sr*cp*sy
        pose_msg.orientation.z = cr*cp*sy - sr*sp*cy
        
        # 通过 Gazebo 服务发号施令，更新姿态
        from gz.msgs10.boolean_pb2 import Boolean
        self._gz_node.request("/world/default/set_pose", pose_msg, Pose, Boolean, 100)
        
        # ==========================================
        # 2. 发送 ROS 2 标准 GPS (Odometry) 信息
        # ==========================================
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "usv_base_link"
        
        # 这里发送的坐标是 NED 标准系 (北东地)
        odom.pose.pose.position.x = current_x
        odom.pose.pose.position.y = -current_y  # Gazebo Y 到 NED Y 取反
        odom.pose.pose.position.z = -self.usv_z
        
        # 极高的置信度 (方差极小：代表 RTK-GPS 的 1cm 级别精度)
        odom.pose.covariance[0] = 0.0001
        odom.pose.covariance[7] = 0.0001
        odom.pose.covariance[14] = 0.0001
        
        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = USVSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
