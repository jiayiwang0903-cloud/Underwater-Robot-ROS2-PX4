"""状态估计模块 — 隔离传感器来源，对外提供统一位姿。

仿真模式: GazeboEstimator  — 订阅 Gazebo 地面真值 (零噪声)
实机模式: EKFEstimator      — 融合 DVL/深度计/IMU/USBL (TODO)

controller_node.py 只调用 estimator.get_state()，不关心数据从哪来。
"""

import math
from dataclasses import dataclass, field
from abc import ABC, abstractmethod

import numpy as np

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V


@dataclass
class RovState:
    """统一位姿输出 (NED 坐标系)"""
    x: float = 0.0       # 北 (m)
    y: float = 0.0       # 东 (m)
    z: float = 0.0       # 下 (m)，正数=水下
    roll: float = 0.0    # rad
    pitch: float = 0.0   # rad
    yaw: float = 0.0     # rad
    vx: float = 0.0      # m/s (机体前)
    vy: float = 0.0      # m/s (机体右)
    vz: float = 0.0      # m/s (机体下)
    ready: bool = False   # 数据是否有效


# ────────────────────────────────────────────────────
#  基类
# ────────────────────────────────────────────────────

class StateEstimator(ABC):
    @abstractmethod
    def get_state(self) -> RovState:
        ...


# ────────────────────────────────────────────────────
#  仿真模式：直接读 Gazebo 地面真值
# ────────────────────────────────────────────────────

class GazeboEstimator(StateEstimator):
    """订阅 /world/<world>/dynamic_pose/info，从中提取指定模型的位姿。
    
    Gazebo 坐标系: FLU (X前 Y左 Z上)
    输出坐标系:     NED (X北 Y东 Z下)
    转换: x_NED = x_Gaz, y_NED = -y_Gaz, z_NED = -z_Gaz
    """

    def __init__(self, model_name: str = 'ustrov_0', world_name: str = 'default'):
        self._model_name = model_name
        self._state = RovState()
        self._prev_pos = None
        self._prev_time = None

        self._gz_node = GzNode()
        topic = f'/world/{world_name}/dynamic_pose/info'
        self._gz_node.subscribe(Pose_V, topic, self._pose_cb)

    def get_state(self) -> RovState:
        return self._state

    def _pose_cb(self, msg: Pose_V):
        for pose in msg.pose:
            if pose.name != self._model_name:
                continue

            p = pose.position
            q = pose.orientation

            # Gazebo FLU → NED
            x_ned = p.x
            y_ned = -p.y
            z_ned = -p.z

            # 四元数 FLU → NED: q_NED = (w, x, -y, -z)
            qw, qx, qy, qz = q.w, q.x, -q.y, -q.z

            roll = math.atan2(2.0 * (qw * qx + qy * qz),
                              1.0 - 2.0 * (qx * qx + qy * qy))
            pitch = math.asin(max(-1.0, min(1.0,
                              2.0 * (qw * qy - qz * qx))))
            yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz))

            self._state.x = x_ned
            self._state.y = y_ned
            self._state.z = z_ned
            self._state.roll = roll
            self._state.pitch = pitch
            self._state.yaw = yaw
            self._state.ready = True
            break


# ────────────────────────────────────────────────────
#  实机模式：EKF 融合 (预留接口)
# ────────────────────────────────────────────────────

class EKFEstimator(StateEstimator):
    """实机/仿真均可使用的 EKF 估计器。
    我们依靠 robot_localization 包的 ekf_node 在后台处理矩阵运算。
    这个类只负责订阅融合后输出的 /odometry/filtered，喂给控制器。
    """

    def __init__(self, node):
        from nav_msgs.msg import Odometry
        
        self.node = node
        self._state = RovState()
        
        # 订阅 EKF 融合后的话题
        self.sub = self.node.create_subscription(
            Odometry, 
            '/odometry/filtered', 
            self._odom_cb, 
            10)
            
    def _odom_cb(self, msg):
        # 位置
        self._state.x = msg.pose.pose.position.x
        self._state.y = msg.pose.pose.position.y
        self._state.z = msg.pose.pose.position.z
        
        # 姿态
        q = msg.pose.pose.orientation
        
        # 四元数转欧拉角
        w, x, y, z = q.w, q.x, q.y, q.z
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        
        self._state.roll = roll
        self._state.pitch = pitch
        self._state.yaw = yaw
        
        # 速度
        self._state.vx = msg.twist.twist.linear.x
        self._state.vy = msg.twist.twist.linear.y
        self._state.vz = msg.twist.twist.linear.z
        
        self._state.ready = True

    def get_state(self) -> RovState:
        return self._state
