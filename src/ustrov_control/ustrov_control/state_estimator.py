"""状态源层（state source layer）— 统一状态读取接口。

对外提供 RovState，隔离传感器来源。
controller_node 只调用 estimator.get_state()，不关心数据从哪来。
当前实现：EKFEstimator 订阅 /odometry/filtered（仿真与实机通用）。
"""

import math
from dataclasses import dataclass
from abc import ABC, abstractmethod



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
    p: float = 0.0       # rad/s (机体角速度 x)
    q: float = 0.0       # rad/s (机体角速度 y)
    r: float = 0.0       # rad/s (机体角速度 z)
    ax: float = 0.0      # m/s^2 (机体线加速度 x)
    ay: float = 0.0      # m/s^2 (机体线加速度 y)
    az: float = 0.0      # m/s^2 (机体线加速度 z)
    ready: bool = False   # 数据是否有效

class StateEstimator(ABC):
    @abstractmethod
    def get_state(self) -> RovState:
        ...

class EKFEstimator(StateEstimator):
    """使用 /odometry/filtered"""

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

        self.node.get_logger().info('EKFEstimator: 使用 /odometry/filtered (real/sim 通用)')
            
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

        # 角速度
        self._state.p = msg.twist.twist.angular.x
        self._state.q = msg.twist.twist.angular.y
        self._state.r = msg.twist.twist.angular.z

        # /odometry/filtered 不包含线加速度，保持默认值。
        self._state.ax = 0.0
        self._state.ay = 0.0
        self._state.az = 0.0
        
        self._state.ready = True

    def get_state(self) -> RovState:
        return self._state
