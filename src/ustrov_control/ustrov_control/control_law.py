"""控制律模块（control law layer）— 纯计算，不依赖 ROS。

输入估计状态与目标，输出 6-DOF 广义力/力矩向量 tau。
当前实现：4-DOF PID（X/Y/Z/Yaw），Mx/My 保持为 0。

无 ROS 依赖，无 rclpy 依赖。
"""

import math
from dataclasses import dataclass

import numpy as np

from ustrov_control.pid import PIDController


@dataclass
class Target:
    """目标位姿 (NED 坐标系)。"""
    x: float = 0.0       # 目标 X (北, m)
    y: float = 0.0       # 目标 Y (东, m)
    z: float = 0.0       # 目标深度 (下, m), z>0 = 更深
    yaw: float = 0.0     # 目标航向 (rad)


class ControlLaw:
    """4-DOF PID 控制律

    compute(state, target) -> tau (6-DOF 广义力/力矩向量)

    内部完成：
      1. NED 世界系误差计算
      2. 世界系→机体系坐标变换
      3. 4 通道 PID 计算
      4. 限幅
      5. 组装 6-DOF tau (Mx=0, My=0)
    """

    def __init__(self,
                 kp_xy: float = 60.0, ki_xy: float = 2.0, kd_xy: float = 10.0, limit_xy: float = 280.0,
                 kp_z: float = 400.0, ki_z: float = 50.0, kd_z: float = 200.0, limit_z: float = 400.0,
                 kp_yaw: float = 5.0, ki_yaw: float = 0.5, kd_yaw: float = 2.0, limit_yaw: float = 14.0,
                 enable_xy: bool = True):

        self.enable_xy = enable_xy
        self.pid_x = PIDController(kp=kp_xy, ki=ki_xy, kd=kd_xy, limit=limit_xy)
        self.pid_y = PIDController(kp=kp_xy, ki=ki_xy, kd=kd_xy, limit=limit_xy)
        self.pid_z = PIDController(kp=kp_z, ki=ki_z, kd=kd_z, limit=limit_z)
        self.pid_yaw = PIDController(kp=kp_yaw, ki=ki_yaw, kd=kd_yaw, limit=limit_yaw)

        self.limit_xy = limit_xy
        self.limit_z = limit_z
        self.limit_yaw = limit_yaw

    def compute(self, state, target: Target, dt: float) -> np.ndarray:
        """根据当前状态和目标计算 6-DOF 广义力/力矩向量。

        Args:
            state: RovState（需有 x, y, z, yaw 属性）
            target: Target 目标位姿
            dt: 控制周期 (s)

        Returns:
            tau: shape (6,) 的力/力矩向量 [Fx, Fy, Fz, Mx, My, Mz]
        """
        # X/Y 控制
        if self.enable_xy:
            err_x_world = target.x - state.x
            err_y_world = target.y - state.y
            cos_yaw = math.cos(state.yaw)
            sin_yaw = math.sin(state.yaw)
            err_x_body = cos_yaw * err_x_world + sin_yaw * err_y_world
            err_y_body = -sin_yaw * err_x_world + cos_yaw * err_y_world
            tau_x = self.pid_x.update(err_x_body, dt)
            tau_y = self.pid_y.update(err_y_body, dt)
        else:
            tau_x = 0.0
            tau_y = 0.0
        tau_z = self.pid_z.update(target.z - state.z, dt)

        err_yaw = (target.yaw - state.yaw + math.pi) % (2 * math.pi) - math.pi
        tau_yaw = self.pid_yaw.update(err_yaw, dt)

        # 限幅
        tau_x = max(-self.limit_xy, min(self.limit_xy, tau_x))
        tau_y = max(-self.limit_xy, min(self.limit_xy, tau_y))
        tau_z = max(-self.limit_z, min(self.limit_z, tau_z))
        tau_yaw = max(-self.limit_yaw, min(self.limit_yaw, tau_yaw))

        return np.array([tau_x, tau_y, tau_z, 0.0, 0.0, tau_yaw])

    def reset(self):
        """重置所有 PID 控制器内部状态。"""
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()
