"""USTROV 控制节点 — 胶水层，组装各模块。"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from pid import PIDController
from allocator import allocate
from gz_thruster import GzThrusterInterface
from px4_interface import PX4Interface
from state_estimator import GazeboEstimator, EKFEstimator


class USTROVDirectController(Node):
    def __init__(self):
        super().__init__('ustrov_direct_controller')

        # 目标位姿 (NED: X前 Y右 Z下)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_depth = 5.0
        self.target_yaw = 0.0
        self.dt = 0.02

        # 4-DOF PID
        self.pid_x = PIDController(kp=20.0, ki=2.0, kd=10.0, limit=100.0)
        self.pid_y = PIDController(kp=20.0, ki=2.0, kd=10.0, limit=100.0)
        self.pid_z = PIDController(kp=20.0, ki=5.0, kd=20.0, limit=200.0)
        self.pid_yaw = PIDController(kp=5.0, ki=0.5, kd=2.0, limit=50.0)

        # 状态估计
        # self.estimator = GazeboEstimator(model_name='ustrov_0', world_name='default')
        self.estimator = EKFEstimator(self)
        # 硬件接口
        self.thrusters = GzThrusterInterface(model_name='ustrov_0')
        self.px4 = PX4Interface(self)

        # 状态变量
        self.tick = 0
        self.control_started = False

        # 控制循环 50 Hz
        self.create_timer(self.dt, self._control_loop)
        self.get_logger().info('USTROV 4-DOF 控制器已启动，等待状态估计数据...')

    # ── 主控制循环 ────────────────────────────────────

    def _control_loop(self):
        self.tick += 1
        self.px4.send_offboard_mode()

        if self.tick >= 50 and self.tick % 10 == 0:
            self.px4.arm()

        state = self.estimator.get_state()
        if not state.ready:
            return

        if not self.control_started:
            self.control_started = True
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
            self.pid_yaw.reset()
            self.get_logger().info(
                f'状态估计就绪: x={state.x:.2f} y={state.y:.2f} z={state.z:.2f}m，控制启动！')

        # NED 世界坐标系下的位置误差
        err_x_world = self.target_x - state.x
        err_y_world = self.target_y - state.y

        # 世界坐标误差 → 机体坐标 (FRD)，按当前航向旋转
        cos_yaw = math.cos(state.yaw)
        sin_yaw = math.sin(state.yaw)
        err_x_body =  cos_yaw * err_x_world + sin_yaw * err_y_world
        err_y_body = -sin_yaw * err_x_world + cos_yaw * err_y_world

        # 4-DOF PID
        tau_x = self.pid_x.update(err_x_body, self.dt)
        tau_y = self.pid_y.update(err_y_body, self.dt)
        tau_z = self.pid_z.update(self.target_depth - state.z, self.dt)
        err_yaw = (self.target_yaw - state.yaw + math.pi) % (2 * math.pi) - math.pi
        tau_yaw = self.pid_yaw.update(err_yaw, self.dt)

        # 分配 → 发送
        tau = np.array([tau_x, tau_y, tau_z, 0.0, 0.0, tau_yaw])
        self.thrusters.send(allocate(tau))

        if self.tick % 50 == 0:
            self.get_logger().info(
                f'X:{state.x:.1f}(→{self.target_x}) '
                f'Y:{state.y:.1f}(→{self.target_y}) '
                f'Z:{state.z:.1f}(→{self.target_depth}) '
                f'Yaw:{math.degrees(state.yaw):.0f}° | '
                f'Fx:{tau_x:.0f} Fy:{tau_y:.0f} Fz:{tau_z:.0f}N')
#先做仿真用的 GazeboEstimator，同时预留 EKFEstimator 接口。