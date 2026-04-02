"""USTROV 控制节点 — 胶水层，组装各模块。实机版。"""

import math
import os
import numpy as np

import rclpy
from rclpy.node import Node

from pid import PIDController
from allocator import allocate
from px4_interface import PX4Interface
from px4_actuator import PX4ActuatorInterface
from state_estimator import EKFEstimator
from sim.gz_thruster import GzThrusterInterface



class USTROVDirectController(Node):
    def __init__(self):
        super().__init__('ustrov_direct_controller')

        # 目标位姿 (NED: X前 Y右 Z下)
        # 通过环境变量配置：
        # USTROV_TARGET_MODE=absolute|relative (默认 absolute)
        # USTROV_TARGET_X / Y / DEPTH / YAW 用于覆盖目标值
        self.target_mode = os.getenv('USTROV_TARGET_MODE', 'relative').strip().lower()
        self.target_x = float(os.getenv('USTROV_TARGET_X', '0.0'))
        self.target_y = float(os.getenv('USTROV_TARGET_Y', '0.0'))
        self.target_depth = float(os.getenv('USTROV_TARGET_DEPTH', '5'))
        self.target_yaw = float(os.getenv('USTROV_TARGET_YAW', '0.0'))
        self._targets_initialized = False
        self.dt = 0.02
        self.log_every_ticks = int(os.getenv('USTROV_LOG_EVERY_TICKS', '1'))

        # 控制器
        self.pid_x = PIDController(kp=20.0, ki=2.0, kd=10.0, limit=100.0)
        self.pid_y = PIDController(kp=20.0, ki=2.0, kd=10.0, limit=100.0)
        self.pid_z = PIDController(kp=20.0, ki=5.0, kd=20.0, limit=100.0)
        self.pid_yaw = PIDController(kp=5.0, ki=0.5, kd=2.0, limit=50.0)

        # 状态估计
        self.estimator = EKFEstimator(self)

        # 推力执行后端硬编码切换: 'gazebo' 或 'px4'
        self.thruster_backend = 'px4'

        # 硬件接口 (PX4) 或 Gazebo 直驱接口
        self.px4 = None
        if self.thruster_backend == 'px4':
            # 修正：后端为 px4 时，加载真实硬件接口
            self.px4 = PX4Interface(self)
            self.thrusters = PX4ActuatorInterface(self)
        else:
            # 修正：否则加载仿真接口
            self.thrusters = GzThrusterInterface(model_name='ustrov_0')

        # 状态变量
        self.tick = 0
        self.control_started = False
        self.offboard_armed = False

        # 控制循环 50 Hz
        self.create_timer(self.dt, self._control_loop)
        self.get_logger().info('USTROV 4-DOF 实机控制器已启动，等待状态估计数据...')

    # ── 主控制循环 ────────────────────────────────────

    def _control_loop(self):
        self.tick += 1
        
        # 1. 无条件：维持 Offboard 心跳
        if self.px4 is not None:
            self.px4.send_offboard_mode()

        # 2. 读取状态
        state = self.estimator.get_state()

        # 3. 如果状态没准备好（或者还没开始正式控制），必须发送一组默认推力维持 PX4 的 setpoint 检查！
        if not state.ready:
            if self.px4 is not None:
                # 喂狗：发全 0 的推力或全 NaN 给 PX4，满足 Offboard 需要 setpoint 的要求
                self.thrusters.send(np.zeros(8)) 
            return

        # ============ 下面的逻辑在 state.ready == True 后才会执行 ============

        if not self.control_started:
            self.control_started = True
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
            self.pid_yaw.reset()

            # relative 模式下：用首次可用状态作为任务零点。
            # absolute 模式下：直接使用给定绝对目标（例如 depth=5m）。
            if not self._targets_initialized and self.target_mode == 'relative':
                self.target_x = state.x + self.target_x
                self.target_y = state.y + self.target_y
                self.target_depth = state.z + self.target_depth
                self.target_yaw = state.yaw + self.target_yaw
                self._targets_initialized = True
            elif not self._targets_initialized:
                self._targets_initialized = True      

            self.get_logger().info(
                f'状态估计就绪: x={state.x:.2f} y={state.y:.2f} z={state.z:.2f}m，控制启动！模式={self.target_mode}, 目标Z={self.target_depth:.2f}')                                                                  
        
        # 4. 状态机：无阻塞的解锁流程
        if self.px4 is not None and not self.offboard_armed:
            if self.tick == 50:
                self.px4.set_offboard_mode()
                self.get_logger().info("请求切换至 Offboard 模式...")
            elif self.tick == 55:  # 利用 5 个 tick (5 * 20ms = 100ms) 替代之前的 sleep(0.1)
                self.px4.arm()
                self.offboard_armed = True
                self.get_logger().info("请求解锁！")

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

        # 限幅 (防止推力越界)
        tau_x = max(-100.0, min(100.0, tau_x))
        tau_y = max(-100.0, min(100.0, tau_y))
        tau_z = max(-100.0, min(100.0, tau_z))
        tau_yaw = max(-50.0, min(50.0, tau_yaw))

        # 分配 → 发送 (给 px4_actuator 发送带有 8个推进器推力数值的数组)
        tau = np.array([tau_x, tau_y, tau_z, 0.0, 0.0, tau_yaw])
        allocated_thrusts = allocate(tau)
        self.thrusters.send(allocated_thrusts)

        if self.tick % max(1, self.log_every_ticks) == 0: 
            thrusters_str = ', '.join(f'T{i}:{v:.1f}' for i, v in enumerate(allocated_thrusts))
            self.get_logger().info(
                f'X:{state.x:.3f}(→  {self.target_x}) \n'
                f'Y:{state.y:.3f}(→  {self.target_y}) \n'
                f'Z:{state.z:.3f}(→  {self.target_depth}) \n'
                f'Yaw:{math.degrees(state.yaw):.0f}(→{math.degrees(self.target_yaw):.0f})\n'
                f'      力[X Y Z Yaw]=[{tau_x:.1f}, {tau_y:.1f}, {tau_z:.1f}, {tau_yaw:.1f}]\n'
                f'      推进器[{thrusters_str}]\n'
                f'      后端:{self.thruster_backend}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = USTROVDirectController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("中断: 关闭控制器...")
    finally:
        # 上锁并停止所有推力
        node.thrusters.stop()
        if node.px4 is not None:
            node.px4._send_command(400, 0.0, 21196.0)
        import time
        time.sleep(0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
