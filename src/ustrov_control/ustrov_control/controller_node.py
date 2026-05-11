"""USTROV 外部 4-DOF 控制节点 — 纯调度器。

该节点是 external low-level controller，不是 PX4 飞控替代品。
调度链路：state + target -> control_law -> allocate -> backend

PX4 模式接口（offboard/arm）由 px4_interface.py 独立承担，
仅在 PX4 backend 模式下激活。

Input / 输入:
  /odometry/filtered       (nav_msgs/Odometry)           — EKF 融合状态
  /ustrov/target_pose      (geometry_msgs/PoseStamped)    — 目标位姿 (NED, z>0=deeper)

Output / 输出:
  由 actuator backend 决定，参见 px4_actuator.py 或 sim/gz_thruster.py。
"""

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from ustrov_control.control_law import ControlLaw, Target
from ustrov_control.allocator import allocate
from ustrov_control.state_estimator import EKFEstimator
from ustrov_control.actuator_backend import ActuatorBackend


class USTROVDirectController(Node):
    def __init__(self, backend_factory):
        """构造控制器节点。

        Args:
            backend_factory: callable(node) -> (actuator_backend, mode_interface, backend_name)
                由 bringup 层提供，在 Node 初始化后调用以创建执行后端。
                Gazebo 模式下 mode_interface 返回 None。
        """
        super().__init__('ustrov_direct_controller')

        self.declare_parameter('hold_zero_until_target', False)
        self.declare_parameter('arm_on_target_only', False)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('enable_xy_control', True)

        self.hold_zero_until_target = self.get_parameter(
            'hold_zero_until_target').get_parameter_value().bool_value
        self.arm_on_target_only = self.get_parameter(
            'arm_on_target_only').get_parameter_value().bool_value
        self.auto_arm = self.get_parameter(
            'auto_arm').get_parameter_value().bool_value

        # 目标位姿 (NED: X前 Y右 Z下)
        # 环境变量仅作为启动默认目标，运行中通过 /ustrov/target_pose 更新。
        # topic 目标始终为 absolute（直接覆盖），不参与 relative 偏移逻辑。
        self.target_mode = os.getenv('USTROV_TARGET_MODE', 'relative').strip().lower()
        self.target = Target(
            x=float(os.getenv('USTROV_TARGET_X', '0.0')),
            y=float(os.getenv('USTROV_TARGET_Y', '0.0')),
            z=float(os.getenv('USTROV_TARGET_DEPTH', '5')),
            yaw=float(os.getenv('USTROV_TARGET_YAW', '0.0')),
        )
        self._targets_initialized = False
        self.target_source = 'default'  # 'default' | 'topic'
        self.dt = 0.02
        self.log_every_ticks = int(os.getenv('USTROV_LOG_EVERY_TICKS', '1'))

        # 目标输入接口：/ustrov/target_pose (PoseStamped, NED, z>0=deeper)
        self.create_subscription(
            PoseStamped, '/ustrov/target_pose', self._target_pose_cb, 10)

        # 控制律（纯计算，不依赖 ROS）
        enable_xy = self.get_parameter('enable_xy_control').get_parameter_value().bool_value
        self.control_law = ControlLaw(enable_xy=enable_xy)

        # 状态估计
        self.estimator = EKFEstimator(self)

        # 执行后端（由 bringup 层通过 factory 创建，控制核心不 import 具体实现）
        self.thrusters, self.mode_interface, self.thruster_backend = backend_factory(self)

        # 状态变量
        self.tick = 0
        self.control_started = False
        self.offboard_armed = False

        # 控制循环 50 Hz
        self.create_timer(self.dt, self._control_loop)
        self.get_logger().info('USTROV 4-DOF 控制器已启动，等待状态估计数据...')

    # ── 目标回调 ───────────────────────────────────────

    def _target_pose_cb(self, msg: PoseStamped):
        """处理 /ustrov/target_pose 目标更新（始终为 absolute target）。"""
        self.target.x = msg.pose.position.x
        self.target.y = msg.pose.position.y
        self.target.z = msg.pose.position.z  # NED: z>0 = deeper

        # 从四元数提取 yaw
        q = msg.pose.orientation
        w, x, y, z = q.w, q.x, q.y, q.z
        self.target.yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        if self.target_source != 'topic':
            self.target_source = 'topic'
            self._targets_initialized = True
            self.get_logger().info(
                f'目标来源切换: default -> topic | '
                f'x={self.target.x:.2f} y={self.target.y:.2f} '
                f'depth={self.target.z:.2f} yaw={math.degrees(self.target.yaw):.1f}deg')
        else:
            self.get_logger().debug(
                f'目标更新(topic): x={self.target.x:.2f} y={self.target.y:.2f} '
                f'depth={self.target.z:.2f} yaw={math.degrees(self.target.yaw):.1f}deg')

    # ── 主控制循环 ────────────────────────────────────

    def _control_loop(self):
        self.tick += 1

        # 1. 无条件：维持 Offboard 心跳
        if self.mode_interface is not None:
            self.mode_interface.send_offboard_mode()

        # 2. 读取状态
        state = self.estimator.get_state()

        # 3. 状态未就绪时，发送零推力维持 PX4 setpoint 检查
        if not state.ready:
            if self.mode_interface is not None:
                self.thrusters.send(np.zeros(8))
            return

        # state.ready == True

        if not self.control_started:
            self.control_started = True
            self.control_law.reset()

            # relative 模式：用首次可用状态作为任务零点
            if not self._targets_initialized and self.target_mode == 'relative':
                self.target.x = state.x + self.target.x
                self.target.y = state.y + self.target.y
                self.target.z = state.z + self.target.z
                self.target.yaw = state.yaw + self.target.yaw
                self._targets_initialized = True
            elif not self._targets_initialized:
                self._targets_initialized = True

            self.get_logger().info(
                f'状态估计就绪: x={state.x:.2f} y={state.y:.2f} z={state.z:.2f}m，'
                f'控制启动！模式={self.target_mode}, 目标Z={self.target.z:.2f}')
            
        if self.hold_zero_until_target and self.target_source != 'topic':
            if self.mode_interface is not None:
                self.thrusters.send(np.zeros(8))
            return

        # 4. PX4 模式状态机：无阻塞的解锁流程
        allow_auto_arm = self.auto_arm
        if self.arm_on_target_only and self.target_source != 'topic':
            allow_auto_arm = False

        if self.mode_interface is not None and not self.offboard_armed and allow_auto_arm:
            if self.tick == 50:
                self.mode_interface.set_offboard_mode()
                self.get_logger().info("请求切换至 Offboard 模式")
            elif self.tick == 55:
                self.mode_interface.arm()
                self.offboard_armed = True
                self.get_logger().info("请求解锁")



        # 5. 控制律 → 分配 → 执行
        tau = self.control_law.compute(state, self.target, self.dt)
        allocated_thrusts = allocate(tau)
        self.thrusters.send(allocated_thrusts)

        # 6. 日志
        if self.tick % max(1, self.log_every_ticks) == 0:
            thrusters_str = ', '.join(f'T{i}:{v:.1f}' for i, v in enumerate(allocated_thrusts))
            self.get_logger().info(
                f'X:{state.x:.3f}(-> {self.target.x:.2f}) '
                f'Y:{state.y:.3f}(-> {self.target.y:.2f}) '
                f'Z:{state.z:.3f}(-> {self.target.z:.2f}) '
                f'Yaw:{math.degrees(state.yaw):.0f}(-> {math.degrees(self.target.yaw):.0f})\n'
                f'      tau=[{tau[0]:.1f}, {tau[1]:.1f}, {tau[2]:.1f}, {tau[5]:.1f}] '
                f'推进器[{thrusters_str}]\n'
                f'      后端:{self.thruster_backend} 目标源:{self.target_source}'
            )


def spin_controller(backend_factory, args=None):
    """启动控制器节点并进入 spin。由 bringup 入口调用。

    Args:
        backend_factory: callable(node) -> (actuator_backend, mode_interface, backend_name)
        args: ROS 2 命令行参数
    """
    import signal
    import time

    rclpy.init(args=args)
    node = USTROVDirectController(backend_factory)

    _shutdown_done = False

    def _safe_shutdown():
        nonlocal _shutdown_done
        if _shutdown_done:
            return
        _shutdown_done = True
        try:
            node.thrusters.stop()
            if node.mode_interface is not None:
                node.mode_interface._send_command(400, 0.0, 21196.0)  # disarm
            time.sleep(0.2)
        except Exception:
            pass

    # 在 rclpy 信号处理之前拦截 SIGINT，确保 stop/disarm 在 context 有效时发出
    original_sigint = signal.getsignal(signal.SIGINT)

    def _sigint_handler(sig, frame):
        _safe_shutdown()
        if callable(original_sigint):
            original_sigint(sig, frame)
        else:
            raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _sigint_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        _safe_shutdown()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
