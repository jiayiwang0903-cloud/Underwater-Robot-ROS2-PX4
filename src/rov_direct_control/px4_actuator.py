"""实体推力发送层 — ROS 2 至 PX4 ActuatorMotors

通过 Micro-XRCE-DDS 将推力指令发送给 Pixhawk，最终由其生成 PWM 信号控制实际电调(ESC)。
"""

import numpy as np
import os
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import ActuatorMotors

NUM_THRUSTERS = 8

class PX4ActuatorInterface:
    def __init__(self, node: Node):
        self._node = node
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 发布给 PX4 的混控器前物理电机输入
        self._pub_motors = self._node.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)

        # 假设最大推力为 100N（根据你的真实浆叶参数修改），用于将物理受力归一化到 [-1.0, 1.0]
        self.max_thrust_newtons = 100.0

        # 默认使用双向推进器语义（[-1, 1] 直传 PX4）。
        # 如需兼容旧配置，可设置环境变量 USTROV_PX4_MAP_01=1 启用 [0,1] 映射。
        self.map_to_zero_one = os.getenv('USTROV_PX4_MAP_01', '0') == '1'

        # 仿真链路中，垂向通道(4~7)方向与期望存在符号相反，默认做一次反号补偿。
        # 如需关闭可设 USTROV_INVERT_VERTICAL=0。
        invert_vertical = os.getenv('USTROV_INVERT_VERTICAL', '1') == '1'
        self.channel_signs = np.ones(NUM_THRUSTERS, dtype=float)
        if invert_vertical:
            self.channel_signs[4:8] = -1.0

    def send(self, thrusts_newtons: np.ndarray):
        """
        接收牛顿(N)为单位的推力数组，并转换为 [-1.0, 1.0] 的归一化控制信号发给 PX4
        """
        msg = ActuatorMotors()
        # 填充时间戳
        msg.timestamp = int(self._node.get_clock().now().nanoseconds / 1000)
        
        # 截断与归一化
        normalized = (thrusts_newtons / self.max_thrust_newtons) * self.channel_signs
        
        # 确保未使用的通道设为 NaN (PX4 的要求)，避免干扰
        control_array = [float('nan')] * 12

        # 前 8 路推进器都声明为可逆，允许负推力（适用于仿真与可逆推进器）。
        # 若启用 [0,1] 映射则关闭可逆标志，交给飞控按单向通道解释。
        msg.reversible_flags = 0 if self.map_to_zero_one else ((1 << NUM_THRUSTERS) - 1)

        for i in range(NUM_THRUSTERS):
            if np.isnan(normalized[i]):
                control_array[i] = float('nan')
            else:
                val_clipped = np.clip(normalized[i], -1.0, 1.0)
                if self.map_to_zero_one:
                    # 兼容旧单向语义：[-1, 1] -> [0, 1]
                    control_array[i] = float((val_clipped + 1.0) / 2.0)
                else:
                    # 双向语义：保持 [-1, 1]，由可逆通道直接解释
                    control_array[i] = float(val_clipped)
            
        msg.control = control_array
        self._pub_motors.publish(msg)

    def stop(self):
        """紧急停止：发送 NaN 放弃控制并触发失能PWM"""
        self.send(np.full(NUM_THRUSTERS, np.nan, dtype=float))
