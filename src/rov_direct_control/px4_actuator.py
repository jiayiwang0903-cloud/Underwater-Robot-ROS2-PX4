"""推力发送层 — ROS 2 至 PX4 ActuatorMotors
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

        self._pub_motors = self._node.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', qos_profile) #发布推力
        self.max_thrust_newtons = 100.0 #最大推力，用于归一化控制输入
        self.channel_signs = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]) #每个通道的符号，允许反向推力


    def send(self, thrusts_newtons: np.ndarray):
        """
        接收牛顿(N)为单位的推力数组，并转换为 [-1.0, 1.0] 的归一化控制信号发给 PX4
        """
        msg = ActuatorMotors()
        msg.timestamp = int(self._node.get_clock().now().nanoseconds / 1000) # PX4时间戳
        
        # 截断与归一化
        normalized = (thrusts_newtons / self.max_thrust_newtons) * self.channel_signs
        
        # 确保未使用的通道设为 NaN (PX4 的要求)，避免干扰
        control_array = [float('nan')] * 12
        msg.reversible_flags = (1 << NUM_THRUSTERS) - 1 # 通道可反转

        for i in range(NUM_THRUSTERS):
            if np.isnan(normalized[i]):
                control_array[i] = float('nan')
            else:
                control_array[i] = float(np.clip(normalized[i], -1.0, 1.0))
            
        msg.control = control_array
        self._pub_motors.publish(msg)

    def stop(self):
        """紧急停止：发送 NaN 放弃控制并触发失能PWM"""
        self.send(np.full(NUM_THRUSTERS, np.nan, dtype=float))
