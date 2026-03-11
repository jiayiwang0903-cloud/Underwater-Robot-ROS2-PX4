"""实体推力发送层 — ROS 2 至 PX4 ActuatorMotors

通过 Micro-XRCE-DDS 将推力指令发送给 Pixhawk，最终由其生成 PWM 信号控制实际电调(ESC)。
"""

import numpy as np
from rclpy.node import Node
from px4_msgs.msg import ActuatorMotors

NUM_THRUSTERS = 8

class PX4ActuatorInterface:
    def __init__(self, node: Node):
        self._node = node
        # 发布给 PX4 的混控器前物理电机输入
        self._pub_motors = self._node.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', 10)

        # 假设最大推力为 100N（根据你的真实浆叶参数修改），用于将物理受力归一化到 [-1.0, 1.0]
        self.max_thrust_newtons = 100.0

    def send(self, thrusts_newtons: np.ndarray):
        """
        接收牛顿(N)为单位的推力数组，并转换为 [-1.0, 1.0] 的归一化控制信号发给 PX4
        """
        msg = ActuatorMotors()
        # 填充时间戳
        msg.timestamp = int(self._node.get_clock().now().nanoseconds / 1000)
        
        # 截断与归一化
        normalized = np.clip(thrusts_newtons / self.max_thrust_newtons, -1.0, 1.0)
        
        # 确保未使用的通道设为 NaN (PX4 的要求)，避免干扰
        control_array = [float('nan')] * 12
        for i in range(NUM_THRUSTERS):
            control_array[i] = float(normalized[i])
            
        msg.control = control_array
        self._pub_motors.publish(msg)

    def stop(self):
        """紧急停止：全通道归零"""
        self.send(np.zeros(NUM_THRUSTERS))
