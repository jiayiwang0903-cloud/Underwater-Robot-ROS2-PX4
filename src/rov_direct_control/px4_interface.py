"""PX4 通信层 — 封装 Offboard 模式切换和 Arm/Disarm。
"""

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand

class PX4Interface:
    def __init__(self, node: Node):
        self._node = node
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # PX4 通常使用 Best Effort QoS
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self._pub_offboard = node.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile) # PX4 Offboard 模式需要持续发送 OffboardControlMode 消息作为心跳
        self._pub_cmd = node.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile) #Arm/Disarm 和模式切换通过 VehicleCommand 消息实现

    def send_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = True  # 直接控制电机输出，发送 ActuatorControl 消息
        msg.timestamp = self._timestamp()
        self._pub_offboard.publish(msg)

    def set_offboard_mode(self):
        """Offboard 模式指令"""
        self._send_command(176, 1.0, 6.0)

    def arm(self):
        """解锁指令"""
        self._send_command(400, 1.0, 21196.0)

    def _send_command(self, command, param1=0.0, param2=0.0):
        """发送 VehicleCommand 消息的通用函数"""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._timestamp()
        self._pub_cmd.publish(msg)

    def _timestamp(self) -> int:
        return int(self._node.get_clock().now().nanoseconds / 1000)
