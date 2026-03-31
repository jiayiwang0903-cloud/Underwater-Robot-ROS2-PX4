"""PX4 通信层 — 封装 Offboard 模式切换和 Arm/Disarm。

将来换成 MAVROS / MAVLink 时，只需替换本文件。
"""

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand


import time

class PX4Interface:
    def __init__(self, node: Node):
        self._node = node
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self._pub_offboard = node.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self._pub_cmd = node.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

    def send_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = True
        msg.timestamp = self._timestamp()
        self._pub_offboard.publish(msg)

    def arm(self):
        # 首先发布切换到 OFFBOARD 模式的指令: 176=DO_SET_MODE, px4_custom_main_mode=6
        self._send_command(176, 1.0, 6.0)
        # 稍微等一小会儿，给飞控0.1秒的时间完成状态机切换
        time.sleep(0.1)
        # 400 is VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0 (arm), param2=21196.0 (magic force arm)
        self._send_command(400, 1.0, 21196.0)

    def _send_command(self, command, param1=0.0, param2=0.0):
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
