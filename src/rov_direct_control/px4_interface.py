"""PX4 通信层 — 封装 Offboard 模式切换和 Arm/Disarm。

将来换成 MAVROS / MAVLink 时，只需替换本文件。
"""

from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand


class PX4Interface:
    def __init__(self, node: Node):
        self._node = node
        self._pub_offboard = node.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self._pub_cmd = node.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

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
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)

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
