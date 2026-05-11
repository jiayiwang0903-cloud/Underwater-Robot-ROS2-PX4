"""Gazebo actuator backend — 通过 Gazebo Transport 发送 8 路推力。

实现 ActuatorBackend 统一接口，可由 backend_factory 创建。
仅用于仿真环境，不依赖 ROS 2 Node。

Output / 输出 (Gazebo Transport):
  /model/{model_name}/joint/rotor_{i}_joint/cmd_thrust  (gz.msgs.Double)  — 8 路推力 (N)
"""

import numpy as np
from gz.transport13 import Node as GzNode
from gz.msgs10.double_pb2 import Double as GzDouble

from ustrov_control.actuator_backend import ActuatorBackend

NUM_THRUSTERS = 8


class GzThrusterInterface(ActuatorBackend):
    def __init__(self, model_name: str = 'ustrov_0'):
        self._node = GzNode()
        self._pubs = []
        for i in range(NUM_THRUSTERS):
            topic = f'/model/{model_name}/joint/rotor_{i}_joint/cmd_thrust'
            self._pubs.append(self._node.advertise(topic, GzDouble))

    def send(self, thrusts: np.ndarray):
        """发送 8 路推力值（牛顿）到 Gazebo Thruster 插件。"""
        for i in range(NUM_THRUSTERS):
            msg = GzDouble()
            msg.data = float(thrusts[i])
            self._pubs[i].publish(msg)

    def stop(self):
        """紧急停止：全部归零。"""
        for pub in self._pubs:
            msg = GzDouble()
            msg.data = 0.0
            pub.publish(msg)
