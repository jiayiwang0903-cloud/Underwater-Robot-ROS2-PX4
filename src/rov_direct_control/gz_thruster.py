"""Gazebo Transport 推力发送层 — 隔离仿真依赖。

将来换成真机 PWM 输出时，只需替换本文件。
"""

import numpy as np
from gz.transport13 import Node as GzNode
from gz.msgs10.double_pb2 import Double as GzDouble

NUM_THRUSTERS = 8


class GzThrusterInterface:
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
