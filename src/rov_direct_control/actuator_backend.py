"""Actuator backend 抽象基类 — 统一执行后端接口。

所有 actuator backend（Gazebo / PX4 / 未来扩展）必须实现此接口。
controller_node 只依赖此抽象，不直接 import 具体实现。
"""

from abc import ABC, abstractmethod
import numpy as np


class ActuatorBackend(ABC):
    """统一的推力执行后端接口。"""

    @abstractmethod
    def send(self, thrusts: np.ndarray) -> None:
        """发送 8 路推力值（牛顿）到执行器。

        Args:
            thrusts: shape (8,) 的推力数组，单位牛顿。
        """
        ...

    @abstractmethod
    def stop(self) -> None:
        """紧急停止：立即停止所有推进器输出。"""
        ...
