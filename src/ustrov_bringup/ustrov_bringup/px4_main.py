#!/usr/bin/env python3
"""PX4 实机模式入口 — 创建 PX4 backend + mode interface 并注入 controller。

用法:
  ros2 run ustrov_bringup px4_main
"""

from ustrov_control.controller_node import spin_controller


def _px4_factory(node):
    """PX4 backend 工厂：使用 controller 的 Node 实例创建 publisher。"""
    from ustrov_px4.px4_actuator import PX4ActuatorInterface
    from ustrov_px4.px4_interface import PX4Interface
    backend = PX4ActuatorInterface(node)
    mode_interface = PX4Interface(node)
    return backend, mode_interface, 'px4'


def main():
    spin_controller(backend_factory=_px4_factory)


if __name__ == '__main__':
    main()
