#!/usr/bin/env python3
"""Gazebo 仿真模式入口 — 创建 Gazebo backend 并注入 controller。

用法:
  ros2 run ustrov_bringup sim_main
"""

from ustrov_control.controller_node import spin_controller


def _gazebo_factory(node):
    """Gazebo backend 工厂：不需要 ROS Node，mode_interface=None。"""
    from ustrov_sim.gz_thruster import GzThrusterInterface
    backend = GzThrusterInterface(model_name='ustrov_0')
    return backend, None, 'gazebo'


def main():
    spin_controller(backend_factory=_gazebo_factory)


if __name__ == '__main__':
    main()
