#!/usr/bin/env python3
"""PX4 实机模式最小 bringup。

启动链路:
  imu_bridge + dvl_bridge + depth_bridge  ->  robot_localization (EKF)  ->  controller_node (px4 backend)

前置条件:
  - Micro XRCE-DDS Agent 已运行
  - PX4 飞控已启动
  - DVL A50 传感器已连接

用法:
  ros2 launch src/rov_direct_control/launch/px4_control.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    # 1. 传感器 bridge
    imu_bridge = ExecuteProcess(
        cmd=['python3', os.path.join(_PKG_DIR, 'imu_bridge.py')],
        name='imu_bridge',
        output='screen',
        cwd=_PKG_DIR,
    )

    dvl_bridge = ExecuteProcess(
        cmd=['python3', os.path.join(_PKG_DIR, 'dvl_bridge.py')],
        name='dvl_bridge',
        output='screen',
        cwd=_PKG_DIR,
    )

    depth_bridge = ExecuteProcess(
        cmd=['python3', os.path.join(_PKG_DIR, 'depth_bridge.py')],
        name='depth_bridge',
        output='screen',
        cwd=_PKG_DIR,
    )

    # 2. EKF（实机配置）
    ekf_config = os.path.join(_PKG_DIR, 'config', 'ekf_real.yaml')
    ekf = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'robot_localization', 'ekf_node',
            '--ros-args', '--params-file', ekf_config,
        ],
        name='ekf_filter_node',
        output='screen',
    )

    # 3. 控制器（延迟 3 秒启动，等 bridge 和 EKF 就绪）
    controller = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3', os.path.join(_PKG_DIR, 'main.py'),
                    '--ros-args', '-p', 'actuator_backend:=px4',
                ],
                name='ustrov_controller',
                output='screen',
                cwd=_PKG_DIR,
            ),
        ],
    )

    return LaunchDescription([
        imu_bridge,
        dvl_bridge,
        depth_bridge,
        ekf,
        controller,
    ])
