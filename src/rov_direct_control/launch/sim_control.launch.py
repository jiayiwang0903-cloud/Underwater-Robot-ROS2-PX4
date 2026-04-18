#!/usr/bin/env python3
"""Gazebo 仿真模式最小 bringup。

启动链路:
  sim_sensors.py  ->  robot_localization (EKF)  ->  controller_node (gazebo backend)

前置条件:
  - Gazebo 仿真环境已启动（ustrov_0 模型已加载）

用法:
  ros2 launch src/rov_direct_control/launch/sim_control.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

# 项目根目录
_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_WS_DIR = os.path.dirname(os.path.dirname(_PKG_DIR))


def generate_launch_description():
    # 1. 仿真传感器（Gazebo truth -> noisy /sensor/imu, /sensor/dvl, /sensor/depth）
    sim_sensors = ExecuteProcess(
        cmd=['python3', os.path.join(_PKG_DIR, 'sim', 'sim_sensors.py')],
        name='sim_sensors',
        output='screen',
        cwd=_PKG_DIR,
    )

    # 2. EKF（融合仿真传感器 -> /odometry/filtered）
    ekf_config = os.path.join(_PKG_DIR, 'config', 'ekf.yaml')
    ekf = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'robot_localization', 'ekf_node',
            '--ros-args', '--params-file', ekf_config,
        ],
        name='ekf_filter_node',
        output='screen',
    )

    # 3. 控制器（延迟 2 秒启动，等 EKF 就绪）
    controller = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3', os.path.join(_PKG_DIR, 'main.py'),
                    '--ros-args', '-p', 'actuator_backend:=gazebo',
                ],
                name='ustrov_controller',
                output='screen',
                cwd=_PKG_DIR,
            ),
        ],
    )

    return LaunchDescription([
        sim_sensors,
        ekf,
        controller,
    ])
