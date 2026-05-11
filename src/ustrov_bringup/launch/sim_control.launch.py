#!/usr/bin/env python3
"""Gazebo 仿真模式最小 bringup。

启动链路:
  sim_sensors  ->  robot_localization (EKF)  ->  sim_main (gazebo backend controller)

前置条件:
  - Gazebo 仿真环境已启动（ustrov_0 模型已加载）

用法:
  ros2 launch ustrov_bringup sim_control.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory('ustrov_bringup')
    ekf_config = os.path.join(bringup_share, 'config', 'ekf.yaml')

    sim_sensors = Node(
        package='ustrov_sim',
        executable='sim_sensors',
        name='sim_sensors',
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ustrov_bringup',
                executable='sim_main',
                name='ustrov_controller',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        sim_sensors,
        ekf,
        controller,
    ])
