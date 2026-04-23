#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
实机直接控制 launch（使用环境变量默认目标，立即启动控制）

定位：与 closed_loop_control.launch.py 同级
- 不启动 bridges / EKF / monitor（由 hw_bringup 负责）
- 只启动 px4_main 闭环控制器
- 区别于 closed_loop_control：不等待 topic 目标，使用启动默认目标直接进入控制

使用流程：
  终端 1: ros2 launch ustrov_bringup hw_bringup.launch.py
  终端 2: ros2 launch ustrov_bringup px4_control.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    enable_xy_arg = DeclareLaunchArgument(
        'enable_xy_control',
        default_value='false',
        description='是否启用 X/Y 水平位置跟踪'
    )

    controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ustrov_bringup',
                executable='px4_main',
                name='ustrov_controller',
                output='screen',
                parameters=[{
                    'hold_zero_until_target': False,
                    'arm_on_target_only': False,
                    'auto_arm': True,
                    'enable_xy_control': LaunchConfiguration('enable_xy_control'),
                }],
            )
        ]
    )

    return LaunchDescription([
        enable_xy_arg,
        controller,
    ])
