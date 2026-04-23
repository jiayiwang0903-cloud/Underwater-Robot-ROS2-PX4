#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
实机闭环目标跟踪 launch

定位是：
1. 先复用 hw_bringup，把基础链路接好
2. 再启动 px4_main（闭环控制器入口）

- 这里不再重复启动 monitor_px4
- 这个 launch 默认假设你已经修改过 controller_node.py：
    * hold_zero_until_target = True
    * arm_on_target_only = True
    启动后不会立刻运动，
    要等真正发布 /ustrov/target_pose 以后，才允许进入自动跟踪
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
     
    # 基础 bringup 参数
     
    start_dvl_driver_arg = DeclareLaunchArgument(
        'start_dvl_driver',
        default_value='false',
        description='是否启动 DVL 驱动'
    )

    enable_dvl_bridge_arg = DeclareLaunchArgument(
        'enable_dvl_bridge',
        default_value='false',
        description='是否启动 DVL bridge'
    )

    dvl_ip_arg = DeclareLaunchArgument(
        'dvl_ip',
        default_value='192.168.194.95',
        description='DVL IP 地址'
    )

    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('ustrov_bringup'),
            'config',
            'ekf_real.yaml'
        ]),
        description='EKF 配置文件路径'
    )


     
    # 闭环控制器
    # 延迟 2 秒再启动 px4_main，
    # 给 imu_bridge / depth_bridge / ekf_node 一点起步时间
    controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ustrov_bringup',
                executable='px4_main',
                name='ustrov_controller',
                output='screen',
                parameters=[{
                    # 没收到 topic 目标前，强制零推力
                    'hold_zero_until_target': True,

                    # 只有收到 topic 目标后，才允许自动解锁
                    'arm_on_target_only': True,

                    # 允许自动解锁，但受上面那个条件约束
                    'auto_arm': True,
                }],
            )
        ]
    )

    return LaunchDescription([
        start_dvl_driver_arg,
        enable_dvl_bridge_arg,
        dvl_ip_arg,
        ekf_config_arg,


        controller,
    ])